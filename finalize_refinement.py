"""Bind refined contacts to PSAP and validate geometry before any new bake."""
import argparse
from collections import Counter,defaultdict
import hashlib
import json
import math
from pathlib import Path
import zipfile

from algo.base import ScreenUtil,TouchAction,VirtualTouchEvent,dump_data
from algo.algo4 import JUDGE_HALF_DRAG,JUDGE_HALF_TAP
from algo.algo5 import Planner,Settings,State
from basis import NoteType
from chart import load_chart
from contact_refinement import ContactGeometry,validate_contact
from handcam import contacts_from_psap
from handcam_motion import attach_plan,finish_motion,point_at
from refine_assignments import audit


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source',type=Path)
    parser.add_argument('chart',type=Path)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    plan = json.loads(args.source.read_text())
    with zipfile.ZipFile(args.chart) as archive:
        chart = load_chart(archive.read('chart.json').decode('utf-8-sig'))
    contacts = sorted(plan['contacts'],key=lambda c:(c['start'],c['pointer']))
    plan['contacts'] = contacts
    allowed = set(plan['settings']['fingers'])
    if any(c['finger'] not in allowed for c in contacts):
        raise ValueError('Contact uses a disabled finger')
    plan['settings'].update(finger_speed=1.2,finger_acceleration=12.,wrist_speed=.55,wrist_acceleration=4.)
    events = defaultdict(list)
    for c in contacts:
        c['start'],c['end'] = round(c['start']*1000)/1000,round(c['end']*1000)/1000
        points = {round(t*1000)/1000:(x*chart.width/chart.width,y*chart.height/chart.height) for t,x,y in c['points']}
        c['points'] = [[t,*xy] for t,xy in sorted(points.items())]
        assert c['points'][0][0]==c['start'] and c['points'][-1][0]<c['end'], c['note_ids']
        for i,(t,x,y) in enumerate(c['points']):
            events[round(t*1000)].append(VirtualTouchEvent(complex(x*chart.width,y*chart.height),
                TouchAction.DOWN if i==0 else TouchAction.MOVE,c['pointer']))
        x,y = c['points'][-1][1:]
        events[round(c['end']*1000)].append(VirtualTouchEvent(complex(x*chart.width,y*chart.height),TouchAction.UP,c['pointer']))
    plan['hand_rest'] = finish_motion(contacts,plan['profile'],plan['physical_screen'],.28/.82)
    data = dump_data(ScreenUtil(chart.width,chart.height),[(t,sorted(items,key=lambda e:(e.action!=TouchAction.UP,e.pointer_id)))
                                                        for t,items in sorted(events.items())])
    plan['psap_sha256'] = hashlib.sha256(data).hexdigest()
    _,decoded = contacts_from_psap(data)
    attach_plan(data,decoded,plan)
    print('PSAP lifecycle and physical identities passed',flush=True)
    geometry = ContactGeometry(chart, physical_screen=plan['physical_screen'],
        extra_clearance_m=plan.get('comfort',{}).get('extra_block_clearance_m',0.))
    notes = geometry.notes
    assert {n for c in contacts for n in c['note_ids']}==set(notes)
    failures = []
    for index,c in enumerate(contacts):
        bad = validate_contact(geometry,c,c['points'])
        if bad:
            failures.append({'notes':c['note_ids'],'times':bad,'kind':'display_or_central_band'})
        native = chart.block_areas.path_violations(c['points'],c['start'],c['end'])
        if native:
            failures.append({'notes':c['note_ids'],'violations':native,'kind':'native_block'})
        if index%200==0:
            print(f'GEOMETRY {index}/{len(contacts)} failures={len(failures)}',flush=True)
    coverage_failures = []
    for nid,(line,note) in notes.items():
        assigned = [c for c in contacts if nid in c['note_ids']]
        times = [note.seconds]
        if note.type==NoteType.HOLD:
            times += [note.seconds+i*.001 for i in range(1,math.ceil(note.hold/.001))]
        half = JUDGE_HALF_DRAG if note.type in (NoteType.DRAG,NoteType.FLICK) else JUDGE_HALF_TAP
        ratio = .5 if note.type in (NoteType.HOLD,NoteType.TAP) else .3 if note.type==NoteType.DRAG else .85
        for t in times:
            if note.type==NoteType.HOLD:
                terminal=[c for c in assigned if c.get('terminal_hold_release_ms') is not None
                          and 0 <= note.seconds+note.hold-c['end'] <= .0200001]
                if terminal and t>=max(c['end'] for c in terminal):
                    continue
            when = max(note.seconds,min(t,note.seconds+note.hold-.001)) if note.type==NoteType.HOLD else note.seconds
            center,angle = line.pos(when,note.offset),line.angle@when
            covered = False
            for c in assigned:
                if not c['start']-.0011<=t<c['end']+.0011:
                    continue
                u,v = point_at(c['points'],t)
                dx,dy = u*chart.width-center.real,v*chart.height-center.imag
                if abs(dx*math.cos(angle)+dy*math.sin(angle)) <= chart.width*half*ratio+1e-7:
                    covered = True
                    break
            if not covered:
                coverage_failures.append({'note':nid,'time':t,'type':int(note.type)})
                break
    topology = audit(contacts,plan['physical_screen'])
    report = dict(notes=len(notes),contacts=len(contacts),allowed_fingers=sorted(allowed),
        used_fingers=sorted({(c['hand'],c['finger']) for c in contacts}),
        boundary_mode='aligned',tap_width_ratio=.5,hold_width_ratio=.5,drag_width_ratio=.3,
        terminal_hold_releases=[dict(notes=c['note_ids'],end=c['end'],early_ms=c['terminal_hold_release_ms'],
                                    reason=c['terminal_hold_release_reason'])
                                for c in contacts if 'terminal_hold_release_ms' in c],
        extra_block_clearance_m=geometry.extra_clearance_m,
        clearance_exceptions=[{'notes':c['note_ids'],'start':c['start'],'end':c['end'],
            'clearance_m':c['block_clearance_m'],'reason':c.get('clearance_reason')}
            for c in contacts if 'block_clearance_m' in c],
        display_guard_normalized=geometry.display_margin,geometry_failures=failures,note_coverage_failures=coverage_failures,
        topology=topology,psap_lifecycle='passed',sample_step_ms=1,block_event_boundaries_checked=True,
        native_ap_validated=False,image_inspection=False)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'refinement-validation.json').write_text(json.dumps(report,indent=2))
    if failures or coverage_failures or topology['finger_overlaps'] or topology['same_hand_crossings']:
        raise RuntimeError({'geometry':failures[:4],'coverage':coverage_failures[:4],
                            'overlaps':topology['finger_overlaps'][:4],'crossings':topology['same_hand_crossings'][:4]})
    checker = Planner(chart,Settings(**plan['settings']),plan['profile'])
    history,degraded = [],[]
    for c in contacts:
        violations,_,_ = checker.constraints(c,State(tuple(history)),-1 if c['hand']=='left' else 1,c['finger'])
        c.setdefault('decision',{})['degraded'] = violations
        if violations:
            degraded.append({'note_ids':c['note_ids'],'time':c['start'],'reasons':violations})
        history.append(c)
    plan['manual_review'] = report
    plan['diagnostics'].update(degraded=degraded,contacts=len(contacts),
        finger_usage=dict(Counter(c['hand']+':'+c['finger'] for c in contacts)),
        review_note='Geometric contact and finger-order checks passed. Actual saved pose requires separate validation.')
    (args.output/'motion-plan.json').write_text(json.dumps(plan,indent=2))
    (args.output/'plan.psap').write_bytes(data)
    print(json.dumps({'notes':len(notes),'contacts':len(contacts),'degraded_motion_estimates':len(degraded),
                      'geometry':'passed','native_ap_validated':False}),flush=True)


if __name__=='__main__':
    main()
