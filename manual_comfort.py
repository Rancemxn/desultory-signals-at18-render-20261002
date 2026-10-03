"""Authored corrections for coincident contacts and constrained block corridors."""
import argparse
import copy
import json
from pathlib import Path
import zipfile

from shapely.geometry import Point
from chart import load_chart
from contact_refinement import ContactGeometry, refine_contact, validate_contact


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('source',type=Path)
    parser.add_argument('chart',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    plan=json.loads(args.source.read_text())
    with zipfile.ZipFile(args.chart) as archive:
        chart=load_chart(archive.read('chart.json').decode('utf-8-sig'))
    geometry=ContactGeometry(chart,physical_screen=plan['physical_screen'],
        extra_clearance_m=plan['comfort']['extra_block_clearance_m'])
    contacts=plan['contacts']
    changed=[]
    for nid,x in ((989,.34),(620,.66)):
        c=next(c for c in contacts if c['note_ids']==[nid])
        c.pop('refinement_region',None)
        c['points']=[[c['start'],x,.70],[round(c['end']-.001,3),x,.70]]
        points,report=refine_contact(geometry,c)
        if points is None:raise RuntimeError((nid,report))
        c['points']=points
        changed.append(c)
    # The two central Holds overlap throughout this final relay. One stationary
    # fingertip covers both, with 4 mm additional clearance in the narrow slot.
    pair=[c for c in contacts if c['note_ids'] in ([621],[990]) and c['start']==86.4]
    merged=copy.deepcopy(pair[0])
    for name in ('refinement_start','refinement_end','refinement_region','refinement_companion'):
        merged.pop(name,None)
    merged.update(note_ids=[621,990],block_clearance_m=.004,joint_note_coverage=True,
        clearance_reason='The authored central slot cannot retain a stationary point with 6 mm additional clearance.',
        points=[[86.4,.499,.56],[88.515,.499,.56]],
        manual_role='Two coincident relay Holds share one central index contact')
    for c in pair:contacts.remove(c)
    contacts.append(merged)
    changed.append(merged)
    # Long Holds at 103 seconds and the Drags along the same two judgement
    # strips share fixed legal points instead of stretching two fingers apart.
    for hold,dragids,x in ((1235,list(range(1005,1029,2)),.1625),
                           (1236,list(range(1006,1029,2)),.8375)):
        ids={hold,*dragids}
        phrase=[c for c in contacts if ids.intersection(c['note_ids'])]
        if {n for c in phrase for n in c['note_ids']}!=ids:raise RuntimeError('Unexpected phrase membership')
        c=copy.deepcopy(next(c for c in phrase if hold in c['note_ids']))
        c.update(note_ids=sorted(ids),joint_note_coverage=True,points=[[c['start'],x,.70],[round(c['end']-.001,3),x,.70]],
            manual_role='103 s Hold and aligned Drags share one stationary contact')
        c.pop('fixed_finger',None)
        for old in phrase:contacts.remove(old)
        contacts.append(c)
        changed.append(c)
    # Validate every edited path and every owned note separately. The generic
    # union of strips must never conceal a missed member of a merged contact.
    for c in changed:
        if validate_contact(geometry,c,c['points']):raise RuntimeError(('invalid contact',c['note_ids']))
        for nid in c['note_ids']:
            one=copy.deepcopy(c);one['note_ids']=[nid]
            line,note=geometry.notes[nid]
            start=max(c['start'],note.seconds)
            end=min(c['end'],note.seconds+max(note.hold,.010))
            if end<=start:continue
            one.update(start=start,end=end)
            if validate_contact(geometry,one,c['points']):raise RuntimeError(('invalid member',nid))
        print(json.dumps({'notes':c['note_ids'],'start':c['start'],'points':c['points'],
                          'clearance_m':c.get('block_clearance_m',.006)}),flush=True)
    for c in contacts:
        if c.get('fixed_finger') and c.get('manual_hand'):
            c['manual_hand']=c['fixed_finger'][0]
    plan['contacts']=sorted(contacts,key=lambda c:(c['start'],c['pointer']))
    args.output.write_text(json.dumps(plan,indent=2))


if __name__=='__main__':main()
