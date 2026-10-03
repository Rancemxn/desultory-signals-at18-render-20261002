"""Refine the existing chart with explicit clearance and finger preferences."""
import argparse
import copy
import json
from pathlib import Path
import zipfile

from chart import load_chart
from contact_refinement import ContactGeometry, refine_contact, route_contact, validate_contact


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source',type=Path)
    parser.add_argument('chart',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--clearance',type=float,default=.006)
    args = parser.parse_args()
    plan = json.loads(args.source.read_text())
    plan['comfort'] = dict(extra_block_clearance_m=args.clearance,
        finger_spacing_m=.020,hand_spacing_m=.070,
        finger_costs=dict(index=0.,middle=3.,ring=55.,thumb=110.,little=280.))
    with zipfile.ZipFile(args.chart) as archive:
        chart=load_chart(archive.read('chart.json').decode('utf-8-sig'))
    geometry=ContactGeometry(chart,physical_screen=plan['physical_screen'],extra_clearance_m=args.clearance)
    args.output.mkdir(parents=True,exist_ok=True)
    reports=[]
    for index,c in enumerate(plan['contacts']):
        # Only genuinely authored phrases retain a finger lock. Earlier global
        # locks accidentally prevented almost the entire song from being searched.
        if not c.get('manual_role') and not c['start']<15.24:
            c.pop('fixed_finger',None)
            c.pop('manual_hand',None)
        bad=validate_contact(geometry,c,c['points'],step=.005,limit=128)
        report=dict(index=index,notes=c['note_ids'],changed=bool(bad),valid=not bad)
        if bad:
            extra=set(bad)
            for attempt in range(5):
                points,result=refine_contact(geometry,c,iterations=80,extra_times=extra)
                if points is None:
                    report.update(result)
                    break
                invalid=validate_contact(geometry,c,points,limit=256)
                if not invalid:
                    c['points']=points
                    report.update(result,valid=True)
                    break
                extra.update(invalid)
                report['invalid_times']=invalid[:12]
            if not report['valid'] and c['kind']!='flick' and report.get('reason') not in ('empty_zone','endpoint_outside_zone'):
                for attempt in range(4):
                    points,result=route_contact(geometry,c,plan['physical_screen'],extra_times=extra)
                    if points is None:
                        report['routing_failure']=result
                        break
                    invalid=validate_contact(geometry,c,points,limit=256)
                    if not invalid:
                        c['points']=points
                        report.update(result,valid=True)
                        break
                    extra.update(invalid)
                    report['invalid_times']=invalid[:12]
        reports.append(report)
        if bad or index%100==0:
            print(json.dumps(report),flush=True)
        if index%100==0:
            (args.output/'candidate-motion-plan.json').write_text(json.dumps(plan,indent=2))
            (args.output/'comfort-refinement.json').write_text(json.dumps(reports,indent=2))
    (args.output/'candidate-motion-plan.json').write_text(json.dumps(plan,indent=2))
    (args.output/'comfort-refinement.json').write_text(json.dumps(reports,indent=2))
    print('COMFORT_RESULT',json.dumps(dict(changed=sum(r['changed'] for r in reports),
        failed=sum(not r['valid'] for r in reports))),flush=True)


if __name__=='__main__':
    main()
