"""Produce a reviewable candidate plan and text diagnostics; never overwrite inputs."""
import argparse
import copy
import json
import math
from pathlib import Path
import zipfile

from chart import load_chart
from contact_refinement import ContactGeometry, refine_contact, route_contact, validate_contact
from handcam_motion import world_xy


def metrics(contact, screen):
    distances = [(math.dist(world_xy(a[1:],screen),world_xy(b[1:],screen)), b[0]-a[0])
                 for a,b in zip(contact['points'],contact['points'][1:]) if b[0]>a[0]]
    return {'travel_mm':1000*sum(d for d,_ in distances),
            'max_speed_mps':max((d/t for d,t in distances),default=0.)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('chart', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--notes', nargs='*', type=int)
    parser.add_argument('--exclude-notes', nargs='*', type=int, default=[])
    args = parser.parse_args()
    plan = json.loads(args.source.read_text())
    with zipfile.ZipFile(args.chart) as archive:
        chart = load_chart(archive.read('chart.json').decode('utf-8-sig'))
    geometry = ContactGeometry(chart)
    reports = []
    for index,c in enumerate(plan['contacts']):
        if args.notes and not set(args.notes).intersection(c['note_ids']):
            continue
        if set(args.exclude_notes).intersection(c['note_ids']):
            continue
        before = metrics(c,plan['physical_screen'])
        extra = set()
        for attempt in range(3):
            points, report = refine_contact(geometry, c, iterations=60, extra_times=sorted(extra))
            if points is None:
                break
            invalid = validate_contact(geometry, c, points, limit=256)
            if not invalid:
                c['points'] = points
                report.update(validated=True, refinement_rounds=attempt+1)
                break
            extra.update(invalid)
            report.update(validated=False, invalid_times=invalid[:12])
        if not report.get('validated') and c['kind'] != 'flick' and report.get('reason') != 'empty_zone':
            extra = set()
            for route_attempt in range(3):
                points, routed = route_contact(geometry,c,plan['physical_screen'],extra_times=sorted(extra))
                if points is None:
                    report['routing_failure'] = routed
                    break
                invalid = validate_contact(geometry,c,points,limit=1000)
                if not invalid:
                    c['points'] = points
                    report = dict(routed,validated=True,refinement_rounds=route_attempt+1)
                    break
                extra.update(invalid)
        report.update(index=index, notes=c['note_ids'], kind=c['kind'], before=before,
                      after=metrics(c,plan['physical_screen']))
        reports.append(report)
        print(json.dumps({k:report[k] for k in ('index','notes','kind','before','after')} |
                         {k:report[k] for k in ('reason','validated','stationary','invalid_times') if k in report}),flush=True)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'candidate-motion-plan.json').write_text(json.dumps(plan,indent=2))
    (args.output/'contact-refinement.json').write_text(json.dumps({
        'boundary_mode':'aligned', 'hold_width_ratio':.5, 'drag_width_ratio':.3,
        'display_guard_normalized':geometry.display_margin,
        'candidate_only':True, 'contacts':reports},indent=2))


if __name__ == '__main__':
    main()
