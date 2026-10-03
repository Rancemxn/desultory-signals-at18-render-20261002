"""Chart-independent contact rules. No song names, note IDs or time ranges."""
import argparse
import copy
import json
import math
from pathlib import Path
import zipfile

from chart import load_chart
from contact_refinement import ContactGeometry, refine_contact, route_contact, validate_contact
from handcam_motion import point_at


def legal_path(g, contact, screen):
    extra = set()
    for attempt in range(4):
        points, report = refine_contact(g, contact, iterations=60, extra_times=extra)
        if points is None:
            break
        bad = validate_contact(g, contact, points, limit=100)
        if not bad:
            return points
        extra.update(bad)
    if contact['kind'] != 'flick':
        for attempt in range(3):
            points, report = route_contact(g, contact, screen, extra_times=extra)
            if points is None:
                break
            bad = validate_contact(g, contact, points, limit=100)
            if not bad:
                return points
            extra.update(bad)
    else:
        # Preserve a visible swipe, searching direction inside the legal corridor.
        origin = point_at(contact['points'], contact['beat'])
        for speed in (.64, .45, .28):
            for i in range(16):
                theta = i * math.pi / 8
                trial = copy.deepcopy(contact)
                times = sorted({contact['start'], contact['end']-.001,
                                *[p[0] for p in contact['points']]})
                trial['points'] = [[t, origin[0]+speed*math.cos(theta)/screen[0]*(t-contact['beat']),
                                   origin[1]+speed*math.sin(theta)/screen[1]*(t-contact['beat'])] for t in times]
                points, _ = refine_contact(g, trial)
                if points is not None and not validate_contact(g, trial, points):
                    return points
    if not validate_contact(g, contact, contact['points']):
        return contact['points']
    raise ValueError(f"No legal contact path: {contact['note_ids']}")


def relay(contact, screen):
    """Split long travelling holds into overlapping contacts; assignment chooses hands."""
    duration = contact['end']-contact['start']
    if contact['kind'] != 'hold' or duration < 3.0:
        return [contact]
    travel = sum(math.hypot((b[1]-a[1])*screen[0], (b[2]-a[2])*screen[1])
                 for a,b in zip(contact['points'],contact['points'][1:]))
    if travel < .10:
        return [contact]
    count = min(math.ceil(duration/2.5), max(2, math.ceil(travel/.12)))
    result = []
    for i in range(count):
        c = copy.deepcopy(contact)
        start = round(contact['start']+i*duration/count,3)
        end = min(contact['end'],round(contact['start']+(i+1)*duration/count+.060,3))
        c.update(start=start,end=end,rule='travelling_hold_relay')
        c['points'] = [[start,*point_at(contact['points'],start)],
                       *[p for p in contact['points'] if start<p[0]<end-.001],
                       [round(end-.001,3),*point_at(contact['points'],end-.001)]]
        result.append(c)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('source',type=Path)
    ap.add_argument('chart',type=Path)
    ap.add_argument('output',type=Path)
    args = ap.parse_args()
    plan = json.loads(args.source.read_text())
    with zipfile.ZipFile(args.chart) as z:
        chart = load_chart(z.read('chart.json').decode('utf-8-sig'))
    screen = plan['physical_screen']
    plan['comfort'] = dict(extra_block_clearance_m=.004, finger_spacing_m=.020,
        hand_spacing_m=.070, finger_costs=dict(index=0.,middle=3.,ring=55.,thumb=110.,little=280.))
    g = ContactGeometry(chart,physical_screen=screen,extra_clearance_m=.004)
    contacts, reports = [], []
    for i,c in enumerate(plan['contacts']):
        for key in ('fixed_finger','manual_hand','manual_role','refinement_region','refinement_start','refinement_end'):
            c.pop(key,None)
        original = copy.deepcopy(c['points'])
        c['points'] = legal_path(g,c,screen)
        pieces = relay(c,screen)
        for piece in pieces:
            piece['points'] = legal_path(g,piece,screen)
        contacts.extend(pieces)
        reports.append(dict(notes=c['note_ids'],changed=original!=c['points'],relay_contacts=len(pieces)))
        if i%100==0:
            print('GENERAL_CONTACT',i,len(plan['contacts']),flush=True)
    plan['contacts'] = sorted(contacts,key=lambda c:(c['start'],c['pointer']))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(plan,indent=2))
    args.output.with_name('general-rules.json').write_text(json.dumps(reports,indent=2))


if __name__ == '__main__':
    main()
