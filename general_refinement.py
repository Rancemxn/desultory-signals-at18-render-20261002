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
from shapely.geometry import Point
from contact_refinement import project


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


def collective_hold(g, contact, max_contacts):
    """Cover discontinuous Hold bands with several stationary, legal contacts.

    Each contact remains clear of blocks for its complete lifetime. The
    final coverage audit still requires a real fingertip in the original
    Hold band at every checked instant; the band is never widened.
    """
    if contact['kind']!='hold':
        raise ValueError('Only Hold contacts support collective coverage')
    times=set(g.times(contact,.001))
    for nid in contact['note_ids']:
        note=g.notes[nid][1]
        times.update(note.seconds+i*.001 for i in range(math.ceil(note.hold/.001))
                     if contact['start']<=note.seconds+i*.001<contact['end'])
    times.add(contact['end']-1e-7)
    times=sorted(times)
    safe=g.screen
    if g.chart.block_areas:
        unowned=dict(contact,note_ids=[],kind='drag')
        for t in times:
            safe=safe.intersection(g.zone(unowned,float(t)))
            if safe.is_empty:
                raise ValueError('No stationary block-free region for collective Hold')
    regions=[]
    for t in times:
        zone=g.zone(contact,float(t)).intersection(safe)
        if zone.is_empty:
            raise ValueError('Hold has an empty judgement band')
        options=[(i,r.intersection(zone)) for i,r in enumerate(regions)]
        options=[(i,r) for i,r in options if not r.buffer(-2e-5).is_empty]
        if options:
            i,region=max(options,key=lambda pair:pair[1].area)
            regions[i]=region
        else:
            regions.append(zone)
            if len(regions)>max_contacts:
                raise ValueError('Collective Hold exceeds the enabled finger count')
    result=[]
    origin=point_at(contact['points'],contact['start'])
    for region in regions:
        xy=project(region.buffer(-2e-5),origin)
        c=copy.deepcopy(contact)
        c.update(collective_hold=True,rule='discontinuous_hold_coverage',
                 points=[[contact['start'],*xy],[round(contact['end']-.001,3),*xy]])
        if validate_contact(g,c,c['points']):
            raise ValueError('Collective contact crosses a blocked region')
        result.append(c)
    for t in times:
        zone=g.zone(contact,float(t)).buffer(1e-9)
        if not any(zone.covers(Point(*c['points'][0][1:])) for c in result):
            raise ValueError('Collective contacts leave a Hold coverage gap')
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
        try:
            c['points'] = legal_path(g,c,screen)
            pieces = relay(c,screen)
        except ValueError:
            if c['kind']!='hold':
                raise
            pieces=collective_hold(g,c,2*len(plan['settings']['fingers']))
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
