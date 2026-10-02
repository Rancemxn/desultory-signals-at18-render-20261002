"""Repair authored touch paths against moving Phigros block areas.

This keeps timestamps and note IDs. Movement along a note's judgement strip is
allowed; the resulting plan must still pass the separate per-note coverage audit.
"""
import cmath
import math
from shapely.geometry import Point, box
from shapely.ops import nearest_points, unary_union
from algo.algo4 import JudgeArea, JUDGE_HALF_DRAG, JUDGE_HALF_TAP
from basis import NoteType
from handcam_motion import point_at


def repair_contacts(chart, contacts, edits):
    if not chart.block_areas:
        return
    notes = {i:(line,note) for i,(line,note) in enumerate((line,n) for line in chart.lines for n in line.notes)}
    screen = box(chart.width*.012, chart.height*.012, chart.width*.988, chart.height*.988)
    pause = box(0,0,chart.width*.1,chart.height*.1).union(
        box(chart.width*.9,0,chart.width,chart.height*.1))
    screen = screen.difference(pause)
    margin = chart.width*.0003
    for ci, contact in enumerate(contacts):
        original = contact['points']
        low, high = round(contact['start']*1000), round(contact['end']*1000)-1
        if not chart.block_areas.path_violations(original, contact['start'], contact['end']):
            continue
        owner_notes = [notes[n] for n in contact['note_ids']]
        times = {low, high, *(round(p[0]*1000) for p in original), *range(low,high+1,10)}
        if any(n.type == NoteType.HOLD for _,n in owner_notes):
            times.update(range(low,high+1,2))
        times.update(round(n.seconds*1000) for _,n in owner_notes if low <= round(n.seconds*1000) <= high)
        for t in chart.block_areas.key_times:
            times.update(ms for ms in (math.floor(t*1000)-1, math.floor(t*1000),math.ceil(t*1000)) if low<=ms<=high)
        times = {ms for ms in times if low<=ms<=high}
        zones = {}

        def zone(ms):
            if ms not in zones:
                t = ms/1000
                candidates = []
                holds = [(line,n) for line,n in owner_notes if n.type==NoteType.HOLD and
                         n.seconds-.002 <= t <= n.seconds+n.hold+.002]
                current = holds or sorted(owner_notes, key=lambda item:abs(item[1].seconds-t))[:1]
                for line,note in current:
                    sec = max(note.seconds,min(t,note.seconds+note.hold-.001)) if note.type==NoteType.HOLD else note.seconds
                    p = line.pos(sec,note.offset)
                    half = JUDGE_HALF_DRAG if note.type in (NoteType.DRAG,NoteType.FLICK) else JUDGE_HALF_TAP
                    candidates.append(JudgeArea(p,cmath.exp(1j*(line.angle@sec)),chart.width,chart.height,half).poly)
                z = screen.intersection(unary_union(candidates))
                inset = z.buffer(-margin*2)
                if not inset.is_empty:
                    z = inset
                # A MOVE is held until the next millisecond, including the last
                # MOVE before UP. Leave clearance on both sides of that interval.
                blocked = unary_union([chart.block_areas.forbidden(when) for when in (
                    max(contact['start'],t-.001),t,min(contact['end']-1e-7,t+.001))])
                z = z.difference(blocked.buffer(margin))
                if z.is_empty:
                    raise ValueError(f'No unblocked judgement position: notes={contact["note_ids"]}, t={t:.3f}')
                zones[ms] = z
            return zones[ms]

        def project(ms):
            xy = point_at(original,ms/1000)
            p = Point(xy[0]*chart.width,xy[1]*chart.height)
            z = zone(ms)
            q = p if z.covers(p) else nearest_points(z,p)[0]
            return [ms/1000,q.x/chart.width,q.y/chart.height]

        for attempt in range(8):
            path = [project(ms) for ms in sorted(times)]
            violations = chart.block_areas.path_violations(path,contact['start'],contact['end'],limit=200)
            if not violations:
                break
            old_count = len(times)
            for v in violations:
                ms = v['time']*1000
                times.update(i for i in (math.floor(ms),math.ceil(ms),math.floor(ms)-1,math.ceil(ms)+1) if low<=i<=high)
            if len(times)==old_count:
                raise ValueError(f'Unresolved block crossing at millisecond precision: {contact["note_ids"]} {violations[:2]}')
        else:
            raise ValueError(f'Block repair did not converge: {contact["note_ids"]}')
        distance = max(math.dist(p[1:],point_at(original,p[0])) for p in path)
        contact['points'] = path
        edits.append({'kind':'block_area_avoidance','notes':contact['note_ids'],'start':contact['start'],
                      'max_normalized_shift':distance,'samples':len(path)})
        print(f'BLOCK_REPAIR contact={ci} notes={contact["note_ids"]} samples={len(path)} shift={distance:.4f}',flush=True)


def audit_contacts(chart, contacts):
    failures = []
    for contact in contacts:
        bad = chart.block_areas.path_violations(contact['points'],contact['start'],contact['end'])
        if bad:
            failures.append({'notes':contact['note_ids'],**bad[0]})
    return failures
