"""Readable per-section motion metrics and a fingering timeline for manual review."""
from collections import Counter
import json
import math
from pathlib import Path
import sys
import skia
from handcam_motion import point_at, world_xy, FINGER_ORDER

root = Path(sys.argv[1])
plan = json.loads((root / 'motion-plan.json').read_text())
contacts = plan['contacts']
screen = plan['physical_screen']
transfers = []
last = {}
for c in contacts:
    key = c['hand'], c['finger']
    if key in last:
        p = last[key]
        gap = c['start'] - p['end']
        distance = math.dist(world_xy(p['points'][-1][1:], screen), world_xy(c['points'][0][1:], screen))
        transfers.append({'time': c['start'], 'finger': ':'.join(key), 'gap_ms': gap * 1000,
                          'distance_mm': distance * 1000, 'speed_mps': distance / max(.001, gap),
                          'from': p['note_ids'], 'to': c['note_ids']})
    last[key] = c
summary = {'finger_usage': Counter(f"{c['hand']}:{c['finger']}" for c in contacts),
           'notes': len([i for c in contacts for i in c['note_ids']]),
           'contacts': len(contacts), 'windows': [],
           'worst_transfers': sorted(transfers, key=lambda c:c['speed_mps'], reverse=True)[:40]}
for start in range(0, 151, 10):
    section = [c for c in contacts if start <= c['start'] < start + 10]
    summary['windows'].append({'start': start, 'contacts': len(section),
        'degraded': sum(bool(c['decision']['degraded']) for c in section),
        'reasons': Counter(r for c in section for r in c['decision']['degraded']),
        'usage': Counter(f"{c['hand']}:{c['finger']}" for c in section)})
(root / 'audit.json').write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))

font = skia.Font(skia.Typeface('Arial'), 13)
small = skia.Font(skia.Typeface('Arial'), 10)
colors = {'left:index':0xFF60A5FA, 'left:middle':0xFF22D3EE, 'left:ring':0xFF2DD4BF,
          'left:thumb':0xFFA78BFA, 'left:little':0xFF818CF8,
          'right:index':0xFFF87171, 'right:middle':0xFFFB923C, 'right:ring':0xFFFACC15,
          'right:thumb':0xFFF472B6, 'right:little':0xFFFDA4AF}
for start in range(0, 151, 20):
    surface = skia.Surface(1500, 1000)
    canvas = surface.getCanvas()
    canvas.clear(0xFF101827)
    for row in range(4):
        low = start + row * 5
        top = 55 + row * 235
        canvas.drawString(f'{low:.1f} - {low+5:.1f} s    note X: left at bottom, right at top', 45, top - 18, font, skia.Paint(Color=0xFFE5E7EB))
        for k in range(11):
            x = 70 + k * 140
            canvas.drawLine(x, top, x, top + 185, skia.Paint(Color=0xFF374151))
            canvas.drawString(f'{low+k*.5:.1f}', x-12, top+204, small, skia.Paint(Color=0xFFCBD5E1))
        for nx in (.25,.5,.75):
            y=top+(1-nx)*185
            canvas.drawLine(70,y,1470,y,skia.Paint(Color=0xFF263244))
        for c in contacts:
            if c['start'] > low+5 or c['end'] < low:
                continue
            color=colors[c['hand']+':'+c['finger']]
            paint=skia.Paint(Color=color,StrokeWidth=2.5,AntiAlias=True)
            points=[[max(low,c['start']),*point_at(c['points'],max(low,c['start']))]]
            points += [p for p in c['points'] if low < p[0] < low+5]
            points += [[min(low+5,c['end']),*point_at(c['points'],min(low+5,c['end']))]]
            xy=[(70+(p[0]-low)*280,top+(1-p[1])*185) for p in points]
            for a,b in zip(xy,xy[1:]):
                canvas.drawLine(*a,*b,paint)
            if low <= c['start'] < low+5:
                x,y=xy[0]
                canvas.drawCircle(x,y,3.5,paint)
                if c['kind']=='hold' or c['decision']['degraded']:
                    canvas.drawString(str(c['note_ids'][0]),x+4,y-4,small,paint)
    surface.makeImageSnapshot().save(str(root / f'timeline-{start}.png'))
