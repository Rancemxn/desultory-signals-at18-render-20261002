"""Measure physical separation of simultaneously active contact paths."""
from collections import Counter
import json
import math
from pathlib import Path
import sys

from handcam_motion import point_at


def closest(a,b,screen):
    low,high=max(a['start'],b['start']),min(a['end'],b['end'])-1e-7
    if high<low:return None
    times=sorted({low,high,*(p[0] for c in (a,b) for p in c['points'] if low<p[0]<high)})
    relative=[]
    for t in times:
        pa,pb=point_at(a['points'],t),point_at(b['points'],t)
        relative.append(tuple((pa[i]-pb[i])*screen[i] for i in (0,1)))
    best=min((math.hypot(*d),t) for d,t in zip(relative,times))
    for i,(da,db) in enumerate(zip(relative,relative[1:])):
        delta=[db[k]-da[k] for k in (0,1)]
        square=sum(v*v for v in delta)
        u=max(0.,min(1.,-sum(da[k]*delta[k] for k in (0,1))/square)) if square else 0.
        best=min(best,(math.hypot(*(da[k]+u*delta[k] for k in (0,1))),times[i]+u*(times[i+1]-times[i])))
    return best


def audit(plan):
    contacts=sorted(plan['contacts'],key=lambda c:c['start'])
    pairs=[]
    for i,a in enumerate(contacts):
        for b in contacts[i+1:]:
            if b['start']>=a['end']:break
            result=closest(a,b,plan['physical_screen'])
            if result:
                distance,t=result
                pairs.append(dict(distance_mm=distance*1000,time=t,a=a['note_ids'],b=b['note_ids'],
                    fingers=[a['hand']+':'+a['finger'],b['hand']+':'+b['finger']],same_hand=a['hand']==b['hand']))
    return dict(finger_usage=dict(Counter(c['finger'] for c in contacts)),
        simultaneous_pairs=len(pairs),below_mm={str(n):sum(p['distance_mm']<n for p in pairs) for n in (5,10,15,20)},
        closest_pairs=sorted(pairs,key=lambda p:p['distance_mm'])[:80],image_inspection=False,
        scope='Planned contact centres, measured at exact minima of piecewise linear paths; excludes airborne fingers and mesh surfaces.')


if __name__=='__main__':
    report=audit(json.loads(Path(sys.argv[1]).read_text()))
    Path(sys.argv[2]).write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='closest_pairs'}))
    print(json.dumps(report['closest_pairs'][:12]))
