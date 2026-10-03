"""Choose full-length Flick directions that preserve same-hand finger order."""
import copy
import json
import math
from pathlib import Path
import sys
import zipfile
import numpy as np
from chart import load_chart
from contact_refinement import ContactGeometry,refine_contact,validate_contact
from handcam_motion import point_at,palm_offsets
from refine_assignments import audit,pair_cost,KEYS


def main():
    source,chartpath,output=map(Path,sys.argv[1:4]);plan=json.loads(source.read_text());cs=plan['contacts'];screen=plan['physical_screen']
    with zipfile.ZipFile(chartpath) as z:chart=load_chart(z.read('chart.json').decode('utf-8-sig'))
    g=ContactGeometry(chart,physical_screen=screen,extra_clearance_m=plan['comfort']['extra_block_clearance_m'])
    measured=palm_offsets(plan['profile']);offsets=np.array([measured[-1 if h=='left' else 1,f][:2] for h,f in KEYS])
    for nid in (1128,1490):
        c=next(c for c in cs if nid in c['note_ids']);origin=point_at(c['points'],c['beat']);neighbors=[b for b in cs if b is not c and b['start']<c['end'] and c['start']<b['end']]
        choices=[]
        for i in range(32):
            angle=i*math.pi/16;trial=copy.deepcopy(c);trial.pop('refinement_region',None)
            trial['points']=[[t,origin[0]+.64*math.cos(angle)/screen[0]*(t-c['beat']),origin[1]+.64*math.sin(angle)/screen[1]*(t-c['beat'])] for t in (c['start'],c['beat'],round(c['end']-.001,3))]
            path,r=refine_contact(g,trial)
            if path is None or validate_contact(g,trial,path):continue
            trial['points']=path
            if audit(sorted([trial,*neighbors],key=lambda c:c['start']),screen)['same_hand_crossings']:continue
            cost=0.
            for b in neighbors:
                a,d=sorted((trial,b),key=lambda c:c['start'])
                cost+=pair_cost(a,d,screen,offsets,plan['comfort'])[KEYS.index((a['hand'],a['finger'])),KEYS.index((d['hand'],d['finger']))]
            choices.append((cost,i,trial))
        if not choices:raise RuntimeError(('No ordered sustained swipe',nid))
        cost,i,trial=min(choices,key=lambda x:x[0]);c.clear();c.update(trial)
        print(json.dumps(dict(note=nid,direction_degrees=i*11.25,cost=cost,points=c['points'])),flush=True)
    # Collapse the converging end of the right-hand relay once both Hold bands
    # cover the same route, preserving an actual continuous held contact.
    a=next(c for c in cs if c['note_ids']==[621] and c['start']==84.208)
    b=next(c for c in cs if c['note_ids']==[990] and c['start']==84.8)
    cut=86.;tail=copy.deepcopy(a)
    tail.update(start=cut,note_ids=[621,990],joint_note_coverage=True,
        manual_role='Right-hand relay shares its converging Hold tail')
    tail['points']=[[cut,*point_at(a['points'],cut)],*[p for p in a['points'] if p[0]>cut]]
    if validate_contact(g,tail,tail['points']):raise RuntimeError('Shared relay tail is invalid')
    # Preserve the original event interval before the transition. PSAP requires
    # a millisecond between release and reuse of one physical pointer, so keep
    # the middle contact continuous and express the extra ownership by time.
    a['note_ids']=[621,990]
    a['joint_note_windows']=[dict(notes=[621,990],start=cut,end=a['end'])]
    a.pop('joint_note_coverage',None)
    endpoint=point_at(b['points'],cut-.001)
    b['end']=cut
    b['points']=[p for p in b['points'] if p[0]<cut]
    if b['points'][-1][0]<cut-.001:b['points'].append([cut-.001,*endpoint])
    output.write_text(json.dumps(plan,indent=2))


if __name__=='__main__':main()
