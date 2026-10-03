"""Restore sustained Flick strokes while preserving the refined legal corridors."""
import argparse
import copy
import json
import math
from pathlib import Path
import zipfile

from basis import NoteType
from chart import load_chart
from contact_refinement import ContactGeometry, refine_contact, validate_contact
from handcam_motion import point_at


def length(c,screen):
    return sum(math.hypot((b[1]-a[1])*screen[0],(b[2]-a[2])*screen[1]) for a,b in zip(c['points'],c['points'][1:]))


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('source',type=Path);ap.add_argument('chart',type=Path);ap.add_argument('output',type=Path)
    args=ap.parse_args();plan=json.loads(args.source.read_text());screen=plan['physical_screen']
    with zipfile.ZipFile(args.chart) as z:chart=load_chart(z.read('chart.json').decode('utf-8-sig'))
    g=ContactGeometry(chart,physical_screen=screen,extra_clearance_m=plan['comfort']['extra_block_clearance_m'])
    changes=[]
    for c in plan['contacts']:
        if c['kind']!='flick' or c['end']-c['start']>=.07:continue
        if any(g.notes[n][1].type!=NoteType.FLICK for n in c['note_ids']):continue
        before=copy.deepcopy(c)
        last=c['points'][-1]
        previous=next((a for a in reversed(c['points'][:-1]) if math.dist(a[1:],last[1:])>1e-7),None)
        if previous is None:raise RuntimeError(('Stationary Flick',c['note_ids']))
        vector=((last[1]-previous[1])*screen[0],(last[2]-previous[2])*screen[1])
        size=math.hypot(*vector)
        # V5 standalone Flicks used a 70 ms swipe at 0.64 m/s (44.8 mm).
        # Keep the later, comfortable DOWN of the opening five-finger chord,
        # and place the extra travel after the held middle fingers release.
        end=round(c['start']+.071,3);t=round(end-.001,3)
        c['end']=end
        c['points'].append([t,last[1]+.64*vector[0]/size/screen[0]*(t-last[0]),
                             last[2]+.64*vector[1]/size/screen[1]*(t-last[0])])
        c.pop('refinement_start',None);c.pop('refinement_end',None)
        extra=set();points=None
        for attempt in range(4):
            points,report=refine_contact(g,c,extra_times=extra)
            if points is None:break
            bad=validate_contact(g,c,points,limit=100)
            if not bad:break
            extra.update(bad)
        if points is None or bad:
            # A longer stroke may need a new direction inside a narrow block
            # corridor. Search rigid swipes without shortening the motion.
            c.pop('refinement_region',None)
            origin=point_at(before['points'],before['beat'])
            angle=math.atan2(vector[1],vector[0]);options=[]
            for i in range(16):
                theta=angle+i*math.pi/8
                trial=copy.deepcopy(c)
                trial['points']=[[when,origin[0]+.64*math.cos(theta)/screen[0]*(when-c['beat']),
                    origin[1]+.64*math.sin(theta)/screen[1]*(when-c['beat'])]
                    for when in (c['start'],c['beat'],t)]
                candidate,r=refine_contact(g,trial)
                if candidate is not None and not validate_contact(g,trial,candidate):
                    beat=point_at(candidate,c['beat'])
                    shift=math.hypot((beat[0]-origin[0])*screen[0],(beat[1]-origin[1])*screen[1])
                    options.append((shift+.003*min(i,16-i),candidate))
            if options:
                _,points=min(options,key=lambda x:x[0])
            else:
                # A newly appearing block can rule out any late continuation.
                # Begin the same 70 ms stroke earlier so it still releases at
                # the already validated instant, rather than clipping travel.
                early=round(before['end']-.071,3)
                for i in range(16):
                    theta=angle+i*math.pi/8
                    trial=copy.deepcopy(c);trial.update(start=early,end=before['end'])
                    trial['points']=[[when,origin[0]+.64*math.cos(theta)/screen[0]*(when-c['beat']),
                        origin[1]+.64*math.sin(theta)/screen[1]*(when-c['beat'])]
                        for when in (early,c['beat'],round(before['end']-.001,3))]
                    candidate,r=refine_contact(g,trial)
                    if candidate is not None and not validate_contact(g,trial,candidate):
                        options.append((min(i,16-i),candidate))
                if not options:raise RuntimeError(('No legal full-length Flick',c['note_ids']))
                _,points=min(options,key=lambda x:x[0])
                c.update(start=early,end=before['end'])
        c['points']=points
        c['flick_style']='70 ms sustained slide restored from V5'
        changes.append(dict(notes=c['note_ids'],start=c['start'],before_ms=round((before['end']-before['start'])*1000),
            after_ms=round((c['end']-c['start'])*1000),before_mm=length(before,screen)*1000,after_mm=length(c,screen)*1000))
        print(json.dumps(changes[-1]),flush=True)
    args.output.write_text(json.dumps(plan,indent=2))
    args.output.with_name('flick-restoration.json').write_text(json.dumps(changes,indent=2))


if __name__=='__main__':main()
