"""Separate simultaneous fingertips by translating complete legal contact paths."""
import argparse
import copy
import json
import math
from pathlib import Path
import zipfile

from shapely.affinity import translate
from shapely.geometry import box
from chart import load_chart
from contact_refinement import ContactGeometry,project,validate_contact
from comfort_audit import closest,audit
from handcam_motion import palm_offsets,point_at


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('source',type=Path);ap.add_argument('chart',type=Path);ap.add_argument('output',type=Path)
    args=ap.parse_args();plan=json.loads(args.source.read_text());screen=plan['physical_screen']
    with zipfile.ZipFile(args.chart) as z:chart=load_chart(z.read('chart.json').decode('utf-8-sig'))
    g=ContactGeometry(chart,physical_screen=screen,extra_clearance_m=plan['comfort']['extra_block_clearance_m'])
    contacts=sorted(plan['contacts'],key=lambda c:(c['start'],c['pointer']));plan['contacts']=contacts
    offsets=palm_offsets(plan['profile']);changes=[]
    def offset(c):return offsets[-1 if c['hand']=='left' else 1,c['finger']][:2]
    def score(c,others,delta):
        result=2e4*sum((delta[i]*screen[i])**2 for i in (0,1))
        for b in others:
            distance,t=closest(c,b,screen)
            result+=3e6*max(0,.022-distance)**2
            pa,pb=point_at(c['points'],t),point_at(b['points'],t)
            if c['hand']==b['hand']:
                oa,ob=offset(c),offset(b)
                reverse=-(pa[0]-pb[0])*screen[0]*(1 if oa[0]>ob[0] else -1)
                if c['finger']!='thumb' and b['finger']!='thumb' and reverse>.002:result+=1e7
                span=math.hypot((pa[0]-pb[0])*screen[0]-oa[0]+ob[0],-(pa[1]-pb[1])*screen[1]-oa[1]+ob[1])
                result+=2e5*max(0,span-.035)**2
            else:
                crossed=(pa[0]-pb[0])*screen[0]*(1 if c['hand']=='left' else -1)
                result+=1e6*max(0,crossed)**2
        return result
    for sweep in range(3):
        close=[]
        for i,a in enumerate(contacts):
            for b in contacts[i+1:]:
                if b['start']>=a['end']:break
                d,t=closest(a,b,screen)
                if d<.020:close.append((d,a,b))
        moved=0
        for distance,a,b in sorted(close,key=lambda x:x[0]):
            if closest(a,b,screen)[0]>=.020:continue
            options=[]
            for c in sorted((a,b),key=lambda c:(c.get('manual_role') is not None,c['end']-c['start'])):
                if c.get('collective_hold') or c.get('continuous_sweep'):continue
                if c.get('refinement_start') is not None or c.get('refinement_end') is not None:continue
                if c.get('manual_role')=='84-89 s two-hand hold relay':continue
                others=[o for o in contacts if o is not c and o['start']<c['end'] and c['start']<o['end']]
                current=score(c,others,(0,0))
                if current<1:continue
                times,zones=g.sample_zones(c,step=.02)
                allowed=box(-.045/screen[0],-.045/screen[1],.045/screen[0],.045/screen[1])
                for t,zone in zip(times,zones):
                    x,y=point_at(c['points'],float(t))
                    allowed=allowed.intersection(translate(zone,xoff=-x,yoff=-y))
                    if allowed.is_empty:break
                if allowed.is_empty:continue
                trials=[];seen=set()
                for radius in (.012,.022,.032,.044):
                    for i in range(16):
                        theta=i*math.pi/8
                        delta=project(allowed,(radius*math.cos(theta)/screen[0],radius*math.sin(theta)/screen[1]))
                        key=tuple(round(float(v),7) for v in delta)
                        if key in seen:continue
                        seen.add(key)
                        trial=copy.deepcopy(c);trial['points']=[[t,x+delta[0],y+delta[1]] for t,x,y in c['points']]
                        value=score(trial,others,delta)
                        if value+1<current:trials.append((value,delta,trial))
                for value,delta,trial in sorted(trials,key=lambda x:x[0])[:8]:
                    if validate_contact(g,trial,trial['points']):continue
                    # Preserve the existing transfer-speed bound for this finger.
                    same=[o for o in contacts if o is not c and (o['hand'],o['finger'])==(c['hand'],c['finger'])]
                    previous=max((o for o in same if o['end']<=c['start']),key=lambda o:o['end'],default=None)
                    following=min((o for o in same if o['start']>=c['end']),key=lambda o:o['start'],default=None)
                    too_fast=False
                    for one,two in ((previous,trial),(trial,following)):
                        if not plan['settings'].get('speed_limits',True):break
                        if one is None or two is None:continue
                        gap=two['start']-one['end']
                        travel=math.hypot(*((two['points'][0][i+1]-one['points'][-1][i+1])*screen[i] for i in (0,1)))
                        if travel>max(.001,1.2*gap):too_fast=True
                    if not too_fast:options.append((current-value,c,trial,delta));break
            if options:
                gain,c,trial,delta=max(options,key=lambda x:x[0])
                c['points']=trial['points'];moved+=1
                entry=dict(notes=c['note_ids'],time=c['start'],translation_mm=[float(delta[i]*screen[i]*1000) for i in (0,1)],cost_reduction=gain)
                changes.append(entry);print(json.dumps(entry),flush=True)
        print('SPREAD_PASS',sweep,moved,flush=True)
        if not moved:break
    args.output.write_text(json.dumps(plan,indent=2))
    args.output.with_name('contact-spacing-refinement.json').write_text(json.dumps(dict(changes=changes,audit=audit(plan)),indent=2))


if __name__=='__main__':main()
