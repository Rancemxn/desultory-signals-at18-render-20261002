"""Desultory Signals-specific manual phrase design plus bounded assignment refinement.

The authored edits are explicit below. Original note timestamps and chart are retained.
The search only assigns the authored touch paths to physical fingers.
"""
from collections import Counter, defaultdict
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import zipfile
import cmath
import numpy as np
from shapely.geometry import Point
from algo.algo4 import JudgeArea, JUDGE_HALF_DRAG, JUDGE_HALF_TAP
from algo.algo5 import Planner, Settings, State
from basis import NoteType
from algo.base import ScreenUtil, TouchAction, VirtualTouchEvent, dump_data
from chart import load_chart
from handcam import contacts_from_psap
from handcam_motion import FINGER_ORDER, attach_plan, finish_motion, point_at, world_xy, travel_time

root = Path(sys.argv[1])
target = Path(sys.argv[2])
target.mkdir(parents=True, exist_ok=True)
plan = json.loads((root / 'motion-plan.json').read_text())
baseline = copy.deepcopy(plan)
contacts = plan['contacts']
archive = Path(sys.argv[3])
with zipfile.ZipFile(archive) as source:
    chart = load_chart(source.read('chart.json').decode('utf-8-sig'))
notes = {i:(line,note) for i,(line,note) in enumerate((line,n) for line in chart.lines for n in line.notes)}
screen = plan['physical_screen']
edits = []


def by_note(nid):
    return next(c for c in contacts if nid in c['note_ids'])


def sample(c, t):
    return [round(t,3), *point_at(c['points'], t)]


# These central holds travel vertically while wide chords stay at the bottom.
# Phigros holds allow movement along the judge strip: park them near the chords
# instead of stretching one palm between opposite screen edges.
for nid in (577,943,1187):
    c=by_note(nid)
    y=c['points'][0][2]
    for p in c['points']: p[2]=y
    edits.append({'kind':'park_vertical_hold_in_judge_strip','notes':[nid],'start':c['start'],'end':c['end'],'y':y})


# At 110.2 s the paired holds cross. Exchange their continuation at the meeting point:
# each index follows the trace entering its own half, maintaining two uninterrupted contacts.
a,b = by_note(1087),by_note(310)
meeting = min((round(t/1000,3) for t in range(110350,110750)),
              key=lambda t:math.dist(point_at(a['points'],t),point_at(b['points'],t)))
ap,bp = copy.deepcopy(a['points']),copy.deepcopy(b['points'])
a['points'] = [p for p in ap if p[0]<meeting]+[sample(a,meeting)]+[p for p in bp if p[0]>meeting]
b['points'] = [p for p in bp if p[0]<meeting]+[[meeting,*point_at(bp,meeting)]]+[p for p in ap if p[0]>meeting]
a['note_ids'] = b['note_ids'] = [1087,310]
a['manual_role'],b['manual_role']='left outer-finger hold exchange','right outer-finger hold exchange'
edits.append({'kind':'paired_hold_exchange','time':meeting,'notes':[1087,310],
              'meeting_distance_mm':1000*math.dist(world_xy(point_at(ap,meeting),screen),world_xy(point_at(bp,meeting),screen))})


# The same short Flick/Drag figure recurs in the opening, the rotating middle,
# and the ending. Author each figure as a stroke, handing it across at mid-screen.
phrases=[]
for line in chart.lines:
    current=[]
    for nid,(owner,note) in notes.items():
        if owner is not line: continue
        if note.type==NoteType.FLICK:
            if len(current)>1: phrases.append(current)
            current=[nid]
        elif (current and note.type==NoteType.DRAG and
              0<note.seconds-notes[current[-1]][1].seconds<=.055 and
              note.seconds-notes[current[0]][1].seconds<=.22):
            current.append(nid)
        else:
            if len(current)>1: phrases.append(current)
            current=[]
    if len(current)>1: phrases.append(current)

def note_point(nid):
    line,note=notes[nid]
    p=line.pos(note.seconds,note.offset)
    return [round(note.seconds,3), min(.99,max(.01,p.real/chart.width)), min(.99,max(.01,p.imag/chart.height))]

# Exact duplicate streams share their Drag continuation. Keep the second Flick
# as an independent swipe, as in the original plan.
unique={}; aliases={}
for phrase in phrases:
    signature=tuple(tuple(round(x,7) for x in note_point(nid)) for nid in phrase)
    if signature in unique:
        for original,duplicate in zip(unique[signature][1:],phrase[1:]):
            aliases.setdefault(original,[]).append(duplicate)
    else:
        unique[signature]=phrase
phrases=list(unique.values())
selected_ids={nid for phrase in phrases for nid in phrase}|{nid for ids in aliases.values() for nid in ids}
originals={nid:copy.deepcopy(by_note(nid)) for nid in selected_ids}
retained=[]
for c in contacts:
    remaining=[nid for nid in c['note_ids'] if nid not in selected_ids]
    if not remaining: continue
    if len(remaining)!=len(c['note_ids']):
        for nid in remaining:
            record=copy.deepcopy(c)
            line,note=notes[nid]
            begin=round(note.seconds,3);end=round(begin+.013,3)
            record.update(note_ids=[nid],start=begin,end=end,beat=begin,
                          points=[[begin,*point_at(c['points'],begin)],[round(end-.001,3),*point_at(c['points'],end-.001)]])
            retained.append(record)
    else: retained.append(c)
contacts[:]=retained
for phrase in phrases:
    chunks=[]
    for nid in phrase:
        point=note_point(nid)
        hand=('left' if point[1]<.5 else 'right') if abs(point[1]-.5)>.015 or not chunks else chunks[-1][0]
        if not chunks or hand!=chunks[-1][0]: chunks.append((hand,[]))
        chunks[-1][1].append(nid)
    for hand,ids in chunks:
        record=copy.deepcopy(originals[ids[0]])
        path=[note_point(nid) for nid in ids]
        flick=notes[ids[0]][1].type==NoteType.FLICK
        start=round(path[0][0]-(.025 if flick else .008),3)
        end=round(path[-1][0]+.014,3)
        # A one-note Flick keeps its actual swipe, rather than becoming stationary.
        if flick and len(ids)==1:
            path=copy.deepcopy(record['points'])
            start,end=record['start'],record['end']
        else:
            path=[[start,*path[0][1:]],*path,[round(end-.001,3),*path[-1][1:]]]
        assigned=[n for nid in ids for n in [nid,*aliases.get(nid,[])]]
        record.update(start=start,end=end,beat=round(notes[ids[0]][1].seconds,3),points=path,
                      note_ids=assigned,kind='flick' if flick else 'drag',manual_hand=hand,
                      manual_role='flick-drag stroke with centre handoff')
        contacts.append(record)
        edits.append({'kind':'continuous_sweep','start':start,'hand':hand,'notes':assigned})


# Long holds spreading from the centre need one hand each. The first two opening
# holds use thumbs so that the long-held contact does not reserve an index finger.
locks={}
for nid,hand,finger in [(933,'left','thumb'),(1356,'right','thumb'),
                        (251,'right','index'),(1029,'left','index'),
                        (577,'right','thumb'),(943,'left','thumb'),(1187,'right','thumb'),
                        (1133,'left','index')]:
    c=by_note(nid);c['hand']=hand;c['finger']=finger
    locks[id(c)]=(hand,finger)
locks[id(a)]=('left','ring');locks[id(b)]=('right','ring')
for c in (a,b): c['hand'],c['finger']=locks[id(c)]

# Bottom-row chords: keep each hand on its own side and reserve the thumbs for
# the central holds. This prevents a global left/right swap between successive chords.
for c in contacts:
    if 60.594<=c['start']<65.32 and id(c) not in locks:
        x=sum(p[1] for p in c['points'])/len(c['points'])
        if abs(x-.5)>.08:
            hand='left' if x<.5 else 'right'
            finger='ring' if x<.22 or x>.78 else 'index'
            c['hand'],c['finger']=hand,finger
            locks[id(c)]=(hand,finger)
        elif c['note_ids']==[1369]:
            c['hand'],c['finger']='left','thumb'
            locks[id(c)]=('left','thumb')


def normal(c):
    line,note=notes[c['note_ids'][0]]
    angle=line.angle@note.seconds
    return -math.sin(angle),math.cos(angle)


# Separate independent contacts that coincide visually by moving along the legal
# judge strip, never sideways into another lane. Keep Hold fan-outs brief.
ordered=sorted(contacts,key=lambda c:(c['beat'], c['kind']=='flick'))
for c in ordered:
    t=c['beat']
    others=[o for o in contacts if o is not c and o['start']<=t<o['end']]
    if not others: continue
    p=world_xy(point_at(c['points'],t),screen)
    distance=min(math.dist(p,world_xy(point_at(o['points'],t),screen)) for o in others)
    if distance>=.014 or c['kind'] not in ('flick','hold'): continue
    nx,ny=normal(c)
    options=[]
    for shift in [-.018,.018,-.030,.030,-.042,.042]:
        # world_xy flips screen Y; nx,ny are chart directions.
        moved=[]
        for old in c['points']:
            weight=1. if c['kind']=='flick' else max(0.,1-(old[0]-c['start'])/.15)
            moved.append([old[0],old[1]+nx*shift/screen[0]*weight,old[2]+ny*shift/screen[1]*weight])
        if any(not(.012<=p[1]<=.988 and .012<=p[2]<=.988) for p in moved): continue
        q=world_xy(point_at(moved,t),screen)
        clearance=min(math.dist(q,world_xy(point_at(o['points'],t),screen)) for o in others)
        options.append((max(0.,.017-clearance)*10000+abs(shift)*5,moved,shift,clearance))
    if options:
        _,moved,shift,clearance=min(options,key=lambda v:v[0])
        if clearance>distance+.002:
            c['points']=moved
            edits.append({'kind':'judge_strip_separation','notes':c['note_ids'],'time':t,'shift_mm':shift*1000})


contacts.sort(key=lambda c:(c['start'],c['pointer']))
keys=[(h,f) for h in ('left','right') for f in ('index','middle','ring','thumb')]
offsets=np.array([plan['profile']['fingers'][f"{-1 if h=='left' else 1}:{f}"]['offset'][:2] for h,f in keys])
samehand=np.array([[ka[0]==kb[0] for kb in keys] for ka in keys])
samefinger=np.eye(8,dtype=bool)
labels=np.array([keys.index((c['hand'],c['finger'])) for c in contacts])
unary=np.zeros((len(contacts),8))
neighbors=[[] for _ in contacts]
locked={i:keys.index(locks[id(c)]) for i,c in enumerate(contacts) if id(c) in locks}

for i,c in enumerate(contacts):
    x=sum(p[1] for p in c['points'])/len(c['points'])
    for k,(hand,finger) in enumerate(keys):
        side=-1 if hand=='left' else 1
        unary[i,k]=100000*max(0.,-side*(x-.5)-.035)**2+{'index':0.,'middle':.3,'ring':2.,'thumb':15.}[finger]
        if c.get('manual_hand') and hand!=c['manual_hand']: unary[i,k]+=1e9
    if i in locked:
        unary[i,:]=1e12;unary[i,locked[i]]=0;labels[i]=locked[i]

for i,a0 in enumerate(contacts):
    for j in range(i+1,len(contacts)):
        b0=contacts[j]
        if b0['start']>a0['end']+.26: break
        gap=b0['start']-a0['end']
        costs=np.zeros((8,8))
        if gap<=0:
            high=min(a0['end'],b0['end'])
            low=b0['start']
            ts=np.linspace(low,max(low,high-.0001),min(21,max(3,int((high-low)/.035)+1)))
            distances=[]
            for t in ts:
                pa=np.array(world_xy(point_at(a0['points'],float(t)),screen))
                pb=np.array(world_xy(point_at(b0['points'],float(t)),screen))
                anchors=(pa-offsets)[:,None,:]-(pb-offsets)[None,:,:]
                distances.append(np.linalg.norm(anchors,axis=2))
            span=np.max(distances,axis=0)
            costs+=samehand*(60*(span/.065)**2+9000*(np.maximum(0,span-.055)/.06)**2)
            costs[samefinger]=1e10
        else:
            pa=np.array(world_xy(a0['points'][-1][1:],screen))
            pb=np.array(world_xy(b0['points'][0][1:],screen))
            distance=float(np.linalg.norm(pa-pb))
            required=max(.014,travel_time(distance,2.2,28.))
            costs+=samefinger*(3*(required/max(.008,gap))**2+220*max(0,required-gap)**2/.01)
            anchors=np.linalg.norm((pa-offsets)[:,None,:]-(pb-offsets)[None,:,:],axis=2)
            costs+=samehand*4*(np.maximum(0,anchors-.04)/max(.04,gap))**2
        neighbors[i].append((j,costs))
        neighbors[j].append((i,costs.T))


def local_scores(i):
    result=unary[i].copy()
    for j,costs in neighbors[i]: result+=costs[:,labels[j]]
    return result


def objective():
    return float(sum(unary[i,labels[i]] for i in range(len(contacts)))+
                 sum(costs[labels[i],labels[j]] for i in range(len(contacts)) for j,costs in neighbors[i] if j>i))


before=objective()
passes=[]
for iteration in range(8):
    changes=0
    order=range(len(contacts)) if iteration%2==0 else reversed(range(len(contacts)))
    for i in order:
        if i in locked: continue
        values=local_scores(i); k=int(np.argmin(values))
        if values[k]+1e-7<values[labels[i]]:
            labels[i]=k;changes+=1
    # Adjacent chord swaps escape a local minimum where neither finger can move alone.
    for i in range(len(contacts)):
        if i in locked: continue
        for j,matrix in neighbors[i]:
            if j<=i or j in locked or abs(contacts[i]['start']-contacts[j]['start'])>.08: continue
            akey,bkey=int(labels[i]),int(labels[j])
            if akey==bkey: continue
            va,vb=local_scores(i),local_scores(j)
            old=va[akey]+vb[bkey]-matrix[akey,bkey]
            new=va[bkey]+vb[akey]-matrix[bkey,bkey]-matrix[akey,akey]+matrix[bkey,akey]
            if new+1e-7<old:
                labels[i],labels[j]=bkey,akey;changes+=2
    passes.append({'iteration':iteration,'changes':changes,'cost':objective()})
    print(passes[-1],flush=True)
    if not changes: break


optimized_cost=objective()
for i,c in enumerate(contacts):
    c['hand'],c['finger']=keys[int(labels[i])]
    c['pointer']=(0 if c['hand']=='left' else 5)+FINGER_ORDER.index(c['finger'])
    c['decision']['manual_refinement']=True
contacts.sort(key=lambda c:(c['start'],c['pointer']))
plan['hand_rest']=finish_motion(contacts,plan['profile'],screen,.28/.82)
events=defaultdict(list)
for c in contacts:
    # PSAP uses milliseconds. Do not leave floating arithmetic in the contract.
    c['start']=round(c['start']*1000)/1000;c['end']=round(c['end']*1000)/1000
    for i,p in enumerate(c['points']):
        t=round(p[0]*1000)/1000;x,y=p[1:]
        p[:]=[t,x*chart.width/chart.width,y*chart.height/chart.height]
        events[round(t*1000)].append(VirtualTouchEvent(complex(x*chart.width,y*chart.height),TouchAction.DOWN if i==0 else TouchAction.MOVE,c['pointer']))
    events[round(c['end']*1000)].append(VirtualTouchEvent(complex(c['points'][-1][1]*chart.width,c['points'][-1][2]*chart.height),TouchAction.UP,c['pointer']))
answer=[(t,sorted(es,key=lambda e:(e.action!=TouchAction.UP,e.pointer_id))) for t,es in sorted(events.items())]
content=dump_data(ScreenUtil(chart.width,chart.height),answer)
plan['psap_sha256']=hashlib.sha256(content).hexdigest()
dimensions,decoded=contacts_from_psap(content)
attach_plan(content,decoded,plan)
before_ids=set(i for c in baseline['contacts'] for i in c['note_ids'])
after_ids=set(i for c in contacts for i in c['note_ids'])
assert before_ids==after_ids==set(notes), (len(before_ids),len(after_ids))
geometry_failures=[]
for nid,(line,note) in notes.items():
    assigned=[c for c in contacts if nid in c['note_ids']]
    times=[round(note.seconds*1000)/1000]
    if note.type==NoteType.HOLD:
        times += [min(round((note.seconds+note.hold)*1000)/1000-.001,note.seconds+i*.02)
                  for i in range(1,math.ceil(note.hold/.02)+1)]
    for t in times:
        when=max(note.seconds,min(t,note.seconds+note.hold-.001)) if note.type==NoteType.HOLD else note.seconds
        center=line.pos(when,note.offset)
        zone=JudgeArea(center,cmath.exp(1j*(line.angle@when)),chart.width,chart.height,
                       JUDGE_HALF_DRAG if note.type in (NoteType.DRAG,NoteType.FLICK) else JUDGE_HALF_TAP).poly
        if not any(c['start']-.001<=t<c['end']+.001 and zone.buffer(1e-8).covers(Point(
            point_at(c['points'],t)[0]*chart.width,point_at(c['points'],t)[1]*chart.height)) for c in assigned):
            geometry_failures.append({'note':nid,'time':t});break
assert not geometry_failures, geometry_failures[:20]
checker=Planner(chart,Settings(**plan['settings']),plan['profile'])
history=[]
degraded=[]
for c in contacts:
    violations,_,_=checker.constraints(c,State(tuple(history)),-1 if c['hand']=='left' else 1,c['finger'])
    c['decision']['baseline_degraded']=c['decision']['degraded']
    c['decision']['degraded']=violations
    if violations: degraded.append({'note_ids':c['note_ids'],'time':c['start'],'reasons':violations})
    history.append(c)
changes=[{'notes':c['note_ids'],'hand':c['hand'],'finger':c['finger']} for c in contacts
         if not any(old['note_ids']==c['note_ids'] and old['hand']==c['hand'] and old['finger']==c['finger'] for old in baseline['contacts'])]
report={'authored_edits':edits,'assignment_changes':changes,'objective_before':before,'objective_after':optimized_cost,
        'passes':passes,'all_2026_notes_retained':True,'psap_lifecycle_validation':'passed',
        'judge_strip_geometry':'passed for all notes and 20 ms hold samples','native_ap_validated':False}
plan['manual_review']=report
plan['diagnostics']['baseline_degraded']=plan['diagnostics'].pop('degraded')
plan['diagnostics'].update(degraded=degraded,contacts=len(contacts),finger_usage=dict(Counter(f"{c['hand']}:{c['finger']}" for c in contacts)),
                          review_note='Motion constraints recomputed; original planner estimates retained as baseline_degraded. Use baked diagnostics for actual fit.')
(target/'plan.psap').write_bytes(content)
(target/'motion-plan.json').write_text(json.dumps(plan,indent=2))
(target/'manual-review.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'authored_edits':len(edits),'assignments_changed':len(changes),'contacts':len(contacts),'objective_before':before,'objective_after':optimized_cost,'degraded':len(degraded)}),flush=True)
