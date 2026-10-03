"""Whole-phrase fingering search with explicit finger order and travel costs."""
import argparse
import copy
import json
import math
from pathlib import Path

import numpy as np

from handcam_motion import FINGER_ORDER, finish_motion, palm_offsets, point_at, travel_time, world_xy


KEYS = [(hand,finger) for hand in ('left','right') for finger in ('index','middle','ring','little','thumb')]


def pair_cost(a,b,screen,offsets,comfort=None):
    comfort = comfort or {}
    samehand = np.array([[x[0]==y[0] for y in KEYS] for x in KEYS])
    samefinger = np.eye(len(KEYS),dtype=bool)
    gap = b['start']-a['end']
    cost = np.zeros((len(KEYS),len(KEYS)))
    if gap < -1e-8:
        low,high = b['start'],min(a['end'],b['end'])-1e-7
        # Linear contact paths attain their extreme relative X at these knots.
        times = sorted({low,high,*(p[0] for c in (a,b) for p in c['points'] if low<p[0]<high)})
        pa = np.array([world_xy(point_at(a['points'],t),screen) for t in times])
        pb = np.array([world_xy(point_at(b['points'],t),screen) for t in times])
        span = np.zeros_like(cost)
        reverse = np.zeros_like(cost)
        natural_x = offsets[:,None,0]-offsets[None,:,0]
        for left,right in zip(pa,pb):
            anchors = (left-offsets)[:,None,:]-(right-offsets)[None,:,:]
            span = np.maximum(span,np.linalg.norm(anchors,axis=2))
            reverse = np.maximum(reverse,-(left[0]-right[0])*np.sign(natural_x))
        ordered = np.array([[x[1]!='thumb' and y[1]!='thumb' and x!=y for y in KEYS] for x in KEYS])
        cost += samehand*(180*(span/.05)**2 + 2e6*np.maximum(0,span-.04)**2)
        cost += samehand*ordered*(reverse>.002)*1e8
        if comfort:
            # Compatible wrist anchors keep adjacent fingers relaxed instead of
            # forcing a splayed grip merely to preserve a finger preference.
            cost += samehand*350*(span/.04)**2
            min_distance = float(np.min(np.linalg.norm(pa-pb,axis=1)))
            cost += samehand*3e6*max(0,comfort.get('finger_spacing_m',.018)-min_distance)**2
        # Crossing complete hands is also expensive; a central relay remains allowed.
        for ka,(ha,_) in enumerate(KEYS):
            for kb,(hb,_) in enumerate(KEYS):
                if ha!=hb:
                    crossed = np.max((pa[:,0]-pb[:,0])*(1 if ha=='left' else -1))
                    cost[ka,kb] += 2e6*max(0,crossed-.025)**2
                    if comfort:
                        anchors_a = pa-offsets[ka]
                        anchors_b = pb-offsets[kb]
                        separation = np.min((anchors_b[:,0]-anchors_a[:,0])*(1 if ha=='left' else -1))
                        cost[ka,kb] += 2e5*max(0,comfort.get('hand_spacing_m',.065)-separation)**2
        cost[samefinger] = 1e12
    else:
        pa = np.array(world_xy(a['points'][-1][1:],screen))
        pb = np.array(world_xy(b['points'][0][1:],screen))
        distance = float(np.linalg.norm(pa-pb))
        required = travel_time(distance,1.2,12.)
        cost += samefinger*(6*(required/max(.008,gap))**2 + 600*max(0,required-gap)**2/.01)
        anchors = np.linalg.norm((pa-offsets)[:,None,:]-(pb-offsets)[None,:,:],axis=2)
        cost += samehand*30*(np.maximum(0,anchors-.015)/max(.06,gap))**2
        cost += samefinger*6*math.exp(-max(0,gap)/.65)*min(1.,a['end']-a['start']+.05)
    return cost


def graph(contacts,plan):
    measured = palm_offsets(plan['profile'])
    offsets = np.array([measured[-1 if h=='left' else 1,f][:2] for h,f in KEYS])
    unary = np.zeros((len(contacts),len(KEYS)))
    neighbors = [[] for _ in contacts]
    comfort = plan.get('comfort',{})
    priorities = comfort.get('finger_costs',{'index':0.,'middle':.1,'ring':1.,'little':12.,'thumb':35.})
    for i,c in enumerate(contacts):
        x = sum(p[1] for p in c['points'])/len(c['points'])
        for k,(hand,finger) in enumerate(KEYS):
            side = -1 if hand=='left' else 1
            unary[i,k] = 3000*max(0,-side*(x-.5)-.06)**2 + priorities[finger]
            if c.get('manual_hand') and hand!=c['manual_hand']:
                unary[i,k] += 1e9
            if c.get('fixed_finger') and (hand,finger)!=tuple(c['fixed_finger']):
                unary[i,k] += 1e12
    for i,a in enumerate(contacts):
        for j in range(i+1,len(contacts)):
            b = contacts[j]
            if b['start']>a['end']+.45:
                break
            matrix = pair_cost(a,b,plan['physical_screen'],offsets,comfort)
            neighbors[i].append((j,matrix))
            neighbors[j].append((i,matrix.T))
    return unary,neighbors


def objective(labels,unary,neighbors):
    return float(sum(unary[i,k] for i,k in enumerate(labels)) +
                 sum(matrix[labels[i],labels[j]] for i,items in enumerate(neighbors) for j,matrix in items if j>i))


def beam_assign(unary,neighbors,width=192):
    n,k = unary.shape
    histories = np.zeros((1,n),dtype=np.int8)
    totals = np.zeros(1)
    last_use = [max((j for j,_ in items),default=i) for i,items in enumerate(neighbors)]
    for i in range(n):
        values = np.tile(unary[i],(len(histories),1))
        for j,matrix in neighbors[i]:
            if j<i:
                values += matrix[:,histories[:,j]].T
        values += totals[:,None]
        ranked = np.argsort(values,axis=None)
        frontier = [j for j in range(i+1) if last_use[j]>i]
        unique,rows,scores = set(),[],[]
        for flat in ranked:
            parent,label = divmod(int(flat),k)
            row = histories[parent].copy()
            row[i] = label
            signature = row[frontier].tobytes()
            if signature in unique:
                continue
            unique.add(signature)
            rows.append(row)
            scores.append(values[parent,label])
            if len(rows)>=width:
                break
        histories,totals = np.array(rows),np.array(scores)
        if i%200==0:
            print(f'ASSIGN_BEAM {i}/{n} states={len(histories)} cost={totals.min():.3f}',flush=True)
    return histories[int(np.argmin(totals))].copy()


def descend(labels,unary,neighbors,passes=20):
    labels = labels.copy()
    def local(i):
        values = unary[i].copy()
        for j,matrix in neighbors[i]:
            values += matrix[:,labels[j]]
        return values
    for iteration in range(passes):
        changes = 0
        order = range(len(labels)) if iteration%2==0 else reversed(range(len(labels)))
        for i in order:
            values = local(i)
            k = int(np.argmin(values))
            if values[k]+1e-8<values[labels[i]]:
                labels[i]=k
                changes+=1
        for i,items in enumerate(neighbors):
            for j,matrix in items:
                if j<=i or labels[i]==labels[j]:
                    continue
                a,b = int(labels[i]),int(labels[j])
                va,vb = local(i),local(j)
                old = va[a]+vb[b]-matrix[a,b]
                new = va[b]+vb[a]-matrix[b,b]-matrix[a,a]+matrix[b,a]
                if new+1e-7<old:
                    labels[i],labels[j]=b,a
                    changes+=2
        print(f'ASSIGN_REFINE pass={iteration} changes={changes} cost={objective(labels,unary,neighbors):.3f}',flush=True)
        if not changes:
            break
    return labels


def audit(contacts,screen):
    crossings,overlaps,transfers = [],[],[]
    last = {}
    rank = {'index':0,'middle':1,'ring':2,'little':3}
    for i,a in enumerate(contacts):
        key = a['hand'],a['finger']
        if key in last:
            old = last[key]
            gap = a['start']-old['end']
            distance = math.dist(world_xy(a['points'][0][1:],screen),world_xy(old['points'][-1][1:],screen))
            transfers.append({'time':a['start'],'hand':key[0],'finger':key[1],
                              'gap_ms':gap*1000,'distance_mm':distance*1000,'average_speed_mps':distance/max(.001,gap)})
        last[key] = a
        for b in contacts[i+1:]:
            if b['start']>=a['end']-1e-8:
                break
            if a['hand']!=b['hand']:
                continue
            if a['finger']==b['finger']:
                overlaps.append({'a':a['note_ids'],'b':b['note_ids'],'time':b['start']})
                continue
            if a['finger'] not in rank or b['finger'] not in rank:
                continue
            low,high = b['start'],min(a['end'],b['end'])-1e-7
            times = {low,high,*(p[0] for c in (a,b) for p in c['points'] if low<p[0]<high)}
            direction = (1 if a['hand']=='right' else -1)*np.sign(rank[a['finger']]-rank[b['finger']])
            worst = max((-(point_at(a['points'],t)[0]-point_at(b['points'],t)[0])*screen[0]*direction,t) for t in times)
            if worst[0]>.002:
                crossings.append({'a':a['note_ids'],'b':b['note_ids'],'time':worst[1],
                                  'reverse_mm':worst[0]*1000,'hand':a['hand']})
    return {'finger_overlaps':overlaps,'same_hand_crossings':crossings,
            'fast_transfers':sorted(transfers,key=lambda x:-x['average_speed_mps'])[:60]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--beam',type=int,default=192)
    parser.add_argument('--special',nargs='*',type=Path,default=[])
    args = parser.parse_args()
    plan = json.loads(args.source.read_text())
    for path in args.special:
        replacement = json.loads(path.read_text())
        ids = {nid for c in replacement for nid in c['note_ids']}
        plan['contacts'] = [c for c in plan['contacts'] if not ids.intersection(c['note_ids'])] + replacement
        for c in replacement:
            if c.get('manual_role')=='84-89 s two-hand hold relay':
                c['fixed_finger'] = [c['hand'],c['finger']]
    contacts = sorted(plan['contacts'],key=lambda c:(c['start'],c['pointer']))
    plan['contacts'] = contacts
    plan['settings']['fingers'] = ['index','middle','ring','little','thumb']
    baseline = audit(contacts,plan['physical_screen'])
    unary,neighbors = graph(contacts,plan)
    initial = np.array([KEYS.index((c['hand'],c['finger'])) for c in contacts])
    before = objective(initial,unary,neighbors)
    beam = beam_assign(unary,neighbors,args.beam)
    a = descend(initial,unary,neighbors)
    b = descend(beam,unary,neighbors)
    labels = min((a,b),key=lambda x:objective(x,unary,neighbors))
    for c,label in zip(contacts,labels):
        c['hand'],c['finger'] = KEYS[label]
        c['pointer'] = (0 if c['hand']=='left' else 5)+FINGER_ORDER.index(c['finger'])
    plan['hand_rest'] = finish_motion(contacts,plan['profile'],plan['physical_screen'],.28/.82)
    result = audit(contacts,plan['physical_screen'])
    report = dict(before=baseline,after=result,cost_before=before,cost_after=objective(labels,unary,neighbors),beam_width=args.beam)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'assigned-motion-plan.json').write_text(json.dumps(plan,indent=2))
    (args.output/'assignment-refinement.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:len(v) for k,v in result.items()}),flush=True)


if __name__=='__main__':
    main()
