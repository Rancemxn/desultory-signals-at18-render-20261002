"""Whole-phrase fingering search with explicit finger order and travel costs."""
import argparse
import copy
import json
import math
from pathlib import Path

import numpy as np

from handcam_motion import FINGER_ORDER, finish_motion, palm_offsets, point_at, travel_time, world_xy


KEYS = [(hand,finger) for hand in ('left','right') for finger in ('index','middle','ring','little','thumb')]


def assignment_keys(plan):
    allowed = plan.get('settings',{}).get('fingers',[k[1] for k in KEYS])
    keys = [k for k in KEYS if k[1] in allowed]
    if not keys:
        raise ValueError('No enabled fingers for assignment')
    return keys


def pair_cost(a,b,screen,offsets,comfort=None,keys=KEYS,load_only=False):
    comfort = comfort or {}
    samehand = np.array([[x[0]==y[0] for y in keys] for x in keys])
    samefinger = np.eye(len(keys),dtype=bool)
    gap = b['start']-a['end']
    cost = np.zeros((len(keys),len(keys)))
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
        ordered = np.array([[x[1]!='thumb' and y[1]!='thumb' and x!=y for y in keys] for x in keys])
        if load_only:
            cost[samehand & ordered & (reverse>.002)] = 1e8
            cost[samefinger] = 1e12
            return cost
        cost += samehand*(180*(span/.05)**2 + 2e6*np.maximum(0,span-.04)**2)
        cost += samehand*ordered*(reverse>.002)*1e8
        if comfort:
            # Compatible wrist anchors keep adjacent fingers relaxed instead of
            # forcing a splayed grip merely to preserve a finger preference.
            cost += samehand*350*(span/.04)**2
            min_distance = float(np.min(np.linalg.norm(pa-pb,axis=1)))
            cost += samehand*3e6*max(0,comfort.get('finger_spacing_m',.018)-min_distance)**2
        # Crossing complete hands is also expensive; a central relay remains allowed.
        for ka,(ha,_) in enumerate(keys):
            for kb,(hb,_) in enumerate(keys):
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
        if load_only:
            return cost
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
    keys = assignment_keys(plan)
    measured = palm_offsets(plan['profile'])
    offsets = np.array([measured[-1 if h=='left' else 1,f][:2] for h,f in keys])
    unary = np.zeros((len(contacts),len(keys)))
    neighbors = [[] for _ in contacts]
    comfort = plan.get('comfort',{})
    load_only = plan.get('settings',{}).get('fingering_objective') == 'load'
    priorities = comfort.get('finger_costs',{'index':0.,'middle':.1,'ring':1.,'little':12.,'thumb':35.})
    for i,c in enumerate(contacts):
        x = sum(p[1] for p in c['points'])/len(c['points'])
        for k,(hand,finger) in enumerate(keys):
            side = -1 if hand=='left' else 1
            unary[i,k] = 0. if load_only else 3000*max(0,-side*(x-.5)-.06)**2 + priorities[finger]
            if not load_only and c.get('manual_hand') and hand!=c['manual_hand']:
                unary[i,k] += 1e9
            if not load_only and c.get('fixed_finger') and (hand,finger)!=tuple(c['fixed_finger']):
                unary[i,k] += 1e12
    for i,a in enumerate(contacts):
        for j in range(i+1,len(contacts)):
            b = contacts[j]
            if b['start']>a['end']+.45:
                break
            if load_only and b['start']>=a['end']:
                break
            matrix = pair_cost(a,b,plan['physical_screen'],offsets,comfort,keys,load_only)
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


def repair_topology(labels,unary,neighbors,contacts=None):
    """Repair forbidden overlaps/order with a small exact neighborhood solve."""
    import z3
    edges=[(i,j,matrix) for i,items in enumerate(neighbors) for j,matrix in items
           if j>i and np.any(matrix>=1e8) and (contacts is None or
           max(contacts[i]['start'],contacts[j]['start'])<min(contacts[i]['end'],contacts[j]['end'])-1e-8)]
    bad=[(i,j) for i,j,m in edges if m[labels[i],labels[j]]>=1e8]
    if not bad:return labels
    affected={i for edge in bad for i in edge}
    for depth in range(4):
        opt=z3.Optimize();opt.set(timeout=30000)
        variables={i:z3.Int('finger_'+str(i)) for i in affected}
        for i,v in variables.items():
            allowed=[k for k in range(unary.shape[1]) if unary[i,k]<1e9]
            opt.add(z3.Or(*[v==k for k in allowed]))
            opt.add_soft(v==int(labels[i]),weight=1)
        for i,j,m in edges:
            if i not in affected and j not in affected:continue
            if i in affected and j in affected:
                forbidden=np.argwhere(m>=1e8)
                for a,b in forbidden:
                    opt.add(z3.Or(variables[i]!=int(a),variables[j]!=int(b)))
            elif i in affected:
                opt.add(z3.Or(*[variables[i]==k for k in range(unary.shape[1]) if m[k,labels[j]]<1e8]))
            else:
                opt.add(z3.Or(*[variables[j]==k for k in range(unary.shape[1]) if m[labels[i],k]<1e8]))
        if opt.check()==z3.sat:
            model=opt.model();result=labels.copy()
            for i,v in variables.items():result[i]=model.eval(v).as_long()
            if any(m[result[i],result[j]]>=1e8 for i,j,m in edges):
                raise AssertionError('Topology repair left a forbidden assignment')
            print('ASSIGN_TOPOLOGY_REPAIR',len(affected),'changed',int(np.sum(result!=labels)),flush=True)
            return result
        affected|={j for i,j,m in edges if i in affected}|{i for i,j,m in edges if j in affected}
    raise ValueError('No non-overlapping ordered finger assignment in conflict neighborhood')


def split_order_conflicts(contacts,screen,max_span,geometry=None):
    """Retain a crossing Hold path while permitting overlapping finger relays."""
    bad_ids={n for v in audit(contacts,screen)['same_hand_crossings'] for n in v['a']+v['b']}
    result=[]
    for old in contacts:
        if (old['kind']!='hold' or old.get('collective_hold') or old['end']-old['start']<=1.
                or not set(old['note_ids'])&bad_ids):
            result.append(copy.deepcopy(old));continue
        count=math.ceil((old['end']-old['start'])/max_span)
        for i in range(count):
            c=copy.deepcopy(old)
            start=round(old['start']+i*(old['end']-old['start'])/count,3)
            end=min(old['end'],round(old['start']+(i+1)*(old['end']-old['start'])/count+.020,3))
            c.update(start=start,end=end,rule='finger_order_relay')
            c['points']=[[start,*point_at(old['points'],start)],
                *[p for p in old['points'] if start<p[0]<end-.001],
                [round(end-.001,3),*point_at(old['points'],end-.001)]]
            if geometry is not None:
                from contact_refinement import validate_contact
                if validate_contact(geometry,c,c['points']):
                    from general_refinement import legal_path
                    c['points']=legal_path(geometry,c,screen)
            result.append(c)
    return sorted(result,key=lambda c:(c['start'],c['pointer']))


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
    parser.add_argument('--chart',type=Path,help='Chart used to validate newly created relay endpoints')
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
    keys = assignment_keys(plan)
    baseline = audit(contacts,plan['physical_screen'])
    unary,neighbors = graph(contacts,plan)
    initial = np.array([keys.index((c['hand'],c['finger'])) for c in contacts])
    load_only = plan['settings'].get('fingering_objective') == 'load'
    if load_only:
        from load_assignment import assign_load, score_labels
        before = score_labels(contacts,keys,plan['settings'],initial)[0]
        labels = assign_load(contacts,keys,plan['settings'],neighbors,args.beam)
        after,decisions = score_labels(contacts,keys,plan['settings'],labels)
        for c,decision in zip(contacts,decisions):
            c.update(effort=decision.pop('effort'),burst_cost=decision.pop('burst_cost'),decision=decision)
    else:
        before = objective(initial,unary,neighbors)
        beam = beam_assign(unary,neighbors,args.beam)
        a = descend(initial,unary,neighbors)
        b = descend(beam,unary,neighbors)
        labels = min((a,b),key=lambda x:objective(x,unary,neighbors))
        try:
            labels = repair_topology(labels,unary,neighbors,contacts)
        except ValueError:
            geometry=None
            if args.chart:
                import zipfile
                from chart import load_chart
                from contact_refinement import ContactGeometry
                with zipfile.ZipFile(args.chart) as z:
                    chart=load_chart(z.read('chart.json').decode('utf-8-sig'))
                geometry=ContactGeometry(chart,physical_screen=plan['physical_screen'],
                    extra_clearance_m=plan.get('comfort',{}).get('extra_block_clearance_m',0.))
            source=copy.deepcopy(contacts)
            for c,label in zip(source,labels):
                c['hand'],c['finger']=keys[label]
            for span in (.8,.4,.2):
                trial=split_order_conflicts(source,plan['physical_screen'],span,geometry)
                if len(trial)==len(source):
                    raise ValueError('No relay-capable Hold at finger-order conflict')
                u,edges=graph(trial,plan)
                chosen=np.array([keys.index((c['hand'],c['finger'])) for c in trial])
                try:chosen=repair_topology(chosen,u,edges,trial)
                except ValueError:continue
                contacts,unary,neighbors,labels=trial,u,edges,chosen
                plan['contacts']=contacts
                print('ASSIGN_ORDER_RELAYS',len(source),len(contacts),'span',span,flush=True)
                break
            else:
                raise ValueError('Finger-order relay search exhausted')
        after = objective(labels,unary,neighbors)
    for c,label in zip(contacts,labels):
        c['hand'],c['finger'] = keys[label]
        c['pointer'] = (0 if c['hand']=='left' else 5)+FINGER_ORDER.index(c['finger'])
    plan['hand_rest'] = finish_motion(contacts,plan['profile'],plan['physical_screen'],.28/.82)
    result = audit(contacts,plan['physical_screen'])
    report = dict(before=baseline,after=result,cost_before=before,cost_after=after,beam_width=args.beam,
                  fingering_objective=plan['settings'].get('fingering_objective','balanced'),
                  speed_limits=plan['settings'].get('speed_limits',True))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'assigned-motion-plan.json').write_text(json.dumps(plan,indent=2))
    (args.output/'assignment-refinement.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:len(v) for k,v in result.items()}),flush=True)


if __name__=='__main__':
    main()
