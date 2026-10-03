"""Eight independent bakes with numerical same-time join verification."""
import bisect
import json
import math
from pathlib import Path
import sys

import refined_task


def bake(part):
    from handcam_blender import sample_times
    start = part*20.
    duration = min(20.,150.75-start)
    out = Path(f'output/part{part}')
    # Retain the exact next segment's first frame for a same-time comparison.
    # Three seconds of history also lets released poses settle before the cut.
    refined_task.bake(out,start,duration+(1/60 if part<7 else 0),warmup=3.)
    job = json.loads((out/'job.json').read_text())
    refined_task.run([refined_task.BLENDER,'--background','--disable-autoexec',out/'handcam.blend',
        '--python-exit-code',1,'--python','capture_bake_boundaries.py','--',out/'job.json',duration])
    end = start+duration
    times = [t for t in sample_times(job) if start<=t<end]
    diagnostics = json.loads((out/'diagnostics.json').read_text())
    diagnostics['contact_samples'] = sum(max(0,bisect.bisect_left(times,c['end'])-
        bisect.bisect_left(times,c['start'])) for c in job['contacts'])
    for key in ('contact_errors_over_1mm','collision_errors'):
        diagnostics[key] = [v for v in diagnostics[key] if start<=v['time']<end]
    diagnostics['max_error_mm'] = max((v['error_mm'] for v in diagnostics['contact_errors_over_1mm']),
                                     default=min(1.,diagnostics['max_error_mm']))
    diagnostics['max_collision_depth_mm'] = max((v['depth_mm'] for v in diagnostics['collision_errors']),default=0.)
    diagnostics['clearance_scope'] = 'segment plus one retained join frame'
    (out/'diagnostics.json').write_text(json.dumps(diagnostics,indent=2))
    numeric = json.loads((out/'pose-numeric.json').read_text())
    for key in ('worst_wrist_steps','worst_joint_steps','finger_order_violations','projected_contacts','mesh_intersections'):
        numeric[key] = [v for v in numeric[key] if start<=v['time']<end]
    numeric['frames'] = round(duration*60)
    (out/'pose-numeric.json').write_text(json.dumps(numeric,indent=2))
    job.update(duration=duration,frames=round(duration*60))
    (out/'job.json').write_text(json.dumps(job,indent=2))
    provenance = json.loads((out/'provenance.json').read_text())
    provenance.update(part=part,duration=duration,frames=job['frames'],warmup_seconds=3.,
                      parallel_bake=True,retained_join_frame=part<7)
    (out/'provenance.json').write_text(json.dumps(provenance,indent=2))


def seam_distance(before,after):
    if abs(before['time']-after['time'])>1e-7:
        raise ValueError('Join states must describe exactly the same time')
    if before['rigs'].keys()!=after['rigs'].keys():
        raise ValueError('Join rig identities differ')
    worst = None
    for name,arm in before['rigs'].items():
        other = after['rigs'][name]
        if arm['bones'].keys()!=other['bones'].keys():
            raise ValueError('Join bone identities differ')
        for bone,points in arm['bones'].items():
            for endpoint in ('head','tail'):
                distance = math.dist(points[endpoint],other['bones'][bone][endpoint])*1000
                if worst is None or distance>worst['distance_mm']:
                    worst = dict(rig=name,bone=bone,endpoint=endpoint,distance_mm=distance)
    return worst


def audit(source):
    paths = sorted(Path(source).rglob('seam-state.json'),key=lambda p:
                   json.loads(p.with_name('provenance.json').read_text())['part'])
    assert len(paths)==8, f'Expected eight bakes, received {len(paths)}'
    provenance = [json.loads(p.with_name('provenance.json').read_text()) for p in paths]
    assert [p['part'] for p in provenance]==list(range(8))
    assert len({p['plan_sha256'] for p in provenance})==1
    assert len({p['commit'] for p in provenance})==1
    states = [json.loads(p.read_text()) for p in paths]
    seams = [dict(time=a['join']['time'],**seam_distance(a['join'],b['first'])) for a,b in zip(states,states[1:])]
    report = dict(passed=all(s['distance_mm']<=1. for s in seams),tolerance_mm=1.,
                  comparison='same-time world-space heads and tails of all rig bones',seams=seams,
                  bakes=provenance,image_inspection=False)
    out = Path('output/full');out.mkdir(parents=True,exist_ok=True)
    (out/'seam-validation.json').write_text(json.dumps(report,indent=2))
    diagnostics = [json.loads(p.with_name('diagnostics.json').read_text()) for p in paths]
    merged = {key:[v for d in diagnostics for v in d[key]] for key in ('contact_errors_over_1mm','collision_errors')}
    merged.update(contact_samples=sum(d['contact_samples'] for d in diagnostics),
        max_error_mm=max(d['max_error_mm'] for d in diagnostics),
        max_collision_depth_mm=max(d['max_collision_depth_mm'] for d in diagnostics),
        min_screen_clearance_mm=min(d['min_screen_clearance_mm'] for d in diagnostics),
        source='eight disjoint rendered intervals; clearance also includes retained join frames')
    (out/'diagnostics.json').write_text(json.dumps(merged,indent=2))
    reports = [json.loads(p.with_name('pose-numeric.json').read_text()) for p in paths]
    numeric = {key:[v for d in reports for v in d[key]] for key in (
        'worst_wrist_steps','worst_joint_steps','finger_order_violations','projected_contacts','mesh_intersections')}
    numeric.update(frames=sum(d['frames'] for d in reports),image_inspection=False)
    (out/'pose-numeric.json').write_text(json.dumps(numeric,indent=2))
    print(json.dumps(report),flush=True)
    if not report['passed']:
        raise RuntimeError('Parallel bake joins differ by more than 1 mm; rendering is blocked')


if __name__=='__main__':
    if sys.argv[1]=='bake':bake(int(sys.argv[2]))
    elif sys.argv[1]=='audit':audit(sys.argv[2])
    elif sys.argv[1]=='render':
        import task
        task.render(int(sys.argv[2]))
    elif sys.argv[1]=='assemble':refined_task.assemble(parallel=True)
    else:raise ValueError('Unknown parallel workflow stage')
