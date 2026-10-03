"""Reconcile a small bake seam and verify contact and mesh constraints.

Run inside Blender; does not render or display images. Only finger rotation
curves and a submillimetre wrist offset are blended. Contact timing is fixed; active pads must
remain within 1 mm of their targets or improve their previous error.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import sys

import bpy
from mathutils import Quaternion

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handcam_blender import screen_mesh_clearance, skin_pads
from inspect_pose_numeric import inspect
from parallel_task import seam_distance
from handcam_motion import point_at, world_xy


def state(time):
    return {'time': time, 'rigs': {arm.name: {
        'matrix': [list(row) for row in arm.matrix_world],
        'location': list(arm.location),
        'euler': list(arm.rotation_euler),
        'bones': {bone.name: {
            'head': list(arm.matrix_world @ bone.head),
            'tail': list(arm.matrix_world @ bone.tail),
            'rotation': list(bone.rotation_quaternion),
        } for bone in arm.pose.bones},
    } for arm in bpy.context.scene.objects if arm.type == 'ARMATURE'}}


def main(before_dir, after_dir, duration=.2):
    before_dir, after_dir = Path(before_dir), Path(after_dir)
    before_job = json.loads((before_dir/'job.json').read_text())
    job = json.loads((after_dir/'job.json').read_text())
    for directory in (before_dir, after_dir):
        provenance = json.loads((directory/'provenance.json').read_text())
        with (directory/'handcam.blend').open('rb') as stream:
            assert hashlib.file_digest(stream, 'sha256').hexdigest() == provenance['blend_sha256']
    start = job['start']
    assert abs(before_job['start']+before_job['duration']-start) < 1e-7
    bpy.ops.wm.open_mainfile(filepath=str((before_dir/'handcam.blend').resolve()))
    bpy.context.scene.frame_set(before_job['frames']+1)
    previous = state(start)
    bpy.ops.wm.open_mainfile(filepath=str((after_dir/'handcam.blend').resolve()))
    scene = bpy.context.scene
    scene.frame_set(1)
    initial = state(start)
    error_before = seam_distance(previous, initial)
    print('SEAM_BEFORE', error_before, flush=True)
    assert 1 < error_before['distance_mm'] <= 5, error_before
    corrections, root_corrections = {}, {}
    for arm_name, arm_state in initial['rigs'].items():
        prior = previous['rigs'][arm_name]
        distance = max(math.dist(prior['bones'][bone][point], arm_state['bones'][bone][point])
                       for bone in arm_state['bones'] for point in ('head','tail'))
        if distance <= .001:
            continue
        translation = [(a-b)*.65 for a,b in zip(prior['location'],arm_state['location'])]
        rotation = [((a-b+math.pi)%(2*math.pi)-math.pi)*.65
                    for a,b in zip(prior['euler'],arm_state['euler'])]
        assert math.dist(translation,(0,0,0)) <= .001
        assert max(abs(v) for v in rotation) < math.radians(1)
        root_corrections[arm_name] = {'location':translation,'rotation_euler':rotation}
        for bone in arm_state['bones']:
            old = Quaternion(prior['bones'][bone]['rotation'])
            current = Quaternion(arm_state['bones'][bone]['rotation'])
            delta = old @ current.inverted()
            if delta.w < 0:
                delta.negate()
            assert math.degrees(delta.angle) <= 15
            corrections[arm_name, bone] = Quaternion().slerp(delta, .65)
    assert corrections
    rig = json.loads((after_dir/'rig.json').read_text())
    arms = {side:bpy.data.objects[name] for side,name in ((-1,'Hand_Left'),(1,'Hand_Right'))}
    meshes = {side:next(o for o in scene.objects if o.type=='MESH' and o.parent==arm)
              for side,arm in arms.items()}
    indices = {(int(k.split(':')[0]), k.split(':')[1]):v for k,v in rig['pad_vertices'].items()}
    # Include every baked key and every video frame through the fade's endpoint.
    frames = {float(i) for i in range(1, round(duration*job['fps'])+3)}
    for arm in arms.values():
        for curve in arm.animation_data.action.fcurves:
            frames.update(float(p.co.x) for p in curve.keyframe_points
                          if 1 <= p.co.x <= duration*job['fps']+1)
    times = [start+(frame-1)/job['fps'] for frame in sorted(frames)]

    def measure():
        pads, targets, clearance = {}, {}, math.inf
        for frame, time in zip(sorted(frames), times):
            scene.frame_set(math.floor(frame), subframe=frame%1)
            evaluated_pads = skin_pads(meshes, indices)
            for c in job['contacts']:
                if c['start'] <= time < c['end']:
                    key = (-1 if c['hand']=='left' else 1, c['finger'])
                    pads[time, *key] = tuple(evaluated_pads[key])
                    targets[time, *key] = (*world_xy(point_at(c['points'],time),job['screen']),job['contact_height'])
            for obj in meshes.values():
                evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
                mesh = evaluated.to_mesh()
                try:
                    clearance = min(clearance, screen_mesh_clearance(mesh, evaluated.matrix_world, job['screen']))
                finally:
                    evaluated.to_mesh_clear()
        return pads, targets, clearance

    pads_before, targets, clearance_before = measure()
    mesh_times = [start+i/job['fps'] for i in range(round(duration*job['fps'])+2)]
    review_before = inspect(job, rig, start=start, end=start+duration+2/job['fps'], mesh_times=mesh_times)
    for arm_name, offsets in root_corrections.items():
        arm = bpy.data.objects[arm_name]
        for curve in arm.animation_data.action.fcurves:
            if curve.data_path not in offsets:
                continue
            offset = offsets[curve.data_path][curve.array_index]
            for key in curve.keyframe_points:
                if not 1 <= key.co.x <= duration*job['fps']+1:
                    continue
                u = min(1., max(0., (key.co.x-1)/(duration*job['fps'])))
                fade = u*u*u*(u*(u*6-15)+10)
                key.co.y += offset*(1-fade)
            curve.update()
    for (arm_name, bone_name), delta in corrections.items():
        arm = bpy.data.objects[arm_name]
        bone = arm.pose.bones[bone_name]
        path = bone.path_from_id('rotation_quaternion')
        curves = {c.array_index:c for c in arm.animation_data.action.fcurves if c.data_path==path}
        if not curves:
            assert delta.angle < .00001, (arm_name, bone_name, 'unanimated rotation differs')
            continue
        assert set(curves)==set(range(4))
        key_times = sorted({float(p.co.x) for c in curves.values() for p in c.keyframe_points
                            if 1 <= p.co.x <= duration*job['fps']+1})
        values = {frame:Quaternion([curves[i].evaluate(frame) for i in range(4)]) for frame in key_times}
        for frame, rotation in values.items():
            u = min(1., max(0., (frame-1)/(duration*job['fps'])))
            fade = u*u*u*(u*(u*6-15)+10)
            corrected = delta.slerp(Quaternion(), fade) @ rotation
            for i, curve in curves.items():
                curve.keyframe_points.insert(frame, corrected[i], options={'FAST', 'REPLACE'})
        for curve in curves.values():
            curve.update()
    pads_after, _, clearance_after = measure()
    pad_change = max((math.dist(value, pads_after[key]) for key,value in pads_before.items()), default=0.)
    assert pad_change <= .001, ('active skin moved more than 1 mm', pad_change)
    errors_before = {key:math.dist(value,targets[key]) for key,value in pads_before.items()}
    errors_after = {key:math.dist(value,targets[key]) for key,value in pads_after.items()}
    assert all(value <= max(.001,errors_before[key])+.00001 for key,value in errors_after.items()), 'Contact fit degraded'
    assert clearance_after >= min(0., clearance_before)-.00001, ('new penetration', clearance_before, clearance_after)
    review_after = inspect(job, rig, start=start, end=start+duration+2/job['fps'], mesh_times=mesh_times)
    assert len(review_after['finger_order_violations']) <= len(review_before['finger_order_violations'])
    def intersections(report):
        return {(round(v['time']*1e6), tuple(v['parts'])):v['intersecting_polygon_pairs']
                for v in report['mesh_intersections']}
    old_intersections = intersections(review_before)
    assert all(count <= old_intersections.get(key, 0)
               for key,count in intersections(review_after).items()), 'New mesh intersection'
    scene.frame_set(1)
    error_after = seam_distance(previous, state(start))
    assert error_after['distance_mm'] <= 1, error_after
    report = dict(start=start, duration=duration, corrected_bones=[list(k) for k in corrections],
        repair_commit=os.environ.get('GITHUB_SHA'),
        repair_tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        wrist_corrections=root_corrections,
        before=error_before, after=error_after, active_pad_change_mm=pad_change*1000,
        active_error_before_mm=max(errors_before.values(),default=0)*1000,
        active_error_after_mm=max(errors_after.values(),default=0)*1000,
        clearance_before_mm=clearance_before*1000, clearance_after_mm=clearance_after*1000,
        samples=len(frames), no_new_sampled_mesh_intersections=True, image_inspection=False)
    old_numeric = json.loads((after_dir/'pose-numeric.json').read_text())
    numeric = inspect(job, rig, mesh_times=sorted(set(old_numeric.get('mesh_check_times', ())) | set(mesh_times)))
    (after_dir/'pose-numeric.json').write_text(json.dumps(numeric, indent=2))
    scene.frame_set(1)
    bpy.ops.wm.save_as_mainfile(filepath=str((after_dir/'handcam.blend').resolve()))
    provenance = json.loads((after_dir/'provenance.json').read_text())
    provenance['original_blend_sha256'] = provenance['blend_sha256']
    with (after_dir/'handcam.blend').open('rb') as stream:
        provenance['blend_sha256'] = hashlib.file_digest(stream, 'sha256').hexdigest()
    provenance['seam_repair'] = report
    (after_dir/'provenance.json').write_text(json.dumps(provenance, indent=2))
    (after_dir/'seam-repair.json').write_text(json.dumps(report, indent=2))
    print('POSE_SEAM_REPAIR', json.dumps(report), flush=True)


if __name__ == '__main__':
    main(*sys.argv[sys.argv.index('--')+1:])
