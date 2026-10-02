"""Blender integration check: load the exported handcam.blend, then --python this file -- OUTPUT [--render]."""
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handcam_blender import FINGERS, contact_position, hand_collisions, render_shadow, sample_times, skin_pads, world_point
from handcam_motion import blend, smooth, wrist_pose


def check(out, render=False):
    job = json.loads((out / 'job.json').read_text(encoding='utf-8'))
    rig = json.loads((out / 'rig.json').read_text(encoding='utf-8'))
    fingering = json.loads((out / 'fingering.json').read_text(encoding='utf-8'))
    motion = json.loads((out / 'motion.json').read_text(encoding='utf-8'))
    original = job['contacts']
    job['contacts'] = motion['contacts']
    for contact in original:
        for point in contact['points']:
            assert (any(c['pointer'] == contact['pointer'] and c['start'] <= point[0] <= c['end'] and point in c['points']
                        for c in job['contacts'])
                    or any(s['pointer'] == contact['pointer'] and s['start'] == contact['start'] and s['shared_time'] == point[0]
                           for s in motion['shared_events'])), 'Position event missing from displayed motion'
    for start, end in job.get('hold_intervals', ()):
        sources = [c for c in original if c['start'] <= start + .002 and c['end'] >= end - .002
                   and any(abs(p[0] - start) < .002 for p in c['points'])]
        if sources:
            assert any(c['pointer'] == source['pointer'] and c['start'] <= start + .002 and c['end'] >= end - .002
                       for source in sources for c in job['contacts']), 'Hold released early'
    for shared in motion['shared_events']:
        source = next(c for c in original if c['pointer'] == shared['pointer'] and c['start'] == shared['start'])
        target = next(c for c in original if c['pointer'] == shared['shared_pointer'] and c['start'] == shared['shared_start'])
        assert target['start'] <= shared['shared_time'] < target['end']
        assert math.dist(contact_position(source, shared['shared_time']), contact_position(target, shared['shared_time'])) < 1e-6
    rigs = {side: bpy.data.objects[name] for side, name in ((-1, 'Hand_Left'), (1, 'Hand_Right'))}
    meshes = {side: next(o for o in bpy.context.scene.objects if o.type == 'MESH' and o.parent == arm)
              for side, arm in rigs.items()}
    indices = {(int(k.split(':')[0]), k.split(':')[1]): v for k, v in rig['pad_vertices'].items()}
    templates = [(tuple(c['key']), c['bone'], Vector(c['a']), Vector(c['b']), c['radius']) for c in rig['capsules']]
    assigned = {(c['pointer'], c['start'], c['end']): (-1 if c['hand'] == 'left' else 1, c['finger']) for c in fingering}
    scene = bpy.context.scene
    max_error, min_clearance, contacts, frames = 0., math.inf, 0, set()
    max_rest_guide_error = 0.
    wrist_frames = {side: [] for side in rigs}
    previous_joints = {}
    max_airborne_joint_speed = 0.
    guides = {(int(k.split(':')[0]), k.split(':')[1]): v for k, v in rig.get('palm_offsets', {}).items()}
    for t in sample_times(job):
        frame = (t - job['start']) * job['fps'] + 1
        if frame < 1:
            continue
        scene.frame_set(math.floor(frame), subframe=frame % 1)
        bpy.context.view_layer.update()
        for gap in job.get('hand_rest', []):
            if gap['mode'] not in ('lower', 'withdraw') or not gap['start'] <= t < gap['end']:
                continue
            if any(c['hand'] == gap['hand'] and c['start'] <= t < c['end'] for c in job['contacts']):
                continue
            side = -1 if gap['hand'] == 'left' else 1
            for a, b in zip(gap['knots'], gap['knots'][1:]):
                if a[0] <= t <= b[0]:
                    expected = Vector(blend(a[1:], b[1:], smooth((t - a[0]) / (b[0] - a[0]))))
                    if job.get('palm_motion') == 'v4':
                        yaw = rigs[side].rotation_euler.z - math.pi
                        rotated = {k: (v[0] * math.cos(yaw) - v[1] * math.sin(yaw),
                                       v[0] * math.sin(yaw) + v[1] * math.cos(yaw), v[2]) for k, v in guides.items()}
                        expected = Vector(wrist_pose(job['contacts'], job['hand_rest'], side, t, job, rotated, guides))
                    wrist = rigs[side].matrix_world @ rigs[side].pose.bones['Bone.016'].head
                    difference = (wrist - expected).length
                    max_rest_guide_error = max(max_rest_guide_error, difference)
                    assert difference < .001, f'Rest/return guide bypassed at {t:.6f}s: {difference * 1000:.3f} mm'
                    break
        for arm in rigs.values():
            assert (arm.matrix_world.to_3x3() @ Vector((0, 0, 1))).z > job['hand_scale'] * .5, 'Palm turned upward'
            for finger, chain in FINGERS.items():
                for joint, name in enumerate(chain):
                    bone = arm.pose.bones[name]
                    assert abs((bone.tail - bone.head).length - bone.bone.length) < 1e-5, 'Stretched finger'
                    assert not any(c.type == 'IK' for c in bone.constraints), 'Scene joints were not baked'
                    if finger != 'thumb':
                        angle = bone.rotation_quaternion.to_euler('XYZ').x
                        assert -1.6 <= angle <= (.45 if joint == 0 else .05), f'Joint reversed: {name}/{angle}'
        pads = skin_pads(meshes, indices)
        for contact in job['contacts']:
            if not contact['start'] <= t < contact['end']:
                continue
            key = assigned[contact['pointer'], contact['start'], contact['end']]
            desired = Vector((*world_point(job, contact_position(contact, t)), job['contact_height']))
            error = (pads[key] - desired).length
            assert error <= .001, f'Baked skin missed contact at {t:.6f}s/{key}: {error * 1000:.3f} mm'
            max_error, contacts = max(max_error, error), contacts + 1
        collisions = hand_collisions(rigs, templates)
        assert not collisions, f'Collision at {t:.6f}s: {collisions[0]}'
        number = round(frame)
        if abs(frame - number) > .00001 or number > job['frames'] or number in frames:
            continue
        for side, arm in rigs.items():
            wrist = arm.matrix_world @ arm.pose.bones['Bone.016'].head
            wrist_frames[side].append((t, *wrist, arm.rotation_euler.z - math.pi))
            for finger, chain in FINGERS.items():
                down = any(c['hand'] == ('left' if side == -1 else 'right') and c['finger'] == finger
                           and c['start'] <= t < c['end'] for c in job['contacts'] if 'hand' in c)
                for name in chain:
                    key = side, name
                    rotation = arm.pose.bones[name].rotation_quaternion.normalized()
                    if key in previous_joints:
                        previous_t, previous_rotation, previous_down = previous_joints[key]
                        if not down and not previous_down:
                            angle = 2 * math.acos(min(1., abs(rotation.dot(previous_rotation))))
                            speed = math.degrees(angle) / (t - previous_t)
                            max_airborne_joint_speed = max(max_airborne_joint_speed, speed)
                            if job.get('palm_lift_mode') == 'gentle':
                                assert speed < 600., f'Airborne joint snaps at {t:.6f}s/{key}: {speed:.1f} deg/s'
                    previous_joints[key] = t, rotation, down
        for obj in meshes.values():
            evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
            mesh = evaluated.to_mesh()
            for vertex in mesh.vertices:
                point = evaluated.matrix_world @ vertex.co
                if abs(point.x) <= job['screen'][0] / 2 and abs(point.y) <= job['screen'][1] / 2:
                    min_clearance = min(min_clearance, point.z)
            evaluated.to_mesh_clear()
        assert min_clearance >= -.0005, f'Mesh crosses screen at {t:.6f}s'
        if render:
            scene.render.filepath = str(out / 'hands' / f'{number:05d}.png')
            bpy.ops.render.render(write_still=True)
            render_shadow(scene, meshes, out / 'shadows' / f'{number:05d}.png')
        frames.add(number)
        if len(frames) % 30 == 0:
            print(f'BAKED CHECK {len(frames)}/{job["frames"]}: skin error {max_error * 1000:.3f} mm', flush=True)
    assert contacts and frames == set(range(1, job['frames'] + 1))
    result = dict(frames=len(frames), contact_samples=contacts, max_skin_error_mm=max_error * 1000,
                  min_screen_clearance_mm=min_clearance * 1000, collision_errors=0, baked=True,
                  shared_events=len(motion['shared_events']), max_rest_guide_error_mm=max_rest_guide_error * 1000,
                  max_airborne_joint_speed_degrees_per_second=max_airborne_joint_speed)
    result['palm_motion'] = {str(side): dict(
        height_range_mm=[min(p[3] for p in rows) * 1000, max(p[3] for p in rows) * 1000],
        yaw_range_degrees=[math.degrees(min(p[4] for p in rows)), math.degrees(max(p[4] for p in rows))],
        max_frame_step_mm=max((math.dist(a[1:4], b[1:4]) * 1000 for a, b in zip(rows, rows[1:])), default=0.))
        for side, rows in wrist_frames.items()}
    (out / 'baked-validation.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(result, flush=True)


if __name__ == '__main__':
    arguments = sys.argv[sys.argv.index('--') + 1:]
    check(Path(arguments[0]).resolve(), '--render' in arguments[1:])
