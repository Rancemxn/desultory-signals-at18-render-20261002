"""Blender integration: six held fingers and crossed hands retain contact without intersections."""
import json
from pathlib import Path
import sys

import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handcam_blender import hand_collisions, main, skin_pads, world_point
from handcam_motion import finish_motion, palm_offsets, point_at


def verify_saved(out, job):
    bpy.ops.wm.open_mainfile(filepath=str(out / 'handcam.blend'))
    rig = json.loads((out / 'rig.json').read_text())
    indices = {(int(k.split(':')[0]), k.split(':')[1]): v for k, v in rig['pad_vertices'].items()}
    arms = {side: bpy.data.objects[name] for side, name in ((-1, 'Hand_Left'), (1, 'Hand_Right'))}
    meshes = {side: next(o for o in bpy.context.scene.objects if o.type == 'MESH' and o.parent == arm)
              for side, arm in arms.items()}
    templates = [(tuple(c['key']), c['bone'], Vector(c['a']), Vector(c['b']), c['radius']) for c in rig['capsules']]
    maximum, intersections, penetration, clearance = 0., 0, 0., float('inf')
    # Quarter frames include interpolated poses between the bake's half-frame samples.
    for quarter_frame in range(4, job['frames'] * 4 + 1):
        frame = quarter_frame / 4
        bpy.context.scene.frame_set(int(frame), subframe=frame % 1)
        t = job['start'] + (frame - 1) / job['fps']
        pads = skin_pads(meshes, indices)
        for depth, *_ in hand_collisions(arms, templates):
            penetration = max(penetration, -depth)
        for contact in job['contacts']:
            if not contact['start'] <= t < contact['end']:
                continue
            side = -1 if contact['hand'] == 'left' else 1
            xy = point_at(contact['points'], t)
            expected = Vector((*world_point(job, xy), job['contact_height']))
            maximum = max(maximum, (pads[side, contact['finger']] - expected).length)
        trees = []
        graph = bpy.context.evaluated_depsgraph_get()
        for obj in meshes.values():
            evaluated = obj.evaluated_get(graph)
            mesh = evaluated.to_mesh()
            try:
                vertices = [evaluated.matrix_world @ v.co for v in mesh.vertices]
                clearance = min(clearance, min((v.z for v in vertices if abs(v.x) <= job['screen'][0] / 2
                                                and abs(v.y) <= job['screen'][1] / 2), default=float('inf')))
                trees.append(BVHTree.FromPolygons(vertices,
                                                 [tuple(p.vertices) for p in mesh.polygons]))
            finally:
                evaluated.to_mesh_clear()
        intersections += len(trees[0].overlap(trees[1]))
    result = dict(saved_max_contact_error_mm=maximum * 1000, intersecting_polygon_pairs=intersections,
                  max_proxy_penetration_mm=penetration * 1000, min_screen_clearance_mm=clearance * 1000)
    (out / 'saved-check.json').write_text(json.dumps(result, indent=2))
    assert maximum < .001, result
    assert intersections == 0, result
    assert penetration == 0. and clearance >= -.0002, result
    return result


def check(out, cases=('multi', 'cross', 'moving_cross')):
    out.mkdir(parents=True, exist_ok=True)
    model = str(Path(bpy.data.filepath).resolve())
    (out / 'profile').mkdir(exist_ok=True)
    main(dict(output=str(out / 'profile'), hand_scale=.27, enable_thumb=True, profile_only=True))
    profile = json.loads((out / 'profile/hand-profile.json').read_text())
    guides = palm_offsets(profile)
    for name in cases:
        bpy.ops.wm.open_mainfile(filepath=model)
        folder = out / name
        folder.mkdir(exist_ok=True)
        if name == 'multi':
            positions = [(side, finger, side * .08 + guides[side, finger][0], -.13 + guides[side, finger][1])
                         for side in (-1, 1) for finger in ('index', 'middle', 'thumb')]
        else:
            positions = [(-1, 'index', .04, .01), (1, 'index', -.04, .01)]
        contacts = []
        for side, finger, x, y in positions:
            drift = -.004 * side if name == 'moving_cross' else .002 if name == 'multi' else 0.
            points = [[t, (x + drift * (t - .5)) / .28 + .5, .5 - y / .1575] for t in (.5, .8, 1., 1.8)]
            contacts.append(dict(hand='left' if side == -1 else 'right', finger=finger, pointer=len(contacts),
                start=.5, end=1.8, points=points, planned=True, kind='hold', note_ids=[len(contacts)]))
        rests = finish_motion(contacts, profile, [.28, .1575], .28 / .82)
        job = dict(start=.8, duration=.2, frames=6, fps=30, width=1280, height=720, output=str(folder),
            screen=[.28, .1575], view_width=.28/.82, camera_y=-.015, engine='workbench', hand_scale=.27,
            contact_height=.0005, lift_height=.045, strict_psap=True, bake_only=True, keyframes=False,
            contacts=contacts, motion_plan_version=1, hand_rest=rests, enable_thumb=True, palm_motion='v4',
            finger_motion='whole_finger', palm_lift_mode='gentle', palm_lift_ratio=.70, pose_avoidance=True)
        (folder / 'job.json').write_text(json.dumps(job, indent=2))
        main(job)
        diag = json.loads((folder / 'diagnostics.json').read_text())
        assert diag['max_error_mm'] < 1., diag['max_error_mm']
        assert not diag['collision_errors'], diag['collision_errors'][:2]
        assert diag['min_screen_clearance_mm'] >= -.2, diag['min_screen_clearance_mm']
        print('AVOIDANCE CASE', name, verify_saved(folder, job), flush=True)
    print('Contact-preserving six-finger and crossed-hand avoidance checks passed', flush=True)


if __name__ == '__main__':
    args = sys.argv[sys.argv.index('--') + 1:]
    check(Path(args[0]).resolve(), tuple(args[1:]) or ('multi', 'cross', 'moving_cross'))
