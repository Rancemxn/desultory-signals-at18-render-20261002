"""Blender half of the experiment; timing/compositing helpers also use ordinary Python."""
import bisect
from array import array
import json
import math
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handcam_motion import MotionIndex, finger_pose, palm_offsets, point_at, stroke_shape, wrist_pose

FINGERS = {
    'index': ('Bone', 'Bone.001', 'Bone.002'),
    'middle': ('Bone.003', 'Bone.004', 'Bone.005'),
    'ring': ('Bone.006', 'Bone.007', 'Bone.008'),
    'little': ('Bone.009', 'Bone.010', 'Bone.011'),
    'thumb': ('Bone.017', 'Bone.018', 'Bone.019'),
}


def solve_steps(limit=6):
    """Deterministic refinement budget, independent of machine load and frame cost."""
    yield from range(limit)


def bake_rotations(rigs, baked):
    """Write solved joint curves in bulk instead of repeatedly inserting/sorting individual keys."""
    for index, ((side, name), keys) in enumerate(baked.items(), 1):
        bone = rigs[side].pose.bones[name]
        bone.rotation_mode = 'QUATERNION'
        # Blender stores frame coordinates as float32; merge event/video samples that become the same frame.
        values = dict(zip(array('f', (frame for frame, _ in keys)), (q for _, q in keys)))
        action = rigs[side].animation_data.action
        path = bone.path_from_id('rotation_quaternion')
        for component in range(4):
            curve = action.fcurves.find(path, index=component)
            if curve is not None:
                action.fcurves.remove(curve)
            curve = action.fcurves.new(path, index=component, action_group=name)
            curve.keyframe_points.add(len(values))
            curve.keyframe_points.foreach_set('co', [v for frame, q in values.items() for v in (frame, q[component])])
            for point in curve.keyframe_points:
                point.interpolation = 'LINEAR'
            curve.update()
        print(f'HANDCAM_STATUS Baking joints {index}/{len(baked)}', flush=True)


def smooth(u):
    u = max(0., min(1., u))
    return u ** 3 * (10 + u * (-15 + 6 * u))


def contact_position(contact, t):
    if contact.get('planned'):
        return point_at(contact['points'], t)
    points = contact['points']
    index = max(0, bisect.bisect_right(points, t, key=lambda p: p[0]) - 1)
    a = points[index]
    if index + 1 == len(points) or t <= a[0]:
        return tuple(a[1:])
    b = points[index + 1]
    start = max(a[0], b[0] - .12)
    u = max(0., min(1., (t - start) / (b[0] - start)))
    return a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u


def coalesce_contacts(contacts, hold_starts=()):
    """Release a parked pointer whose final reused event is covered by another finger."""
    def parked(contact):
        points = contact['points']
        return (len(points) > 1 and points[-1][0] - points[-2][0] > .25
                and contact['end'] - points[-1][0] < .02 and math.dist(points[-1][1:], points[-2][1:]) > .002
                and not any(abs(p[0] - t) < .002 for p in points for t in hold_starts))

    visual, shared = [], []
    for contact in contacts:
        points = contact['points']
        replacement = contact
        if parked(contact):
            last = points[-1]
            for other in contacts:
                if other is contact or parked(other) or not other['start'] <= last[0] - .18 < last[0] < other['end']:
                    continue
                if math.dist(last[1:], contact_position(other, last[0])) > 1e-6:
                    continue
                if math.dist(last[1:], contact_position(other, last[0] - .18)) > .002:
                    continue
                end = max(points[-2][0] + .05, other['start'] - .18)
                if end > last[0] - .18:
                    continue
                replacement = dict(contact, points=points[:-1], end=end)
                shared.append(dict(pointer=contact['pointer'], start=contact['start'], source_end=contact['end'],
                                   visual_end=end, shared_time=last[0], shared_pointer=other['pointer'],
                                   shared_start=other['start']))
                break
        visual.append(replacement)
    return visual, shared


def world_point(job, pos):
    return ((pos[0] - .5) * job['screen'][0], (.5 - pos[1]) * job['screen'][1])


def release_idle_contacts(contacts, hold_intervals=()):
    """Split parked pointer reuse into airborne transfers, retaining every position event."""
    visual, releases = [], []
    for contact in contacts:
        holds = [(a, b) for a, b in hold_intervals if any(abs(p[0] - a) < .002 for p in contact['points'])]
        points = []
        start = contact['start']
        for point in contact['points']:
            if points:
                end = max(points[-1][0] + .04, max((b + .001 for a, b in holds if a <= points[-1][0] <= b), default=0.))
                if point[0] - end > .12:
                    visual.append(dict(contact, start=start, end=end, points=points))
                    releases.append(dict(pointer=contact['pointer'], start=start, release=end, resume=point[0]))
                    start, points = point[0], []
            points.append(point)
        end = max(points[-1][0] + .04, max((b + .001 for a, b in holds if a <= points[-1][0] <= b), default=0.))
        if contact['end'] - end > .12:
            releases.append(dict(pointer=contact['pointer'], start=start, release=end, resume=None))
        else:
            end = contact['end']
        visual.append(dict(contact, start=start, end=end, points=points))
    return visual, releases


def sample_times(job):
    start, end = job['start'], job['start'] + job['duration']
    times = {start + i / job['fps'] for i in range(-job['fps'], job['frames'])}
    phases = (.14, .25, .28, .5, .68, .75, .84) if job.get('palm_lift_mode') in ('accent', 'gentle') else (.14, .28, .5, .68, .84)
    contract_times = set()
    for contact in job['contacts']:
        event_times = {t for t in (contact['start'], contact['end'], *(p[0] for p in contact['points']))
                       if start - 1 <= t <= end}
        contract_times.update(event_times)
        times.update(event_times)
        if contact.get('planned'):
            times.update(t for t in (contact['prepare'], contact['release_until']) if start - 1 <= t <= end)
    for gap in job.get('hand_rest', []):
        times.update(p[0] for p in gap['knots'] if start - 1 <= p[0] <= end)
        if (job.get('palm_lift_ratio') is not None and gap['mode'] == 'hover'
                and 0 < gap['end'] - gap['start'] <= job.get('stroke_gap_limit', .75)):
            times.update(t for u in phases
                         if start - 1 <= (t := gap['start'] + u * (gap['end'] - gap['start'])) <= end)
    if job.get('finger_motion') == 'whole_finger':
        tracks = {}
        for c in job['contacts']:
            tracks.setdefault((c['hand'], c['finger']), []).append(c)
        for events in tracks.values():
            events.sort(key=lambda c: c['start'])
            for previous, following in zip(events, events[1:]):
                gap = following['start'] - previous['end']
                if 0 < gap <= job.get('stroke_gap_limit', .75):
                    times.update(t for u in phases
                                 if start - 1 <= (t := previous['end'] + u * gap) <= end)
    ordered = sorted(times)
    times.update((a + b) / 2 for a, b in zip(ordered, ordered[1:]) if b - a > 1 / 120)
    # Equivalent event/phase expressions can differ by a few floating-point
    # ulps. Do not run a second pose solve with effectively zero elapsed time.
    # Preserve the exact PSAP event value when it shares such a sample.
    canonical = {}
    for t in sorted(times):
        key = round(t*1e9)
        if key not in canonical or t in contract_times:
            canonical[key] = t
    return [canonical[key] for key in sorted(canonical)]


def assign_contacts(job, offsets):
    tracks = {key: [] for key in offsets}
    if job.get('motion_plan_version'):
        for contact in sorted(job['contacts'], key=lambda c: (c['start'], c['pointer'])):
            if not contact.get('planned') or contact.get('hand') not in ('left', 'right'):
                raise ValueError('Missing algo5 physical finger assignment')
            key = (-1 if contact['hand'] == 'left' else 1, contact['finger'])
            if key not in tracks:
                raise ValueError(f'Uncalibrated finger in motion plan: {key}')
            if tracks[key] and tracks[key][-1]['end'] >= contact['start']:
                raise ValueError(f'Overlapping planned contacts: {key}')
            tracks[key].append(contact)
        return tracks
    preference = {'index': 0., 'middle': .00008, 'ring': .0003, 'little': .0006, 'thumb': .0005}
    ordered = sorted(job['contacts'], key=lambda c: (c['start'], c['points'][0][1]))
    for index, contact in enumerate(ordered):
        t = contact['start']
        following = [c for c in ordered[index + 1:] if c['start'] <= contact['end']]
        candidates = []
        for key, offset in offsets.items():
            # The supplied thumb rig needs a different opposition model; keep it relaxed for now.
            if key[1] == 'thumb':
                continue
            history = tracks[key]
            if history and history[-1]['end'] >= t:
                continue
            hand, finger = key
            # Evaluate the entire contact, including its held endpoint after a Flick.
            times = {t, contact['end'], *(p[0] for p in contact['points'])}
            others = [(other, c) for other, events in tracks.items() if other[0] == hand
                      for c in events if c['end'] > t and c['start'] < contact['end']]
            for _, c in others:
                times.update(max(t, min(contact['end'], v)) for v in (c['end'], *(p[0] for p in c['points'])))
            costs = []
            for when in times:
                x, y = world_point(job, contact_position(contact, when))
                wrist = (x - offset[0], y - offset[1])
                anchors = []
                for other, c in others:
                    if c['start'] <= when <= c['end']:
                        px, py = world_point(job, contact_position(c, when))
                        anchors.append((px - offsets[other][0], py - offsets[other][1]))
                cost = sum((wrist[0] - a[0]) ** 2 + (wrist[1] - a[1]) ** 2 for a in anchors) / max(1, len(anchors))
                cost += max(0., .045 - hand * wrist[0]) ** 2 * 5
                costs.append(cost)
            cost = max(costs) + preference[finger]
            # Reserve a plausible neighbour for a later overlapping touch, before fixing this finger.
            future_costs = []
            for later in following:
                px, py = world_point(job, later['points'][0][1:])
                if hand * px < -.015:
                    continue
                x, y = world_point(job, contact_position(contact, later['start']))
                fits = [((x - offset[0]) - (px - other_offset[0])) ** 2
                        + ((y - offset[1]) - (py - other_offset[1])) ** 2
                        for other, other_offset in offsets.items() if other[0] == hand and other != key]
                if fits:
                    future_costs.append(min(fits))
            cost += .5 * max(future_costs, default=0.)
            if history:
                x, y = world_point(job, contact['points'][0][1:])
                px, py = world_point(job, history[-1]['points'][-1][1:])
                cost += .03 * ((px - x) ** 2 + (py - y) ** 2) / max(.05, t - history[-1]['end'])
            candidates.append((cost, key))
        if not candidates:
            raise ValueError(f'More than eight non-thumb contacts at {t:.3f}s')
        # ponytail: greedy whole-trajectory scoring; use joint reassignment when reaches still fail.
        key = min(candidates)[1]
        tracks[key].append(contact)
    return tracks


def finger_sample(events, t, rest, job):
    if job.get('motion_plan_version'):
        return finger_pose(events, t, rest, job)
    i = bisect.bisect_right(events, t, key=lambda c: c['start']) - 1
    previous = events[i] if i >= 0 else None
    following = events[i + 1] if i + 1 < len(events) else None
    h = job['contact_height']
    if previous and t < previous['end']:
        return (*world_point(job, contact_position(previous, t)), h), 1., True
    if previous and following and following['start'] - previous['end'] < .6:
        a = world_point(job, previous['points'][-1][1:])
        b = world_point(job, following['points'][0][1:])
        gap = following['start'] - previous['end']
        u = smooth((t - previous['end']) / max(.000001, gap))
        lift = min(job.get('lift_height', .025), .008 + .18 * math.dist(a, b)) * min(1., gap / .12) * math.sin(math.pi * u)
        weight = max(1 - smooth((t - previous['end']) / .16), 1 - smooth((following['start'] - t) / .20))
        return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, h + lift), weight, False
    if following and (not previous or following['start'] - t < .22):
        b = world_point(job, following['points'][0][1:])
        u = 1 - smooth((following['start'] - t) / .22)
        return (rest[0] * (1 - u) + b[0] * u, rest[1] * (1 - u) + b[1] * u,
                rest[2] * (1 - u) + h * u), u, False
    if previous:
        a = world_point(job, previous['points'][-1][1:])
        u = smooth((t - previous['end']) / .18)
        return (a[0] * (1 - u) + rest[0] * u, a[1] * (1 - u) + rest[1] * u,
                h * (1 - u) + rest[2] * u), 1 - u, False
    return rest, 0., False


def skin_pads(meshes, indices):
    """Lowest patch of each distal phalanx on the evaluated (deformed) mesh."""
    import bpy
    from mathutils import Vector

    pads = {}
    for side, obj in meshes.items():
        evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
        mesh = evaluated.to_mesh()
        for key, vertices in indices.items():
            if key[0] != side:
                continue
            points = [evaluated.matrix_world @ mesh.vertices[i].co for i in vertices]
            z = min(p.z for p in points)
            # A small patch centroid avoids jumping between almost-level adjacent vertices.
            patch = [p for p in points if p.z <= z + .0002]
            pad = sum(patch, Vector()) / len(patch)
            pad.z = z
            pads[key] = pad
        evaluated.to_mesh_clear()
    return pads


def cache_static_geometry(meshes):
    """Evaluate static subdivision once; keep the following armature live."""
    import bpy

    cached = []
    for obj in meshes.values():
        modifiers = list(obj.modifiers)
        armature = next((i for i, m in enumerate(modifiers) if m.type == 'ARMATURE'), 0)
        prefix = modifiers[:armature]
        if (not prefix or any(m.type != 'MULTIRES' for m in prefix)
                or any(m.show_viewport != m.show_render or m.levels != m.render_levels for m in prefix)
                or obj.data.shape_keys or obj.data.animation_data or obj.animation_data):
            continue
        original = obj.data
        flags = [(m, m.show_viewport, m.show_render) for m in modifiers]
        prepared = None
        try:
            for modifier in modifiers[armature:]:
                modifier.show_viewport = False
            bpy.context.view_layer.update()
            graph = bpy.context.evaluated_depsgraph_get()
            evaluated = obj.evaluated_get(graph)
            prepared = bpy.data.meshes.new_from_object(evaluated, preserve_all_data_layers=True, depsgraph=graph)
            obj.data = prepared
            for modifier, viewport, render in flags:
                modifier.show_viewport = False if modifier in prefix else viewport
                modifier.show_render = False if modifier in prefix else render
            cached.append((obj, original, prepared, flags))
        except Exception:
            obj.data = original
            for modifier, viewport, render in flags:
                modifier.show_viewport, modifier.show_render = viewport, render
            if prepared is not None:
                bpy.data.meshes.remove(prepared)
            raise
    bpy.context.view_layer.update()
    return cached


def restore_static_geometry(cached):
    import bpy

    for obj, original, prepared, flags in cached:
        obj.data = original
        for modifier, viewport, render in flags:
            modifier.show_viewport, modifier.show_render = viewport, render
        bpy.data.meshes.remove(prepared)
    bpy.context.view_layer.update()


def mesh_world_coordinates(mesh, matrix):
    """Match mathutils: float32 products, double accumulation, float32 coordinates."""
    import numpy as np

    vertices = np.empty((len(mesh.vertices), 3), dtype=np.float32)
    mesh.vertices.foreach_get('co', vertices.ravel())
    transform = np.asarray(matrix, dtype=np.float32)
    points = (vertices[:, 0:1] * transform[:3, 0]).astype(np.float64)
    points += vertices[:, 1:2] * transform[:3, 1]
    points += vertices[:, 2:3] * transform[:3, 2]
    points += transform[:3, 3]
    return points.astype(np.float32)


def project_shadow_mesh(mesh, matrix):
    """Apply the existing world-space shadow projection without per-vertex RNA calls."""
    import numpy as np

    # Python's scalar projection used double arithmetic on mathutils float32 values.
    points = mesh_world_coordinates(mesh, matrix).astype(np.float64)
    points[:, 0] += .35 * points[:, 2]
    points[:, 1] -= .45 * points[:, 2]
    points[:, 2] = 0.
    mesh.vertices.foreach_set('co', points.astype(np.float32).ravel())
    mesh.update()


def screen_mesh_clearance(mesh, matrix, screen):
    import numpy as np

    points = mesh_world_coordinates(mesh, matrix)
    xy = points[:, :2].astype(np.float64)
    inside = (np.abs(xy[:, 0]) <= screen[0] / 2) & (np.abs(xy[:, 1]) <= screen[1] / 2)
    return float(points[inside, 2].min()) if inside.any() else math.inf


def closest_segments(a, b, c, d):
    """Closest points of finite segments, including parallel/zero-length segments."""
    u, v, w = (tuple(x - y for x, y in zip(p, q)) for p, q in ((b, a), (d, c), (a, c)))
    aa, bb, cc, dd, ee = (sum(x * y for x, y in zip(p, q))
                          for p, q in ((u, u), (u, v), (v, v), (u, w), (v, w)))
    clamp = lambda x: max(0., min(1., x))
    if aa < 1e-12:
        s, t = 0., clamp(ee / cc) if cc > 1e-12 else 0.
    elif cc < 1e-12:
        s, t = clamp(-dd / aa), 0.
    else:
        determinant = aa * cc - bb * bb
        s = clamp((bb * ee - cc * dd) / determinant) if determinant > 1e-12 * aa * cc else 0.
        t = (bb * s + ee) / cc
        if t < 0:
            s, t = clamp(-dd / aa), 0.
        elif t > 1:
            s, t = clamp((bb - dd) / aa), 1.
    return tuple(x + s * y for x, y in zip(a, u)), tuple(x + t * y for x, y in zip(c, v))


def hand_collisions(rigs, templates, margin=0.):
    from mathutils import Vector

    capsules = []
    for key, bone, a, b, radius in templates:
        arm = rigs[key[0]]
        transform = arm.matrix_world @ arm.pose.bones[bone].matrix
        a, b = transform @ a, transform @ b
        center = (a + b) / 2
        half = Vector((abs(a.x - b.x) / 2 + radius, abs(a.y - b.y) / 2 + radius, abs(a.z - b.z) / 2 + radius))
        capsules.append((key, a, b, radius, center, half))
    collisions = []
    # ponytail: fewer than 40 capsules; use spatial hashing if the rig grows substantially.
    for i, (key, a, b, radius, center, half) in enumerate(capsules):
        for other, c, d, other_radius, other_center, other_half in capsules[i + 1:]:
            if key[0] == other[0]:
                if (key[1] == 'palm' and key[2] >= 100) or (other[1] == 'palm' and other[2] >= 100):
                    # Broad skin-group bounds include the connected webbing of this same hand.
                    # Use them for opposite-hand coverage, with the original proxies for self-collision.
                    continue
                if key[1] == other[1] and (key[1] == 'palm' or abs(key[2] - other[2]) <= 1):
                    continue
                # Proximal fingers and the thumb root are continuous with the palm mesh.
                if key[1] == 'palm' and other[2] == 0 or other[1] == 'palm' and key[2] == 0:
                    continue
            # Disjoint capsule bounds cannot collide; avoid the costly segment solve.
            if (abs(center.x - other_center.x) > half.x + other_half.x + margin
                    or abs(center.y - other_center.y) > half.y + other_half.y + margin
                    or abs(center.z - other_center.z) > half.z + other_half.z + margin):
                continue
            p, q = closest_segments(a, b, c, d)
            delta = Vector(p) - Vector(q)
            clearance = delta.length - radius - other_radius
            if clearance < margin - .0005:
                direction = delta.normalized() if delta.length > 1e-8 else Vector((key[0], 0, 1)).normalized()
                collisions.append((clearance, key, other, direction))
    return sorted(collisions, key=lambda item: item[0])


def screen_rect(job):
    w, h = job['width'], job['height']
    sw = max(2, round(w * job['screen'][0] / job['view_width']) // 2 * 2)
    sh = max(2, round(sw * job['screen'][1] / job['screen'][0]) // 2 * 2)
    x = (w - sw) // 2
    y = round(h / 2 + (job['camera_y'] - job['screen'][1] / 2) * w / job['view_width'])
    if x < 8 or x + sw + 8 > w or y < 8 or y + sh + 8 > h:
        raise ValueError('No room for the tablet; reduce --screen-fill or use 720x480')
    return x, y, sw, sh


def configure_camera(scene, job):
    """Keep the screen plane aligned while letting raised hands approach the lens."""
    camera = scene.camera
    camera.rotation_euler = (0, 0, 0)
    camera.data.shift_x = camera.data.shift_y = 0
    camera.data.sensor_fit = 'HORIZONTAL'
    if job.get('camera_projection') == 'perspective':
        height = job.get('camera_height', .45)
        offset_y = job.get('camera_offset_y', 0.)
        camera.data.type = 'PERSP'
        camera.location = (0, job['camera_y'] + offset_y, height)
        camera.data.lens = camera.data.sensor_width * height / job['view_width']
        # Move the lens toward the player to keep raised palms in frame, then
        # compensate with lens shift so the screen plane stays pixel-aligned.
        camera.data.shift_y = -offset_y / job['view_width']
    else:
        camera.data.type, camera.data.ortho_scale = 'ORTHO', job['view_width']
        camera.location = (0, job['camera_y'], 1)


def camera_pixel(scene, point):
    """Project a world point into the full frame, including perspective depth."""
    from bpy_extras.object_utils import world_to_camera_view

    uv = world_to_camera_view(scene, scene.camera, point)
    return uv.x * scene.render.resolution_x, (1 - uv.y) * scene.render.resolution_y


def composite_command(job, frame=None):
    out = Path(job['output'])
    w, h, fps = job['width'], job['height'], job['fps']
    x, y, sw, sh = screen_rect(job)
    command = ['ffmpeg', '-v', 'warning', '-y']
    for directory in ('screen', 'hands', 'shadows'):
        if frame is None and directory == 'screen' and job.get('screen_video'):
            command += ['-i', job['screen_video']]
        elif frame is None:
            command += ['-framerate', str(fps), '-start_number', '1', '-i', str(out / directory / '%05d.png')]
        else:
            command += ['-i', str(out / directory / f'{frame:05d}.png')]
    if frame is None:
        command += ['-ss', '0' if job.get('mixed_audio') else str(job['start']),
                    '-i', job.get('mixed_audio', job['audio'])]
    background = job.get('background')
    if background:
        command += ['-loop', '1', '-framerate', str(fps), '-i', background]
    # The Blender pass projects the actual mesh onto the screen, including its current height.
    base = (f'[{4 if frame is None else 3}:v][0:v]overlay={x}:{y}:shortest=1,' if background
            else f'[0:v]pad={w}:{h}:{x}:{y}:color=0x343c46,')
    graph = (base + f'drawbox=x={x-7}:y={y-7}:w={sw+14}:h={sh+14}:color=0x1d2530:t=7[bg];'
             '[2:v]alphaextract,boxblur=3:1,lutyuv=y=val*0.20[alpha];'
             f'color=black:s={w}x{h}:r={fps},format=rgba[black];'
             '[black][alpha]alphamerge[shadow];[bg][shadow]overlay=0:0:shortest=1[under];'
             '[under][1:v]overlay=0:0:shortest=1,format=yuv420p[video]')
    command += ['-filter_complex', graph, '-map', '[video]']
    if frame is None:
        command += ['-map', '3:a:0', '-c:v', 'libx264', '-preset', 'fast', '-crf', '20',
                    '-c:a', 'aac', '-b:a', '192k', '-t', str(job['duration']), '-movflags', '+faststart',
                    str(out / 'handcam.partial.mp4')]
    else:
        command += ['-frames:v', '1', '-update', '1', str(out / 'inspection' / f'{frame:05d}.png')]
    return command


def render_shadow(scene, meshes, path):
    """Directional projection of evaluated skin; no additional renderer/dependency."""
    import bpy

    objects = []
    for obj in meshes.values():
        evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
        mesh = bpy.data.meshes.new_from_object(evaluated)
        for vertex in mesh.vertices:
            point = evaluated.matrix_world @ vertex.co
            vertex.co = (point.x + .35 * point.z, point.y - .45 * point.z, 0.)
        shadow = bpy.data.objects.new('Handcam shadow', mesh)
        scene.collection.objects.link(shadow)
        shadow.color = (0, 0, 0, 1)
        objects.append(shadow)
        obj.hide_render = True
    engine = scene.render.engine
    scene.render.engine = 'BLENDER_WORKBENCH'
    scene.display.shading.light = 'FLAT'
    scene.display.shading.show_cavity = False
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)
    for obj in objects:
        mesh = obj.data
        bpy.data.objects.remove(obj, do_unlink=True)
        bpy.data.meshes.remove(mesh)
    for obj in meshes.values():
        obj.hide_render = False
    scene.render.engine = engine
    scene.display.shading.light = 'STUDIO'
    scene.display.shading.show_cavity = True


def main(job):
    import bpy
    from mathutils import Matrix, Vector

    scene = bpy.context.scene
    use_thumb = job.get('enable_thumb', False)
    planned_pose = bool(job.get('motion_plan_version') or job.get('profile_only'))
    out = Path(job['output'])
    (out / 'hands').mkdir(exist_ok=True)
    (out / 'inspection').mkdir(exist_ok=True)
    (out / 'shadows').mkdir(exist_ok=True)
    for collection in bpy.data.collections:
        collection.hide_render = collection.hide_viewport = False
    for obj in bpy.data.objects:
        obj.hide_set(False)
        obj.hide_render = obj.hide_viewport = False
        if obj.type == 'MESH':
            for modifier in obj.modifiers:
                if modifier.type == 'MULTIRES':
                    modifier.levels = modifier.render_levels = 1
                elif modifier.type == 'ARMATURE':
                    modifier.use_deform_preserve_volume = True
            for polygon in obj.data.polygons:
                polygon.use_smooth = True
            obj.color = (.82, .60, .46, 1)
            material = bpy.data.materials.new('Handcam skin')
            material.diffuse_color = obj.color
            material.use_nodes = True
            shader = material.node_tree.nodes.get('Principled BSDF')
            shader.inputs['Base Color'].default_value = obj.color
            shader.inputs['Roughness'].default_value = .5
            obj.data.materials.clear()
            obj.data.materials.append(material)
    rigs, roots, neutral, offsets, bases, lengths, targets = {}, {}, {}, {}, {}, {}, {}
    meshes, pad_indices, collider_templates = {}, {}, []
    # The source fingers point along -Y, with the back of the hand toward +Z.
    for side, name in ((-1, 'Hand_Left'), (1, 'Hand_Right')):
        if name not in bpy.data.objects:
            raise ValueError(f'Model needs the supplied hand rig: missing {name}')
        arm = bpy.data.objects[name]
        arm.rotation_mode = 'XYZ'
        rigs[side] = arm
        meshes[side] = next(obj for obj in scene.objects if obj.type == 'MESH' and obj.parent == arm)
        for bone in arm.pose.bones:
            bone.matrix_basis = Matrix.Identity(4)
            for constraint in list(bone.constraints):
                bone.constraints.remove(constraint)
            bone.ik_stretch = 0
        wrist = arm.data.bones['Bone.016'].head_local.copy()
        roots[side] = Vector((side * .075, -.16, .065 * job['hand_scale'] / .29))
        neutral[side] = Matrix.Rotation(math.pi, 4, 'Z') @ Matrix.Scale(job['hand_scale'], 4) @ Matrix.Translation(-wrist)
        arm.matrix_world = Matrix.Translation(roots[side]) @ neutral[side]
        # Tuck idle thumbs alongside the palm to avoid crossing the other hand.
        thumb_base = arm.pose.bones['Bone.021']
        thumb_base.rotation_mode = 'XYZ'
        thumb_base.rotation_euler.z = -side * (.3 if planned_pose else .12)
        for finger, chain in FINGERS.items():
            angles = ((-.35, -.4, -.3) if planned_pose else (-.1, -.15, -.1)) if finger == 'thumb' else (.12, -.6, -.35)
            for joint, (bone_name, angle) in enumerate(zip(chain, angles)):
                bone = arm.pose.bones[bone_name]
                bone.rotation_mode = 'XYZ'
                bone.rotation_euler.x = angle
                if finger == 'thumb' and joint == 0:
                    bone.rotation_euler.z = -side * .15
                elif joint == 0:
                    bone.rotation_euler.z = -side * {'index': -.08, 'middle': -.02, 'ring': .08, 'little': .16}[finger]
                bone.lock_ik_y = finger != 'thumb' or joint > 0
                if finger == 'thumb' and joint == 0 and use_thumb:
                    bone.use_ik_limit_y = True
                    bone.ik_min_y, bone.ik_max_y = -.55, .55
                bone.use_ik_limit_x = True
                bone.ik_min_x, bone.ik_max_x = -1.55, (.4 if joint == 0 else 0.)
                if joint:
                    bone.lock_ik_z = True
                else:
                    bone.use_ik_limit_z = True
                    spread = .65 if finger=='thumb' else {'index':.35,'middle':.22,'ring':.28,'little':.35}[finger]
                    bone.ik_min_z, bone.ik_max_z = -spread, spread
        bpy.context.view_layer.update()
        for finger, chain in FINGERS.items():
            key = (side, finger)
            bases[key] = arm.matrix_world @ arm.pose.bones[chain[0]].head - roots[side]
            lengths[key] = sum(arm.data.bones[n].length for n in chain) * job['hand_scale']
            target = bpy.data.objects.new(f'{side}_{finger}_target', None)
            scene.collection.objects.link(target)
            target.empty_display_size = .004
            target.location = arm.matrix_world @ arm.pose.bones[chain[-1]].tail
            targets[key] = target
            ik = arm.pose.bones[chain[-1]].constraints.new('IK')
            ik.target, ik.chain_count, ik.use_stretch = target, 3, False
            ik.iterations = 32
            if finger == 'thumb':
                ik.influence = 0
        obj = meshes[side]
        evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
        mesh = evaluated.to_mesh()
        for finger, chain in FINGERS.items():
            for joint, bone_name in enumerate(chain):
                bone = arm.pose.bones[bone_name]
                group = obj.vertex_groups[bone_name].index
                local = bone.matrix.inverted() @ arm.matrix_world.inverted() @ evaluated.matrix_world
                points = [local @ v.co for v in mesh.vertices
                          if any(g.group == group and g.weight > .5 for g in v.groups)]
                low = Vector(tuple(min(p[i] for p in points) for i in range(3)))
                high = Vector(tuple(max(p[i] for p in points) for i in range(3)))
                section = [p for p in points if low.y + (high.y - low.y) * .3 < p.y < low.y + (high.y - low.y) * .75]
                if section:
                    for axis in (0, 2):
                        low[axis], high[axis] = min(p[axis] for p in section), max(p[axis] for p in section)
                center = (low + high) / 2
                radius = max(high.x - low.x, high.z - low.z) / 2
                a, b = center.copy(), center.copy()
                a.y, b.y = min(center.y, low.y + radius), max(center.y, high.y - radius)
                if joint == 0:
                    # The webbing/base belongs to the palm proxy, not two colliding fingers.
                    a.y = max(a.y, low.y + (high.y - low.y) * .45)
                collider_templates.append(((side, finger, joint), bone_name, a, b, radius * job['hand_scale']))
            if finger == 'thumb' and not use_thumb:
                continue
            bone = arm.pose.bones[chain[-1]]
            group = obj.vertex_groups[chain[-1]].index
            local = bone.matrix.inverted() @ arm.matrix_world.inverted() @ evaluated.matrix_world
            pad_indices[side, finger] = [v.index for v in mesh.vertices
                if any(g.group == group and g.weight > .5 for g in v.groups)
                and (local @ v.co).y > bone.length * .55]
            if not pad_indices[side, finger]:
                raise ValueError(f'No fingertip skin vertices: {name}/{finger}')
            palm = arm.pose.bones['Bone.016']
            end = palm.matrix.inverted() @ arm.pose.bones[chain[0]].head
            collider_templates.append(((side, 'palm', len(collider_templates)), 'Bone.016',
                                       end * .15, end * .8, .012 * job['hand_scale'] / .29))
        if job.get('pose_avoidance', True):
            # Cover the metacarpals, thumb base and forearm as well as the finger chains.
            # These connected palm regions are excluded from self-palm collision checks.
            for index, name in enumerate(('Bone.012', 'Bone.013', 'Bone.014', 'Bone.015', 'Bone.021', 'Bone.020')):
                bone = arm.pose.bones[name]
                group = obj.vertex_groups[name].index
                local = bone.matrix.inverted() @ arm.matrix_world.inverted() @ evaluated.matrix_world
                points = [local @ v.co for v in mesh.vertices
                          if any(g.group == group and g.weight > .5 for g in v.groups)]
                if not points:
                    continue
                low = Vector(tuple(min(p[i] for p in points) for i in range(3)))
                high = Vector(tuple(max(p[i] for p in points) for i in range(3)))
                center = (low + high) / 2
                radius = max(high.x - low.x, high.z - low.z) / 2
                a, b = center.copy(), center.copy()
                a.y, b.y = min(center.y, low.y + radius), max(center.y, high.y - radius)
                collider_templates.append(((side, 'palm', 100 + index), name, a, b, radius * job['hand_scale']))
        evaluated.to_mesh_clear()
    pads = skin_pads(meshes, pad_indices)
    pad_offsets = {key: pad - targets[key].location for key, pad in pads.items()}
    offsets = {key: pad - roots[key[0]] for key, pad in pads.items()}
    rest_offsets, rest_bases = offsets.copy(), bases.copy()
    natural_palm = job.get('palm_motion') == 'v4'
    guide_offsets = palm_offsets(dict(scale=job['hand_scale'], fingers={
        f'{side}:{finger}': dict(offset=list(offset), base=list(bases[side, finger]),
                                length=lengths[side, finger])
        for (side, finger), offset in rest_offsets.items()}))
    yaw = {side: 0. for side in rigs}
    (out / 'rig.json').write_text(json.dumps({
        'palm_offsets': {f'{side}:{finger}': list(offset) for (side, finger), offset in guide_offsets.items()},
        'pad_vertices': {f'{side}:{finger}': indices for (side, finger), indices in pad_indices.items()},
        'capsules': [{'key': key, 'bone': bone, 'a': list(a), 'b': list(b), 'radius': radius}
                     for key, bone, a, b, radius in collider_templates]}, indent=2), encoding='utf-8')
    if job.get('profile_only'):
        profile = dict(version=1, scale=job['hand_scale'], source='measured supplied Blender rig',
                       fingers={f'{side}:{finger}': dict(offset=list(offset), base=list(bases[side, finger]),
                                length=lengths[side, finger]) for (side, finger), offset in offsets.items()})
        (out / 'hand-profile.json').write_text(json.dumps(profile, indent=2), encoding='utf-8')
        print('HANDCAM_PROGRESS 1', flush=True)
        return
    # Targets and sampling now refer to skin, not to the bone inside the fingertip.
    visual_contacts, shared = ((job['contacts'], []) if job.get('strict_psap') or job.get('motion_plan_version')
                               else coalesce_contacts(job['contacts'], job.get('hold_starts', ())))
    releases = []
    if not job.get('strict_psap') and not job.get('motion_plan_version'):
        visual_contacts, releases = release_idle_contacts(visual_contacts, job.get('hold_intervals', ()))
    motion_job = dict(job, contacts=visual_contacts)
    motion_index = (MotionIndex(visual_contacts, job.get('hand_rest', ()))
                    if job.get('motion_plan_version') and job.get('bake_motion_index', True) else None)
    (out / 'motion.json').write_text(json.dumps({'contacts': visual_contacts, 'shared_events': shared,
                                               'idle_releases': releases}, indent=2), encoding='utf-8')

    static_geometry = cache_static_geometry(meshes) if job.get('bake_static_geometry', True) else []
    cached_pads = skin_pads(meshes, pad_indices)
    cache_error = max(((cached_pads[key] - pads[key]).length for key in pads), default=0.)
    if cache_error > 1e-7:
        restore_static_geometry(static_geometry)
        static_geometry = []
    fitting_stats = dict(calls=0, updates=0)

    def fit_skin(samples, steps=None):
        fitting_stats['calls'] += 1
        origins = {key: target.location.copy() for key, target in targets.items()}
        def correct(key, delta, gain):
            target = targets[key]
            offset = target.location + delta * gain - origins[key]
            limit = .018 * job['hand_scale'] / .29
            target.location = origins[key] + (offset.normalized() * limit if offset.length > limit else offset)
        # Stop on convergence or the fixed iteration budget. A wall-clock cutoff
        # made adjacent samples depend on unrelated CPU load and caused jitter.
        for _ in refinement_steps if steps is None else steps:
            fitting_stats['updates'] += 1
            bpy.context.view_layer.update()
            pads = skin_pads(meshes, pad_indices)
            # Skin fitting is a contact constraint. Chasing airborne targets can fold idle joints.
            residuals = {key: Vector(samples[key][0]) - pad for key, pad in pads.items() if samples[key][2]}
            if all(delta.length < .0002 for delta in residuals.values()):
                return pads
            for key, delta in residuals.items():
                correct(key, delta, .9)
        fitting_stats['updates'] += 1
        bpy.context.view_layer.update()
        return skin_pads(meshes, pad_indices)
    tracks = assign_contacts(motion_job, offsets)
    from handcam_avoidance import PoseAvoidance
    avoidance = PoseAvoidance(rigs, targets, FINGERS,
        motion_limits=job.get('avoidance_motion_limits',dict(wrist_speed=.18,max_wrist_shift=.025))) if job.get('pose_avoidance', True) else None
    airborne_floor_corrections = 0

    def clear_airborne_skin(samples,pads,t):
        """Keep inactive distal skin above the screen without moving held targets."""
        nonlocal airborne_floor_corrections
        if avoidance is None:
            return pads
        for _ in range(3):
            changed = False
            for key,pad in pads.items():
                if samples[key][2]:
                    continue
                events = tracks[key]
                at = bisect.bisect_right(events,t,key=lambda c:c['start'])-1
                previous = events[at] if at>=0 else None
                following = events[at+1] if at+1<len(events) else None
                release = smooth((t-previous['end'])/.08) if previous else 1.
                landing = smooth((following['start']-t)/.08) if following else 1.
                floor = job['contact_height']+.0075*release*landing
                if pad.z < floor-.0001:
                    avoidance.move_finger(key,Vector((0.,0.,min(.012,floor-pad.z))),samples)
                    airborne_floor_corrections += 1
                    changed = True
            if not changed:
                break
            pads = fit_skin(samples,range(2))
        return pads
    palm_bones = {(side, arm.pose.bones[chain[0]].parent.name): arm.pose.bones[chain[0]].parent
                  for side, arm in rigs.items() for chain in FINGERS.values()}
    palm_rest = {key: bone.matrix_basis.to_euler('XYZ') for key, bone in palm_bones.items()}
    if avoidance:
        for bone in palm_bones.values():
            bone.rotation_mode = 'XYZ'

    def mesh_clearance():
        lowest = math.inf
        graph = bpy.context.evaluated_depsgraph_get()
        for obj in meshes.values():
            evaluated = obj.evaluated_get(graph)
            mesh = evaluated.to_mesh()
            try:
                lowest = min(lowest, screen_mesh_clearance(mesh, evaluated.matrix_world, job['screen']))
            finally:
                evaluated.to_mesh_clear()
        return lowest
    contact_rotations = {}
    previous_down = set()
    contact_root_heights = {}
    relaxed_rotations = {key: [rigs[key[0]].pose.bones[name].rotation_euler.copy() for name in FINGERS[key[1]]]
                         for key in tracks}
    assignments = [{'hand': 'left' if k[0] == -1 else 'right', 'finger': k[1],
                    'pointer': c['pointer'], 'start': c['start'], 'end': c['end']}
                   for k, events in tracks.items() for c in events]
    (out / 'fingering.json').write_text(json.dumps(assignments, indent=2), encoding='utf-8')
    scene.render.engine = {'workbench': 'BLENDER_WORKBENCH', 'eevee': 'BLENDER_EEVEE_NEXT', 'cycles': 'CYCLES'}[job['engine']]
    scene.render.resolution_x, scene.render.resolution_y = job['width'], job['height']
    scene.render.resolution_percentage = 100
    scene.render.fps = job['fps']
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGBA'
    scene.render.image_settings.compression = 15
    scene.render.film_transparent = True
    scene.use_nodes = False
    scene.view_settings.view_transform = 'AgX'
    scene.view_settings.exposure, scene.view_settings.gamma = 0, 1
    scene.view_settings.look = 'AgX - Medium Low Contrast'
    scene.display.shading.light = 'STUDIO'
    scene.display.shading.color_type = 'OBJECT'
    scene.display.shading.show_shadows = True
    scene.display.shading.show_cavity = True
    scene.display.shading.cavity_type = 'BOTH'
    scene.display.render_aa = 'FXAA'
    if job['engine'] == 'cycles':
        scene.cycles.samples, scene.cycles.use_denoising = 8, True
        scene.cycles.device = 'CPU'
    elif job['engine'] == 'eevee':
        scene.eevee.taa_render_samples = 8
    configure_camera(scene, job)
    for obj in list(scene.objects):
        if obj.type == 'LIGHT':
            bpy.data.objects.remove(obj, do_unlink=True)
    light = bpy.data.lights.new('Handcam softbox', 'AREA')
    lamp = bpy.data.objects.new('Handcam softbox', light)
    scene.collection.objects.link(lamp)
    lamp.location, light.energy, light.size = (0, -.1, .5), 20, .45
    scene.world.use_nodes = True
    scene.world.node_tree.nodes.clear()
    bg = scene.world.node_tree.nodes.new('ShaderNodeBackground')
    bg.inputs['Color'].default_value, bg.inputs['Strength'].default_value = (.3, .34, .4, 1), .5
    world_out = scene.world.node_tree.nodes.new('ShaderNodeOutputWorld')
    scene.world.node_tree.links.new(bg.outputs[0], world_out.inputs['Surface'])
    scene.frame_start, scene.frame_end = 1, job['frames']
    selected = {1, job['frames'] // 4 + 1, job['frames'] // 2 + 1, job['frames'] * 3 // 4 + 1, job['frames']}
    errors, collision_errors = [], []
    collision_corrections = 0
    baked = {(side, name): [] for side in rigs for finger, chain in FINGERS.items()
             if finger != 'thumb' or use_thumb for name in chain}
    if avoidance:
        baked.update({key: [] for key in palm_bones})
    max_error, contact_samples = 0., 0
    min_clearance, mesh_samples = math.inf, 0
    # Warm up before the clip so a contact already on screen starts in the correct pose.
    previous_time = job['start'] - 1 - 1 / job['fps']
    motion_stats = dict(wrist_speed_clamps=0,max_wrist_lateral_speed_mps=0.)
    last_progress = 0
    for t in sample_times(motion_job):
        dt = t - previous_time
        previous_time = t
        previous_output_roots = {side:root.copy() for side,root in roots.items()}
        frame = (t - job['start']) * job['fps'] + 1
        video_frame = round(frame)
        is_video_frame = abs(frame - video_frame) < .00001 and 1 <= video_frame <= job['frames']
        scene.frame_set(max(1, math.floor(frame)), subframe=frame % 1 if frame >= 1 else 0)
        if avoidance:
            for key, bone in palm_bones.items():
                bone.rotation_euler = palm_rest[key]
        samples = {}
        for key, events in tracks.items():
            rest = roots[key[0]] + offsets[key]
            rest.z = max(rest.z, .012)
            samples[key] = finger_sample(events, t, rest, motion_job)
        for side, arm in rigs.items():
            previous_root = roots[side].copy()
            if avoidance:
                previous_root -= avoidance.wrists[side]
            reach_samples = dict(samples)
            # Planned wrist guides already reach the next contact at its deadline.
            # Projecting a future DOWN here pulled a withdrawing palm back early,
            # bypassing its smooth return trajectory and saturating the speed cap.
            active = [(Vector(pos) - offsets[k]) for k, (pos, _, down) in samples.items() if k[0] == side and down]
            aiming = [(Vector(pos), weight) for k, (pos, weight, _) in samples.items()
                      if k[0] == side and weight > .001]
            wanted = yaw[side]
            if aiming and (not job.get('motion_plan_version') or natural_palm):
                aim = sum((p * w for p, w in aiming), Vector()) / sum(w for _, w in aiming)
                wanted = max(-.45, min(.45, -math.atan2(aim.x - side * .12, max(.08, aim.y + .24))))
            for guide in job.get('pose_guides',[]):
                if guide['hand'] != ('left' if side==-1 else 'right') or not guide['prepare']<=t<=guide['release']:
                    continue
                weight = (smooth((t-guide['prepare'])/max(1e-6,guide['start']-guide['prepare'])) if t<guide['start'] else
                          smooth((guide['release']-t)/max(1e-6,guide['release']-guide['end'])) if t>guide['end'] else 1.)
                wanted += (guide['yaw']-wanted)*weight
            yaw[side] += (wanted-yaw[side])*(1-math.exp(-10*dt))
            rotation = Matrix.Rotation(yaw[side], 3, 'Z')
            for key in offsets:
                if key[0] == side:
                    offsets[key] = rotation @ rest_offsets[key]
                    bases[key] = rotation @ rest_bases[key]
            weighted = [(Vector(pos) - offsets[k], weight) for k, (pos, weight, _) in samples.items()
                        if k[0] == side and weight > .001]
            if weighted:
                desired = sum((p * w for p, w in weighted), Vector()) / sum(w for _, w in weighted)
                desired.z = min(.085 * job['hand_scale'] / .29, max(.055 * job['hand_scale'] / .29, desired.z))
                roots[side] = desired
            if job.get('motion_plan_version'):
                rotated_guide = {key: rotation @ Vector(offset) for key, offset in guide_offsets.items()}
                roots[side] = Vector(wrist_pose(motion_job['contacts'], job['hand_rest'], side, t, job,
                    rotated_guide if natural_palm else rest_offsets, guide_offsets if natural_palm else None, motion_index))
            if natural_palm:
                active = [(Vector(pos) - rotated_guide[k])
                          for k, (pos, _, down) in samples.items() if k[0] == side and down]
            if active:
                anchor = sum(active, Vector()) / len(active)
                delta = Vector((roots[side].x - anchor.x, roots[side].y - anchor.y, 0.))
                reach = .015 * job['hand_scale'] / .29
                if delta.length > reach:
                    center = anchor + delta.normalized() * reach
                    roots[side].x, roots[side].y = center.x, center.y
            # Project wrist XY into the active fingers' reach disks before solving the joints.
            for _ in range(16):
                for key, (pos, _, active) in reach_samples.items():
                    if key[0] != side or not active:
                        continue
                    base = bases[key]
                    radius = math.sqrt(max(.000001, (lengths[key] * .94) ** 2 - (roots[side].z + base.z - pos[2]) ** 2))
                    center = Vector((pos[0] - base.x, pos[1] - base.y, roots[side].z))
                    delta = roots[side] - center
                    if delta.length > radius:
                        roots[side] = center + delta.normalized() * radius
                if not job.get('motion_plan_version'):
                    roots[side].x = side * max(.045, side * roots[side].x)
            # Smooth after reach corrections, which can otherwise teleport the whole hand on a sudden MOVE.
            # ponytail: abrupt contacts may miss; use a motion-aware planner if exact contact is required.
            if job.get('motion_plan_version'):
                # The guide already anticipates contacts; bound only residual IK corrections.
                delta = roots[side] - previous_root
                maximum = job.get('wrist_speed', .9) * max(0., dt)
                if job.get('palm_lift_mode') in ('accent', 'gentle') and dt > 0:
                    # Respect the planned vertical stroke independently of the
                    # lateral transfer; gentle strokes are bounded before solving.
                    lateral = Vector((delta.x, delta.y, 0.))
                    if lateral.length > maximum:
                        lateral *= maximum / lateral.length
                    vertical = job.get('palm_strike_speed', 4.5) * dt
                    roots[side] = previous_root + lateral + Vector((0., 0., max(-vertical, min(vertical, delta.z))))
                elif delta.length > maximum and dt > 0:
                    roots[side] = previous_root + delta.normalized() * maximum
            else:
                roots[side] = previous_root.lerp(roots[side], 1 - math.exp(-18 * dt))
            arm.matrix_world = Matrix.Translation(roots[side]) @ rotation.to_4x4() @ neutral[side]
            arm.rotation_euler.z = math.pi + yaw[side]  # Keep the angle continuous through 180 degrees.
        for key, target in targets.items():
            if key not in samples:
                continue
            pos, weight, active = samples[key]
            # Recompute idle points after moving the wrist, so they follow the palm.
            rest = roots[key[0]] + offsets[key]
            rest.z = max(rest.z, .012)
            samples[key] = finger_sample(tracks[key], t, rest, motion_job)
            if job.get('motion_plan_version') and not samples[key][2]:
                # An idle/preparing finger travels with the palm. A world-space
                # interpolation toward a later note can otherwise drag it across
                # its neighbours while another finger controls this hand.
                pos, weight, down = samples[key]
                pos = Vector(pos)
                clamped = pos.copy()
                clamped.x = max(rest.x - .012, min(rest.x + .012, pos.x))
                clamped.y = max(rest.y - .025, min(rest.y + .025, pos.y))
                events = tracks[key]
                at = bisect.bisect_right(events,t,key=lambda c:c['start'])-1
                previous = events[at] if at>=0 else None
                following = events[at+1] if at+1<len(events) else None
                landing = smooth(1-(following['start']-t)/.12) if following else 0.
                release = smooth(1-(t-previous['end'])/.12) if previous else 0.
                # Removing the clamp only at DOWN caused a target discontinuity
                # and a missed first contact. Blend it out before landing and
                # back in after release, preserving both contact endpoints.
                samples[key] = (clamped.lerp(pos,max(landing,release)),weight,down)
            # Unused fingers keep their relaxed joints; fingertip-only IK can fold them arbitrarily.
            influence = samples[key][1]
            if job.get('finger_motion') == 'whole_finger' and key[1] == 'thumb' and not samples[key][2]:
                for name, angles in zip(FINGERS['thumb'], relaxed_rotations[key]):
                    rigs[key[0]].pose.bones[name].rotation_euler = angles
            if job.get('finger_motion') == 'whole_finger' and key[1] != 'thumb':
                bones = [rigs[key[0]].pose.bones[name] for name in FINGERS[key[1]]]
                pose = contact_rotations.get(key, relaxed_rotations[key])
                events = tracks[key]
                i = bisect.bisect_right(events, t, key=lambda c: c['start']) - 1
                previous = events[i] if i >= 0 else None
                following = events[i + 1] if i + 1 < len(events) else None
                if samples[key][2] and key not in previous_down:
                    # Start contact IK from the prepared pose, not a pose cached
                    # at the end of an unrelated earlier contact.
                    pose = [bone.bone.convert_local_to_pose(bone.matrix,bone.bone.matrix_local,
                        parent_matrix=bone.parent.matrix,parent_matrix_local=bone.parent.bone.matrix_local,
                        invert=True).to_euler('XYZ') for bone in bones]
                elif not samples[key][2] and previous and (
                        following is None or following['start']-previous['end']>job.get('stroke_gap_limit',.75)):
                    # A long idle gap releases the old grip. Keeping its curled
                    # FK pose indefinitely can cross a neighbour's new stroke.
                    settle = smooth((t-previous['end'])/.18)
                    pose = [a.to_quaternion().slerp(b.to_quaternion(),settle).to_euler('XYZ')
                            for a,b in zip(pose,relaxed_rotations[key])]
                for bone, angles in zip(bones, pose):
                    bone.rotation_euler = angles
                if not samples[key][2] and 0 <= i < len(events) - 1:
                    previous, following = events[i:i + 2]
                    gap = following['start'] - previous['end']
                    if 0 < gap <= job.get('stroke_gap_limit', .75):
                        u = (t - previous['end']) / gap
                        raised = stroke_shape(u, job)
                        height = max(0., samples[key][0][2] - job['contact_height'])
                        height = max(0., height - (roots[key[0]].z - contact_root_heights.get(key, roots[key[0]].z)))
                        angle = math.asin(min(.8, height / max(.03, lengths[key] * .8)))
                        bones[0].rotation_euler.x = min(.4, pose[0].x + angle)
                        # In the raised phase, rotate the finger as a curved unit
                        # about the MCP joint. Blend back into contact IK before
                        # landing; do not let a fixed-XY tip target re-curl it.
                        influence *= 1 - raised
                if job.get('palm_lift_mode') == 'gentle' and not samples[key][2]:
                    following = events[i + 1] if i + 1 < len(events) else None
                    previous = events[i] if i >= 0 else None
                    short_gap = previous and following and following['start'] - previous['end'] <= job.get('stroke_gap_limit', .75)
                    if not short_gap:
                        # During withdrawal/return the palm carries the curled
                        # finger. Early IK chased a still-distant note and folded
                        # the finger sharply for one or two output frames.
                        duration = min(.18, following['start'] - following['prepare']) if following else 0.
                        influence = smooth((t - following['start'] + duration) / max(1e-6, duration)) if following else 0.
            rigs[key[0]].pose.bones[FINGERS[key[1]][-1]].constraints[0].influence = influence
            target.location = Vector(samples[key][0]) - pad_offsets[key]
        refinement_steps = solve_steps()
        pads = fit_skin(samples)
        pads = clear_airborne_skin(samples,pads,t)
        if avoidance:
            pads = avoidance.prepare(roots, samples, dt, pads, lambda values: fit_skin(values, range(3)))
            pads, corrections = avoidance.resolve(roots, samples, pads,
                lambda: hand_collisions(rigs, collider_templates, margin=.001),
                lambda values: fit_skin(values, range(3)), mesh_clearance, job['hand_scale'] / .27)
            collision_corrections += corrections
        # The earlier guide limit precedes collision avoidance. Enforce it again
        # on the final wrist so an accepted pose correction (or a discarded old
        # correction) cannot teleport the whole hand after that limit.
        bounded = False
        for side,arm in rigs.items():
            delta = roots[side]-previous_output_roots[side]
            lateral = Vector((delta.x,delta.y,0.))
            maximum = job.get('wrist_speed',.55)*max(0.,dt)
            if lateral.length > maximum + 1e-9:
                wanted = lateral*(maximum/lateral.length)
                shift = wanted-lateral
                arm.location += shift
                roots[side] += shift
                for key,(pos,weight,down) in list(samples.items()):
                    if key[0]==side and not down:
                        samples[key] = (Vector(pos)+shift,weight,down)
                        targets[key].location += shift
                motion_stats['wrist_speed_clamps'] += 1
                bounded = True
            if dt>0:
                actual = roots[side]-previous_output_roots[side]
                motion_stats['max_wrist_lateral_speed_mps'] = max(
                    motion_stats['max_wrist_lateral_speed_mps'],math.hypot(actual.x,actual.y)/dt)
        if bounded:
            pads = fit_skin(samples,range(4))
        pads = clear_airborne_skin(samples,pads,t)
        for _ in range(0 if avoidance else (3 if job.get('motion_plan_version') else 2)):
            collisions = hand_collisions(rigs, collider_templates)
            if not collisions:
                break
            shifts = {}
            for clearance, first, second, direction in collisions:
                free = [(key[:2], sign) for key, sign in ((first, 1), (second, -1))
                        if key[:2] in samples and not samples[key[:2]][2]
                        and (not job.get('motion_plan_version') or tracks[key[:2]])]
                if free:
                    for key, sign in free:
                        # A fingertip has more leverage than the proximal segment being separated.
                        delta = direction * sign * min(.012, (-clearance + .0005) * 3) / len(free)
                        delta.z = max(0., delta.z)
                        shifts[key] = shifts.get(key, Vector()) + delta
                # Two active contacts may demand overlapping fingers. Keep their wrist anchors stable.
            if not shifts:
                break
            for key, delta in shifts.items():
                pos, weight, _ = samples[key]
                samples[key] = (Vector(pos) + delta, weight, False)
                rigs[key[0]].pose.bones[FINGERS[key[1]][-1]].constraints[0].influence = max(.5, weight)
                targets[key].location += delta
            collision_corrections += 1
            pads = fit_skin(samples)
        if job.get('finger_motion') == 'whole_finger':
            for key, (_, _, down) in samples.items():
                if not down or key[1] == 'thumb':
                    continue
                pose = []
                for name in FINGERS[key[1]]:
                    bone = rigs[key[0]].pose.bones[name]
                    basis = bone.bone.convert_local_to_pose(bone.matrix, bone.bone.matrix_local,
                        parent_matrix=bone.parent.matrix, parent_matrix_local=bone.parent.bone.matrix_local, invert=True)
                    pose.append(basis.to_euler('XYZ'))
                contact_rotations[key] = pose
                contact_root_heights[key] = roots[key[0]].z
        if frame >= 1:
            for arm in rigs.values():
                arm.keyframe_insert('location', frame=frame)
                arm.keyframe_insert('rotation_euler', frame=frame)
        previous_down = {key for key,value in samples.items() if value[2]}
        for key, pad in pads.items():
            arm = rigs[key[0]]
            pad_offsets[key] = pad - arm.matrix_world @ arm.pose.bones[FINGERS[key[1]][-1]].tail
            if frame >= 1:
                targets[key].keyframe_insert('location', frame=frame)
        if frame < 1:
            continue
        for (side, name), keys in baked.items():
            bone = rigs[side].pose.bones[name]
            basis = bone.bone.convert_local_to_pose(bone.matrix, bone.bone.matrix_local,
                parent_matrix=bone.parent.matrix, parent_matrix_local=bone.parent.bone.matrix_local, invert=True)
            keys.append((frame, basis.to_quaternion()))
        for clearance, first, second, _ in hand_collisions(rigs, collider_templates):
            collision_errors.append({'time': t, 'first': first, 'second': second, 'depth_mm': -clearance * 1000})
        for key, (_, _, active) in samples.items():
            if active:
                error = (pads[key] - Vector(samples[key][0])).length
                max_error = max(max_error, error)
                contact_samples += 1
                if error > .001:
                    errors.append({'frame': frame, 'time': t, 'hand': key[0], 'finger': key[1], 'error_mm': error * 1000})
        if is_video_frame:
            for obj in scene.objects:
                if obj.type != 'MESH':
                    continue
                evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
                mesh = evaluated.to_mesh()
                if job.get('bake_bulk_geometry', True):
                    min_clearance = min(min_clearance, screen_mesh_clearance(mesh, evaluated.matrix_world, job['screen']))
                else:
                    for vertex in mesh.vertices:
                        point = evaluated.matrix_world @ vertex.co
                        if abs(point.x) <= job['screen'][0] / 2 and abs(point.y) <= job['screen'][1] / 2:
                            min_clearance = min(min_clearance, point.z)
                evaluated.to_mesh_clear()
                mesh_samples += 1
        if is_video_frame and not job.get('bake_only') and (not job['keyframes'] or video_frame in selected):
            scene.render.filepath = str(out / 'hands' / f'{video_frame:05d}.png')
            bpy.ops.render.render(write_still=True)
            render_shadow(scene, meshes, out / 'shadows' / f'{video_frame:05d}.png')
            if video_frame in selected:
                subprocess.run(composite_command(job, video_frame), check=True)
        if is_video_frame and video_frame > last_progress:
            last_progress = video_frame
            print(f'HANDCAM_PROGRESS {video_frame}', flush=True)
        if is_video_frame and video_frame % 30 == 0:
            print(f'HANDCAM {video_frame}/{job["frames"]}, max skin error {max_error * 1000:.3f} mm, '
                  f'min mesh clearance {min_clearance * 1000:.3f} mm, '
                  f'{len(collision_errors)} unresolved collision samples', flush=True)
    (out / 'diagnostics.json').write_text(json.dumps({'contact_measurement': 'evaluated distal skin patch',
                'contact_errors_over_1mm': errors,
                'max_error_mm': max_error * 1000, 'contact_samples': contact_samples,
                'min_screen_clearance_mm': min_clearance * 1000 if math.isfinite(min_clearance) else None,
                'mesh_samples': mesh_samples,
                'collision_errors': collision_errors,
                'collision_corrections': collision_corrections,
                'pose_avoidance': avoidance.stats if avoidance else None,
                'fitting_stats': fitting_stats,
                'motion_stats': motion_stats,
                'airborne_floor_corrections': airborne_floor_corrections,
                'static_geometry_cache': dict(meshes=len(static_geometry), initial_pad_error_mm=cache_error * 1000),
                'motion_index': motion_index is not None,
                'bulk_geometry': job.get('bake_bulk_geometry', True),
                'shared_events': shared,
                'idle_releases': releases,
                'max_collision_depth_mm': max((c['depth_mm'] for c in collision_errors), default=0.),
                'assignments': len(assignments)}, indent=2), encoding='utf-8')
    # Bake the solved joints so opening/rendering the saved scene cannot choose a different IK pose.
    for arm in rigs.values():
        for bone in arm.pose.bones:
            for constraint in list(bone.constraints):
                if constraint.type == 'IK':
                    bone.constraints.remove(constraint)
    bake_rotations(rigs, baked)
    restore_static_geometry(static_geometry)
    for obj in (*rigs.values(), *targets.values()):
        if obj.animation_data and obj.animation_data.action:
            for curve in obj.animation_data.action.fcurves:
                for key in curve.keyframe_points:
                    key.interpolation = 'LINEAR'
    scene.frame_set(1)
    print('HANDCAM_STATUS Saving animation', flush=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(out / 'handcam.blend'))
    print(f'HANDCAM DONE: {len(errors)} finger samples exceeded 1 mm', flush=True)


if __name__ == '__main__':
    main(json.loads(Path(sys.argv[sys.argv.index('--') + 1]).read_text(encoding='utf-8')))
