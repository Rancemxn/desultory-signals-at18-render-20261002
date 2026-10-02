"""Blender check: screen alignment and visible whole-hand depth under the render camera."""
import json
import math
from pathlib import Path
import statistics
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handcam_blender import FINGERS, camera_pixel, configure_camera, contact_position, sample_times, screen_rect, skin_pads


def check():
    scene = bpy.context.scene
    if scene.camera is None:
        camera = bpy.data.objects.new('Camera', bpy.data.cameras.new('Camera'))
        scene.collection.objects.link(camera)
        scene.camera = camera
    for width, height in ((1280, 720), (1920, 1080), (720, 480)):
        scene.render.resolution_x, scene.render.resolution_y = width, height
        job = dict(width=width, height=height, screen=[.28, .1575], view_width=.28 / .82, camera_y=-.015)
        x, y, sw, sh = screen_rect(job)
        points = [Vector(((u - .5) * .28, (.5 - v) * .1575, 0))
                  for u, v in ((0, 0), (1, 0), (0, 1), (1, 1), (.5, .5))]
        configure_camera(scene, job)
        bpy.context.view_layer.update()
        baseline = [camera_pixel(scene, p) for p in points]
        for projected, uv in zip(baseline, ((0, 0), (1, 0), (0, 1), (1, 1), (.5, .5))):
            assert math.dist(projected, (x + uv[0] * sw, y + uv[1] * sh)) < 2, 'Screen composite drift'
        raised = (Vector((.08, -.10, .051)), Vector((.08, -.10, .093)))
        assert math.dist(*(camera_pixel(scene, p) for p in raised)) < .001, 'Legacy camera changed'
        for camera_height, offset_y in ((.45, 0.), (.45, -.15), (.65, -.15)):
            configure_camera(scene, dict(job, camera_projection='perspective', camera_height=camera_height, camera_offset_y=offset_y))
            bpy.context.view_layer.update()
            for point, expected in zip(points, baseline):
                assert math.dist(camera_pixel(scene, point), expected) < .001, 'Perspective shifted screen plane'
            assert math.dist(*(camera_pixel(scene, p) for p in raised)) > 15, 'Whole-hand lift remains invisible'
            # A half-millimetre skin clearance must not visibly move the contact away from its note.
            for point, expected in zip(points, baseline):
                assert math.dist(camera_pixel(scene, point + Vector((0, 0, .0005))), expected) < 1.8
    print('Camera alignment, depth visibility and legacy projection checks passed', flush=True)


def check_saved(out):
    """Measure the rendered projection of baked contacts and real palm strokes, without images."""
    job = json.loads((out / 'job.json').read_text(encoding='utf-8'))
    motion = json.loads((out / 'motion.json').read_text(encoding='utf-8'))
    rig = json.loads((out / 'rig.json').read_text(encoding='utf-8'))
    fingering = json.loads((out / 'fingering.json').read_text(encoding='utf-8'))
    strokes = json.loads((out / 'stroke-validation.json').read_text(encoding='utf-8'))['strokes']
    job['contacts'] = motion['contacts']
    scene = bpy.context.scene
    assert scene.camera.data.type == 'PERSP', 'Saved camera was not updated'
    configure_camera(scene, job)
    arms = {side: bpy.data.objects[name] for side, name in ((-1, 'Hand_Left'), (1, 'Hand_Right'))}
    meshes = {side: next(o for o in scene.objects if o.type == 'MESH' and o.parent == arm)
              for side, arm in arms.items()}
    indices = {(int(k.split(':')[0]), k.split(':')[1]): v for k, v in rig['pad_vertices'].items()}
    assigned = {(c['pointer'], c['start'], c['end']): (-1 if c['hand'] == 'left' else 1, c['finger'])
                for c in fingering}
    x, y, sw, sh = screen_rect(job)
    errors = []

    def set_time(t):
        frame = (t - job['start']) * job['fps'] + 1
        scene.frame_set(math.floor(frame), subframe=frame % 1)
        bpy.context.view_layer.update()

    for t in sample_times(job):
        if t < job['start']:
            continue
        active = [c for c in job['contacts'] if c['start'] <= t < c['end']]
        if not active:
            continue
        set_time(t)
        pads = skin_pads(meshes, indices)
        for c in active:
            point = pads[assigned[c['pointer'], c['start'], c['end']]]
            u, v = contact_position(c, t)
            error = math.dist(camera_pixel(scene, point), (x + u * sw, y + v * sh))
            assert error < 2., f'Contact projection missed screen target at {t}: {error:.3f} px'
            errors.append(error)
    assert errors, 'No baked contacts checked'

    rows = []
    for stroke in strokes:
        side = -1 if stroke['hand'] == 'left' else 1
        arm = arms[side]
        positions = []
        for phase in ('pressed', 'lifted'):
            set_time(stroke[phase]['time'])
            # The middle-finger base is a palm landmark, independent of finger curl;
            # the wrist can lie below the video, so it is not useful as a visibility probe.
            positions.append(arm.matrix_world @ arm.pose.bones[FINGERS['middle'][0]].head)
        pressed, lifted = positions
        # Hold planar motion fixed to isolate the visible contribution of the actual vertical stroke.
        before = Vector((lifted.x, lifted.y, pressed.z))
        pixels = math.dist(camera_pixel(scene, before), camera_pixel(scene, lifted))
        magnification = (scene.camera.location.z - pressed.z) / (scene.camera.location.z - lifted.z)
        projected = [camera_pixel(scene, p) for p in positions]
        assert all(0 <= px < job['width'] and 0 <= py < job['height'] for px, py in projected), (
            f'Palm landmark outside the frame: {projected}')
        minimum_pixels = 5. if job.get('palm_lift_mode') == 'gentle' else 10.
        minimum_scale = 1.015 if job.get('palm_lift_mode') == 'gentle' else 1.05
        assert pixels > minimum_pixels, f'Baked palm lift remains too small in the frame: {pixels:.3f} px'
        assert magnification > minimum_scale, f'Baked palm depth has little scale change: {magnification:.4f}'
        # Also check actual output frames: a subframe peak alone does not establish video visibility.
        start = stroke['lifted']['time'] - stroke['gap'] / 2
        stop = stroke['lifted']['time'] + stroke['gap'] / 2
        first = max(1, math.ceil((start - job['start']) * job['fps'] + 1))
        last = min(job['frames'] + 1, math.ceil((stop - job['start']) * job['fps'] + 1))
        rendered = []
        for frame in range(first, last):
            set_time(job['start'] + (frame - 1) / job['fps'])
            rendered.append((frame, arm.matrix_world @ arm.pose.bones[FINGERS['middle'][0]].head))
        assert rendered, 'No output frame samples the raised phase'
        frame, peak = max(rendered, key=lambda row: row[1].z)
        flat = Vector((peak.x, peak.y, pressed.z))
        peak_pixel = camera_pixel(scene, peak)
        rendered_pixels = math.dist(camera_pixel(scene, flat), peak_pixel)
        rendered_scale = (scene.camera.location.z - pressed.z) / (scene.camera.location.z - peak.z)
        assert 0 <= peak_pixel[0] < job['width'] and 0 <= peak_pixel[1] < job['height'], 'Rendered palm peak outside frame'
        assert rendered_pixels > minimum_pixels, f'Output frames missed the palm lift: {rendered_pixels:.3f} px'
        rows.append(dict(hand=stroke['hand'], time=stroke['lifted']['time'], gap=stroke['gap'],
                         palm_landmark_pixels=projected, depth_only_displacement_px=pixels,
                         palm_scale_increase_percent=(magnification - 1) * 100,
                         rendered_peak_frame=frame, rendered_peak_pixel=peak_pixel,
                         rendered_depth_only_displacement_px=rendered_pixels,
                         rendered_palm_scale_increase_percent=(rendered_scale - 1) * 100))
    summary = dict(contact_samples=len(errors), max_contact_projection_error_px=max(errors), strokes=len(rows),
                   palm_landmark='middle MCP (finger base)', all_palm_landmarks_in_frame=True)
    for name, selected in (('ordinary', [r for r in rows if r['gap'] >= .1]),
                           ('dense', [r for r in rows if r['gap'] < .1])):
        if selected:
            summary[name] = dict(strokes=len(selected),
                median_depth_only_displacement_px=statistics.median(r['depth_only_displacement_px'] for r in selected),
                median_palm_scale_increase_percent=statistics.median(r['palm_scale_increase_percent'] for r in selected),
                median_rendered_depth_only_displacement_px=statistics.median(r['rendered_depth_only_displacement_px'] for r in selected),
                median_rendered_palm_scale_increase_percent=statistics.median(r['rendered_palm_scale_increase_percent'] for r in selected))
    (out / 'camera-validation.json').write_text(json.dumps(dict(summary=summary, strokes=rows), indent=2), encoding='utf-8')
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    if '--' in sys.argv:
        check_saved(Path(sys.argv[sys.argv.index('--') + 1]).resolve())
    else:
        check()
