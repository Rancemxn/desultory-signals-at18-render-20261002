"""Inspect a saved whole-finger animation in Blender; does not render images."""
import json
import math
from pathlib import Path
import statistics
import sys

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handcam_blender import FINGERS, skin_pads


def check(out):
    job = json.loads((out / 'job.json').read_text(encoding='utf-8'))
    assert job.get('finger_motion') == 'whole_finger'
    rig = json.loads((out / 'rig.json').read_text(encoding='utf-8'))
    indices = {(int(k.split(':')[0]), k.split(':')[1]): v for k, v in rig['pad_vertices'].items()}
    arms = {side: bpy.data.objects[name] for side, name in ((-1, 'Hand_Left'), (1, 'Hand_Right'))}
    meshes = {side: next(o for o in bpy.data.objects if o.type == 'MESH' and o.parent == arm)
              for side, arm in arms.items()}

    def sample(key, t):
        frame = (t - job['start']) * job['fps'] + 1
        bpy.context.scene.frame_set(math.floor(frame), subframe=frame % 1)
        bpy.context.view_layer.update()
        bones = [arms[key[0]].pose.bones[name] for name in FINGERS[key[1]]]
        wrist = arms[key[0]].matrix_world @ arms[key[0]].pose.bones['Bone.016'].head
        return dict(time=t, skin_height_mm=skin_pads(meshes, indices)[key].z * 1000,
                    wrist_height_mm=wrist.z * 1000,
                    angles_degrees=[math.degrees(b.rotation_quaternion.to_euler('XYZ').x) for b in bones])

    rows = []
    for side in arms:
        for finger in ('index', 'middle'):
            events = [c for c in job['contacts'] if c['hand'] == ('left' if side == -1 else 'right') and c['finger'] == finger]
            for previous, following in zip(events, events[1:]):
                gap = following['start'] - previous['end']
                minimum_gap = .035 if job.get('palm_lift_mode') in ('accent', 'gentle') else .1
                if (previous['kind'] != 'tap' or following['kind'] != 'tap' or not minimum_gap <= gap <= .75
                        or previous['start'] < job['start'] or following['start'] >= job['start'] + job['duration']):
                    continue
                key = side, finger
                pressed = sample(key, previous['end'] - .001)
                lifted = sample(key, previous['end'] + gap / 2)
                change = [b - a for a, b in zip(pressed['angles_degrees'], lifted['angles_degrees'])]
                assert lifted['skin_height_mm'] > 10., f'Finger still hugs screen at {lifted["time"]}'
                assert change[0] > (2. if job.get('palm_lift_ratio') is not None else 8.), f'MCP did not lift the finger at {lifted["time"]}: {change}'
                assert max(abs(v) for v in change[1:]) < 2., f'Distal curl changed during raised phase: {change}'
                palm_rise = lifted['wrist_height_mm'] - pressed['wrist_height_mm']
                finger_rise = lifted['skin_height_mm'] - pressed['skin_height_mm']
                hand_free = not any(c['hand'] == previous['hand'] and c['start'] <= lifted['time'] < c['end'] for c in job['contacts'])
                if job.get('palm_lift_ratio') is not None and hand_free:
                    assert .3 * finger_rise < palm_rise < .9 * finger_rise, f'Whole-hand lift has wrong proportion: {palm_rise}/{finger_rise}'
                rows.append(dict(hand=previous['hand'], finger=finger, gap=gap, pressed=pressed, lifted=lifted,
                                 palm_rise_mm=palm_rise, finger_rise_mm=finger_rise,
                                 joint_changes_degrees=change))
    assert rows, 'No ordinary repeated taps in this clip'
    summary = dict(strokes=len(rows), median_peak_skin_height_mm=statistics.median(r['lifted']['skin_height_mm'] for r in rows),
                   median_palm_rise_mm=statistics.median(r['palm_rise_mm'] for r in rows),
                   mcp_swing_degrees=[min(r['joint_changes_degrees'][0] for r in rows), max(r['joint_changes_degrees'][0] for r in rows)],
                   max_distal_change_degrees=max(abs(v) for r in rows for v in r['joint_changes_degrees'][1:]))
    for name, selected in (('ordinary', [r for r in rows if r['gap'] >= .1]),
                           ('dense', [r for r in rows if r['gap'] < .1])):
        if selected:
            summary[name] = dict(strokes=len(selected), median_palm_rise_mm=statistics.median(r['palm_rise_mm'] for r in selected))
    if job.get('palm_lift_mode') == 'accent' and 'dense' in summary and 'ordinary' in summary:
        assert summary['dense']['median_palm_rise_mm'] > summary['ordinary']['median_palm_rise_mm'], 'Dense hand motion was flattened'
    (out / 'stroke-validation.json').write_text(json.dumps(dict(summary=summary, strokes=rows), indent=2), encoding='utf-8')
    print(summary, flush=True)


if __name__ == '__main__':
    check(Path(sys.argv[sys.argv.index('--') + 1]).resolve())
