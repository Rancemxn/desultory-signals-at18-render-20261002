"""Blender integration: source rig -> planned thumb/index contacts -> saved pose checks."""
import json
from pathlib import Path
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handcam_blender import FINGERS, main, skin_pads, world_point
from handcam_motion import finish_motion


def check(out):
    out.mkdir(parents=True, exist_ok=True)
    main(dict(output=str(out), hand_scale=.27, enable_thumb=True, profile_only=True))
    profile = json.loads((out / 'hand-profile.json').read_text())
    contacts = []
    for index, finger in enumerate(('thumb', 'index', 'middle', 'ring', 'little')):
        for side in (-1, 1):
            offset = profile['fingers'][f'{side}:{finger}']['offset']
            xy = (side * .10 + offset[0], -.13 + offset[1])
            start = .3 + index * .5
            contacts.append(dict(hand='left' if side == -1 else 'right', finger=finger,
                pointer=len(contacts), start=start, end=start + .20,
                points=[[start, xy[0] / .28 + .5, .5 - xy[1] / .1575]],
                planned=True, kind='tap', note_ids=[len(contacts)]))
    rests = finish_motion(contacts, profile, [.28, .1575], .28 / .82)
    job = dict(start=0., duration=2.7, frames=81, fps=30, width=1280, height=720,
               output=str(out), screen=[.28, .1575], view_width=.28/.82, camera_y=-.015,
               engine='workbench', hand_scale=.27, contact_height=.0005, lift_height=.025,
               strict_psap=True, bake_only=True, keyframes=False, contacts=contacts,
               motion_plan_version=1, hand_rest=rests, enable_thumb=True, palm_motion='v4')
    (out / 'job.json').write_text(json.dumps(job, indent=2))
    main(job)
    diagnostics = json.loads((out / 'diagnostics.json').read_text())
    print('ALGO5 RIG DIAGNOSTICS', diagnostics['max_error_mm'], diagnostics['min_screen_clearance_mm'], flush=True)
    assert diagnostics['max_error_mm'] < 1., 'Planned contacts exceed 1 mm fitting tolerance'
    assert diagnostics['min_screen_clearance_mm'] >= -.5, 'Mesh penetrates screen'
    bpy.ops.wm.open_mainfile(filepath=str(out / 'handcam.blend'))
    rig = json.loads((out / 'rig.json').read_text())
    indices = {(int(k.split(':')[0]), k.split(':')[1]): v for k, v in rig['pad_vertices'].items()}
    arms = {side: bpy.data.objects[name] for side, name in ((-1, 'Hand_Left'), (1, 'Hand_Right'))}
    meshes = {side: next(obj for obj in bpy.data.objects if obj.type == 'MESH' and obj.parent == arm)
              for side, arm in arms.items()}
    for c in contacts:
        t = c['start'] + .1
        frame = t * job['fps'] + 1
        bpy.context.scene.frame_set(int(frame), subframe=frame % 1)
        bpy.context.view_layer.update()
        side = -1 if c['hand'] == 'left' else 1
        pad = skin_pads(meshes, indices)[side, c['finger']]
        expected = Vector((*world_point(job, c['points'][0][1:]), job['contact_height']))
        assert (pad - expected).length < .001, f'Saved {c["hand"]}/{c["finger"]} contact drift'
        for bone in FINGERS[c['finger']]:
            assert not arms[side].pose.bones[bone].constraints, 'Saved contact still depends on live IK'
    print('algo5 ten-finger fitting and saved-animation checks passed', flush=True)


if __name__ == '__main__':
    check(Path(sys.argv[sys.argv.index('--') + 1]).resolve())
