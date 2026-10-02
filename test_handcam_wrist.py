"""Load the source hand model in Blender, then --python this_file; no video rendering."""
from pathlib import Path
import sys
import tempfile

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handcam_blender import main


with tempfile.TemporaryDirectory() as directory:
    # A Hold tail can jump from the bottom edge to the center in the last 4 ms.
    job = dict(start=0., duration=.15, frames=9, fps=60, width=720, height=480,
               output=directory, screen=[.28, .1575], view_width=.28/.82, camera_y=-.015,
               engine='workbench', hand_scale=.27, contact_height=.0005, lift_height=.025,
               strict_psap=True, bake_only=True, keyframes=False,
               contacts=[dict(pointer=i, start=-1., end=.101,
                   points=[[-1., x, 1.], [.046, x, 1.], [.05, x, .5]]) for i, x in enumerate((.2, .8))])
    main(job)
    for name in ('Hand_Left', 'Hand_Right'):
        arm = bpy.data.objects[name]
        positions = []
        for frame in range(1, job['frames'] + 1):
            bpy.context.scene.frame_set(frame)
            positions.append(arm.matrix_world @ arm.pose.bones['Bone.016'].head)
        jump = max((b - a).length for a, b in zip(positions, positions[1:]))
        travel = (positions[-1] - positions[0]).length
        print(f'{name}: maximum frame step {jump * 1000:.2f} mm, travel {travel * 1000:.2f} mm', flush=True)
        assert jump < .025, f'{name}: touch constraints bypassed wrist smoothing ({jump * 1000:.2f} mm)'
        assert travel > .035, f'{name}: wrist stopped following the target'
print('Wrist continuity check passed')
