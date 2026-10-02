"""Run with Blender --background --factory-startup --python-exit-code 1 --python this_file; no rendering."""
from pathlib import Path
import sys

import bpy
from mathutils import Quaternion

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handcam_blender import bake_rotations

data = bpy.data.armatures.new('Bake check')
arm = bpy.data.objects.new('Bake check', data)
bpy.context.collection.objects.link(arm)
bpy.context.view_layer.objects.active = arm
arm.select_set(True)
bpy.ops.object.mode_set(mode='EDIT')
bone = data.edit_bones.new('Finger')
bone.head, bone.tail = (0, 0, 0), (0, 1, 0)
bpy.ops.object.mode_set(mode='OBJECT')
arm.keyframe_insert('location', frame=1)
bone = arm.pose.bones['Finger']
bone.rotation_mode = 'QUATERNION'
bone.keyframe_insert('rotation_quaternion', frame=3)  # Replace a partially written curve.
keys = [(1., Quaternion()), (1.000000001, Quaternion((1, 0, 0), .2)), (4., Quaternion((1, 0, 0), .8))]
bake_rotations({1: arm}, {(1, 'Finger'): keys})
curves = [c for c in arm.animation_data.action.fcurves if 'rotation_quaternion' in c.data_path]
assert len(curves) == 4 and all(len(c.keyframe_points) == 2 for c in curves)
for curve in curves:
    assert abs(curve.evaluate(1) - keys[1][1][curve.array_index]) < 1e-6
    assert abs(curve.evaluate(4) - keys[2][1][curve.array_index]) < 1e-6
    assert all(p.interpolation == 'LINEAR' for p in curve.keyframe_points)
print('Bulk bake check passed')
