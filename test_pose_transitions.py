"""Blender regression checks for airborne FK/IK transitions and wrist continuity."""
from pathlib import Path
import sys
import math
import bpy
from mathutils import Vector

sys.path.insert(0,str(Path(__file__).resolve().parent))
from handcam_avoidance import PoseAvoidance

bpy.ops.wm.read_factory_settings(use_empty=True)
data = bpy.data.armatures.new('transition-test')
arm = bpy.data.objects.new('transition-test',data)
bpy.context.collection.objects.link(arm)
bpy.context.view_layer.objects.active = arm
arm.select_set(True)
bpy.ops.object.mode_set(mode='EDIT')
parent = None
for i in range(4):
    bone = data.edit_bones.new(str(i))
    bone.head,bone.tail = (0,i*.03,0),(0,(i+1)*.03,0)
    if parent:
        bone.parent, bone.use_connect = parent,True
    parent = bone
bpy.ops.object.mode_set(mode='OBJECT')
target = bpy.data.objects.new('distant-inactive-target',None)
bpy.context.collection.objects.link(target)
target.location = (1,1,1)
for i,bone in enumerate(arm.pose.bones):
    bone.rotation_mode = 'XYZ'
    bone.rotation_euler.x = -.3 if i else 0.
    bone.ik_stretch = 0.
constraint = arm.pose.bones['3'].constraints.new('IK')
constraint.target,constraint.chain_count,constraint.use_stretch = target,3,False
constraint.influence = 0.
bpy.context.view_layer.update()
key = (1,'index')
avoid = PoseAvoidance({1:arm},{key:target},{'index':('1','2','3')},
                      motion_limits={'wrist_speed':.18,'max_wrist_shift':.025})
samples = {key:(target.location.copy(),0.,False)}
before = arm.pose.bones['3'].tail.copy()
avoid.move_finger(key,Vector((.001,0,0)),samples)
bpy.context.view_layer.update()
after = arm.pose.bones['3'].tail.copy()
assert (after-before).length < .003, ('Airborne IK jumped toward an inactive target',before,after)
assert (after-(before+Vector((.001,0,0)))).length < .001

before = [bone.matrix.copy() for bone in arm.pose.bones]
avoid.curl_finger(key,Vector())
bpy.context.view_layer.update()
for matrix,bone in zip(before,arm.pose.bones):
    assert (matrix.translation-bone.matrix.translation).length < 1e-5
    assert abs(matrix.to_quaternion().dot(bone.matrix.to_quaternion())) > .99999

roots = {1:Vector()}
channels = {b.name:(tuple(b.location),tuple(b.scale)) for b in arm.pose.bones}
for i in range(1000):
    state = avoid.snapshot(roots,samples)
    avoid.previous_contacts = {key:(9.,9.,9.)}
    avoid.restore(state,roots,samples)
    assert avoid.previous_contacts == state['previous_contacts']
    for b in arm.pose.bones:
        assert (tuple(b.location),tuple(b.scale)) == channels[b.name]

avoid.step_dt = 1/60
shift = avoid.move_wrist(1,Vector((.05,0,0)),Vector(),roots,samples,project=True)
assert shift.length <= .18/60+1e-7, shift
print('Airborne FK/IK transitions, 1000 exact restorations and final correction speed checks passed')
