"""Make a render segment from the same continuous animation, without re-solving IK."""
from array import array
import hashlib
import json
import math
from pathlib import Path
import sys
import bpy

args = sys.argv[sys.argv.index('--')+1:]
source,out = Path(args[0]).resolve(),Path(args[1]).resolve()
start,duration = float(args[2]),float(args[3])
job = json.loads((source/'job.json').read_text())
offset,frames = round(start*job['fps']),round(duration*job['fps'])
assert out != source
out.mkdir(parents=True,exist_ok=True)
scene = bpy.context.scene


def landmarks(frame):
    scene.frame_set(frame)
    return {(arm.name,bone.name):arm.matrix_world@bone.tail
            for arm in scene.objects if arm.type=='ARMATURE' for bone in arm.pose.bones}


reference = {1:landmarks(offset+1),frames:landmarks(offset+frames)}
for action in bpy.data.actions:
    for curve in action.fcurves:
        values = array('f',[0.]*(2*len(curve.keyframe_points)))
        curve.keyframe_points.foreach_get('co',values)
        for i in range(0,len(values),2):
            values[i] -= offset
        curve.keyframe_points.foreach_set('co',values)
        curve.update()
maximum = 0.
for frame,expected in reference.items():
    actual = landmarks(frame)
    maximum = max(maximum,max((actual[key]-point).length for key,point in expected.items()))
assert maximum < 2e-6, ('Slice changed the shared animation',maximum)
scene.frame_start,scene.frame_end = 1,frames
scene.frame_set(1)
job.update(start=start,duration=duration,frames=frames,output=str(out),source_full_bake=str(source))
(out/'job.json').write_text(json.dumps(job,indent=2))
for name in ('rig.json','diagnostics.json'):
    (out/name).write_bytes((source/name).read_bytes())
provenance = json.loads((source/'provenance.json').read_text())
provenance.update(start=start,duration=duration,frames=frames,
                  source_blend_sha256=provenance['blend_sha256'],max_slice_bone_error_m=maximum,
                  diagnostic_scope='full_song',source_frame_start=offset+1,source_frame_end=offset+frames)
(out/'provenance.json').write_text(json.dumps(provenance,indent=2))
bpy.ops.wm.save_as_mainfile(filepath=str(out/'handcam.blend'))
print('SLICE_VERIFIED',json.dumps({'start':start,'frames':frames,'max_bone_error_m':maximum}),flush=True)
