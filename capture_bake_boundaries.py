"""Save same-time joint coordinates for parallel-bake joins; no image I/O."""
import json
from pathlib import Path
import sys
import bpy

job_path, duration = sys.argv[sys.argv.index('--')+1:]
path = Path(job_path)
job = json.loads(path.read_text())
frames = round(float(duration)*job['fps'])

def capture(frame):
    bpy.context.scene.frame_set(frame)
    return {'time':job['start']+(frame-1)/job['fps'], 'rigs':{
        arm.name:{'matrix':[list(row) for row in arm.matrix_world],
                  'bones':{bone.name:{'head':list(arm.matrix_world@bone.head),
                                      'tail':list(arm.matrix_world@bone.tail)} for bone in arm.pose.bones}}
        for arm in bpy.context.scene.objects if arm.type=='ARMATURE'}}

states = {'first':capture(1), 'last':capture(frames), 'join':capture(frames+1)}
path.with_name('seam-state.json').write_text(json.dumps(states,indent=2))
path.with_name('boundary-poses.json').write_text(json.dumps([states['first'],states['last']],indent=2))

