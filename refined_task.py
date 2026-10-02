"""Refined export workflow with numerical pose review and a single shared bake."""
import hashlib
import json
import math
from pathlib import Path
import os
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
BLENDER = os.environ.get('HANDCAM_BLENDER','/home/runner/blender-headless')
PROBES = [(70.,2.),(78.5,2.),(84.,5.),(110.,3.),(125.5,1.)]


def run(args):
    subprocess.run([str(a) for a in args],check=True)


def bake(out,start,duration):
    out = Path(out)
    out.mkdir(parents=True,exist_ok=True)
    started = time.monotonic()
    run([sys.executable,'-u','handcam.py','inputs/desultory-signals.zip',
         '--model','inputs/hands.blend','--resources','inputs/resources.zip','--blender',BLENDER,
         '--psap','output/plan/plan.psap','--motion-plan','output/plan/motion-plan.json',
         '--title','Desultory Signals','--level','AT 18','--width',1920,'--height',1080,'--fps',60,
         '--start',start,'--duration',duration,'--bake-only','--output',out])
    (out/'bake-timing.json').write_text(json.dumps({'seconds':time.monotonic()-started}))
    diagnostics = json.loads((out/'diagnostics.json').read_text())
    times = {start+i*.25 for i in range(math.ceil(duration/.25))}
    worst = sorted(diagnostics['contact_errors_over_1mm'],key=lambda x:-x['error_mm'])[:10]
    times.update(x['time'] for x in worst if start<=x['time']<start+duration)
    run([BLENDER,'--background','--disable-autoexec',out/'handcam.blend','--python-exit-code',1,
         '--python',ROOT/'inspect_pose_numeric.py','--',out/'job.json',out/'pose-numeric.json',
         '--mesh-times',*sorted(times)])
    report = json.loads((out/'pose-numeric.json').read_text())
    (out/'boundary-poses.json').write_text(json.dumps(report['boundary_poses'],indent=2))
    with (out/'handcam.blend').open('rb') as stream:
        digest = hashlib.file_digest(stream,'sha256').hexdigest()
    provenance = {'commit':os.environ.get('GITHUB_SHA'),'workflow_run':os.environ.get('GITHUB_RUN_ID'),
        'start':start,'duration':duration,'frames':round(duration*60),'fps':60,
        'plan_sha256':hashlib.sha256(Path('output/plan/motion-plan.json').read_bytes()).hexdigest(),
        'psap_sha256':hashlib.sha256(Path('output/plan/plan.psap').read_bytes()).hexdigest(),
        'blend_sha256':digest,'image_inspection':False}
    (out/'provenance.json').write_text(json.dumps(provenance,indent=2))
    print('REFINED_BAKE_SUMMARY',json.dumps({'start':start,'duration':duration,
        'max_contact_error_mm':diagnostics['max_error_mm'],
        'min_screen_clearance_mm':diagnostics['min_screen_clearance_mm'],
        'collision_samples':len(diagnostics['collision_errors']),
        'finger_order_samples':len(report['finger_order_violations']),
        'max_wrist_speed_mps':max((x['lateral_speed_mps'] for x in report['worst_wrist_steps']),default=0),
        'max_joint_step_degrees':max((x['step_degrees'] for x in report['worst_joint_steps']),default=0)}),flush=True)


if __name__=='__main__':
    if sys.argv[1]=='probe':
        index = int(sys.argv[2])
        bake(f'output/probe{index}',*PROBES[index])
    elif sys.argv[1]=='bake':
        bake('output/full',0.,150.75)
    else:
        raise ValueError('Unknown refined workflow stage')
