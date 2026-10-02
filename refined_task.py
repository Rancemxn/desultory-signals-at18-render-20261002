"""Refined export workflow with numerical pose review and a single shared bake."""
import hashlib
import json
import math
from pathlib import Path
import os
import subprocess
import sys
import time
import shutil

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


def render(part):
    import task
    start,duration = part*20.,min(20.,150.75-part*20)
    out = Path(f'output/part{part}')
    run([BLENDER,'--background','--disable-autoexec','output/full/handcam.blend','--python-exit-code',1,
         '--python','slice_bake.py','--','output/full',out,start,duration])
    run([BLENDER,'--background','--disable-autoexec',out/'handcam.blend','--python-exit-code',1,
         '--python','inspect_pose_numeric.py','--',out/'job.json',out/'pose-numeric.json'])
    numeric = json.loads((out/'pose-numeric.json').read_text())
    (out/'boundary-poses.json').write_text(json.dumps(numeric['boundary_poses'],indent=2))
    task.render(part)


def assemble():
    from handcam import unpack,prepare_resources
    from chart import load_chart
    from phigros_renderer import mix_audio
    from task import validate
    delivery = Path('delivery')
    delivery.mkdir(exist_ok=True)
    chartpath,music,_ = unpack(Path('inputs/desultory-signals.zip'),Path('inputs/chart'))
    chart = load_chart(chartpath.read_text(encoding='utf-8-sig'),source=chartpath)
    resources = prepare_resources(Path('inputs/resources.zip'),Path('inputs/resources'))
    hits = sorted((v.note.seconds+chart.offset,int(v.note.type))
                  for line in chart.lines for v in line.visual_notes if not v.is_fake)
    mixed = delivery/'audio.wav'
    audio = mix_audio(music,resources,hits,0.,150.75,.35,mixed)
    videos = []
    for part in range(8):
        source = next(Path('assembled').rglob(f'part{part}.mp4'))
        target = delivery/f'video-{part}.mp4'
        run(['ffmpeg','-v','error','-y','-i',source,'-map','0:v:0','-c:v','copy','-an',target])
        videos.append(target)
    listing = delivery/'concat.txt'
    listing.write_text(''.join(f"file '{p.name}'\n" for p in videos))
    target = delivery/'Desultory-Signals-AT18-1080p60-refined.mp4'
    run(['ffmpeg','-v','error','-y','-f','concat','-safe',0,'-i',listing,'-i',mixed,
         '-map','0:v:0','-map','1:a:0','-c:v','copy','-c:a','aac','-b:a','192k',
         '-t',150.75,'-movflags','+faststart',target])
    media = validate(target,9045)
    audio_stream = next(s for s in media['streams'] if s['codec_type']=='audio')
    assert abs(float(audio_stream['duration'])-150.75)<.03
    diagnostics = json.loads(Path('output/full/diagnostics.json').read_text())
    numeric = json.loads(Path('output/full/pose-numeric.json').read_text())
    bakes = [json.loads(next(Path('assembled').rglob(f'part{i}-provenance.json')).read_text()) for i in range(8)]
    assert len({b['source_blend_sha256'] for b in bakes})==1
    assert all(b['max_slice_bone_error_m']<2e-6 for b in bakes)
    boundaries = [json.loads(next(Path('assembled').rglob(f'part{i}-boundary-poses.json')).read_text()) for i in range(8)]
    seams = []
    for before,after in zip(boundaries,boundaries[1:]):
        a,b = before[-1],after[0]
        distances = [math.dist(bone['tail'],b['rigs'][name]['bones'][key]['tail'])*1000
                     for name,arm in a['rigs'].items() for key,bone in arm['bones'].items()]
        seams.append({'time':b['time'],'max_bone_tail_step_mm':max(distances),
                      'shared_continuous_animation':True})
    with target.open('rb') as stream:
        digest = hashlib.file_digest(stream,'sha256').hexdigest()
    summary = {'max_error_mm':diagnostics['max_error_mm'],
        'contact_errors_over_1mm':len(diagnostics['contact_errors_over_1mm']),
        'contact_samples':diagnostics['contact_samples'],'collision_samples':len(diagnostics['collision_errors']),
        'max_collision_depth_mm':diagnostics['max_collision_depth_mm'],
        'min_screen_clearance_mm':diagnostics['min_screen_clearance_mm'],
        'finger_order_samples':len(numeric['finger_order_violations']),
        'max_wrist_lateral_speed_mps':max((x['lateral_speed_mps'] for x in numeric['worst_wrist_steps']),default=0),
        'max_joint_step_degrees':max((x['step_degrees'] for x in numeric['worst_joint_steps']),default=0)}
    geometry = json.loads(Path('output/plan/refinement-validation.json').read_text())
    report = {'media':media,'sha256':digest,'bytes':target.stat().st_size,'full_decode':'passed',
        'audio_mix':audio,'baked_pose_diagnostics':summary,'bakes':bakes,'segment_boundaries':seams,
        'contact_refinement':geometry,'native_ap_validated':False,'image_inspection':False}
    (delivery/'validation.json').write_text(json.dumps(report,indent=2))
    shutil.copyfile('output/full/diagnostics.json',delivery/'pose-diagnostics.json')
    shutil.copyfile('output/full/pose-numeric.json',delivery/'pose-numeric.json')
    (delivery/'DELIVERY.md').write_text(
        '# Desultory Signals AT18 — refined export\n\n'
        '1920 × 1080, 60 FPS, 9045 frames, 150.750 seconds. Full decoding passed.\n\n'
        'Aligned block boundaries; central 50% Hold / 30% Drag planning bands; '
        'authored held-contact relays and connected Drag strokes. All render segments '
        'come from one shared continuous bake. No image inspection was performed.\n\n'
        f'Actual maximum skin contact error: {summary["max_error_mm"]:.3f} mm. '
        f'Contact samples over 1 mm: {summary["contact_errors_over_1mm"]}. '
        f'Collision samples: {summary["collision_samples"]}. '
        'See the JSON reports for remaining pose limitations; native Phigros AP is not validated.\n\n'
        f'SHA-256: `{digest}`\n',encoding='utf-8')
    for temporary in [*videos,listing,mixed]:
        temporary.unlink()
    print('REFINED_DELIVERY',json.dumps({'sha256':digest,'bytes':target.stat().st_size,'pose':summary}),flush=True)


if __name__=='__main__':
    if sys.argv[1]=='probe':
        index = int(sys.argv[2])
        bake(f'output/probe{index}',*PROBES[index])
    elif sys.argv[1]=='bake':
        bake('output/full',0.,150.75)
    elif sys.argv[1]=='render':
        render(int(sys.argv[2]))
    elif sys.argv[1]=='assemble':
        assemble()
    else:
        raise ValueError('Unknown refined workflow stage')
