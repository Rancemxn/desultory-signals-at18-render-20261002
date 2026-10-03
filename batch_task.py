"""One selected chart per run: general rules, eight bakes, 1080p60 delivery."""
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import zipfile

from task import run, validate

BLENDER = os.environ.get('HANDCAM_BLENDER','/home/runner/blender-headless')
CHARTS = {
    'AboutTheUniverse': 'IN',
    'TrueHomeTrueWorldRework': 'IN',
    'Implexrough': 'IN',
    'EntrancetotheChaos': 'IN',
    'ExoplanetaryMirage': 'AT',
    'Hate': 'AT',
    'OblivionPHIN': 'IN',
    'EntrancetotheChaos-IN-index2': 'IN',
    'ExoplanetaryMirage-IN-index2': 'IN',
    'DesultorySignals-AT-load': 'AT',
}


def configuration(key):
    special = key=='DesultorySignals-AT-load'
    load_only = special or key=='ExoplanetaryMirage-IN-index2'
    return dict(source_key='desultory-signals' if special else key.removesuffix('-IN-index2'),
                title='Desultory Signals' if special else 'ハテ' if key=='Hate' else key.removesuffix('-IN-index2'),
                input_release='render-inputs' if special else 'batch-inputs-v1',
                fingering_objective='load' if load_only else 'balanced',speed_limits=not load_only,
                allowed_fingers=['index'] if key.endswith('-IN-index2') else ['index','middle','ring','thumb','little'])


def prepare(key):
    level = CHARTS[key]
    config = configuration(key)
    source_key = config['source_key']
    source = Path('inputs')/(source_key+'.zip')
    run(['gh','release','download',config['input_release'],'--pattern',source.name,'--dir','inputs','--clobber'])
    with zipfile.ZipFile(source) as z, zipfile.ZipFile('inputs/selected.zip','w',zipfile.ZIP_DEFLATED) as target:
        chart_name = f'chart_{level}.json' if f'chart_{level}.json' in z.namelist() else 'chart.json'
        names = [(chart_name,'chart.json'),('music.wav','music.wav')]
        picture = next((n for n in ('illustration.jpg','illustration.png') if n in z.namelist()),None)
        if picture is None:
            raise ValueError('Input archive has no illustration')
        names.append((picture,picture))
        names.extend((n,n) for n in ('info.yml','info.txt') if n in z.namelist())
        for name, renamed in names:
            target.writestr(renamed,z.read(name))
    Path('output').mkdir(exist_ok=True)
    with zipfile.ZipFile('inputs/selected.zip') as z:
        Path('inputs/music.wav').write_bytes(z.read('music.wav'))
    media = json.loads(subprocess.check_output(['ffprobe','-v','error','-show_format','-of','json','inputs/music.wav'],text=True))
    frames = math.ceil(float(media['format']['duration'])*60)
    meta = dict(key=key,**config,level=level,frames=frames,duration=frames/60,
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),parts=list(range(8)))
    if key=='DesultorySignals-AT-load':
        meta['level']='AT 18'
    Path('output/batch.json').write_text(json.dumps(meta,indent=2))
    output = os.environ.get('GITHUB_OUTPUT')
    if output:
        with open(output,'a') as f:
            f.write('parts='+json.dumps(meta['parts'])+'\n')
    return meta


def metadata():
    return json.loads(Path('output/batch.json').read_text())


def common(meta):
    return [sys.executable,'-u','handcam.py','inputs/selected.zip','--model','inputs/hands.blend',
        '--resources','inputs/resources.zip','--blender',BLENDER,'--algorithm',5,
        '--title',meta['title'],'--level',meta['level'],'--width',1920,'--height',1080,'--fps',60,
        '--fingers',*meta.get('allowed_fingers',['index','middle','ring','thumb','little']),
        '--fingering-objective',meta.get('fingering_objective','balanced'),
        *([] if meta.get('speed_limits',True) else ['--no-speed-limits'])]


def planning_stage(name, args):
    started = time.monotonic()
    print('PLAN_STAGE', name, 'started', flush=True)
    process = subprocess.Popen(list(map(str,args)))
    try:
        while True:
            try:
                code = process.wait(timeout=30)
                if code:
                    raise subprocess.CalledProcessError(code,args)
                break
            except subprocess.TimeoutExpired:
                log = Path('output/base/solve.log')
                last = log.read_text(encoding='utf-8').splitlines()[-1:] if log.exists() else []
                print('PLAN_HEARTBEAT',name,'elapsed_seconds',round(time.monotonic()-started),*last,flush=True)
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    print('PLAN_STAGE',name,'completed_seconds',round(time.monotonic()-started,1),flush=True)


def plan(key):
    meta = prepare(key)
    if key=='DesultorySignals-AT-load':
        run(['gh','release','download','refined-plan-v21','--pattern','motion-plan.json',
             '--dir','output/base','--clobber'])
        source = Path('output/base/motion-plan.json')
        original = source.read_bytes()
        baseline = json.loads(original)
        baseline['settings'].update(fingering_objective='load',speed_limits=False)
        baseline['assignment_source'] = dict(release='refined-plan-v21',sha256=hashlib.sha256(original).hexdigest(),
                                            preserved_contact_paths=True)
        Path('output/rules').mkdir(parents=True,exist_ok=True)
        Path('output/rules/merged.json').write_text(json.dumps(baseline,indent=2))
        Path('output/rules/general-rules.json').write_text(json.dumps(baseline['assignment_source'],indent=2))
    else:
        planning_stage('initial-fingering',common(meta)+['--full','--plan-only','--output','output/base'])
        planning_stage('contact-paths',[sys.executable,'-u','general_refinement.py','output/base/motion-plan.json','inputs/selected.zip','output/rules/motion-plan.json'])
        planning_stage('shared-drags',[sys.executable,'-u','merge_shared_drags.py','output/rules/motion-plan.json','inputs/selected.zip','output/rules/merged.json'])
    planning_stage('assignment',[sys.executable,'-u','refine_assignments.py','output/rules/merged.json','output/assigned','--beam',96])
    if key=='DesultorySignals-AT-load':
        shutil.copyfile('output/assigned/assigned-motion-plan.json','output/rules/spaced.json')
    else:
        planning_stage('contact-spacing',[sys.executable,'-u','spread_contacts.py','output/assigned/assigned-motion-plan.json','inputs/selected.zip','output/rules/spaced.json'])
    if key=='DesultorySignals-AT-load':
        Path('output/assigned-final').mkdir(parents=True,exist_ok=True)
        for name in ('assigned-motion-plan.json','assignment-refinement.json'):
            shutil.copyfile(Path('output/assigned')/name,Path('output/assigned-final')/name)
    else:
        planning_stage('final-assignment',[sys.executable,'-u','refine_assignments.py','output/rules/spaced.json','output/assigned-final','--beam',96])
    planning_stage('validation',[sys.executable,'-u','finalize_refinement.py','output/assigned-final/assigned-motion-plan.json','inputs/selected.zip','output/plan'])


def resume_plan(key):
    previous = metadata()
    meta = prepare(key)
    for field in ('key','level','source_sha256','fingering_objective','speed_limits'):
        if field not in previous and field in ('fingering_objective','speed_limits'):
            previous[field] = 'balanced' if field=='fingering_objective' else True
        if previous[field] != meta[field]:
            raise ValueError('Refinement artifact does not match requested chart: '+field)
    source = 'output/assigned-final/assigned-motion-plan.json'
    plan = json.loads(Path(source).read_text())
    if set(plan['settings']['fingers']) != set(meta['allowed_fingers']):
        raise ValueError('Refinement artifact uses a different finger configuration')
    planning_stage('validation',[sys.executable,'-u','finalize_refinement.py',source,'inputs/selected.zip','output/plan'])


def interval(meta, part):
    if not 0 <= part < 8 or meta['frames'] < 8:
        raise ValueError('Eight nonempty frame intervals required')
    first, last = meta['frames']*part//8, meta['frames']*(part+1)//8
    return first/60, last-first


def bake(part):
    meta = metadata()
    start, frames = interval(meta,part)
    duration = frames/60
    full = Path(f'output/part{part}')
    run(common(meta)+['--psap','output/plan/plan.psap','--motion-plan','output/plan/motion-plan.json',
        '--start',start,'--duration',(frames+(part<7))/60-1e-9,'--consistent-history',
        '--bake-only','--output',full])
    run([BLENDER,'--background','--disable-autoexec',full/'handcam.blend','--python-exit-code',1,
         '--python','capture_bake_boundaries.py','--',full/'job.json',duration])
    run([BLENDER,'--background','--disable-autoexec',full/'handcam.blend','--python-exit-code',1,
         '--python','inspect_pose_numeric.py','--',full/'job.json',full/'pose-numeric.json'])
    job = json.loads((full/'job.json').read_text())
    job.update(duration=duration,frames=frames)
    (full/'job.json').write_text(json.dumps(job,indent=2))
    # Exclude the retained comparison frame from aggregate contact statistics.
    from handcam_blender import sample_times
    import bisect
    times = [t for t in sample_times(job) if start<=t<start+duration]
    diagnostics = json.loads((full/'diagnostics.json').read_text())
    diagnostics['contact_samples'] = sum(max(0,bisect.bisect_left(times,c['end'])-
        bisect.bisect_left(times,c['start'])) for c in job['contacts'])
    for key in ('contact_errors_over_1mm','collision_errors'):
        diagnostics[key] = [v for v in diagnostics[key] if start<=v['time']<start+duration]
    (full/'diagnostics.json').write_text(json.dumps(diagnostics,indent=2))
    numeric = json.loads((full/'pose-numeric.json').read_text())
    for key in ('worst_wrist_steps','worst_joint_steps','finger_order_violations','projected_contacts','mesh_intersections'):
        numeric[key] = [v for v in numeric[key] if start<=v['time']<start+duration]
    numeric['frames'] = frames
    (full/'pose-numeric.json').write_text(json.dumps(numeric,indent=2))
    with (full/'handcam.blend').open('rb') as f:
        digest = hashlib.file_digest(f,'sha256').hexdigest()
    provenance = dict(commit=os.environ.get('GITHUB_SHA'),workflow_run=os.environ.get('GITHUB_RUN_ID'),
        blend_sha256=digest,plan_sha256=hashlib.sha256(Path('output/plan/motion-plan.json').read_bytes()).hexdigest(),
        psap_sha256=hashlib.sha256(Path('output/plan/plan.psap').read_bytes()).hexdigest(),**meta)
    provenance.update(part=part,start=start,duration=duration,frames=frames,warmup_seconds=start+1.,
                      initialization_time=-1.,full_contact_context=True,
                      parallel_bake=True,retained_join_frame=part<7)
    (full/'provenance.json').write_text(json.dumps(provenance,indent=2))


def render(part):
    from handcam import unpack,prepare_resources,render_screen,render_saved
    from chart import load_chart
    from phigros_renderer import background_image,mix_audio
    meta = metadata()
    start, frames = interval(meta,part)
    duration = frames/60
    out = Path(f'output/part{part}').resolve()
    job = json.loads((out/'job.json').read_text())
    assert job['start']==start and job['frames']==frames
    chartpath,music,picture = unpack(Path('inputs/selected.zip'),out/'input')
    chart = load_chart(chartpath.read_text(encoding='utf-8-sig'),source=chartpath)
    job.update(resources=str(prepare_resources(Path('inputs/resources.zip'),out/'resources')),
        background=str(out/'background.png'),screen_video=str(out/'screen.mp4'),mixed_audio=str(out/'audio.wav'))
    background_image(picture,1920,1080,.2,1080*.045).save(job['background'])
    renderer = render_screen(chart,picture,out/'screen',job)
    mix_audio(music,job['resources'],renderer.hits,start,duration,.35,Path(job['mixed_audio']))
    (out/'job.json').write_text(json.dumps(job,indent=2))
    render_saved(out,BLENDER)
    delivery = Path('delivery'); delivery.mkdir(exist_ok=True)
    shutil.copyfile(out/'handcam.mp4',delivery/f'part{part}.mp4')
    shutil.copyfile(out/'provenance.json',delivery/f'part{part}-provenance.json')
    (delivery/f'part{part}-validation.json').write_text(json.dumps(validate(delivery/f'part{part}.mp4',frames),indent=2))


def assemble():
    from handcam import unpack,prepare_resources
    from chart import load_chart
    from phigros_renderer import mix_audio
    meta = metadata()
    delivery = Path('delivery'); delivery.mkdir(exist_ok=True)
    chartpath,music,_ = unpack(Path('inputs/selected.zip'),Path('inputs/chart'))
    chart = load_chart(chartpath.read_text(encoding='utf-8-sig'),source=chartpath)
    resources = prepare_resources(Path('inputs/resources.zip'),Path('inputs/resources'))
    hits = sorted((v.note.seconds+chart.offset,int(v.note.type)) for line in chart.lines for v in line.visual_notes if not v.is_fake)
    audio = mix_audio(music,resources,hits,0.,meta['duration'],.35,delivery/'audio.wav')
    videos,provenance = [],[]
    for part in meta['parts']:
        source = Path('assembled')/f'part{part}.mp4'
        target = delivery/f'video-{part}.mp4'
        run(['ffmpeg','-v','error','-y','-i',source,'-map','0:v:0','-c:v','copy','-an',target])
        videos.append(target)
        provenance.append(json.loads((Path('assembled')/f'part{part}-provenance.json').read_text()))
    assert len({p['plan_sha256'] for p in provenance})==1
    seams = json.loads(Path('output/full/seam-validation.json').read_text())
    assert seams['passed']
    assert [p['blend_sha256'] for p in provenance]==[p['blend_sha256'] for p in seams['bakes']]
    listing = delivery/'concat.txt'
    listing.write_text(''.join(f"file '{p.name}'\n" for p in videos))
    name = meta['key'] if meta['key'].endswith(('-IN-index2','-AT-load')) else f"{meta['key']}-{meta['level']}"
    target = delivery/f"{name}-1080p60.mp4"
    run(['ffmpeg','-v','error','-y','-f','concat','-safe',0,'-i',listing,'-i',delivery/'audio.wav',
        '-map','0:v:0','-map','1:a:0','-c:v','copy','-c:a','aac','-b:a','192k','-t',meta['duration'],'-movflags','+faststart',target])
    media = validate(target,meta['frames'])
    assert abs(float(next(s for s in media['streams'] if s['codec_type']=='audio')['duration'])-meta['duration'])<.04
    with target.open('rb') as f:
        digest = hashlib.file_digest(f,'sha256').hexdigest()
    report = dict(chart=meta,media=media,sha256=digest,full_decode='passed',audio_mix=audio,
        parallel_bakes=8,seam_validation=seams,provenance=provenance,native_ap_validated=False,
        contact_validation=json.loads(Path('output/plan/refinement-validation.json').read_text()),
        pose_diagnostics=json.loads(Path('output/full/diagnostics.json').read_text()))
    plan = json.loads(Path('output/plan/motion-plan.json').read_text())
    used = sorted({(c['hand'],c['finger']) for c in plan['contacts']})
    assert all(f in meta.get('allowed_fingers',plan['settings']['fingers']) for _,f in used)
    report['used_fingers'] = used
    (delivery/'validation.json').write_text(json.dumps(report,indent=2))
    for source in ('output/plan/motion-plan.json','output/plan/plan.psap','output/rules/general-rules.json'):
        shutil.copyfile(source,delivery/Path(source).name)
    for path in [*videos,listing,delivery/'audio.wav']:
        path.unlink()
    print('BATCH_DELIVERY',target,digest,flush=True)


if __name__=='__main__':
    stage = sys.argv[1]
    if stage=='plan': plan(sys.argv[2])
    elif stage=='resume-plan': resume_plan(sys.argv[2])
    elif stage=='bake': bake(int(sys.argv[2]))
    elif stage=='render': render(int(sys.argv[2]))
    elif stage=='assemble': assemble()
    else: raise ValueError(stage)
