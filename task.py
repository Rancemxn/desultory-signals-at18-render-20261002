"""Isolated rendering driver. All large intermediates stay on Actions runners."""
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time
import hashlib
import os

ROOT = Path(__file__).resolve().parent
BLENDER = '/home/runner/blender-headless'


def run(args):
    subprocess.run(list(map(str, args)), check=True)


def common():
    return [sys.executable, '-u', 'handcam.py', 'inputs/desultory-signals.zip',
            '--model', 'inputs/hands.blend', '--resources', 'inputs/resources.zip',
            '--blender', BLENDER, '--algorithm', '5', '--title', 'Desultory Signals',
            '--level', 'AT 18', '--width', '1920', '--height', '1080', '--fps', '60',
            '--background', 'inputs/illustration.jpg']


def illustration():
    import zipfile
    with zipfile.ZipFile('inputs/desultory-signals.zip') as archive:
        Path('inputs/illustration.jpg').write_bytes(archive.read('illustration.jpg'))


def plan():
    illustration()
    run(common() + ['--full', '--plan-only', '--output', 'output/plan'])


def bake(part, start_override=None, duration_override=None):
    illustration()
    out = Path(f'output/part{part}')
    start = part * 20 if start_override is None else start_override
    duration = min(20, 150.75 - start) if duration_override is None else duration_override
    started = time.monotonic()
    run(common() + ['--psap', 'output/plan/plan.psap', '--motion-plan', 'output/plan/motion-plan.json',
                    '--bake-only', '--start', str(start), '--duration', str(duration), '--output', out])
    (out / 'bake-timing.json').write_text(json.dumps({'seconds': time.monotonic() - started}))
    # Inspect eight evenly spaced poses plus the largest solved contact/collision errors.
    job = json.loads((out / 'job.json').read_text())
    diagnostics = json.loads((out / 'diagnostics.json').read_text())
    times = [start + .5 + i * (duration - 1) / 7 for i in range(8)]
    worst = sorted(diagnostics['contact_errors_over_1mm'], key=lambda e: e['error_mm'], reverse=True)
    for e in worst:
        if all(abs(e['time'] - t) > .4 for t in times):
            times.append(e['time'])
        if len(times) >= 12:
            break
    times = sorted(times)
    times = sorted(set([start, *times, start + (job['frames'] - 1) / job['fps']]))
    (out / 'review-times.json').write_text(json.dumps(times))
    (out / 'provenance.json').write_text(json.dumps({
        'commit': os.environ.get('GITHUB_SHA'), 'workflow_run': os.environ.get('GITHUB_RUN_ID'),
        'plan_sha256': hashlib.sha256((Path('output/plan/motion-plan.json')).read_bytes()).hexdigest(),
        'psap_sha256': hashlib.sha256((Path('output/plan/plan.psap')).read_bytes()).hexdigest(),
        'start': start, 'frames': job['frames'], 'fps': job['fps'],
    }, indent=2))
    run([BLENDER, '--background', '--disable-autoexec', out / 'handcam.blend',
         '--python-exit-code', '1', '--python', ROOT / 'review_blender.py', '--', out.resolve()])
    run([sys.executable, 'review_sheet.py', out])
    shutil.copyfile(out / 'review.jpg', out / f'review-{part}.jpg')
    shutil.copyfile(out / 'diagnostics.json', out / f'diagnostics-{part}.json')


def validate(path, frames):
    run(['ffmpeg', '-v', 'error', '-xerror', '-i', path, '-f', 'null', '-'])
    probe = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-show_streams',
                    '-show_format', '-of', 'json', path], text=True))
    video = next(s for s in probe['streams'] if s['codec_type'] == 'video')
    assert (video['width'], video['height'], video['r_frame_rate'], int(video['nb_frames'])) == (1920, 1080, '60/1', frames), video
    assert abs(float(video['duration']) - frames / 60) < .001
    return probe


def render(part):
    illustration()
    out = Path(f'output/part{part}')
    start = part * 20
    duration = min(20, 150.75 - start)
    if not (out / 'handcam.blend').is_file():
        raise ValueError('Final rendering requires the manually reviewed saved bake')
    from handcam import unpack, prepare_resources, render_screen, render_saved
    from chart import load_chart
    from phigros_renderer import background_image, mix_audio
    out = out.resolve()
    job = json.loads((out / 'job.json').read_text())
    assert job['start'] == start and job['frames'] == round(duration * 60)
    chartpath, music, _ = unpack(Path('inputs/desultory-signals.zip').resolve(), out / 'input')
    chart = load_chart(chartpath.read_text(encoding='utf-8-sig'), source=chartpath)
    job.update(output=str(out),resources=str(prepare_resources(Path('inputs/resources.zip').resolve(),out/'resources')),
               background=str(out/'background.png'),screen_video=str(out/'screen.mp4'),mixed_audio=str(out/'audio.wav'))
    picture = Path('inputs/illustration.jpg')
    background_image(picture,1920,1080,.2,1080*.045).save(job['background'])
    renderer = render_screen(chart,picture,out/'screen',job)
    mix_audio(music,job['resources'],renderer.hits,start,duration,.35,Path(job['mixed_audio']))
    (out/'job.json').write_text(json.dumps(job,indent=2))
    render_saved(out,BLENDER)
    delivery = Path('delivery')
    delivery.mkdir(exist_ok=True)
    shutil.copyfile(out / 'handcam.mp4', delivery / f'part{part}.mp4')
    for name in ('diagnostics', 'render-timings', 'boundary-poses', 'provenance'):
        shutil.copyfile(out / f'{name}.json', delivery / f'part{part}-{name}.json')
    probe = validate(delivery / f'part{part}.mp4', round(duration * 60))
    (delivery / f'part{part}-validation.json').write_text(json.dumps(probe, indent=2))


def assemble():
    from PIL import Image, ImageDraw
    from delivery_report import attach_plan_review, delivery_notes
    from handcam import unpack, prepare_resources
    from chart import load_chart
    from phigros_renderer import PhigrosRenderer, mix_audio
    delivery = Path('delivery')
    delivery.mkdir(exist_ok=True)
    chartpath, audio, picture = unpack(Path('inputs/desultory-signals.zip'), Path('inputs/chart'))
    chart = load_chart(chartpath.read_text(encoding='utf-8-sig'), source=chartpath)
    resources = prepare_resources(Path('inputs/resources.zip'), Path('inputs/resources'))
    renderer = PhigrosRenderer(chart, 1920, 1080, resources, 'Desultory Signals', 'AT 18', 150.75)
    mixed = delivery / 'audio.wav'
    audio_stats = mix_audio(audio, resources, renderer.hits, 0, 150.75, .35, mixed)
    parts = [next(Path('assembled').rglob(f'part{i}.mp4')).resolve() for i in range(8)]
    # Strip per-part AAC first; join video, then encode the continuous mixed audio once.
    videos = []
    for i, path in enumerate(parts):
        video = delivery / f'video-{i}.mp4'
        run(['ffmpeg', '-v', 'error', '-y', '-i', path, '-map', '0:v:0', '-c:v', 'copy', '-an', video])
        videos.append(video)
    listing = delivery / 'concat.txt'
    listing.write_text(''.join(f"file '{p.name}'\n" for p in videos))
    target = delivery / 'Desultory-Signals-AT18-1080p60.mp4'
    run(['ffmpeg', '-v', 'error', '-y', '-f', 'concat', '-safe', '0', '-i', listing,
         '-i', mixed, '-map', '0:v:0', '-map', '1:a:0', '-c:v', 'copy', '-c:a', 'aac',
         '-b:a', '192k', '-t', '150.75', '-movflags', '+faststart', target])
    probe = validate(target, 9045)
    audio_stream = next(s for s in probe['streams'] if s['codec_type'] == 'audio')
    assert abs(float(audio_stream['duration']) - 150.75) < .03
    with target.open('rb') as f:
        digest = hashlib.file_digest(f, 'sha256').hexdigest()
    report = {'media': probe, 'sha256': digest, 'bytes': target.stat().st_size,
              'audio_mix': audio_stats, 'full_decode': 'passed', 'native_ap_validated': False}
    diagnostics = [json.loads(next(Path('assembled').rglob(f'part{i}-diagnostics.json')).read_text()) for i in range(8)]
    report['baked_pose_diagnostics'] = {
        'contact_samples': sum(d['contact_samples'] for d in diagnostics),
        'contact_errors_over_1mm': sum(len(d['contact_errors_over_1mm']) for d in diagnostics),
        'max_error_mm': max(d['max_error_mm'] for d in diagnostics),
        'collision_samples': sum(len(d['collision_errors']) for d in diagnostics),
        'max_collision_depth_mm': max(d['max_collision_depth_mm'] for d in diagnostics),
        'min_screen_clearance_mm': min(d['min_screen_clearance_mm'] for d in diagnostics),
    }
    report['bakes'] = [json.loads(next(Path('assembled').rglob(f'part{i}-provenance.json')).read_text()) for i in range(8)]
    boundaries = [json.loads(next(Path('assembled').rglob(f'part{i}-boundary-poses.json')).read_text()) for i in range(8)]
    report['segment_boundaries'] = []
    for i in range(7):
        before, after = boundaries[i][-1], boundaries[i + 1][0]
        distances = [math.dist(bone['tail'], after['rigs'][name]['bones'][key]['tail']) * 1000
                     for name, arm in before['rigs'].items() for key, bone in arm['bones'].items()]
        report['segment_boundaries'].append({'time': after['time'], 'max_bone_tail_step_mm': max(distances)})
    attach_plan_review(report, ROOT / 'reviewed-plan-validation.json')
    (delivery / 'validation.json').write_text(json.dumps(report, indent=2))
    times = [4.76, 10.5, 15.223, 26, 31, 38, 45.45, 55, 62.376, 78, 92, 101, 108.12, 111.36, 115.248, 125.05, 132, 146.7]
    times = sorted(set(times + [t + delta for t in range(20,141,20) for delta in (-1/60,0)]))
    sheet = Image.new('RGB', (1920, math.ceil(len(times) / 3) * 384), '#101827')
    draw = ImageDraw.Draw(sheet)
    for i, t in enumerate(times):
        frame = delivery / f'frame-{i}.png'
        run(['ffmpeg', '-v', 'error', '-y', '-ss', str(t), '-i', target, '-frames:v', '1', '-vf', 'scale=640:360', frame])
        col, row = i % 3, i // 3
        sheet.paste(Image.open(frame), (col * 640, row * 384 + 24))
        draw.text((col * 640 + 10, row * 384 + 5), f'{t:.3f} s', fill='white')
        frame.unlink()
    sheet.save(delivery / 'final-review.jpg', quality=90)
    (delivery / 'pose-diagnostics.json').write_text(json.dumps(diagnostics, indent=2))
    notes = delivery_notes(report)
    Path('DELIVERY.md').write_text(notes)
    (delivery / 'DELIVERY.md').write_text(notes)
    for p in [*videos, listing, mixed]:
        p.unlink()


if __name__ == '__main__':
    if sys.argv[1] == 'plan':
        plan()
    elif sys.argv[1] == 'bake':
        bake(int(sys.argv[2]))
    elif sys.argv[1] == 'smoke':
        part=int(sys.argv[2])
        bake(part,{100:27.,101:61.,102:108.,103:110.2}[part],1.5)
    elif sys.argv[1] == 'render':
        render(int(sys.argv[2]))
    elif sys.argv[1] == 'assemble':
        assemble()
