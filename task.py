"""Isolated rendering driver. All large intermediates stay on Actions runners."""
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time

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


def bake(part):
    illustration()
    out = Path(f'output/part{part}')
    start = part * 20
    duration = min(20, 150.75 - start)
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
    (out / 'review-times.json').write_text(json.dumps(times))
    run([BLENDER, '--background', '--disable-autoexec', out / 'handcam.blend',
         '--python-exit-code', '1', '--python', ROOT / 'review_blender.py', '--', out.resolve()])
    run([sys.executable, 'review_sheet.py', out])
    shutil.copyfile(out / 'review.jpg', out / f'review-{part}.jpg')
    shutil.copyfile(out / 'diagnostics.json', out / f'diagnostics-{part}.json')


if __name__ == '__main__':
    if sys.argv[1] == 'plan':
        plan()
    elif sys.argv[1] == 'bake':
        bake(int(sys.argv[2]))
