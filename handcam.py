"""Offline handcam experiment. Run with the project's Python, not Blender's."""
from __future__ import annotations

import argparse
import copy
import importlib
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import zipfile

from rich.progress import BarColumn, MofNCompleteColumn, Progress, TimeElapsedColumn, TimeRemainingColumn

from algo.base import TouchAction, dump_data, load_data
from basis import NoteType
from chart import load_chart


def progress_bar():
    return Progress('[progress.description]{task.description}', BarColumn(), '[progress.percentage]{task.percentage:>3.0f}%',
                    MofNCompleteColumn(), TimeElapsedColumn(), TimeRemainingColumn())


def run_blender(command, log_path, description, total):
    """Keep Blender chatter in the log while displaying its completed work in the terminal."""
    with log_path.open('w', encoding='utf-8') as log, progress_bar() as progress:
        task = progress.add_task(description, total=total)
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, encoding='utf-8', errors='replace', bufsize=1) as process:
            try:
                for line in process.stdout:
                    log.write(line)
                    log.flush()
                    if line.startswith('HANDCAM_PROGRESS '):
                        completed = int(line.split()[1])
                        progress.update(task, completed=min(completed, total))
                    elif line.startswith('HANDCAM_STATUS '):
                        progress.update(task, description=line.strip().split(' ', 1)[1])
                if process.wait():
                    raise RuntimeError(f'{description} failed; see {log_path}')
            except BaseException:
                if process.poll() is None:
                    process.terminate()
                raise
        progress.update(task, completed=total)


def render_saved(out, blender, resume=False):
    job = json.loads((out / 'job.json').read_text(encoding='utf-8'))
    if Path(job['output']).resolve() != out:
        raise ValueError('Saved job belongs to a different output directory')
    for path in (out / 'handcam.blend', *(Path(job[key]) for key in ('screen_video', 'mixed_audio', 'background'))):
        if not path.is_file():
            raise ValueError(f'Render input missing: {path}')
    command = [blender, '--background', '--disable-autoexec', str(out / 'handcam.blend'),
               '--python-exit-code', '1', '--python', str(Path(__file__).with_name('handcam_render.py')),
               '--', str(out / 'job.json')]
    if resume:
        command.append('--resume')
    run_blender(command, out / ('render-resume.log' if resume else 'render.log'),
                'Render and composite', job['frames'] * 3)


def contacts_from_psap(content: bytes, time_offset: float = 0.) -> tuple[tuple[int, int], list[dict]]:
    if not math.isfinite(time_offset):
        raise ValueError('Chart offset must be finite')
    # Validate before the legacy reader, whose header check uses assert.
    if len(content) < 12 or content[:4] != b'PSAP':
        raise ValueError('Not a PSAP file')
    offset = 12
    while offset < len(content):
        if offset + 5 > len(content):
            raise ValueError('Truncated PSAP timestamp')
        count = content[offset + 4]
        if count > 127 or offset + 5 + count * 21 > len(content):
            raise ValueError('Invalid or truncated PSAP events')
        offset += 5 + count * 21
    screen, frames = load_data(content)
    if screen.width <= 0 or screen.height <= 0:
        raise ValueError('Invalid PSAP screen size')
    active, contacts = {}, []
    previous = -math.inf
    for timestamp, events in frames:
        if timestamp < previous:
            raise ValueError('PSAP timestamps must be ordered')
        previous = timestamp
        t = timestamp / 1000 + time_offset
        for event in events:
            pid, action = event.pointer_id, event.action
            x, y = event.pos.real / screen.width, event.pos.imag / screen.height
            if not all(math.isfinite(v) for v in (x, y)) or not (0 <= x <= 1 and 0 <= y <= 1):
                raise ValueError(f'Pointer {pid} has an invalid screen position at {t}s')
            if action == TouchAction.DOWN:
                if pid in active:
                    raise ValueError(f'Duplicate DOWN for pointer {pid} at {t}s')
                contact = {'pointer': pid, 'start': t, 'end': None, 'points': [[t, x, y]]}
                contacts.append(contact)
                active[pid] = contact
            elif action == TouchAction.MOVE:
                if pid not in active:
                    raise ValueError(f'MOVE without DOWN for pointer {pid} at {t}s')
                points = active[pid]['points']
                if points[-1][0] == t:
                    if points[-1][1:] != [x, y]:
                        raise ValueError(f'Conflicting positions at {t}s for pointer {pid}')
                else:
                    points.append([t, x, y])
            elif action in (TouchAction.UP, TouchAction.CANCEL):
                if pid not in active:
                    raise ValueError(f'UP without DOWN for pointer {pid} at {t}s')
                contact = active.pop(pid)
                contact['end'] = t
                # Some planners put a stale position in UP. Releasing does not teleport a fingertip.
            else:
                raise ValueError(f'Unsupported PSAP action: {action.name}')
    if active:
        raise ValueError(f'PSAP ends with unreleased pointers: {sorted(active)}')
    return (screen.width, screen.height), contacts


def extract_archive(source: Path, destination: Path):
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(source) as archive:
        # Extract only files inside this dedicated directory; reject traversal and zip bombs.
        root = destination.resolve()
        if sum(i.file_size for i in archive.infolist()) > 512 * 1024 * 1024:
            raise ValueError('Chart archive exceeds 512 MiB unpacked')
        for info in archive.infolist():
            target = (root / info.filename).resolve()
            if not target.is_relative_to(root):
                raise ValueError(f'Unsafe archive path: {info.filename}')
            if info.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.open(info) as src, target.open('wb') as dst:
                    shutil.copyfileobj(src, dst)


def unpack(source: Path, destination: Path) -> tuple[Path, Path | None, Path | None]:
    if not zipfile.is_zipfile(source):
        return source, None, None
    extract_archive(source, destination)
    candidates = sorted(p for p in destination.rglob('*') if p.suffix.lower() in ('.json', '.pec'))
    charts = []
    for path in candidates:
        try:
            load_chart(path.read_text(encoding='utf-8-sig'), source=path)
            charts.append(path)
        except (ValueError, KeyError, TypeError):
            continue
    if len(charts) != 1:
        raise ValueError('Archive must contain one chart; extract it and pass its path explicitly')
    audio = sorted(p for p in destination.rglob('*') if p.suffix.lower() in ('.ogg', '.mp3', '.wav', '.flac'))
    images = sorted(p for p in destination.rglob('*') if p.suffix.lower() in ('.png', '.jpg', '.jpeg', '.webp'))
    return charts[0], audio[0] if len(audio) == 1 else None, images[0] if images else None


def prepare_resources(source: Path, destination: Path):
    if source.is_dir():
        return source.resolve()
    if not source.is_file() or not zipfile.is_zipfile(source):
        raise ValueError(f'Resources must be a directory or ZIP: {source}')
    extract_archive(source, destination)
    packs = [p.parent for p in destination.rglob('info.yml') if (p.parent / 'click.png').is_file()]
    if len(packs) != 1:
        raise ValueError('Resource ZIP must contain one pack with info.yml and click.png')
    return packs[0].resolve()


def find_blender(value: str | None) -> str:
    if value:
        executable = shutil.which(value) or (str(Path(value).resolve()) if Path(value).is_file() else None)
    else:
        executable = shutil.which('blender')
        if not executable:
            installed = sorted(Path('C:/Program Files/Blender Foundation').glob('Blender */blender.exe'))
            executable = str(installed[-1]) if installed else None
    if not executable:
        raise ValueError('Blender not found; specify --blender /path/to/blender')
    return executable


def render_screen(chart, image_path, directory, job):
    import skia
    from phigros_renderer import PhigrosRenderer, background_image

    directory.mkdir(parents=True, exist_ok=True)
    from handcam_blender import screen_rect
    _, _, width, height = screen_rect(job)
    renderer = PhigrosRenderer(chart, width, height, job['resources'], job['title'], job['level'], job['music_duration'])
    surface = skia.Surface(width, height)
    canvas = surface.getCanvas()
    background = background_image(image_path, width, height, job['background_dim'], height * .015)
    streaming = job.get('screen_video')
    selected = {1, job['frames'] // 4 + 1, job['frames'] // 2 + 1, job['frames'] * 3 // 4 + 1, job['frames']}
    process = None
    if streaming:
        process = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pixel_format', 'bgra',
                                    '-video_size', f'{width}x{height}', '-framerate', str(job['fps']), '-i', '-',
                                    '-an', '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '12', '-pix_fmt', 'yuv420p',
                                    str(directory.parent / 'screen.partial.mp4')], stdin=subprocess.PIPE)
    try:
        with progress_bar() as progress:
            task = progress.add_task('Chart video', total=job['frames'])
            for index in range(job['frames']):
                canvas.drawImage(background, 0, 0)
                renderer.draw(canvas, job['start'] + index / job['fps'] - chart.offset)
                image = surface.makeImageSnapshot()
                if process:
                    process.stdin.write(image.tobytes())
                if not process or index + 1 in selected:
                    image.save(str(directory / f'{index + 1:05d}.png'))
                progress.update(task, advance=1)
    finally:
        if process:
            process.stdin.close()
            if process.wait():
                raise RuntimeError('Screen video encoding failed')
    if process:
        (directory.parent / 'screen.partial.mp4').replace(streaming)
    print(f'Screen: {job["frames"]} frames (Phigros Autoplay)', flush=True)
    return renderer


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('chart', type=Path, help='chart ZIP/PEZ, JSON or PEC')
    parser.add_argument('--model', type=Path, required=True, help='Hands + armature.blend')
    parser.add_argument('--audio', type=Path, help='overrides the music in the chart archive')
    parser.add_argument('--psap', type=Path, help='replay an existing plan instead of running a planner')
    parser.add_argument('--motion-plan', type=Path, help='algo5 motion-plan.json accompanying --psap')
    parser.add_argument('--strict-psap', action='store_true', help='display every raw contact, including redundant overlaps')
    parser.add_argument('--algorithm', type=int, choices=(1, 2, 3, 4, 5), default=1,
                        help='existing planner; algo1 usually releases idle contacts sooner (default: 1)')
    parser.add_argument('--video', type=Path, help='full-song Autoplay video, aligned with the music')
    parser.add_argument('--background', type=Path, help='illustration for the playfield and blurred surround')
    parser.add_argument('--resources', type=Path, default=Path(__file__).parent / 'refer/phira/assets',
                        help='Phira resource ZIP, pack directory, or assets directory')
    parser.add_argument('--title', help='override song title from chart metadata')
    parser.add_argument('--level', help='override chart difficulty label')
    parser.add_argument('--screen-fill', type=float, default=.82, help='playfield fraction of output width (default: .82)')
    parser.add_argument('--background-dim', type=float, default=.45, help='playfield illustration dimming, 0–1')
    parser.add_argument('--hitsound-volume', type=float, default=.35, help='hit sound gain, 0 disables (default: .35)')
    parser.add_argument('--start', type=float, default=84., help='music time in seconds')
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--full', action='store_true', help='render the complete music, including the final partial frame')
    parser.add_argument('--fps', type=int, default=30, help='output frame rate (default: 30)')
    parser.add_argument('--width', type=int, default=1280, help='output width in pixels (default: 1280)')
    parser.add_argument('--height', type=int, default=720, help='output height in pixels (default: 720)')
    parser.add_argument('--output', type=Path, default=Path('build/handcam'))
    parser.add_argument('--blender')
    parser.add_argument('--engine', choices=('workbench', 'eevee', 'cycles'), default='workbench')
    parser.add_argument('--hand-scale', type=float, default=.27, help='model scale; source model units to metres (default: .27)')
    parser.add_argument('--contact-height', type=float, default=.0005, help='fingertip skin clearance in metres')
    parser.add_argument('--lift-height', type=float, help='maximum airborne finger lift in metres (algo5: .045; others: .025)')
    parser.add_argument('--prepare-only', action='store_true', help='write PSAP, motion job and screen frames')
    parser.add_argument('--plan-only', action='store_true', help='write touch/motion plans and diagnostics without baking or rendering')
    parser.add_argument('--keyframes', action='store_true', help='render five inspection frames instead of the video')
    parser.add_argument('--bake-only', action='store_true', help='solve and save the full hand animation without rendering media')
    parser.add_argument('--warmup', type=float, default=1., help='seconds of motion history before a bake segment')
    parser.add_argument('--screen-video', action='store_true', help='stream chart frames into video instead of saving every PNG')
    parser.add_argument('--stream', action='store_true', help='render a baked animation in small video chunks, saving disk space')
    parser.add_argument('--resume', action='store_true', help='resume video rendering from the saved job in --output; uses saved settings')
    args = parser.parse_args(argv)
    if args.lift_height is None:
        args.lift_height = .045 if args.algorithm == 5 or args.motion_plan else .025
    if args.motion_plan and not args.psap:
        parser.error('--motion-plan requires its matching --psap')
    if not math.isfinite(args.warmup) or not 0 <= args.warmup <= 20:
        parser.error('--warmup must be between 0 and 20 seconds')
    if args.stream and (args.video or args.keyframes or args.bake_only or args.engine != 'workbench'):
        parser.error('--stream requires Workbench and built-in chart rendering; omit --video, --keyframes and --bake-only')
    if not all(math.isfinite(v) for v in (args.start, args.duration, args.hand_scale, args.contact_height, args.lift_height,
                                        args.screen_fill, args.background_dim, args.hitsound_volume)):
        parser.error('Times and scales must be finite')
    if args.start < 0 or args.duration <= 0 or args.hand_scale <= 0 or args.contact_height < 0 or args.lift_height < 0:
        parser.error('Invalid time, hand scale or contact height')
    if not (1 <= args.fps <= 120) or min(args.width, args.height) < 64 or args.width % 2 or args.height % 2:
        parser.error('Use 1–120 fps and even dimensions of at least 64 pixels')
    if not .4 <= args.screen_fill <= .95 or not 0 <= args.background_dim <= 1 or not 0 <= args.hitsound_volume <= 2:
        parser.error('Use screen fill .4–.95, background dim 0–1, and hit sound gain 0–2')
    try:
        if args.resume:
            out = args.output.resolve()
            render_saved(out, find_blender(args.blender), resume=True)
            print(f'Video: {out / "handcam.mp4"}', flush=True)
            return 0
        for path in (args.chart, args.model, args.audio, args.psap, args.motion_plan, args.video, args.background):
            if path is not None and not path.is_file():
                raise ValueError(f'File does not exist: {path}')
        if not shutil.which('ffmpeg') or not shutil.which('ffprobe'):
            raise ValueError('FFmpeg and ffprobe are required on PATH')
        blender = find_blender(args.blender)
        out = args.output.resolve()
        if args.model.resolve() == out / 'handcam.blend' or (args.video and args.video.resolve() == out / 'handcam.mp4'):
            raise ValueError('Output would overwrite an input model or video; choose another --output directory')
        out.mkdir(parents=True, exist_ok=True)
        chart_path, music, picture = unpack(args.chart.resolve(), out / 'input')
        picture = args.background.resolve() if args.background else picture
        music = args.audio.resolve() if args.audio else music
        if music is None:
            raise ValueError('Provide --audio, or a chart archive with one music file')
        probe = json.loads(subprocess.check_output([
            'ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'json', str(music),
        ], text=True))
        music_duration = float(probe['format']['duration'])
        if args.full:
            args.start, args.duration = 0., music_duration
        frame_count = math.ceil(args.duration * args.fps)
        duration = frame_count / args.fps
        if not args.full and args.start + duration > music_duration + .01:
            raise ValueError('Requested clip extends beyond the audio')
        chart = load_chart(chart_path.read_text(encoding='utf-8-sig'), source=chart_path)
        from phigros_renderer import background_image, mix_audio, read_info
        from handcam_blender import screen_rect
        metadata = read_info(chart_path.parent / 'info.yml') | read_info(chart_path.parent / 'info.txt')
        if chart.format == 'rpe':
            meta = json.loads(chart_path.read_text(encoding='utf-8-sig')).get('META', {})
            metadata = {key.lower(): value for key, value in meta.items()} | metadata
        if not math.isfinite(chart.offset):
            raise ValueError('Chart offset must be finite')
        motion_plan = None
        if args.psap:
            content = args.psap.read_bytes()
            if args.motion_plan:
                motion_plan = json.loads(args.motion_plan.read_text(encoding='utf-8'))
        else:
            from preview import cdc
            from rich.console import Console
            planning = copy.copy(chart)
            planning.lines = [copy.copy(line) for line in chart.lines]
            begin, end = args.start - chart.offset - 2, args.start + duration - chart.offset + 2
            if args.algorithm != 5:
                for line in planning.lines:
                    line.notes = [n for n in line.notes if n.seconds + n.hold >= begin and n.seconds <= end]
            if not any(line.notes for line in planning.lines):
                raise ValueError('No notes overlap the planning window')
            config = cdc()
            config['algo4_continue_when_failed'] = False
            profile = None
            if args.algorithm == 5:
                profile_job = dict(output=str(out), hand_scale=args.hand_scale, enable_thumb=True, profile_only=True)
                (out / 'profile-job.json').write_text(json.dumps(profile_job), encoding='utf-8')
                run_blender([blender, '--background', '--factory-startup', '--disable-autoexec', str(args.model.resolve()),
                    '--python-exit-code', '1', '--python', str(Path(__file__).with_name('handcam_blender.py').resolve()),
                    '--', str(out / 'profile-job.json')], out / 'profile.log', 'Measure hand rig', 1)
                profile = json.loads((out / 'hand-profile.json').read_text(encoding='utf-8'))
                profile['model_sha256'] = hashlib.sha256(args.model.read_bytes()).hexdigest()
            with Console().status(f'Planning touches (algo{args.algorithm})...'), (out / 'solve.log').open('w', encoding='utf-8') as log:
                algorithm = importlib.import_module(f'algo.algo{args.algorithm}')
                if args.algorithm == 5:
                    screen, answer, motion_plan = algorithm.plan(planning, config, Console(file=log),
                                                               profile=profile, view_width=.28 / args.screen_fill)
                else:
                    screen, answer = algorithm.solve(planning, config, Console(file=log))
            content = dump_data(screen, answer)
        # Convert chart time to music time once, including fractional offsets.
        dimensions, contacts = contacts_from_psap(content, chart.offset)
        if motion_plan:
            from handcam_motion import attach_plan
            profile = motion_plan['profile']
            if not math.isclose(profile['scale'], args.hand_scale, rel_tol=1e-9):
                raise ValueError('Motion plan uses a different --hand-scale; replan with algo5')
            if profile.get('model_sha256') and profile['model_sha256'] != hashlib.sha256(args.model.read_bytes()).hexdigest():
                raise ValueError('Motion plan was measured for a different hand model; replan with algo5')
            expected_screen = [.28, .28 * chart.height / chart.width]
            if any(not math.isclose(a, b, rel_tol=1e-9) for a, b in zip(motion_plan['physical_screen'], expected_screen)):
                raise ValueError('Motion plan uses a different physical screen size')
            contacts = attach_plan(content, contacts, motion_plan, chart.offset)
            (out / 'motion-plan.json').write_text(json.dumps(motion_plan, indent=2), encoding='utf-8')
            (out / 'planning-diagnostics.json').write_text(json.dumps(motion_plan['diagnostics'], indent=2), encoding='utf-8')
            if motion_plan['diagnostics']['degraded']:
                print(f'Planning: {len(motion_plan["diagnostics"]["degraded"])} contacts exceed motion estimates; '
                      'see planning-diagnostics.json', flush=True)
        if not contacts:
            raise ValueError('The plan has no contacts')
        if not math.isclose(dimensions[0] / dimensions[1], chart.width / chart.height, rel_tol=.001):
            raise ValueError('PSAP and chart aspect ratios differ')
        (out / 'plan.psap').write_bytes(content)
        if args.plan_only:
            print(f'Plan: {out / "plan.psap"}' + (f' + {out / "motion-plan.json"}' if motion_plan else ''), flush=True)
            return 0
        if not motion_plan and not any(c['end'] >= args.start and c['start'] < args.start + duration for c in contacts):
            raise ValueError('No contacts overlap the requested clip')
        # Algo5 has already planned the whole song; keep its rest schedule but only solve nearby fingers.
        contacts = [c for c in contacts if c['end'] >= args.start - args.warmup and c['start'] <= args.start + duration + 1]
        screen_w = .28
        screen_h = screen_w * chart.height / chart.width
        resources = prepare_resources(args.resources.resolve(), out / 'resources') if not args.video else args.resources.resolve()
        job = dict(start=args.start, duration=duration, frames=frame_count, fps=args.fps, warmup=args.warmup,
                   width=args.width, height=args.height, output=str(out), model=str(args.model.resolve()),
                   contacts=contacts, screen=[screen_w, screen_h], view_width=screen_w / args.screen_fill, camera_y=-.015,
                   resources=str(resources), illustration=str(picture) if picture else None,
                   title=args.title if args.title is not None else metadata.get('name', args.chart.stem),
                   level=args.level if args.level is not None else metadata.get('level', ''),
                   music_duration=music_duration, background_dim=args.background_dim,
                   renderer='external' if args.video else 'phigros-autoplay', hitsound_volume=args.hitsound_volume,
                   hold_starts=[n.seconds + chart.offset for line in chart.lines for n in line.notes if n.type == NoteType.HOLD],
                   hold_intervals=[(n.seconds + chart.offset, n.seconds + n.hold + chart.offset)
                                   for line in chart.lines for n in line.notes if n.type == NoteType.HOLD],
                   engine=args.engine, hand_scale=args.hand_scale, contact_height=args.contact_height,
                   lift_height=args.lift_height, strict_psap=args.strict_psap,
                   keyframes=args.keyframes, bake_only=args.bake_only or args.stream, audio=str(music), source=str(args.chart.resolve()),
                   algorithm=None if args.psap else args.algorithm)
        if motion_plan:
            rests = copy.deepcopy(motion_plan['hand_rest'])
            for gap in rests:
                gap['start'] += chart.offset
                gap['end'] += chart.offset
                for point in gap['knots']:
                    point[0] += chart.offset
            job.update(motion_plan_version=motion_plan['version'], hand_rest=rests,
                       palm_motion=motion_plan.get('palm_motion'),
                       finger_motion='whole_finger' if motion_plan.get('palm_motion') == 'v4' else None,
                       palm_lift_ratio=.70 if motion_plan.get('palm_motion') == 'v4' else None,
                       palm_lift_mode='gentle' if motion_plan.get('palm_motion') == 'v4' else None,
                       palm_strike_speed=1.05,
                       palm_lift_low=.042, palm_lift_high=.035,
                       stroke_gap_limit=1., transfer_sway=.004,
                       camera_projection='perspective' if motion_plan.get('palm_motion') == 'v4' else 'orthographic',
                       camera_height=.65,
                       camera_offset_y=-.15,
                       enable_thumb=True, pose_avoidance=True,
                       wrist_speed=motion_plan['settings']['wrist_speed'])
            guides = copy.deepcopy(motion_plan.get('pose_guides',[]))
            for guide in guides:
                for name in ('prepare','start','end','release'):
                    guide[name] += chart.offset
            job['pose_guides'] = guides
            for name in ('palm_lift_low','palm_lift_high','palm_strike_speed','transfer_sway','finger_lateral_limit'):
                if name in motion_plan.get('pose_style',{}):
                    job[name] = motion_plan['pose_style'][name]
        _, _, sw, sh = screen_rect(job)
        if not args.bake_only:
            background_image(picture, args.width, args.height, .2, args.height * .045).save(str(out / 'background.png'))
        job['background'] = str(out / 'background.png')
        if (args.screen_video or args.stream) and not args.video:
            job['screen_video'] = str(out / 'screen.mp4')
        (out / 'job.json').write_text(json.dumps(job, indent=2), encoding='utf-8')
        if args.bake_only:
            pass
        elif args.video:
            (out / 'screen').mkdir(exist_ok=True)
            # A shorter new input must not silently reuse frames from a previous export.
            for frame in (out / 'screen').glob('[0-9][0-9][0-9][0-9][0-9].png'):
                frame.unlink()
            subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', str(args.start), '-i', str(args.video),
                            '-vf', f'fps={args.fps},scale={sw}:{sh}', '-frames:v', str(frame_count),
                            str(out / 'screen/%05d.png')], check=True)
            if not (out / 'screen' / f'{frame_count:05d}.png').exists():
                raise ValueError('Autoplay video is shorter than the requested clip')
        else:
            renderer = render_screen(chart, picture, out / 'screen', job)
            audio_stats = mix_audio(music, resources, renderer.hits, args.start, duration,
                                    args.hitsound_volume, out / 'audio.wav')
            job['mixed_audio'] = str(out / 'audio.wav')
            (out / 'audio-mix.json').write_text(json.dumps(audio_stats, indent=2), encoding='utf-8')
        (out / 'job.json').write_text(json.dumps(job, indent=2), encoding='utf-8')
        for warning in chart.warnings:
            print(f'Chart warning: {warning}', flush=True)
        print(f'Prepared {len(contacts)} contact lifetimes in {out}', flush=True)
        if args.prepare_only:
            return 0
        command = [blender, '--background', '--factory-startup', '--disable-autoexec',
                   str(args.model.resolve()), '--python-exit-code', '1', '--python',
                   str(Path(__file__).with_name('handcam_blender.py').resolve()), '--', str(out / 'job.json')]
        run_blender(command, out / 'blender.log', 'Hand animation', job['frames'])
        diagnostics = json.loads((out / 'diagnostics.json').read_text(encoding='utf-8'))
        if diagnostics['shared_events']:
            print(f'Animation: {len(diagnostics["shared_events"])} redundant events shared by an existing contact; '
                  'see motion.json, or use --strict-psap to keep every raw contact', flush=True)
        if diagnostics.get('idle_releases'):
            print(f'Animation: {len(diagnostics["idle_releases"])} idle pointer intervals released; see motion.json', flush=True)
        if diagnostics['contact_errors_over_1mm']:
            print(f'Warning: fingertip skin error reaches {diagnostics["max_error_mm"]:.2f} mm; see diagnostics.json', flush=True)
        if diagnostics['collision_errors']:
            print(f'Warning: unresolved hand collision reaches {diagnostics["max_collision_depth_mm"]:.2f} mm; '
                  'see diagnostics.json', flush=True)
        if diagnostics['min_screen_clearance_mm'] is not None and diagnostics['min_screen_clearance_mm'] < 0:
            print(f'Warning: hand mesh crosses the screen by {-diagnostics["min_screen_clearance_mm"]:.2f} mm; '
                  'increase --contact-height and inspect the pose', flush=True)
        if args.keyframes:
            print(f'Inspection frames: {out / "inspection"}', flush=True)
            return 0
        if args.bake_only:
            print(f'Animation: {out / "handcam.blend"}', flush=True)
            return 0
        if args.stream:
            render_saved(out, blender)
        else:
            from handcam_blender import composite_command
            subprocess.run(composite_command(job), check=True)
            (out / 'handcam.partial.mp4').replace(out / 'handcam.mp4')
        print(f'Video: {out / "handcam.mp4"}', flush=True)
        return 0
    except (ValueError, OSError, RuntimeError, struct.error, subprocess.CalledProcessError) as error:
        parser.exit(1, f'handcam: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
