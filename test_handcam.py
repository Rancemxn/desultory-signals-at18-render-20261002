"""Run: uv run python test_handcam.py. No Blender or external assets needed."""
import math
import json
import runpy
import sys
import tempfile
import zipfile
from pathlib import Path
from unittest.mock import patch

from algo.base import ScreenUtil, TouchAction as A, VirtualTouchEvent as E, dump_data
from handcam import contacts_from_psap, render_saved, run_blender, unpack
from handcam_render import completed_chunk
from handcam_blender import assign_contacts, closest_segments, coalesce_contacts, contact_position, finger_sample, release_idle_contacts, sample_times, smooth, solve_steps, world_point


def check():
    with patch('handcam_blender.time.monotonic', return_value=0):
        assert len(list(solve_steps())) == 6  # Bounded even if the clock has not advanced.
    with patch('handcam_blender.time.monotonic', side_effect=[0., .1, .3]):
        steps = solve_steps()
        assert next(steps) == 0  # First fit.
        assert next(steps) == 1  # A second caller shares the remaining budget.
        assert list(steps) == []  # Slow native IK cannot trigger another full refinement cycle.
    # Collision distances must work for crossing, parallel and degenerate segments.
    for segments, distance in [(((0, 0, 0), (1, 0, 0), (.5, -1, 0), (.5, 1, 0)), 0),
                               (((0, 0, 0), (1, 0, 0), (0, 2, 0), (1, 2, 0)), 2),
                               (((0, 0, 0), (0, 0, 0), (1, 0, 0), (2, 0, 0)), 1),
                               (((0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)), 0)]:
        assert math.isclose(math.dist(*closest_segments(*segments)), distance, abs_tol=1e-10)
        assert math.isclose(math.dist(*closest_segments(*segments[2:], *segments[:2])), distance, abs_tol=1e-10)
    assert smooth(-1) == smooth(0) == 0 and smooth(1) == smooth(2) == 1
    assert smooth(.0001) < 1e-10 and 1 - smooth(.9999) < 1e-10  # Smooth starts/stops.
    parked = dict(pointer=1, start=0., end=2.001, points=[[0., .2, .5], [2., .8, .5]])
    held = dict(pointer=2, start=1., end=2.001, points=[[1., .8, .5]])
    visual, shared = coalesce_contacts([parked, held])
    assert math.isclose(visual[0]['end'], .82) and visual[1] is held and len(shared) == 1
    assert parked['end'] == 2.001 and len(parked['points']) == 2  # Source remains intact.
    assert coalesce_contacts([parked])[1] == []
    assert coalesce_contacts([parked, dict(held, end=2.)])[1] == []  # UP cannot cover a same-time event.
    assert coalesce_contacts([parked, held], hold_starts=[0.])[1] == []  # Preserve actual Hold contacts.
    assert coalesce_contacts([parked, dict(held, points=[[1., .7, .5]])])[1] == []
    # Two pointers converging together must not both disappear by aliasing one another.
    assert coalesce_contacts([parked, dict(held, points=[[1., .3, .5], [2., .8, .5]])])[1] == []
    split, released = release_idle_contacts([parked])
    assert len(split) == 2 and split[0]['end'] == .04 and split[1]['start'] == 2.
    assert [p for c in split for p in c['points']] == parked['points'] and released
    protected, released = release_idle_contacts([parked], [(0., 2.)])
    assert protected == [parked] and not released
    protected, released = release_idle_contacts([dict(held, start=0., points=[[0., .8, .5]])], [(0., 2.)])
    assert protected[0]['end'] == 2.001 and not released
    def event(x, y, action, pid=7):
        return E(complex(x, y), action, pid)

    def encode(frames):
        return dump_data(ScreenUtil(100, 100), frames)

    frames = [
        (-20, [event(20, 30, A.DOWN)]),
        (12, [event(20, 30, A.UP), event(80, 70, A.DOWN)]),
        (14, [event(85, 70, A.MOVE)]),
        (16, [event(0, 0, A.UP)]),  # UP position may be stale in existing plans.
    ]
    dimensions, contacts = contacts_from_psap(encode(frames))
    assert dimensions == (100, 100) and len(contacts) == 2
    assert contacts[0]['end'] == contacts[1]['start'] == .012
    assert contacts[1]['points'][-1][1:] == [.85, .7]
    assert contact_position(contacts[1], .013) == (.825, .7)
    assert contact_position(contacts[1], 10) == (.85, .7)
    job = dict(start=0, duration=1, fps=30, frames=30, contacts=contacts, screen=[.28, .28], contact_height=.006)
    times = sample_times(job)
    assert all(t in times for t in (-.02, .012, .014, .016))  # Click shorter than a video frame.
    assert 0 in times and 29 / 30 in times
    rest = (0., -.1, .04)
    pos, _, down = finger_sample(contacts, 0., rest, job)  # Contact crossing clip start.
    assert down and pos[:2] == world_point(job, (.2, .3)) and pos[2] == .006
    for t, expected in ((.012, (.8, .7)), (.014, (.85, .7))):
        pos, _, down = finger_sample(contacts, t, rest, job)
        assert down and pos[:2] == world_point(job, expected) and pos[2] == .006
    assert not finger_sample(contacts, .017, rest, job)[2]
    assert not finger_sample([contacts[0]], .012, rest, job)[2]  # UP releases at this timestamp.
    assert finger_sample(contacts, .03, rest, job)[0][2] > .006
    spaced = [dict(contacts[0], start=0., end=.001), dict(contacts[1], start=2., end=2.01)]
    assert finger_sample(spaced, 1., rest, job) == (rest, 0., False)  # Relax during a long gap.

    # Fractional chart offsets must not round DOWN to a time before the actual contact.
    offset = .123456789123
    _, shifted = contacts_from_psap(encode(frames), offset)
    shifted_job = dict(job, start=offset, contacts=shifted)
    for original, moved in zip(contacts, shifted):
        assert moved['start'] == original['start'] + offset
        assert moved['end'] == original['end'] + offset
        assert moved['start'] in sample_times(shifted_job)
        assert finger_sample([moved], moved['start'], rest, shifted_job)[2]

    invalid = [b'', b'PSAP', encode(frames)[:-1],
               encode([(0, [event(0, 0, A.MOVE)])]),
               encode([(0, [event(0, 0, A.UP)])]),
               encode([(0, [event(0, 0, A.DOWN), event(0, 0, A.DOWN)])]),
               encode([(0, [event(math.nan, 0, A.DOWN)])]),
               encode([(0, [event(101, 0, A.DOWN)])]),
               encode([(0, [event(0, 0, A.DOWN)])])]
    for content in invalid:
        try:
            contacts_from_psap(content)
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid PSAP accepted')

    # Reused pointer IDs may change fingers only between completed contact lifetimes.
    offsets = {(side, finger): (side * dx, .14, 0.)
               for side in (-1, 1) for finger, dx in [('index', -.02), ('middle', 0.), ('ring', .02)]}
    tracks = assign_contacts(job, offsets)
    assert sum(map(len, tracks.values())) == len(contacts)
    assert all(a['end'] < b['start'] for values in tracks.values() for a, b in zip(values, values[1:]))
    wide = dict(job, contacts=[dict(pointer=1, start=0., end=1., points=[[0., .75, .5]]),
                               dict(pointer=2, start=.5, end=.501, points=[[.5, .5, .5]])])
    offsets = {(1, f): (x, .14, 0.) for f, x in [('index', -.03), ('middle', 0.), ('ring', .03), ('little', .05)]}
    assignment = assign_contacts(wide, offsets)
    assert assignment[1, 'ring'][0]['pointer'] == 1  # Outer touch leaves room for the later centre tap.

    # Archive paths cannot escape the requested extraction directory.
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        archive = root / 'bad.zip'
        with zipfile.ZipFile(archive, 'w') as z:
            z.writestr('../outside.txt', 'no')
        try:
            unpack(archive, root / 'input')
        except ValueError:
            pass
        else:
            raise AssertionError('Archive traversal accepted')
        assert not (root / 'outside.txt').exists()
        # Resume uses saved media/settings and never invokes chart preparation or hand solving.
        for name in ('handcam.blend', 'screen.mp4', 'audio.wav', 'background.png'):
            (root / name).touch()
        saved = dict(output=str(root), frames=12, fps=60, width=1920, height=1080,
                     screen_video=str(root / 'screen.mp4'), mixed_audio=str(root / 'audio.wav'), background=str(root / 'background.png'))
        (root / 'job.json').write_text(json.dumps(saved), encoding='utf-8')
        with patch('handcam.run_blender') as worker:
            render_saved(root.resolve(), 'blender', resume=True)
        assert worker.call_args.args[0][-1] == '--resume' and worker.call_args.args[-1] == 36
        info = dict(width=1920, height=1080, r_frame_rate='60/1', nb_frames='12')
        with patch('handcam_render.subprocess.check_output', return_value=json.dumps({'streams': [info]})):
            assert completed_chunk(root / 'screen.mp4', saved, 12)
            assert not completed_chunk(root / 'screen.mp4', saved, 13)
            assert not completed_chunk(root / 'missing.mp4', saved, 12)
        with patch('handcam_render.subprocess.check_output', return_value='broken'):
            assert not completed_chunk(root / 'screen.mp4', saved, 12)
        # Progress and failure reporting are tested with a tiny subprocess, without Blender/rendering.
        log = root / 'worker.log'
        run_blender([sys.executable, '-c', "print('HANDCAM_PROGRESS 1'); print('HANDCAM_PROGRESS 2')"], log, 'Check', 2)
        assert log.read_text().splitlines()[-1] == 'HANDCAM_PROGRESS 2'
        try:
            run_blender([sys.executable, '-c', 'raise SystemExit(3)'], log, 'Check', 2)
        except RuntimeError as error:
            assert str(log) in str(error)
        else:
            raise AssertionError('Worker failure was ignored')
    # The one-command entry keeps all requested inputs and the 720p30/full-song defaults.
    for algorithm in range(1, 6):
        argv = ['export_handcam.py', 'song.zip', '--algorithm', str(algorithm), '--resources', 'skin.zip', '--model', 'hands.blend']
        with patch.object(sys, 'argv', argv), patch('handcam.main', return_value=0) as export:
            try:
                runpy.run_path(str(Path(__file__).with_name('export_handcam.py')), run_name='__main__')
            except SystemExit as result:
                assert result.code == 0
        passed = export.call_args.args[0]
        assert passed[-len(argv[1:]):] == argv[1:]
        assert [passed[passed.index(flag) + 1] for flag in ('--width', '--height', '--fps')] == ['1280', '720', '30']
        assert '--full' in passed and '--stream' in passed
    print('handcam checks passed')


if __name__ == '__main__':
    check()
