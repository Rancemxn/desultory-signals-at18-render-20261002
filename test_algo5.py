"""Run with project Python. Exercises planning semantics without Blender or video output."""
import copy
import math
from types import SimpleNamespace

from bamboo import BambooShoot
from basis import Note, NoteType as N
from algo.algo5 import Planner, PlanningError, Settings, load_at, plan
from algo.base import dump_data
from handcam import contacts_from_psap
from handcam_blender import assign_contacts, finger_sample, sample_times
from handcam_motion import attach_plan, default_profile, finish_motion, palm_offsets, point_at, travel_time, wrist_pose


class Line:
    def __init__(self, notes, rotation=0., moving=False):
        self.notes = notes
        self.angle = BambooShoot(rotation)
        self.moving = moving

    def pos(self, t, offset):
        return complex(800 + (100 * math.sin(t * 2) if self.moving else 0), 450) + offset


def chart(notes, **kwargs):
    return SimpleNamespace(width=1600, height=900, offset=0., lines=[Line(notes, **kwargs)])


def fails(call):
    try:
        call()
    except ValueError:
        return
    raise AssertionError('Invalid input accepted')


def check():
    settings = Settings(beam_width=4, allow_degraded=False)
    notes = [Note(N.TAP, 1 + i * .125, 0, (-220 if i % 2 else 220) + 0j) for i in range(12)]
    source = chart(notes)
    screen, answer, motion = plan(source, settings=settings)
    raw = dump_data(screen, answer)
    assert not motion['diagnostics']['degraded']
    assert set(motion['diagnostics']['finger_usage']) == {'left:index', 'right:index'}
    assert all(.024 <= c['end'] - c['start'] <= .077 for c in motion['contacts'])
    for c in motion['contacts']:
        note = source.lines[0].notes[c['note_ids'][0]]
        target = source.lines[0].pos(note.seconds, note.offset)
        x, y = point_at(c['points'], note.seconds)
        assert math.hypot(x * source.width - target.real, y * source.height - target.imag) <= source.width * .012 + 1e-6
    assert dump_data(*plan(source, settings=settings)[:2]) == raw
    # Keep the chart's exact timestamp for visual geometry. Rounding to integer
    # milliseconds can select the other side of an instantaneous line event.
    jump_note = Note(N.TAP, math.nextafter(1., math.inf), 0., 0j)
    jump = chart([jump_note])
    jump.lines[0].pos = lambda t, offset: complex(1000 if t > 1. else 800, 450) + offset
    _, _, jump_motion = plan(jump, settings=settings)
    x, y = jump_motion['contacts'][0]['points'][0][1:]
    assert math.hypot(x * 1600 - 1000, y * 900 - 450) <= 1600 * .012 + 1e-6
    offset = .123456789123
    _, contacts = contacts_from_psap(raw, offset)
    attached = attach_plan(raw, contacts, motion, offset)
    assert all(a['prepare'] == b['prepare'] + offset for a, b in zip(attached, motion['contacts']))
    altered = copy.deepcopy(motion)
    altered['contacts'][0]['points'][0][1] += .01
    fails(lambda: attach_plan(raw, contacts, altered, offset))
    fails(lambda: attach_plan(raw + b'wrong', contacts, motion, offset))
    altered = copy.deepcopy(motion)
    altered['version'] = 50
    fails(lambda: attach_plan(raw, contacts, altered, offset))
    altered = copy.deepcopy(motion)
    altered['contacts'][0]['prepare'] = math.nan
    fails(lambda: attach_plan(raw, contacts, altered, offset))
    profile = default_profile()
    offsets = {(int(k.split(':')[0]), k.split(':')[1]): v['offset'] for k, v in profile['fingers'].items()}
    job = dict(start=0., duration=4., frames=120, fps=30, contacts=motion['contacts'],
               screen=[.28, .1575], contact_height=.0005, lift_height=.025, hand_scale=.27,
               motion_plan_version=1, hand_rest=motion['hand_rest'])
    tracks = assign_contacts(job, offsets)
    assert sum(map(len, tracks.values())) == len(notes)
    for c in motion['contacts']:
        key = (-1 if c['hand'] == 'left' else 1, c['finger'])
        assert c in tracks[key]
        pose = finger_sample(tracks[key], c['start'], (0., -.1, .03), job)
        assert pose[2] and pose[0][2] == job['contact_height']
        assert not finger_sample(tracks[key], c['end'], (0., -.1, .03), job)[2]
        # Sampling resolution never changes the continuous motion at the same instant.
        for when in (c['start'] - .012, c['start'], c['end'] + .008):
            assert finger_sample(tracks[key], when, (0., -.1, .03), job) == finger_sample(
                tracks[key], when, (0., -.1, .03), dict(job, fps=60, frames=240))
    assert all(c['start'] in sample_times(job) for c in job['contacts'])
    # Airborne phase boundaries must not introduce a jump in target position.
    for events in tracks.values():
        for c in events:
            for t in (c['prepare'], c['start'], c['end'], c['release_until']):
                before = finger_sample(events, t - 1e-7, (0., -.1, .03), job)[0]
                after = finger_sample(events, t + 1e-7, (0., -.1, .03), job)[0]
                assert math.dist(before, after) < 1e-5, f'Airborne target discontinuity at {t}'
    assert any(g['mode'] == 'withdraw' and g['intro'] for g in motion['hand_rest'])
    for side in (-1, 1):
        parked = wrist_pose(job['contacts'], job['hand_rest'], side, -2., job, offsets)
        assert abs(parked[0]) < .14 and parked[1] < -.27, 'Rest must leave through the bottom'
    # Sustained Hold loads the finger/hand even without a new DOWN or MOVE.
    hold = dict(hand='left', finger='index', kind='hold', start=0., end=3., effort=.055, burst_cost=.4)
    f0, b0 = load_at([hold], .1, 'left', 'index')
    f1, b1 = load_at([hold], 2., 'left', 'index')
    assert f1 > f0 and b1 > b0
    assert load_at([hold], 2., 'left')[0] > 0
    assert load_at([hold], 2., 'left', 'middle')[0] == 0
    assert load_at([hold], 7., 'left', 'index')[0] < load_at([hold], 3., 'left', 'index')[0]
    duplicated_moves = dict(hold, points=[[i / 1000, .3, .5] for i in range(3001)])
    assert load_at([hold], 2., 'left') == load_at([duplicated_moves], 2., 'left')
    assert travel_time(.1, .5, 5.) >= 1.875 * .1 / .5
    # Rotating judge bands/long contacts remain inside the sampled valid region.
    moving = chart([Note(N.HOLD, .5, .7, -180 + 0j), Note(N.TAP, .7, 0, 230 + 0j),
                    Note(N.FLICK, 1.5, 0, 260 + 0j)], rotation=.4, moving=True)
    screen, answer, meta = plan(moving, settings=settings)
    verifier = Planner(moving, settings, default_profile())
    from shapely.geometry import Point
    for c in meta['contacts']:
        task = next(t for t in verifier.tasks if t.id == c['note_ids'][0])
        for ms in range(task.start, round(c['end'] * 1000), 5):
            x, y = point_at(c['points'], ms / 1000)
            assert verifier.zone(task, ms).buffer(1e-7).covers(Point(x * screen.width, y * screen.height))
    held = next(c for c in meta['contacts'] if c['kind'] == 'hold')
    assert held['start'] == .5 and held['end'] >= 1.2
    # A Hold follows the visible note even along the judgement strip's free axis.
    stationary_hold = chart([Note(N.HOLD, 1., 1., -220 + 0j)])
    stationary_hold.lines[0].pos = lambda t, offset: complex(800, 450 + 180 * math.sin(t * 4)) + offset
    _, _, stable = plan(stationary_hold, settings=settings)
    assert len(stable['contacts'][0]['points']) > 1, 'Hold parked inside its judgement strip'
    held_note = stationary_hold.lines[0].notes[0]
    held_pos = stationary_hold.lines[0].pos(held_note.seconds, held_note.offset)
    x, y = stable['contacts'][0]['points'][0][1:]
    assert math.hypot(x * 1600 - held_pos.real, y * 900 - held_pos.imag) <= 1600 * .012 + 1e-6
    translating_hold = chart([Note(N.HOLD, 1., 1., -220 + 0j)])
    translating_hold.lines[0].pos = lambda t, offset: complex(800 + 450 * (t - 1), 450) + offset
    _, _, shifted = plan(translating_hold, settings=settings)
    shifted_contact = shifted['contacts'][0]
    distance = math.dist(shifted_contact['points'][0][1:], shifted_contact['points'][-1][1:])
    assert math.isclose(distance, 450 * .999 / 1600, abs_tol=1e-9), 'Hold did not follow its note one-to-one'
    moving_verifier = Planner(translating_hold, Settings(hold_margin=0), default_profile())
    for ms in range(1000, 2000, 5):
        x, y = point_at(shifted_contact['points'], ms / 1000)
        assert moving_verifier.zone(moving_verifier.tasks[0], ms).buffer(1e-7).covers(Point(x * 1600, y * 900))
    # The hand itself lifts between taps, and rotation affects its wrist anchor.
    natural_job = dict(job, palm_motion='v4', lift_height=.028)
    for events in tracks.values():
        for previous, following in zip(events, events[1:]):
            t = (previous['end'] + following['start']) / 2
            old = finger_sample(events, t, (0., -.1, .03), dict(job, motion_plan_version=None))[0][2] - job['contact_height']
            new = finger_sample(events, t, (0., -.1, .03), natural_job)[0][2] - job['contact_height']
            assert math.isclose(new, old * 1.12, rel_tol=1e-8), 'Stroke does not track fourth-round amplitude'
    raised_job = dict(natural_job, finger_motion='whole_finger')
    for events in tracks.values():
        for previous, following in zip(events, events[1:]):
            gap = following['start'] - previous['end']
            for u in (.25, .5, .75):
                t = previous['end'] + u * gap
                raised = finger_sample(events, t, (0., -.1, .03), raised_job)
                original = finger_sample(events, t, (0., -.1, .03), natural_job)
                assert not raised[2] and raised[0][2] > original[0][2]
                assert raised[0][2] <= raised_job['contact_height'] + raised_job['lift_height']
            for t in (previous['end'], following['start']):
                before = finger_sample(events, t - 1e-8, (0., -.1, .03), raised_job)[0]
                after = finger_sample(events, t + 1e-8, (0., -.1, .03), raised_job)[0]
                assert math.dist(before, after) < 1e-5, 'Raised stroke jumps at contact boundary'
            assert previous['end'] + .28 * gap in sample_times(raised_job)
        for contact in events:
            t = (contact['start'] + contact['end']) / 2
            assert finger_sample(events, t, (0., -.1, .03), raised_job) == finger_sample(
                events, t, (0., -.1, .03), natural_job), 'Raised stroke changed a held contact'
    guides = palm_offsets(default_profile())
    gap = next(g for g in motion['hand_rest'] if g['mode'] == 'hover' and not g['intro'] and not g['outro'])
    side = -1 if gap['hand'] == 'left' else 1
    middle = (gap['start'] + gap['end']) / 2
    p = wrist_pose(job['contacts'], job['hand_rest'], side, middle, natural_job, guides, guides)
    assert p[2] > gap['knots'][0][3] + .001, 'Palm remained at a fixed height between taps'
    coupled_job = dict(raised_job, palm_lift_ratio=.65, wrist_speed=.9)
    coupled = wrist_pose(job['contacts'], job['hand_rest'], side, middle, coupled_job, guides, guides)
    assert coupled[2] > p[2], 'Palm did not accompany the larger finger stroke'
    assert coupled == wrist_pose(job['contacts'], job['hand_rest'], side, middle,
                                 dict(coupled_job, fps=60), guides, guides)
    duration = gap['end'] - gap['start']
    heights = [wrist_pose(job['contacts'], job['hand_rest'], side, gap['start'] + duration * i / 100,
                          coupled_job, guides, guides)[2] for i in range(101)]
    assert max(abs(b - a) / (duration / 100) for a, b in zip(heights, heights[1:])) <= .8 * coupled_job['wrist_speed'] + 1e-6
    assert math.isclose(heights[0], heights[-1], abs_tol=1e-6), 'Palm did not return for the next contact'
    assert gap['start'] + .28 * duration in sample_times(coupled_job)
    # The accented policy must grow in dense passages, not shrink under the old
    # transfer-speed cap. Compare identical positions at different tap intervals.
    accent_job = dict(coupled_job, palm_lift_mode='accent', palm_lift_ratio=.78,
                      lift_height=.056, palm_strike_speed=4.5)
    amplitudes = []
    for free_time in (.35, .16, .06):
        second = 1.05 + free_time
        pair = [dict(hand='left', finger='index', start=1., end=1.05, points=[[1., .25, .5]]),
                dict(hand='left', finger='index', start=second, end=second + .05, points=[[second, .25, .5]])]
        paired_rests = finish_motion(pair, default_profile(), [.28, .1575], .28 / .82)
        heights = [wrist_pose(pair, paired_rests, -1, 1.05 + free_time * i / 100,
                              accent_job, guides, guides)[2] for i in range(101)]
        amplitudes.append(max(heights) - heights[0])
        assert math.isclose(heights[0], heights[-1], abs_tol=1e-6)
        assert max(abs(b - a) / (free_time / 100) for a, b in zip(heights, heights[1:])) < accent_job['palm_strike_speed']
    assert .03 < amplitudes[0] < amplitudes[1] < amplitudes[2], 'Dense strikes must increase whole-hand amplitude'
    gentle_job = dict(accent_job, palm_lift_mode='gentle', palm_lift_ratio=.70,
                      lift_height=.034, palm_strike_speed=.9)
    for free_time in (.35, .16, .06, .035):
        second = 1.05 + free_time
        pair = [dict(hand='left', finger='index', start=1., end=1.05, points=[[1., .25, .5]]),
                dict(hand='left', finger='index', start=second, end=second + .05, points=[[second, .25, .5]])]
        paired_rests = finish_motion(pair, default_profile(), [.28, .1575], .28 / .82)
        heights = [wrist_pose(pair, paired_rests, -1, 1.05 + free_time * i / 100,
                              gentle_job, guides, guides)[2] for i in range(101)]
        assert .009 < max(heights) - heights[0] < .023, 'Restrained palm lift exceeded its intended range'
        assert math.isclose(heights[0], heights[-1], abs_tol=1e-6), 'Gentle wrist missed the next contact'
        speed = max(abs(b-a) / (free_time/100) for a,b in zip(heights,heights[1:]))
        assert speed <= gentle_job['palm_strike_speed'] + 1e-6, 'Short gap needs a smaller stroke, not a late landing'
    from handcam_motion import transfer_progress
    progress = [transfer_progress(i / 1000) for i in range(1001)]
    steps = [b-a for a,b in zip(progress,progress[1:])]
    assert progress[0] == 0 and progress[-1] == 1 and min(steps) >= 0
    assert max(steps) * 1000 < 1.26, 'Lateral travel is still concentrated into a few frames'
    assert steps[0] < 1e-7 and steps[-1] < 1e-7, 'Transfer starts or stops abruptly'
    angle = .3
    rotated = {k: (o[0] * math.cos(angle) - o[1] * math.sin(angle),
                   o[0] * math.sin(angle) + o[1] * math.cos(angle), o[2]) for k, o in guides.items()}
    for t in (gap['start'] + 1e-7, gap['end'] - 1e-7):
        before = wrist_pose(job['contacts'], job['hand_rest'], side, t - 2e-7, natural_job, rotated, guides)
        after = wrist_pose(job['contacts'], job['hand_rest'], side, t + 2e-7, natural_job, rotated, guides)
        assert math.dist(before, after) < 1e-5, 'Rotated rest endpoint disagrees with contact anchor'
    # Consecutive covered Drags may remain down; each note retains its identity.
    drags = chart([Note(N.DRAG, 1. + i * .12, 0, -220 + 0j) for i in range(5)])
    _, _, chain = plan(drags, settings=settings)
    assert sorted(i for c in chain['contacts'] for i in c['note_ids']) == list(range(5))
    assert chain['diagnostics']['joined_drags'] > 0
    assert all(c['finger'] in settings.fingers for c in chain['contacts'])
    # Large exact Drag stacks share their path, preserving all source note IDs.
    stacked = chart([Note(N.DRAG, 1., 0, x + 0j) for x in (-220, 220) for _ in range(47)])
    _, _, stack_plan = plan(stacked, settings=settings)
    assert len(stack_plan['contacts']) == 2
    assert sorted(i for c in stack_plan['contacts'] for i in c['note_ids']) == list(range(94))
    _, _, pair_plan = plan(chart([Note(N.DRAG, 1., 0, x + 0j) for x in (-220, 220)]), settings=settings)
    for stacked_contact, single_contact in zip(stack_plan['contacts'], pair_plan['contacts']):
        assert stacked_contact['points'] == single_contact['points']
        assert stacked_contact['finger'] == single_contact['finger']
        assert stacked_contact['effort'] == single_contact['effort']
    # A held left hand cannot withdraw just because the right hand is idle.
    fixed = [dict(hand='left', finger='index', start=1., end=4., points=[[1., .25, .5]]),
             dict(hand='right', finger='index', start=1., end=1.1, points=[[1., .75, .5]]),
             dict(hand='right', finger='index', start=5., end=5.1, points=[[5., .75, .5]])]
    rests = finish_motion(fixed, default_profile(), [.28, .1575], .28 / .82)
    assert not any(g['hand'] == 'left' and g['start'] <= 2. < g['end'] for g in rests)
    assert any(g['hand'] == 'right' and g['mode'] == 'withdraw' and g['start'] <= 2. < g['end'] for g in rests)
    # Preparing another finger must not pull the wrist away from a held note.
    upcoming = dict(hand='left', finger='middle', start=2.1, end=2.2, prepare=1.8,
                    release_until=2.3, points=[[2.1, .7, .5]])
    base_wrist = wrist_pose(fixed, rests, -1, 2., job, offsets)
    assert wrist_pose([*fixed, upcoming], rests, -1, 2., job, offsets) == base_wrist
    # A held finger anchors its hand even when another finger prepares to tap.
    held_wrist = wrist_pose(fixed, rests, -1, 2., coupled_job, guides, guides)
    assert wrist_pose([*fixed, upcoming], rests, -1, 2., coupled_job, guides, guides) == held_wrist
    assert held_wrist == wrist_pose(fixed, rests, -1, 2., raised_job, guides, guides)
    assert held_wrist == wrist_pose(fixed, rests, -1, 2., accent_job, guides, guides)
    assert held_wrist == wrist_pose(fixed, rests, -1, 2., gentle_job, guides, guides)
    # Unsupported motion must be explicit; hard occupancy is never downgraded.
    one_finger = default_profile()
    one_finger['fingers'] = {'-1:index': one_finger['fingers']['-1:index']}
    fast = chart([Note(N.TAP, 1., 0, -500 + 0j), Note(N.TAP, 1.09, 0, 500 + 0j)])
    fails(lambda: plan(fast, settings=settings, profile=one_finger))
    _, _, degraded = plan(fast, settings=Settings(beam_width=1), profile=one_finger)
    assert degraded['diagnostics']['degraded']
    # No planner may disguise eleven simultaneous independent taps as ten fingers.
    impossible = chart([Note(N.TAP, 1., 0, complex((i - 5) * 50, 0)) for i in range(11)])
    fails(lambda: plan(impossible, settings=Settings(beam_width=1)))
    print('algo5 planning and motion contract checks passed')


if __name__ == '__main__':
    check()
