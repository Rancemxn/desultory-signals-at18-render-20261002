"""Indexed wrist guides preserve overlap order, boundaries and backward queries."""
import math
import random

from handcam_motion import IntervalIndex, MotionIndex, default_profile, finish_motion, palm_offsets, wrist_pose


def check():
    randomizer = random.Random(19)
    rows = [(0., 100., 'long')]
    rows += [(t := randomizer.uniform(1., 99.), t + randomizer.uniform(.01, 4.), i) for i in range(100)]
    lookup = IntervalIndex(rows)
    times = [-1., 0., 100., 101., *(v for row in rows for v in row[:2])]
    randomizer.shuffle(times)
    for t in times:
        expected = [value for a, b, value in rows if a <= t <= b]
        assert lookup.at(t) == expected
        assert lookup.at(t) == expected  # Repeated cached queries preserve order too.
    assert IntervalIndex([]).at(0.) == []

    contacts = []
    for hand, finger, start, end in [('left', 'ring', 1., 4.), ('left', 'middle', 2., 2.08),
                                    ('left', 'index', 2.3, 2.4), ('right', 'middle', 2., 2.2),
                                    ('left', 'little', 2., 2.08), ('left', 'middle', 4.5, 4.7),
                                    ('right', 'middle', 8., 8.2)]:
        contacts.append(dict(hand=hand, finger=finger, start=start, end=end,
                             points=[[start, .4, .5], [end, .43, .53]]))
    contacts.sort(key=lambda c: c['start'])
    profile = default_profile()
    rests = finish_motion(contacts, profile, [.28, .1575], .28 / .82)
    index = MotionIndex(contacts, rests)
    job = dict(hand_scale=.27, screen=[.28, .1575], palm_motion='v4', finger_motion='whole_finger',
               palm_lift_ratio=.7, palm_lift_mode='gentle', lift_height=.045,
               palm_lift_low=.042, palm_lift_high=.035, palm_strike_speed=1.05,
               stroke_gap_limit=1., transfer_sway=.004)
    offsets = palm_offsets(profile)
    rotated = {key: (value[0] + .003, value[1] - .002, value[2]) for key, value in offsets.items()}
    times = [-100., 0., 100.]
    times += [c[key] + delta for c in contacts for key in ('prepare', 'start', 'end', 'release_until')
              for delta in (-1e-8, 0., 1e-8)]
    times += [knot[0] + delta for rest in rests for knot in rest['knots'] for delta in (-1e-8, 0., 1e-8)]
    times += [i / 50 for i in range(500)]
    randomizer.shuffle(times)
    for t in times:
        for side in (-1, 1):
            a = wrist_pose(contacts, rests, side, t, job, rotated, offsets, index)
            b = wrist_pose(contacts, rests, side, t, job, rotated, offsets)
            assert math.dist(a, b) < 1e-12, (t, side, a, b)
    print('Indexed wrist guides match across long Holds, ties, rest boundaries and backward queries')


if __name__ == '__main__':
    check()
