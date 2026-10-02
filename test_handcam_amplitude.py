"""Revised strokes retain contact boundaries, speed limits and matching bake samples."""
import math

from handcam_blender import sample_times
from handcam_motion import default_profile, finger_pose, finish_motion, palm_offsets, stroke_lift, wrist_pose


def check():
    job = dict(palm_motion='v4', finger_motion='whole_finger', palm_lift_mode='gentle',
               palm_lift_ratio=.70, palm_strike_speed=1.05, lift_height=.045,
               palm_lift_low=.042, palm_lift_high=.035, transfer_sway=.004,
               stroke_gap_limit=1., contact_height=.0005, hand_scale=.27, screen=[.28, .1575])
    old = dict(job, palm_lift_low=.030, palm_lift_high=.032, lift_height=.034,
               palm_strike_speed=.9, stroke_gap_limit=.75, transfer_sway=0.)
    profile = default_profile()
    offsets = palm_offsets(profile)
    boosts = []
    for hand, side, x in (('left', -1, .25), ('right', 1, .75)):
        for gap in (.45, .1, .035, .8):
            pair = [dict(hand=hand, finger='index', start=1., end=1.05, points=[[1., x, .5]]),
                    dict(hand=hand, finger='index', start=1.05 + gap, end=1.1 + gap,
                         points=[[1.05 + gap, x, .5]])]
            rests = finish_motion(pair, profile, job['screen'], .28 / .82)
            heights = [wrist_pose(pair, rests, side, 1.05 + gap * i / 100,
                                  job, offsets, offsets)[2] for i in range(101)]
            assert max(heights) - heights[0] > .01
            speed = max(abs(b - a) / (gap / 100) for a, b in zip(heights, heights[1:]))
            assert speed <= 1.05 + 1e-6, (gap, speed)
            for t in (1.05, 1.05 + gap):
                before = finger_pose(pair, t - 1e-8, (0., -.1, .03), job)[0]
                after = finger_pose(pair, t + 1e-8, (0., -.1, .03), job)[0]
                assert math.dist(before, after) < 1e-5, (gap, t)
            for contact in pair:
                t = contact['start'] + .025
                assert finger_pose(pair, t, (0., -.1, .03), job) == finger_pose(pair, t, (0., -.1, .03), old)

            # Both the finger and palm move outward in the air, with no offset
            # at either touch boundary. The two hands use opposite directions.
            straight = dict(job, transfer_sway=0.)
            for fraction in (0., .5, 1.):
                t = 1.05 + gap * fraction
                finger = finger_pose(pair, t, (0., -.1, .03), job)[0]
                flat_finger = finger_pose(pair, t, (0., -.1, .03), straight)[0]
                palm = wrist_pose(pair, rests, side, t, job, offsets, offsets)
                flat_palm = wrist_pose(pair, rests, side, t, straight, offsets, offsets)
                finger_shift, palm_shift = finger[0] - flat_finger[0], palm[0] - flat_palm[0]
                assert math.isclose(finger_shift, palm_shift, abs_tol=1e-12)
                if fraction == .5:
                    assert 0 < side * finger_shift <= .004 + 1e-12
                else:
                    assert abs(finger_shift) < 1e-12

            sampling = dict(job, contacts=pair, hand_rest=rests, start=.95,
                            duration=1.2, frames=36, fps=30)
            times = sample_times(sampling)
            for phase in (.14, .25, .28, .5, .68, .75, .84):
                assert any(abs(t - (1.05 + phase * gap)) < 1e-12 for t in times)
            if side == -1:
                boosts.append(stroke_lift(pair[1], gap, job) - stroke_lift(pair[1], gap, old))
    assert boosts[0] > boosts[1] > 0, boosts
    print('Stroke amplitude, speed, touch boundaries, outward sway and extended bake sampling checks passed')


if __name__ == '__main__':
    check()
