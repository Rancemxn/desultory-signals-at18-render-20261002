"""Sustained contacts track notes; thumbs participate without banning crossed hands."""
import math

from algo.algo5 import PREFERENCE, Planner, Settings, State, plan
from basis import Note, NoteType as N
from handcam_motion import default_profile, point_at
from test_algo5 import chart


def check():
    assert Settings().fingers == ('index', 'middle', 'thumb')
    assert max(PREFERENCE.values()) - min(PREFERENCE.values()) < 1.
    for kind, hold in ((N.HOLD, 1.), (N.DRAG, 0.)):
        source = chart([Note(kind, 1., hold, -150 + 0j)], rotation=.7)
        source.lines[0].pos = lambda t, offset: complex(800 + 70 * math.sin(t * 3), 450 + 100 * math.cos(t * 2)) + offset
        planner = Planner(source, Settings(), default_profile())
        _, _, motion = plan(source)
        contact = motion['contacts'][0]
        assert len(contact['points']) > 1
        for t, x, y in contact['points']:
            at = planner.target_time(planner.tasks[0], t * 1000)
            expected = source.lines[0].pos(at, source.lines[0].notes[0].offset)
            assert abs(complex(x * 1600, y * 900) - expected) < 1e-7
        # Interpolated frames remain close between 20 ms planning samples.
        for i in range(40):
            t = 1. + (contact['points'][-1][0] - 1.) * i / 40
            x, y = point_at(contact['points'], t)
            expected = source.lines[0].pos(t, source.lines[0].notes[0].offset)
            assert abs(complex(x * 1600, y * 900) - expected) < .1
    source = chart([Note(N.HOLD, 1., .5, complex(x, 0)) for x in (-420, -260, -100, 100, 260, 420)])
    _, _, motion = plan(source, settings=Settings(beam_width=2))
    assert len(motion['contacts']) == 6
    assert sum(c['finger'] == 'thumb' for c in motion['contacts']) == 2
    assert all(c['end'] >= 1.5 for c in motion['contacts'])
    # Two planar wrist guides can coincide while the baked palms pass at different heights.
    planner = Planner(source, Settings(), default_profile())
    offsets = [planner.offset(side, 'index') for side in (-1, 1)]
    contacts = [dict(hand=hand, finger='index', start=1., end=2.,
                     points=[[1., .5 + offset[0] / .28, .5 - offset[1] / .1575]])
                for hand, offset in zip(('left', 'right'), offsets)]
    errors, _, _ = planner.constraints(contacts[1], State((contacts[0],)), 1, 'index')
    assert 'palm_collision' not in errors
    print('Thumb defaults, note tracking and crossed-hand planning checks passed')


if __name__ == '__main__':
    check()
