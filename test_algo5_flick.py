"""Flicks cross the judged note once, with physical distance independent of aspect ratio."""
import cmath
import math

from shapely.geometry import box

from algo.algo5 import Planner, Settings, State, plan
from algo.base import dump_data
from basis import Note, NoteType as N
from handcam import contacts_from_psap
from handcam_motion import attach_plan, default_profile, point_at, world_xy
from test_algo5 import chart, fails


def check():
    settings = Settings(beam_width=4, allow_degraded=False)
    for distance in (.09, .26, float('nan')):
        fails(lambda: Settings(flick_distance=distance))

    source = chart([Note(N.FLICK, 1., 0., -220 + 0j)])
    screen, answer, motion = plan(source, settings=settings)
    raw = dump_data(screen, answer)
    _, contacts = contacts_from_psap(raw)
    attached = attach_plan(raw, contacts, motion)
    flick = attached[0]
    assert flick['start'] == .975 and flick['end'] == 1.046
    distance = math.dist(world_xy(flick['points'][0][1:], motion['physical_screen']),
                         world_xy(flick['points'][-1][1:], motion['physical_screen']))
    # The early movement budget may prefer a sideways path clipped by the
    # judgement strip. It still needs a readable stroke, within the 44.8 mm target.
    assert .04 <= distance <= .0448 + 1e-7, distance

    # Try every seed and direction on rotating, moving lines. Anchoring seeds at
    # the wider pre-beat zone used to allow a reversal when approaching the note.
    for rotation in (0., .6, 1.4):
        source = chart([Note(N.FLICK, 1., 0., 0j)], rotation=rotation, moving=True)
        planner = Planner(source, settings, default_profile())
        task = planner.tasks[0]
        center = source.lines[0].pos(1., 0j)
        for side in (-1, 1):
            for seed in planner.seeds(task, State(), side, 'index'):
                for direction in (1, -1, 2, -2):
                    points = planner.path(task, seed, task.end, direction)
                    assert points is not None
                    at_beat = point_at(points, 1.)
                    assert math.hypot(at_beat[0] * 1600 - center.real,
                                      at_beat[1] * 900 - center.imag) <= 1600 * .012 + 1e-6
                    axis = cmath.exp(1j * rotation) * 1j
                    if abs(direction) == 2:
                        axis *= -1j
                    axis *= 1 if direction > 0 else -1
                    progress = [complex(p[1] * 1600, p[2] * 900) for p in points]
                    assert all(((b - a) * axis.conjugate()).real >= -1e-6
                               for a, b in zip(progress, progress[1:])), points

    # A vertical path can be long in normalized Y but short in physical metres.
    # Reject a clipped stroke shorter than 28 mm on the 280 mm-wide screen.
    planner = Planner(chart([Note(N.FLICK, 1., 0., 0j)]), settings, default_profile())
    planner.zone = lambda task, ms: box(700, 400, 900, 500)
    assert planner.path(planner.tasks[0], (800, 450), 1045, 1) is None
    print('Flick timing, PSAP binding, beat alignment, monotonic paths and physical clipping checks passed')


if __name__ == '__main__':
    check()
