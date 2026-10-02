"""Numerical regression checks; no raster assets or rendered frames are read."""
from pathlib import Path
from types import SimpleNamespace
import math
import unittest

from basis import NoteType
from block_area import BlockAreas
from contact_refinement import ContactGeometry, refine_contact, validate_contact


class Angle:
    def __matmul__(self, t):
        return 0.


def fixture(kind, centers, seconds, duration=0.):
    notes = [SimpleNamespace(type=kind, seconds=t, hold=duration, offset=complex(x,0))
             for x,t in zip(centers,seconds)]
    line = SimpleNamespace(notes=notes, angle=Angle(), pos=lambda t,offset: offset+4.5j)
    chart = SimpleNamespace(width=16.,height=9.,lines=[line],block_areas=BlockAreas())
    return chart,line


class RefinementTests(unittest.TestCase):
    def test_hold_keeps_a_legal_point_when_line_moves(self):
        chart,line = fixture(NoteType.HOLD,[8.],[0.],1.)
        line.pos = lambda t,offset: offset + .45*math.sin(t*6)*1.0 + 4.5j
        c = dict(kind='hold',note_ids=[0],start=0.,end=1.,
                 points=[[i/100,(8+.45*math.sin(i*.06))/16,.5] for i in range(100)])
        g = ContactGeometry(chart)
        path,report = refine_contact(g,c)
        self.assertTrue(report['stationary'])
        self.assertEqual(path[0][1:],path[-1][1:])
        self.assertFalse(validate_contact(g,c,path))

    def test_ratios_are_fractions_of_full_width(self):
        for kind,ratio,half in ((NoteType.HOLD,.5,.106875),(NoteType.DRAG,.3,.118125)):
            chart,_ = fixture(kind,[8.],[0.],1. if kind==NoteType.HOLD else 0.)
            g = ContactGeometry(chart)
            c = dict(kind='hold' if kind==NoteType.HOLD else 'drag',note_ids=[0],start=0.,end=1.)
            bounds = g.zone(c,0.).bounds
            self.assertAlmostEqual(bounds[2]-bounds[0],2*half*ratio,places=10)

    def test_drag_phrase_can_stay_inside_overlapping_bands(self):
        chart,_ = fixture(NoteType.DRAG,[8.,8.7],[0.,.2])
        c = dict(kind='drag',note_ids=[0,1],start=0.,end=.212,
                 points=[[0.,.5,.5],[.199,8.7/16,.5],[.211,8.7/16,.5]])
        g = ContactGeometry(chart)
        path,report = refine_contact(g,c)
        self.assertTrue(report['stationary'])
        self.assertFalse(validate_contact(g,c,path))

    def test_drag_travel_is_not_constrained_to_nearest_note_strip(self):
        chart,_ = fixture(NoteType.DRAG,[4.,12.],[0.,.2])
        c = dict(kind='drag',note_ids=[0,1],start=0.,end=.212,
                 points=[[0.,.25,.5],[.199,.75,.5],[.211,.75,.5]])
        g = ContactGeometry(chart)
        from shapely.geometry import Point
        self.assertTrue(g.zone(c,.1).covers(Point(.5,.5)))
        self.assertFalse(g.zone(c,0.).covers(Point(.5,.5)))
        self.assertFalse(g.zone(c,.2).covers(Point(.5,.5)))

    def test_flick_refinement_preserves_swipe(self):
        chart,_ = fixture(NoteType.FLICK,[8.],[.025])
        c = dict(kind='flick',note_ids=[0],start=0.,end=.08,
                 points=[[0.,.5,.5],[.025,.5,.54],[.079,.5,.60]])
        path,report = refine_contact(ContactGeometry(chart),c)
        for old,new in zip(c['points'],path):
            self.assertEqual(old[0],new[0])
        self.assertAlmostEqual(path[-1][2]-path[0][2],.1)

    def test_updated_shader_compiles_without_loading_textures(self):
        import skia
        source = (Path(__file__).parent/'block_assets'/'compose.sksl').read_text()
        self.assertIsNotNone(skia.RuntimeEffect.MakeForShader(source))


if __name__ == '__main__':
    unittest.main()
