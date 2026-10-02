"""Legality/transform regression cases for the 4.0.1 block implementation."""
import math
import unittest
from shapely.geometry import Point
from block_area import BlockAreas, ease
from chart import load_chart


def area(lo=(.2,.2), hi=(.8,.8), **values):
    return dict(bottomLeftPercentage=dict(zip(('x','y'),lo)),
                topRightPercentage=dict(zip(('x','y'),hi)),
                appearTime=0.,enableTime=1.,disableTime=4.,disappearTime=5.,
                isSubtract=False,rotateEvents=[],scaleEvents=[],moveEvents=[],**values)


class TestBlocks(unittest.TestCase):
    def test_empty_and_timing(self):
        b=BlockAreas([area()])
        self.assertFalse(b.contains(.999,8,4.5))
        self.assertTrue(b.contains(1,8,4.5))
        self.assertFalse(b.contains(4,8,4.5))
        self.assertEqual([b.areas[0].phase(t) for t in (-1,0,.5,1,4,5)],
                         ['hidden','disabled','ready','active','disabled','hidden'])
        self.assertTrue(BlockAreas().forbidden(0).is_empty)

    def test_xor_is_not_difference_or_union(self):
        normal=area(); subtract=area((.4,.4),(.9,.9));subtract['isSubtract']=True
        b=BlockAreas([normal,subtract])
        self.assertTrue(b.contains(2,4,6))       # normal only
        self.assertFalse(b.contains(2,8,4.5))   # both: open window
        self.assertTrue(b.contains(2,13.6,1.2)) # subtract only: blocked
        self.assertFalse(BlockAreas([subtract,subtract]).contains(2,8,4.5))
        self.assertTrue(BlockAreas([normal,normal]).contains(2,8,4.5))

    def test_inset_uses_screen_height_and_both_masks(self):
        b=BlockAreas([area()])
        self.assertFalse(b.contains(2,3.3,4.5)) # visual edge x=3.2; 0.27 inset
        self.assertTrue(b.contains(2,3.5,4.5))
        b=BlockAreas([area((.48,.2),(.52,.8))])
        self.assertFalse(b.contains(2,7.80,4.5)) # cap .25 local on thin block
        self.assertTrue(b.contains(2,8,4.5))

    def test_ease_table(self):
        self.assertEqual([ease(.5,i) for i in (1,2,4,5,7,8,10,11,13,14)],
                         [.25,.75,.125,.875,.0625,.9375,.03125,.96875,0.,1.])
        # Native table interpolates its 1% samples; not exact t**2.
        self.assertAlmostEqual(ease(.125,1),(.12**2+.13**2)/2,places=7)
        with self.assertRaises(ValueError): ease(.5,15)

    def test_scale_rotate_then_move(self):
        d=area((.1,.1),(.3,.3))
        d['scaleEvents']=[dict(time=1.,scale=dict(x=1.,y=1.),anchor=dict(x=.5,y=.5),easeTypeX=0,easeTypeY=0),
                          dict(time=2.,scale=dict(x=2.,y=2.),anchor=dict(x=0.,y=0.),easeTypeX=0,easeTypeY=0)]
        d['rotateEvents']=[dict(time=1.,rotation=0.,anchor=dict(x=.5,y=.5),easeType=0),
                           dict(time=2.,rotation=90.,anchor=dict(x=0.,y=0.),easeType=0)]
        d['moveEvents']=[dict(time=1.,endPosition=dict(x=.2,y=.2),easeTypeX=0,easeTypeY=0),
                         dict(time=2.,endPosition=dict(x=.3,y=.4),easeTypeX=0,easeTypeY=0)]
        r=BlockAreas([d],10,10).areas[0].rectangle(2.)
        self.assertAlmostEqual(r.center[0],12.,places=5)
        self.assertAlmostEqual(r.center[1],9.,places=5)
        self.assertAlmostEqual(r.size[0],4.,places=5)
        self.assertAlmostEqual(r.angle,-math.pi/2)

    def test_midsegment_and_stationary_dwell(self):
        b=BlockAreas([area((.48,.2),(.52,.8))])
        self.assertTrue(b.path_violations([[2,.3,.5],[2.02,.7,.5]],end=2.021))
        self.assertTrue(b.path_violations([[.9,.5,.5]],end=1.01))
        self.assertFalse(b.path_violations([[2,.2,.5],[2.02,.3,.5]],end=2.021))

    def test_polygon_agrees_with_point_predicate(self):
        d=area(); d['rotateEvents']=[dict(time=0.,rotation=31.,anchor=dict(x=.5,y=.5),easeType=0)]
        other=area((.1,.3),(.45,.7)); other['isSubtract']=True
        b=BlockAreas([d,other])
        p=b.forbidden(2)
        for x in range(32):
            for y in range(18):
                self.assertEqual(b.contains(2,x*.5+.1,y*.5+.1),p.covers(Point(x*.5+.1,y*.5+.1)))

    def test_chart_retains_blocks(self):
        chart=load_chart('{"formatVersion":3,"offset":0,"judgeLineList":[],"blockAreaList":[]}')
        self.assertFalse(chart.block_areas)


if __name__=='__main__': unittest.main()
