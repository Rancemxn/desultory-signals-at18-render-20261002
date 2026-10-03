import unittest
from general_refinement import relay, legal_path
from test_contact_refinement import fixture
from basis import NoteType
from contact_refinement import ContactGeometry, validate_contact


class GeneralRulesTests(unittest.TestCase):
    def test_planner_corridor_preserves_block_legality(self):
        from algo.algo5 import Planner,Settings
        from handcam_motion import default_profile
        from block_area import BlockAreas
        from test_block_area import area
        chart,_ = fixture(NoteType.HOLD,[8.],[2.],.5)
        chart.block_areas = BlockAreas([area((.40,.40),(.60,.60))])
        planner = Planner(chart,Settings(),default_profile())
        task = planner.tasks[0]
        path = planner.corridor_path(task,task.end)
        self.assertIsNotNone(path)
        self.assertFalse(chart.block_areas.path_violations(path,2.,2.501))
        c = dict(kind='hold',note_ids=[0],start=2.,end=2.501)
        self.assertFalse(validate_contact(ContactGeometry(chart,extra_clearance_m=.004),c,path))
        chart.block_areas = BlockAreas([area((0.,0.),(1.,1.))])
        planner = Planner(chart,Settings(),default_profile())
        self.assertIsNone(planner.corridor_path(planner.tasks[0],planner.tasks[0].end))

    def test_travelling_relay_has_continuous_coverage(self):
        c = dict(kind='hold',note_ids=[91],start=10.,end=16.,points=[[10.,.1,.5],[15.999,.9,.5]])
        pieces = relay(c,(.28,.1575))
        self.assertGreater(len(pieces),1)
        self.assertEqual(pieces[0]['start'],c['start'])
        self.assertEqual(pieces[-1]['end'],c['end'])
        for left,right in zip(pieces,pieces[1:]):
            self.assertGreaterEqual(left['end']-right['start'],.059)
        for piece in pieces:
            self.assertEqual(piece['points'][0][0],piece['start'])
            self.assertLess(piece['points'][-1][0],piece['end'])
            self.assertEqual(piece['note_ids'],[91])

    def test_stationary_hold_does_not_force_relay(self):
        c = dict(kind='hold',note_ids=[43],start=27.,end=39.,points=[[27.,.5,.5]])
        self.assertEqual(relay(c,(.28,.1575)),[c])

    def test_rules_work_with_shifted_time_and_note_id(self):
        for start in (0.,73.125):
            chart,line = fixture(NoteType.HOLD,[8.],[start],1.)
            g = ContactGeometry(chart)
            c = dict(kind='hold',note_ids=[0],start=start,end=start+1,
                     points=[[start,.52,.5],[start+.999,.48,.5]])
            points = legal_path(g,c,(.28,.1575))
            self.assertFalse(validate_contact(g,c,points))
            self.assertEqual(points[0][1:],points[-1][1:])


if __name__=='__main__': unittest.main()
