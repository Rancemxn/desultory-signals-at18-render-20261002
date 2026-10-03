import unittest
from general_refinement import relay, legal_path
from test_contact_refinement import fixture
from basis import NoteType
from contact_refinement import ContactGeometry, validate_contact


class GeneralRulesTests(unittest.TestCase):
    def test_two_stacked_taps_and_a_flick_keep_two_down_contacts(self):
        from types import SimpleNamespace
        from algo.algo5 import Planner,Settings
        from handcam_motion import default_profile
        chart,line=fixture(NoteType.FLICK,[8.],[2.])
        line.notes.extend(SimpleNamespace(type=NoteType.TAP,seconds=2.,hold=0.,offset=8+0j) for _ in range(2))
        contacts=Planner(chart,Settings(fingers=('index',)),default_profile()).run()
        self.assertEqual(len(contacts),2)
        self.assertEqual({n for c in contacts for n in c['note_ids']},{0,1,2})
        self.assertTrue(all(sum(n in (1,2) for n in c['note_ids'])==1 for c in contacts))
        for c in contacts:self.assertFalse(validate_contact(ContactGeometry(chart),c,c['points']))

    def test_discontinuous_hold_keeps_both_original_bands_covered(self):
        from types import SimpleNamespace
        from general_refinement import collective_hold
        from shapely.geometry import Point
        chart,line=fixture(NoteType.HOLD,[8.],[2.],.12)
        class Vertical:
            def __matmul__(self,t):return __import__('math').pi/2
        line.angle=Vertical()
        line.pos=lambda t,offset: complex(8,2 if int((t-2)*1000)%20<10 else 7)
        contact=dict(kind='hold',note_ids=[0],start=2.,end=2.12,points=[[2.,.5,2/9],[2.119,.5,7/9]])
        geometry=ContactGeometry(chart)
        contacts=collective_hold(geometry,contact,2)
        self.assertEqual(len(contacts),2)
        for t in geometry.times(contact,.001):
            zone=geometry.zone(contact,float(t))
            self.assertTrue(any(zone.covers(Point(*c['points'][0][1:])) for c in contacts))
        with self.assertRaises(ValueError):collective_hold(geometry,contact,1)

    def test_invisible_drag_can_be_hit_immediately_before_line_cut(self):
        from algo.algo5 import Planner,Settings
        from handcam_motion import default_profile
        chart,line=fixture(NoteType.DRAG,[8.],[2.])
        line.pos=lambda t,offset: complex(8,4.5) if t<2 else complex(80,45)
        contacts=Planner(chart,Settings(fingers=('index',)),default_profile()).run()
        self.assertEqual(len(contacts),1)
        c=contacts[0]
        self.assertLess(c['judgement_times']['0'],2.)
        self.assertLessEqual(2.-c['judgement_times']['0'],.0155)
        self.assertFalse(validate_contact(ContactGeometry(chart),c,c['points']))

    def test_hold_tail_flick_uses_the_existing_index(self):
        from types import SimpleNamespace
        from algo.algo5 import Planner,Settings
        from handcam_motion import default_profile
        chart,line=fixture(NoteType.HOLD,[4.,12.],[2.,2.],.5)
        line.notes.extend(SimpleNamespace(type=NoteType.FLICK,seconds=2.5,hold=0.,offset=complex(x,0))
                          for x in (4.,12.))
        contacts=Planner(chart,Settings(fingers=('index',)),default_profile()).run()
        self.assertEqual(len(contacts),2)
        self.assertEqual({n for c in contacts for n in c['note_ids']},{0,1,2,3})
        for c in contacts:
            self.assertFalse(validate_contact(ContactGeometry(chart),c,c['points']))

    def test_two_index_fingers_share_simultaneous_flick_hold_pairs(self):
        from types import SimpleNamespace
        from algo.algo5 import Planner,Settings
        from handcam_motion import default_profile
        chart,line=fixture(NoteType.FLICK,[4.,12.],[2.,2.])
        line.notes.extend(SimpleNamespace(type=NoteType.HOLD,seconds=2.,hold=.15,offset=complex(x,0))
                          for x in (4.,12.))
        contacts=Planner(chart,Settings(fingers=('index',)),default_profile()).run()
        self.assertEqual(len(contacts),2)
        self.assertEqual({n for c in contacts for n in c['note_ids']},{0,1,2,3})
        self.assertEqual({(c['hand'],c['finger']) for c in contacts},{('left','index'),('right','index')})
        g=ContactGeometry(chart,extra_clearance_m=.004)
        for c in contacts:
            self.assertFalse(validate_contact(g,c,c['points']))
            swipe=[p for p in c['points'] if p[0]<=2.045]
            self.assertGreaterEqual(__import__('math').dist(swipe[0][1:],swipe[-1][1:]),.10)

    def test_busy_tap_contacts_can_cover_following_drags(self):
        from types import SimpleNamespace
        from algo.algo5 import Planner,Settings
        from handcam_motion import default_profile
        chart,line=fixture(NoteType.TAP,[4.,12.],[2.,2.])
        line.notes.extend(SimpleNamespace(type=NoteType.DRAG,seconds=t,hold=0.,offset=complex(x,0))
                          for x,t in ((4.,2.018),(12.,2.036)))
        contacts=Planner(chart,Settings(fingers=('index',)),default_profile()).run()
        self.assertEqual(len(contacts),2)
        self.assertEqual({n for c in contacts for n in c['note_ids']},{0,1,2,3})
        for c in contacts:
            self.assertFalse(validate_contact(ContactGeometry(chart),c,c['points']))

    def test_terminal_hold_release_is_limited_to_twenty_milliseconds(self):
        from algo.algo5 import Planner,Settings,State
        from handcam_motion import default_profile
        for cut,expected in ((2.49,True),(2.45,False)):
            chart,line=fixture(NoteType.HOLD,[8.],[2.],.5)
            line.pos=lambda t,offset,cut=cut: complex(8,4.5) if t<cut else complex(80,45)
            planner=Planner(chart,Settings(fingers=('index',)),default_profile())
            choices=planner.choices(planner.tasks[0],State(),degraded=True)
            self.assertEqual(bool(choices),expected)
            for _,c in choices:
                self.assertLessEqual(c['terminal_hold_release_ms'],20.)
                self.assertGreater(c['terminal_hold_release_ms'],0.)
                self.assertFalse(validate_contact(ContactGeometry(chart,extra_clearance_m=.004),c,c['points']))

    def test_nine_simultaneous_contacts_can_use_auxiliary_fingers(self):
        from algo.algo5 import Planner,Settings
        from handcam_motion import default_profile
        chart,_=fixture(NoteType.TAP,[2.+i*1.4 for i in range(9)],[2.]*9)
        contacts=Planner(chart,Settings(beam_width=2,candidates_per_finger=1),default_profile()).run()
        self.assertEqual(len(contacts),9)
        self.assertEqual(len({(c['hand'],c['finger']) for c in contacts}),9)

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
