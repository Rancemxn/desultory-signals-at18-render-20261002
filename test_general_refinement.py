import unittest
from general_refinement import relay, legal_path
from test_contact_refinement import fixture
from basis import NoteType
from contact_refinement import ContactGeometry, validate_contact


class GeneralRulesTests(unittest.TestCase):
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
