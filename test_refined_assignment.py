"""Fingering regressions expressed as physical contact arrangements."""
import unittest
import numpy as np
from handcam_motion import default_profile
from refine_assignments import KEYS,audit,beam_assign,graph,objective


class AssignmentTests(unittest.TestCase):
    def test_order_relays_keep_crossing_hold_paths_and_continuous_coverage(self):
        from refine_assignments import split_order_conflicts
        from handcam_motion import point_at
        contacts=[dict(start=3.,end=5.1,pointer=i,points=[[3.,a,.5],[5.099,b,.5]],
                       kind='hold',note_ids=[i],hand='left',finger=finger)
                  for i,(a,b,finger) in enumerate(((.7,.3,'index'),(.3,.7,'middle')))]
        result=split_order_conflicts(contacts,[.28,.1575],.4)
        self.assertGreater(len(result),2)
        for old in contacts:
            pieces=[c for c in result if c['note_ids']==old['note_ids']]
            self.assertEqual(pieces[0]['start'],old['start'])
            self.assertEqual(pieces[-1]['end'],old['end'])
            for a,b in zip(pieces,pieces[1:]):self.assertGreaterEqual(a['end']-b['start'],.0199)
            for c in pieces:
                for t,x,y in c['points']:
                    self.assertAlmostEqual(x,point_at(old['points'],t)[0])
                    self.assertAlmostEqual(y,point_at(old['points'],t)[1])

    def test_exact_repair_removes_forbidden_pair_without_reusing_finger(self):
        from refine_assignments import repair_topology
        unary=np.zeros((3,3))
        conflict=np.eye(3)*1e12
        neighbors=[[(1,conflict),(2,conflict)],[(0,conflict),(2,conflict)],[(0,conflict),(1,conflict)]]
        labels=repair_topology(np.array([0,0,1]),unary,neighbors)
        self.assertEqual(len(set(labels)),3)

    def test_index_only_domain_survives_global_assignment(self):
        from refine_assignments import assignment_keys,descend
        contacts = [dict(start=0.,end=.08,points=[[0.,x,.5],[.079,x,.5]],
                         kind='tap',note_ids=[i],hand=hand,finger='index')
                    for i,(x,hand) in enumerate(((.2,'left'),(.8,'right')))]
        plan = dict(profile=default_profile(),physical_screen=[.28,.1575],settings={'fingers':['index']})
        keys = assignment_keys(plan)
        self.assertEqual(keys,[('left','index'),('right','index')])
        unary,neighbors = graph(contacts,plan)
        self.assertEqual(unary.shape,(2,2))
        labels = descend(beam_assign(unary,neighbors,32),unary,neighbors)
        self.assertEqual({keys[k] for k in labels},set(keys))

    def test_left_hand_chord_does_not_cross_or_reuse_a_finger(self):
        contacts = [dict(start=0.,end=1.,points=[[0.,x,.5],[.999,x,.5]],
                    kind='hold',note_ids=[i],manual_hand='left',hand='left',finger=finger)
                    for i,(x,finger) in enumerate(((.36,'index'),(.46,'middle')))]
        plan = dict(profile=default_profile(),physical_screen=[.28,.1575])
        unary,neighbors = graph(contacts,plan)
        wrong = np.array([KEYS.index(('left','index')),KEYS.index(('left','middle'))])
        labels = beam_assign(unary,neighbors,32)
        self.assertLess(objective(labels,unary,neighbors),objective(wrong,unary,neighbors))
        for c,label in zip(contacts,labels):
            c['hand'],c['finger'] = KEYS[label]
            self.assertEqual(c['hand'],'left')
        report = audit(contacts,plan['physical_screen'])
        self.assertFalse(report['finger_overlaps'])
        self.assertFalse(report['same_hand_crossings'])

    def test_authored_relay_finger_is_preserved(self):
        contact = dict(start=0.,end=1.,points=[[0.,.6,.6],[.999,.6,.6]],
                       kind='hold',note_ids=[0],fixed_finger=['right','thumb'])
        unary,neighbors = graph([contact],dict(profile=default_profile(),physical_screen=[.28,.1575]))
        label = beam_assign(unary,neighbors,32)[0]
        self.assertEqual(KEYS[label],('right','thumb'))


if __name__=='__main__':
    unittest.main()
