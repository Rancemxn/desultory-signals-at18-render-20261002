"""Fingering regressions expressed as physical contact arrangements."""
import unittest
import numpy as np
from handcam_motion import default_profile
from refine_assignments import KEYS,audit,beam_assign,graph,objective


class AssignmentTests(unittest.TestCase):
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
