import copy
import unittest
import numpy as np

from algo.algo5 import Planner,Settings,State
from basis import NoteType
from handcam_motion import default_profile
from load_assignment import assign_load,score_labels
from refine_assignments import graph,assignment_keys,audit
from test_contact_refinement import fixture


def contact(t,end,x=.5,kind='tap',nid=0):
    return dict(start=t,end=end,points=[[t,x,.5],[round(end-.001,3),x,.5]],
                kind=kind,note_ids=[nid],hand='left',finger='index',pointer=0,beat=t)


def plan(contacts):
    return dict(contacts=contacts,settings=vars(Settings(fingers=('index',),
                fingering_objective='load',speed_limits=False)),
                profile=default_profile(),physical_screen=[.28,.1575])


class LoadAssignmentTests(unittest.TestCase):
    def test_load_scores_and_assignments_ignore_travel_distance(self):
        a=[contact(i*.16,i*.16+.024,nid=i) for i in range(10)]
        p=plan(a); keys=assignment_keys(p)
        _,neighbors=graph(a,p)
        labels=assign_load(a,keys,p['settings'],neighbors,width=16)
        b=copy.deepcopy(a)
        for i,c in enumerate(b):
            c['points']=[[t,.1 if i%2 else .9,y] for t,x,y in c['points']]
        _,far_neighbors=graph(b,p)
        far=assign_load(b,keys,p['settings'],far_neighbors,width=16)
        self.assertEqual(list(labels),list(far))
        score,decisions=score_labels(a,keys,p['settings'],labels)
        self.assertEqual(score,score_labels(b,keys,p['settings'],labels)[0])
        self.assertLess(score,score_labels(a,keys,p['settings'],np.zeros(len(a),dtype=int))[0])
        self.assertTrue(all(d['costs']['movement']==0 for d in decisions))

    def test_hold_accumulator_matches_reference_and_occupancy_stays_hard(self):
        contacts=[contact(0.,1.8,kind='hold',nid=0),
                  *[contact(.2+i*.11,.224+i*.11,nid=i+1) for i in range(12)],
                  contact(2.,2.025,nid=13)]
        p=plan(contacts); keys=assignment_keys(p)
        _,neighbors=graph(contacts,p)
        labels=assign_load(contacts,keys,p['settings'],neighbors,width=16)
        self.assertTrue(all(label!=labels[0] for label in labels[1:-1]))
        for c,label in zip(contacts,labels):c['hand'],c['finger']=keys[label]
        self.assertFalse(audit(contacts,p['physical_screen'])['finger_overlaps'])

    def test_third_simultaneous_contact_cannot_use_a_disabled_finger(self):
        contacts=[contact(1.,1.1,x,nid=i) for i,x in enumerate((.2,.5,.8))]
        p=plan(contacts); keys=assignment_keys(p)
        _,neighbors=graph(contacts,p)
        with self.assertRaises(ValueError):
            assign_load(contacts,keys,p['settings'],neighbors)

    def test_speed_limits_are_optional_but_occupancy_is_not(self):
        chart,_=fixture(NoteType.TAP,[8.],[1.])
        old=contact(0.,.99,.1)
        new=contact(1.,1.02,.9,nid=1)
        new['points'][-1][1]=.1
        enabled=Planner(chart,Settings(),default_profile())
        disabled=Planner(chart,Settings(speed_limits=False),default_profile())
        state=State((old,))
        violations=enabled.constraints(new,state,-1,'index')[0]
        self.assertTrue({'finger_transfer','wrist_transfer','contact_speed'} & set(violations))
        self.assertFalse({'finger_transfer','wrist_transfer','contact_speed'} & set(disabled.constraints(new,state,-1,'index')[0]))
        old['end']=1.01
        self.assertIn('occupied',disabled.constraints(new,state,-1,'index')[0])

    def test_initial_load_objective_removes_hidden_movement_effort(self):
        chart,_=fixture(NoteType.TAP,[8.],[1.])
        p=Planner(chart,Settings(fingering_objective='load',speed_limits=False),default_profile())
        choices=p.choices(p.tasks[0],State((contact(.5,.6,.05),)),degraded=True)
        self.assertTrue(choices)
        for _,c in choices:
            self.assertEqual(c['effort'],.055)
            for name in ('movement','posture','visual','side','habit','preference','degraded'):
                self.assertEqual(c['decision']['costs'][name],0.)

    def test_load_mode_scores_burst_overload_instead_of_filtering_candidates(self):
        chart,_=fixture(NoteType.TAP,[8.],[1.])
        p=Planner(chart,Settings(fingers=('index',),fingering_objective='load',speed_limits=False),default_profile())
        choices=p.choices(p.tasks[0],State((contact(.95,.99,.5),)))
        overloaded=[c for _,c in choices if c['hand']=='left']
        self.assertTrue(overloaded)
        self.assertTrue(all(c['decision']['costs']['burst']>0 for c in overloaded))
        self.assertTrue(all(c['decision']['costs']['degraded']==0 for c in overloaded))

    def test_hold_tail_allows_same_beat_flick_without_losing_hold_time(self):
        from types import SimpleNamespace
        from contact_refinement import ContactGeometry,validate_contact
        chart,line=fixture(NoteType.HOLD,[8.],[1.],1.)
        line.notes.append(SimpleNamespace(type=NoteType.FLICK,seconds=2.,hold=0.,offset=complex(13.,0)))
        p=Planner(chart,Settings(fingers=('index',),fingering_objective='load',speed_limits=False),default_profile())
        hold=next(c for _,c in p.choices(p.tasks[0],State()) if c['hand']=='left')
        task=next(t for t in p.tasks if t.note.type==NoteType.FLICK)
        states=p.flick_after_hold(task,State((hold,)))
        self.assertTrue(states)
        for state in states:
            old,new=state.contacts
            self.assertEqual(old['end'],2.)
            self.assertEqual(new['start'],2.)
            self.assertEqual(old['finger'],new['finger'])
            self.assertEqual(old['hand'],new['hand'])
            self.assertGreater(__import__('math').dist(new['points'][0][1:],new['points'][-1][1:]),.05)
            for c in state.contacts:
                self.assertFalse(validate_contact(ContactGeometry(chart),c,c['points']))
        self.assertEqual(hold['end'],2.001)
        line.notes[0].hold=1.1
        p=Planner(chart,Settings(fingers=('index',),speed_limits=False),default_profile())
        hold=p.choices(p.tasks[0],State())[0][1]
        task=next(t for t in p.tasks if t.note.type==NoteType.FLICK)
        self.assertFalse(p.flick_after_hold(task,State((hold,))))

    def test_task_modes_are_isolated(self):
        from batch_task import configuration,common
        for key in ('DesultorySignals-AT-load','ExoplanetaryMirage-IN-index2'):
            c=configuration(key)
            self.assertEqual(c['fingering_objective'],'load')
            self.assertFalse(c['speed_limits'])
            self.assertIn('--no-speed-limits',common(dict(c,level='IN')))
        self.assertEqual(configuration('ExoplanetaryMirage-IN-index2')['allowed_fingers'],['index'])
        self.assertTrue(configuration('ExoplanetaryMirage')['speed_limits'])
        self.assertEqual(configuration('EntrancetotheChaos-IN-index2')['fingering_objective'],'balanced')


if __name__=='__main__':unittest.main()
