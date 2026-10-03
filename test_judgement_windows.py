import copy
import unittest
from types import SimpleNamespace

from algo.algo5 import Planner,Settings,State
from basis import NoteType
from chord_sweep import outward_group,sweep_states
from contact_refinement import ContactGeometry,validate_contact
from handcam_motion import default_profile
from judgement_windows import check_offset,audit_sweeps
from test_contact_refinement import fixture


class JudgementWindowTests(unittest.TestCase):
    def test_outward_fan_uses_two_unbroken_index_contacts(self):
        for origin in (2.,58.75):
            chart,line=fixture(NoteType.TAP,[8.],[origin])
            line.notes += [SimpleNamespace(type=kind,seconds=origin,hold=0.,offset=complex(x,0))
                for kind,x in ((NoteType.DRAG,6.2),(NoteType.DRAG,4.4),(NoteType.FLICK,2.6),
                               (NoteType.DRAG,9.8),(NoteType.DRAG,11.6),(NoteType.FLICK,13.4))]
            p=Planner(chart,Settings(fingers=('index',),fingering_objective='load',speed_limits=False,
                                    judgement_windows=True),default_profile())
            group=outward_group(p,p.tasks[0]);self.assertIsNotNone(group)
            states=sweep_states(p,group,State());self.assertTrue(states)
            for state in states:
                self.assertEqual(len(state.contacts),2)
                self.assertEqual({c['finger'] for c in state.contacts},{'index'})
                self.assertEqual({n for c in state.contacts for n in c['note_ids']},set(range(7)))
                self.assertEqual(len(audit_sweeps(chart,state.contacts)),2)
                for c in state.contacts:
                    self.assertTrue(c['continuous_sweep'])
                    self.assertFalse(validate_contact(ContactGeometry(chart),c,c['points']))
                    for n,when in c['judgement_times'].items():check_offset(line.notes[int(n)],when)
            contacts=p.run()
            self.assertEqual(len(contacts),2)

    def test_sweep_does_not_release_unfinished_hold(self):
        chart,line=fixture(NoteType.TAP,[8.],[2.])
        line.notes += [SimpleNamespace(type=kind,seconds=2.,hold=0.,offset=complex(x,0))
            for kind,x in ((NoteType.DRAG,5.),(NoteType.FLICK,2.6),(NoteType.DRAG,11.),(NoteType.FLICK,13.4))]
        p=Planner(chart,Settings(fingers=('index',),fingering_objective='load',speed_limits=False,
                                judgement_windows=True),default_profile())
        occupied=[dict(hand=h,finger='index',start=1.,end=3.,kind='tap',note_ids=[],points=[[1.,.5,.5]])
                  for h in ('left','right')]
        self.assertFalse(sweep_states(p,outward_group(p,p.tasks[0]),State(tuple(occupied))))

    def test_two_parallel_tap_drag_flick_branches(self):
        chart,line=fixture(NoteType.TAP,[],[])
        line.notes=[SimpleNamespace(type=kind,seconds=3.,hold=0.,offset=complex(x,0))
            for kind,x in ((NoteType.FLICK,2.75),(NoteType.DRAG,4.85),(NoteType.TAP,6.95),
                           (NoteType.FLICK,9.05),(NoteType.DRAG,11.15),(NoteType.TAP,13.25))]
        p=Planner(chart,Settings(fingers=('index',),fingering_objective='load',speed_limits=False,
                                judgement_windows=True),default_profile())
        group=outward_group(p,p.tasks[0]);self.assertIsNotNone(group)
        states=sweep_states(p,group,State());self.assertTrue(states)
        self.assertEqual(len(states[0].contacts),2)
        self.assertEqual(len(audit_sweeps(chart,states[0].contacts)),2)

    def test_window_bounds_reject_late_tap(self):
        note=SimpleNamespace(type=NoteType.TAP,seconds=2.)
        self.assertEqual(check_offset(note,2.06)['window_ms'],80)
        with self.assertRaises(ValueError):check_offset(note,2.061)


if __name__=='__main__':unittest.main()
