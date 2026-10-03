"""Nearly identical phase/event times must not become zero-duration pose steps."""
import unittest
from handcam_blender import sample_times


class SamplingTests(unittest.TestCase):
    def test_parallel_prefix_samples_are_identical(self):
        contacts=[dict(start=34.731,end=34.933,points=[[34.731,.5,.5],[34.932,.5,.5]])]
        full=sample_times(dict(start=0.,duration=140.,frames=8400,fps=60,warmup=1.,contacts=contacts))
        for first in (1047,2095,3142,4190,5237,6285,7332):
            start=first/60
            part=sample_times(dict(start=start,duration=10.,frames=600,fps=60,warmup=start+1.,contacts=contacts))
            # Exclude only the final interval, where midpoint sampling requires
            # knowledge of the next sample beyond the segment's endpoint.
            self.assertEqual([t for t in full if t<start+9.9],[t for t in part if t<start+9.9])

    def test_segment_warmup_includes_earlier_motion_events(self):
        job=dict(start=20.,duration=1.,fps=60,frames=60,warmup=4.,
            contacts=[dict(start=17.123,end=17.234,points=[[17.123,.5,.5],[17.233,.5,.5]])])
        times=sample_times(job)
        self.assertEqual(times[0],16.)
        self.assertIn(17.123,times)
        self.assertIn(17.234,times)
        self.assertIn(20.,times)

    def test_near_duplicate_phase_keeps_exact_contact_event(self):
        job = dict(start=22.,duration=1.,fps=60,frames=60,
            contacts=[dict(start=22.3,end=22.4,points=[[22.3,.5,.5],[22.399,.5,.5]])],
            hand_rest=[dict(knots=[[22.299999999999997,0,0,0]])])
        times = sample_times(job)
        near = [t for t in times if abs(t-22.3)<1e-8]
        self.assertEqual(near,[22.3])
        self.assertTrue(all(b-a>1e-9 for a,b in zip(times,times[1:])))
        self.assertIn(22.399,times)
        self.assertIn(22.4,times)


if __name__=='__main__':
    unittest.main()
