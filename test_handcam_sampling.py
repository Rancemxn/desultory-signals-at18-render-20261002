"""Nearly identical phase/event times must not become zero-duration pose steps."""
import unittest
from handcam_blender import sample_times


class SamplingTests(unittest.TestCase):
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
