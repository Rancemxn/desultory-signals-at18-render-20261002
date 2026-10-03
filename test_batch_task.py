import unittest
from batch_task import interval


class BatchIntervalsTests(unittest.TestCase):
    def test_queue_respects_two_cloud_workflows(self):
        from batch_queue import available_slots
        self.assertEqual(available_slots([]),2)
        self.assertEqual(available_slots([dict(status='in_progress'),dict(status='completed')]),1)
        self.assertEqual(available_slots([dict(status='in_progress'),dict(status='queued')]),0)
        self.assertEqual(available_slots([dict(status='waiting')]*3),0)

    def test_eight_parts_cover_every_frame_exactly_once(self):
        for total in (8, 8380, 10342, 8221, 8661, 10725, 10498):
            parts = [interval({'frames':total},i) for i in range(8)]
            cursor = 0
            for start, frames in parts:
                self.assertEqual(round(start*60),cursor)
                self.assertGreater(frames,0)
                cursor += frames
            self.assertEqual(cursor,total)
            self.assertLessEqual(max(n for _,n in parts)-min(n for _,n in parts),1)


if __name__=='__main__': unittest.main()
