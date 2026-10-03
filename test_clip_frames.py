import unittest

from handcam import clip_frames


class ClipFramesTests(unittest.TestCase):
    def test_last_audio_frame_is_preserved(self):
        self.assertEqual(clip_frames(164.31666666666666,23.483333332333334,60,187.787188),1409)
        self.assertEqual(clip_frames(0.,187.787188,60,187.787188),11268)

    def test_full_extra_frame_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'extends beyond the audio'):
            clip_frames(164.31666666666666,23.5,60,187.787188)

    def test_exact_audio_frame_boundary_is_not_extended(self):
        self.assertEqual(clip_frames(1.,1.,60,2.),60)
        with self.assertRaises(ValueError):
            clip_frames(1.,1.001,60,2.)


if __name__=='__main__':
    unittest.main()
