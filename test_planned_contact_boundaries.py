"""The animation assignment must preserve PSAP's half-open contact lifetimes."""
import unittest

from handcam_blender import assign_contacts, finger_sample, world_point


class PlannedContactBoundaryTests(unittest.TestCase):
    def contacts(self):
        return [dict(pointer=5,hand='right',finger='index',planned=True,
                     start=start,end=end,points=[[start,x,.5]],
                     prepare=start,release_until=end,lift=.01)
                for start,end,x in ((1.,1.1,.4),(1.1,1.2,.6))]

    def job(self,contacts):
        return dict(motion_plan_version=1,contacts=contacts,screen=[.28,.1575],
                    contact_height=.0005)

    def test_same_time_up_down_keeps_both_contacts_and_samples_new_down(self):
        contacts=self.contacts()
        job=self.job(list(reversed(contacts)))
        tracks=assign_contacts(job,{(1,'index'):(0.,0.,0.)})
        self.assertEqual(tracks[(1,'index')],contacts)
        position,_,down=finger_sample(tracks[(1,'index')],1.1,(0.,0.,.04),job)
        self.assertTrue(down)
        self.assertEqual(position,(*world_point(job,(.6,.5)),job['contact_height']))

    def test_real_overlap_still_rejected(self):
        contacts=self.contacts()
        contacts[0]['end']=1.101
        with self.assertRaisesRegex(ValueError,'Overlapping planned contacts'):
            assign_contacts(self.job(contacts),{(1,'index'):(0.,0.,0.)})


if __name__=='__main__':
    unittest.main()
