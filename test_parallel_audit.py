import contextlib
import io
import json
import os
from pathlib import Path
import tempfile
import unittest

from parallel_task import audit


class ParallelAuditTests(unittest.TestCase):
    def test_nullable_clearance_keeps_other_segments_and_seam_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            old = Path.cwd()
            try:
                os.chdir(directory)
                def write(path,value):
                    path.write_text(json.dumps(value))
                def state(t):
                    return dict(time=t,rigs={'hand':dict(bones={'bone':dict(head=[0,0,0],tail=[0,1,0])})})
                for i in range(8):
                    p=Path('baked')/str(i);p.mkdir(parents=True)
                    write(p/'provenance.json',dict(part=i,plan_sha256='same',commit='same'))
                    write(p/'seam-state.json',dict(first=state(i),join=state(i+1)))
                    write(p/'diagnostics.json',dict(contact_samples=0,contact_errors_over_1mm=[],collision_errors=[],
                        max_error_mm=0.,max_collision_depth_mm=0.,min_screen_clearance_mm=None if i%2==0 else .2))
                    write(p/'pose-numeric.json',dict(frames=60,worst_wrist_steps=[],worst_joint_steps=[],
                        finger_order_violations=[],projected_contacts=[],mesh_intersections=[]))
                with contextlib.redirect_stdout(io.StringIO()): audit('baked')
                self.assertEqual(json.loads(Path('output/full/diagnostics.json').read_text())['min_screen_clearance_mm'],.2)
                self.assertTrue(json.loads(Path('output/full/seam-validation.json').read_text())['passed'])
            finally:
                os.chdir(old)


if __name__=='__main__': unittest.main()
