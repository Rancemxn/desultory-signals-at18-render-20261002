import contextlib
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from parallel_task import audit,bake_compatibility


class ParallelAuditTests(unittest.TestCase):
    def test_mixed_commit_recovery_requires_matching_solver_inputs_and_rig(self):
        with tempfile.TemporaryDirectory() as directory:
            paths,provenance=[],[]
            for i in range(8):
                root=Path(directory)/str(i);root.mkdir()
                paths.append(root/'seam-state.json')
                provenance.append(dict(commit='a'*40 if i<7 else 'b'*40,psap_sha256='same',
                    full_contact_context=True,initialization_time=-1.,start=float(i),frames=60))
                job=dict(start=float(i),duration=1.,frames=60,warmup=i+1.,fps=60,
                         contacts=[dict(start=0.,end=.1)],output=str(root),contact_height=.0005)
                (root/'job.json').write_text(json.dumps(job))
                (root/'rig.json').write_text(json.dumps(dict(pads=[1,2])))
            with patch('parallel_task.solver_tree',return_value={'solver':'same'}):
                self.assertEqual(len(bake_compatibility(paths,provenance)['commits']),2)
                target=paths[-1].with_name('job.json');job=json.loads(target.read_text())
                job['contact_height']=.002;target.write_text(json.dumps(job))
                with self.assertRaisesRegex(ValueError,'different motion inputs'):
                    bake_compatibility(paths,provenance)
                job['contact_height']=.0005;target.write_text(json.dumps(job))
                paths[-1].with_name('rig.json').write_text(json.dumps(dict(pads=[1,3])))
                with self.assertRaisesRegex(ValueError,'different rig calibration'):
                    bake_compatibility(paths,provenance)
            with patch('parallel_task.solver_tree',side_effect=[{'solver':'a'},{'solver':'b'}]):
                with self.assertRaisesRegex(ValueError,'different solver'):
                    bake_compatibility(paths,provenance)

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
