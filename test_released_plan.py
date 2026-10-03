import hashlib,json,tempfile,unittest,zipfile
from pathlib import Path
from unittest.mock import patch
from batch_task import configuration,released_plan


class ReleasedPlanTests(unittest.TestCase):
    def test_released_plan_checks_source_and_load_settings_before_validation(self):
        key='ExoplanetaryMirage-IN-index2'
        meta=dict(configuration(key),key=key,level='IN',source_sha256='abc')
        for mismatch in (False,True):
            with tempfile.TemporaryDirectory() as folder:
                root=Path(folder);(root/'inputs').mkdir()
                settings=dict(fingers=['index'],fingering_objective='load',speed_limits=mismatch,judgement_windows=True)
                data=json.dumps(dict(settings=settings)).encode()
                source=dict(meta,plan_sha256=hashlib.sha256(data).hexdigest())
                with zipfile.ZipFile(root/'inputs'/(key+'-plan.zip'),'w') as z:
                    z.writestr('source.json',json.dumps(source));z.writestr('motion-plan.json',data)
                import os
                previous=os.getcwd()
                try:
                    os.chdir(root)
                    with patch('batch_task.prepare',return_value=meta),patch('batch_task.run'),patch('batch_task.planning_stage') as validate:
                        if mismatch:
                            with self.assertRaises(ValueError):released_plan(key,'test-release')
                            validate.assert_not_called()
                        else:
                            released_plan(key,'test-release');validate.assert_called_once()
                finally:os.chdir(previous)


if __name__=='__main__':unittest.main()
