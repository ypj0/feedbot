import importlib.util,pathlib,tempfile,types,unittest,sys
from unittest.mock import patch

class RestorationTest(unittest.TestCase):
    def load(self):
        stub=types.ModuleType('maintenance_guard');stub.current=lambda scan: {'fresh': True}
        with patch.dict(sys.modules,{'maintenance_guard':stub}):
            spec=importlib.util.spec_from_file_location('trial',pathlib.Path(__file__).with_name('stationary_trial.py'))
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module
    def test_recreates_observer_after_factory_start(self):
        m=self.load()
        with tempfile.TemporaryDirectory() as d:
            m.ROOT=pathlib.Path(d);m.MANAGER=m.ROOT/'manager';m.MANAGER.write_bytes(b'factory')
            m.SHA=m.hashlib.sha256(b'factory').hexdigest()
            with patch.object(m.subprocess,'check_call') as calls:
                m.restore()
            self.assertEqual([c.args[0] for c in calls.call_args_list],
                [['systemctl','start','spirit'],['systemctl','restart','feedbot-maintenance-observer']])
            self.assertTrue((m.ROOT/'restored.json').exists())
    def test_no_success_record_without_fresh_feedback(self):
        m=self.load()
        with tempfile.TemporaryDirectory() as d:
            m.ROOT=pathlib.Path(d)
            with patch.object(m.subprocess,'check_call'),patch.object(m.time,'monotonic',side_effect=[0,41]):
                with self.assertRaises(RuntimeError):m.restore()
            self.assertFalse((m.ROOT/'restored.json').exists())
if __name__=='__main__':unittest.main()
