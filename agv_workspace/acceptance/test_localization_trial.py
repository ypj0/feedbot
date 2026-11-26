import ast
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock


SOURCE_PATH = Path(__file__).with_name('localization_trial.py')


def functions(namespace):
    tree = ast.parse(SOURCE_PATH.read_text())
    tree.body = [item for item in tree.body if isinstance(item, ast.FunctionDef)
                 and item.name in ('container', 'run_container', 'main')]
    exec(compile(tree, str(SOURCE_PATH), 'exec'), namespace)
    return namespace


class LocalizationToolsTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        self.command = Mock(return_value='tool-container-id')
        self.namespace = functions(dict(
            trial=SimpleNamespace(DOCKER=['docker'], command=self.command),
            ownership=SimpleNamespace(DDS_ENVIRONMENT=['-e', 'ROS_DOMAIN_ID=89']),
            RELEASE=root / 'release', LOCALIZATION=root / 'localization',
            LIFECYCLE=root / 'lifecycle', SOURCE=root / 'source', TOOLS=root / 'tools',
            ROOT=root / 'results', TOOL_CONTAINER='feedbot-localization-tools',
            json=__import__('json'), time=__import__('time')))

    def test_one_detached_idle_container_has_all_evidence_and_tools(self):
        arguments = self.namespace['container'](
            'feedbot-localization-tools', self.namespace['ROOT'], 'exec sleep infinity', True)
        self.assertEqual(arguments[:5], ['docker', 'run', '--detach', '--init', '--rm'])
        self.assertIn(str(self.namespace['ROOT']) + ':/evidence', arguments)
        self.assertIn(str(self.namespace['TOOLS']) + ':/tools:ro', arguments)
        self.assertEqual(arguments[-1], 'source /opt/ros/jazzy/setup.bash; exec sleep infinity')
        self.assertNotIn('--device', arguments)

    def test_foreground_recording_container_keeps_existing_lifecycle(self):
        arguments = self.namespace['container'](
            'recorder', self.namespace['ROOT'], 'exec python3 -B /trial/record.py')
        self.assertNotIn('--init', arguments)
        self.assertNotIn('--detach', arguments)

    def test_three_stages_reuse_container_with_distinct_output_directories(self):
        for stage in ('seed', 'seed', 'convergence'):
            self.namespace['run_container']('stage', self.namespace['ROOT'] / stage,
                                            'exec python3 -B /tools/tool.py', 15, 'owned-id')
        for call, stage in zip(self.command.call_args_list, ('seed', 'seed', 'convergence')):
            self.assertEqual(call.args[1][:5], [
                'docker', 'exec', '--workdir', '/evidence/' + stage,
                'owned-id'])
            self.assertEqual(call.args[2], 15)

    def test_tool_failure_is_not_retried(self):
        self.command.side_effect = RuntimeError('tool failed')
        with self.assertRaisesRegex(RuntimeError, 'tool failed'):
            self.namespace['run_container']('stage', self.namespace['ROOT'] / 'seed', 'false', 15,
                                            'owned-id')
        self.assertEqual(self.command.call_count, 1)

    def prepare_inputs(self):
        for group, names in (('TOOLS', ('acquire_seed.py', 'converge.py')),
                             ('SOURCE', ('record.py', 'initialize.py')),
                             ('LOCALIZATION', ('start.sh',)),
                             ('LIFECYCLE', ('start_lifecycle.sh',))):
            self.namespace[group].mkdir()
            for name in names:
                (self.namespace[group] / name).touch()

    def test_base_start_failure_releases_owned_tool_container(self):
        self.prepare_inputs()

        def command(name, arguments, timeout):
            if name == 'start_base':
                raise RuntimeError('base start failed')
            return 'tool-container-id'

        self.command.side_effect = command
        self.assertEqual(self.namespace['main'](), 1)
        self.assertEqual([call.args[0] for call in self.command.call_args_list],
                         ['start_tools', 'start_base', 'stop_tools'])
        self.assertEqual(self.command.call_args_list[-1].args[1],
                         ['docker', 'stop', '-t', '5', 'tool-container-id'])

    def test_failed_tool_start_does_not_stop_unowned_container(self):
        self.prepare_inputs()
        self.command.side_effect = RuntimeError('container name in use')
        self.assertEqual(self.namespace['main'](), 1)
        self.assertEqual([call.args[0] for call in self.command.call_args_list], ['start_tools'])

    def test_tool_outputs_are_relative_to_exec_working_directory(self):
        tools = SOURCE_PATH.parent / 'localization_tools'
        for filename in ('acquire_seed.py', 'converge.py'):
            tree = ast.parse((tools / filename).read_text())
            paths = [node.args[0].value for node in ast.walk(tree)
                     if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                     and node.func.id == 'Path' and node.args
                     and isinstance(node.args[0], ast.Constant)]
            self.assertTrue(paths)
            self.assertTrue(all(not path.startswith('/') for path in paths))


if __name__ == '__main__':
    unittest.main()
