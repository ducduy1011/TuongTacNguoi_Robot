#!/usr/bin/env python3
"""Check spawn requests without starting Gazebo or creating any real entity."""
import importlib.util
from pathlib import Path
from subprocess import CompletedProcess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


spawn = load('spawn_objects', ROOT / 'src/turtlebot4_btth5/scripts/spawn_objects.py')
launch = load('objects_launch', ROOT / 'src/turtlebot4_btth5/launch/objects.launch.py')


class ObjectSpawnTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.models = Path(self.temp.name)
        for _, folder, *_ in spawn.SPECS:
            path = self.models / folder / 'model.sdf'
            path.parent.mkdir(exist_ok=True)
            path.write_text('<sdf/>')

    def tearDown(self):
        self.temp.cleanup()

    def test_world_matches_tb4_maze(self):
        self.assertEqual(launch.default_world(), 'maze')

    def test_missing_service_never_sends_create(self):
        with patch.object(spawn.subprocess, 'run', return_value=CompletedProcess([], 0, '/world/other/create', '')) as run:
            with self.assertRaisesRegex(RuntimeError, '/world/maze/create is not available'):
                spawn.create_objects('maze', self.models)
            self.assertEqual(run.call_count, 1)

    def test_success_requires_a_true_response_for_every_object(self):
        responses = [CompletedProcess([], 0, '/world/maze/create\n', '')] + [
            CompletedProcess([], 0, 'data: true\n', '') for _ in spawn.SPECS]
        with patch.object(spawn.subprocess, 'run', side_effect=responses) as run:
            spawn.create_objects('maze', self.models)
            self.assertEqual(run.call_count, len(spawn.SPECS) + 1)
            for call in run.call_args_list[1:]:
                args = call.args[0]
                self.assertEqual(args[args.index('-s') + 1], '/world/maze/create')
                self.assertIn('allow_renaming: false', args[-1])

    def test_false_response_stops_sequence(self):
        responses = [CompletedProcess([], 0, '/world/maze/create\n', ''),
                     CompletedProcess([], 0, 'data: false\n', '')]
        with patch.object(spawn.subprocess, 'run', side_effect=responses) as run:
            with self.assertRaisesRegex(RuntimeError, 'Could not create btth5_person'):
                spawn.create_objects('maze', self.models)
            self.assertEqual(run.call_count, 2)

    def test_timeout_with_zero_exit_is_not_success(self):
        responses = [CompletedProcess([], 0, '/world/maze/create\n', ''),
                     CompletedProcess([], 0, '', 'Service call timed out')]
        with patch.object(spawn.subprocess, 'run', side_effect=responses):
            with self.assertRaisesRegex(RuntimeError, 'timed out'):
                spawn.create_objects('maze', self.models)


if __name__ == '__main__':
    unittest.main(verbosity=2)
