#!/usr/bin/env python3
"""Readiness RPCs must retry slowly, without aborting normal Nav2 startup."""
from concurrent.futures import Future
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('prepare', ROOT / 'src/turtlebot4_btth5/scripts/prepare_robot.py')
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)


class Client:
    def __init__(self):
        self.calls = []
    def service_is_ready(self):
        return True
    def call_async(self, request):
        future = Future()
        self.calls.append(future)
        return future


class PreparationTest(unittest.TestCase):
    def setUp(self):
        self.state = SimpleNamespace(readiness_futures={}, readiness_values={}, readiness_poll_times={})
        self.client = Client()
    def poll(self, now):
        with patch.object(prepare.time, 'monotonic', return_value=now):
            return prepare.PrepareRobot.poll_service(self.state, 'test', self.client,
                                                     lambda: None, lambda response: response == 'active')
    def test_slow_startup_does_not_abort_after_five_seconds(self):
        for now in [100,101,105,110,114]:
            self.assertFalse(self.poll(now))
        self.assertEqual(len(self.client.calls), 1)
        self.assertFalse(self.poll(116))
        self.assertEqual(len(self.client.calls), 2)
        self.assertTrue(self.client.calls[0].cancelled())
    def test_active_reply_allows_start(self):
        self.assertFalse(self.poll(100))
        self.client.calls[0].set_result('active')
        self.assertTrue(self.poll(100.2))
    def test_inactive_reply_cannot_flood_service(self):
        self.poll(100)
        self.client.calls[0].set_result('inactive')
        for _ in range(20):
            self.assertFalse(self.poll(100.2))
        self.assertEqual(len(self.client.calls), 1)
        self.poll(101.2)
        self.assertEqual(len(self.client.calls), 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
