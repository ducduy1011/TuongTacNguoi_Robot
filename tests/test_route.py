#!/usr/bin/env python3
"""Offline clearance/connectivity check for the unchanged map and 8 objects."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('scene', ROOT / 'src/turtlebot4_btth5/scripts/spawn_objects.py')
scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene)


class RouteTest(unittest.TestCase):
    def check_connected(self, points, wall_clearance, object_exclusion):
        path = ROOT / 'src/turtlebot4_btth4/maps/map.yaml'
        metadata = yaml.safe_load(path.read_text())
        pixels = np.array(Image.open(path.parent / metadata['image']))
        resolution = metadata['resolution']
        distance = distance_transform_edt(np.pad(pixels == 254, 1))[1:-1, 1:-1] * resolution
        free = distance > wall_clearance
        rows, cols = np.indices(free.shape)
        def cell(x, y):
            return int((x-metadata['origin'][0])/resolution), pixels.shape[0]-1-int((y-metadata['origin'][1])/resolution)
        self.assertEqual(len(scene.SPECS), 8)
        self.assertEqual(len({p[0] for p in scene.SPECS}), 8)
        for name, _, world_x, world_y, _ in scene.SPECS:
            # Approximate existing map/world offset, not a localization algorithm.
            c, r = cell(world_x + 2, world_y)
            self.assertGreaterEqual(distance[r, c], .5, name)
            free[(cols-c)**2 + (rows-r)**2 < (object_exclusion/resolution)**2] = False
        components, _ = label(free)
        group = None
        for x, y in points:
            c, r = cell(x, y)
            self.assertTrue(0 <= c < pixels.shape[1] and 0 <= r < pixels.shape[0])
            self.assertGreaterEqual(distance[r, c], .65)
            self.assertNotEqual(components[r, c], 0)
            if group is None:
                group = components[r, c]
            self.assertEqual(components[r, c], group)

    def test_previous_wide_route_stays_connected(self):
        self.check_connected([(0., 0.), (-5., -2.), (-3., -8.), (6., -8.)], .65, .9)

    def test_selected_goals_fit_narrower_passages(self):
        goals = yaml.safe_load((ROOT / 'src/turtlebot4_btth5/config/goals.yaml').read_text())['goals']
        # Robot radius .25 m plus .20 m wall margin. The selected route uses
        # narrower passages than the earlier .65 m clearance reference route.
        # .75 m is an approximate object-centre exclusion, not exact mesh geometry.
        # This offline check does not alter Nav2 radius/inflation/safety settings.
        self.check_connected([(0., 0.)] + [(g['x'], g['y']) for g in goals], .45, .75)


if __name__ == '__main__':
    unittest.main(verbosity=2)
