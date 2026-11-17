"""Offline tests of the avoid controller (no ROS needed: rospy and msgs are stubbed).

Includes a closed-loop run in a kinematic model of the arena that reuses the
simulator's own lidar model (sim/psr_sim/geometry.py).

Run from the kit root:  python tests/test_avoid.py
"""
import importlib.util
import math
import os
import random
import sys
import types
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros", "psr_hero", "src"))
sys.path.insert(0, os.path.join(ROOT, "exercicios", "avoid"))

for name in ("rospy", "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg",
             "sensor_msgs", "sensor_msgs.msg"):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules["geometry_msgs.msg"].Twist = object
sys.modules["nav_msgs.msg"].Odometry = object
sys.modules["sensor_msgs.msg"].LaserScan = object

import avoid  # noqa: E402
from psr_hero.utils import V_MAX, W_MAX  # noqa: E402

# the simulator's lidar model, loaded by path (the package __init__ needs Genesis)
_spec = importlib.util.spec_from_file_location("psr_geometry", os.path.join(ROOT, "sim", "psr_sim", "geometry.py"))
geometry = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(geometry)

ARENA_SEGMENTS = np.array([(-1, -1, 1, -1), (1, -1, 1, 1), (1, 1, -1, 1), (-1, 1, -1, -1)], dtype=float)
CYLINDER = np.array([[0.0, 0.0, 0.15]])
COLLISION = 0.045


def simulate(cfg, seed, duration=300.0, start=(-0.7, 0.0, 0.0), dt=0.05, lag=0.15):
    """Unicycle with a first-order velocity lag. Returns dict of metrics."""
    ctrl = avoid.Controller(cfg, random.Random(seed))
    x, y, th = start
    v = w = 0.0
    t, dist, collisions, below = 0.0, 0.0, 0, False
    cells = set()
    while t < duration:
        rng = geometry.cast_beams(np.array([x, y]), th, CYLINDER, ARENA_SEGMENTS)
        cv, cw = ctrl.step(list(rng), t)
        a = dt / (lag + dt)
        v += a * (cv - v)
        w += a * (cw - w)
        nx, ny = x + v * math.cos(th) * dt, y + v * math.sin(th) * dt
        # keep the robot out of the solid shapes, like the physics would
        if abs(nx) < 1 - 0.037 and abs(ny) < 1 - 0.037 and math.hypot(nx, ny) > 0.15 + 0.037:
            dist += math.hypot(nx - x, ny - y)
            x, y = nx, ny
        th += w * dt
        t += dt
        m = ctrl.min_range
        if m < COLLISION and not below:
            collisions += 1
        below = m < COLLISION
        cells.add((int((x + 1) / 0.25), int((y + 1) / 0.25)))
    return {"collisions": collisions, "distance": dist, "coverage": len(cells) / 64.0}


class SanitizeTests(unittest.TestCase):
    def test_inf_nan_zero(self):
        last = [0.5] * 8
        raw = [float("inf"), float("nan"), 0.0, -1.0, 0.3, 0.9, 0.01, 0.2]
        out = avoid.sanitize(raw, last)
        self.assertEqual(out[0], 0.5)  # inf -> max range
        self.assertEqual(out[1], 0.5)  # nan, no history -> max range
        self.assertEqual(out[2], 0.5)  # zero, no history
        self.assertEqual(out[3], 0.5)
        self.assertEqual(out[4], 0.3)
        self.assertEqual(out[5], 0.5)  # above max
        self.assertEqual(out[6], 0.037)  # below min
        self.assertEqual(out[7], 0.2)

    def test_invalid_keeps_last_valid(self):
        last = [0.5] * 8
        avoid.sanitize([0.2] * 8, last)
        out = avoid.sanitize([float("nan")] * 8, last)
        self.assertEqual(out, [0.2] * 8)

    def test_short_list_is_safe(self):
        out = avoid.sanitize([0.3, 0.3], [0.5] * 8)
        self.assertEqual(len(out), 8)


class ControllerTests(unittest.TestCase):
    def ctrl(self, name="A"):
        return avoid.Controller(avoid.CONFIGS[name], random.Random(0))

    def test_free_space_goes_straight_at_cruise(self):
        for name, cfg in avoid.CONFIGS.items():
            v, w = self.ctrl(name).step([0.5] * 8, 0.0)
            self.assertAlmostEqual(v, cfg.v_cruise)
            self.assertLessEqual(abs(w), cfg.w_wander + 1e-12)  # only the slow wander

    def test_wander_changes_over_time_and_is_bounded(self):
        for name, cfg in avoid.CONFIGS.items():
            c = self.ctrl(name)
            ws = set()
            for i in range(400):  # 20 s
                _, w = c.step([0.5] * 8, i * 0.05)
                self.assertLessEqual(abs(w), cfg.w_wander + 1e-12)
                ws.add(round(w, 6))
            self.assertGreater(len(ws), 3)  # redrawn several times

    def test_wander_fades_when_obstacle_is_near(self):
        c = self.ctrl("B")
        c.wander, c.wander_until = 0.5, 1e9
        r = [0.5] * 8
        r[3] = 0.037  # p3 = 1: wander fully faded
        _, w = c.step(r, 0.0)
        c2 = self.ctrl("B")
        c2.wander, c2.wander_until = 0.0, 1e9
        _, w0 = c2.step(r, 0.0)
        self.assertAlmostEqual(w, w0)

    def test_obstacle_on_right_turns_left(self):
        r = [0.5] * 8
        r[3] = r[2] = 0.15  # right side
        v, w = self.ctrl().step(r, 0.0)
        self.assertGreater(w, 0.0)  # w > 0 is counter-clockwise = left

    def test_obstacle_on_left_turns_right(self):
        r = [0.5] * 8
        r[5] = r[6] = 0.15
        _, w = self.ctrl().step(r, 0.0)
        self.assertLess(w, 0.0)

    def test_front_blocked_turns_to_freer_side(self):
        r = [0.5] * 8
        r[4] = 0.1
        r[3] = 0.15  # right is tighter than left
        v, w = self.ctrl().step(r, 0.0)
        self.assertGreater(w, 0.0)
        self.assertLess(v, 0.02)  # almost stopped

    def test_symmetric_front_still_turns(self):
        r = [0.5] * 8
        r[4] = 0.1
        _, w = self.ctrl().step(r, 0.0)
        self.assertNotEqual(w, 0.0)

    def test_turn_direction_is_latched(self):
        c = self.ctrl()
        r = [0.5] * 8
        r[4] = 0.15
        r[3] = 0.2
        c.step(r, 0.0)
        self.assertEqual(c.turn_dir, 1)  # left is freer
        r[3], r[5] = 0.5, 0.2  # sides swap: without the latch the side would flip
        c.step(r, 0.05)
        self.assertEqual(c.turn_dir, 1)  # still latched while the front is blocked
        c.step([0.5] * 8, 0.1)
        self.assertEqual(c.turn_dir, 0)  # released when the front is clear

    def test_speed_never_negative_and_limits(self):
        rng = random.Random(3)
        for name in avoid.CONFIGS:
            c = self.ctrl(name)
            for i in range(2000):
                raw = [rng.choice([rng.uniform(0.0, 0.6), float("nan"), float("inf")]) for _ in range(8)]
                v, w = c.step(raw, i * 0.05)
                self.assertTrue(0.0 <= v <= V_MAX)
                self.assertLessEqual(abs(w), W_MAX)

    def test_escape_after_blocked_time(self):
        c = self.ctrl()
        r = [0.5] * 8
        r[4] = r[3] = r[5] = 0.06  # wedged
        t, modes = 0.0, set()
        while t < 4.0:
            v, w = c.step(r, t)
            modes.add(c.mode)
            t += 0.05
        self.assertIn("escape", modes)

    def test_no_odometry_in_controller(self):
        # the controller only receives lidar ranges and time
        import inspect
        self.assertEqual(list(inspect.signature(avoid.Controller.step).parameters), ["self", "raw_ranges", "t"])


class ClosedLoopTests(unittest.TestCase):
    def test_no_collisions_and_moves(self):
        for name, cfg in avoid.CONFIGS.items():
            for seed, start in enumerate([(-0.7, 0.0, 0.0), (0.5, 0.5, 2.0), (0.9, -0.9, 0.7)]):
                m = simulate(cfg, seed, duration=120.0, start=start)
                print(name, seed, {k: round(val, 2) for k, val in m.items()})
                self.assertEqual(m["collisions"], 0, "config %s seed %d" % (name, seed))
                self.assertGreater(m["distance"], 5.0, "robot barely moved (config %s)" % name)


if __name__ == "__main__":
    unittest.main(verbosity=2)
