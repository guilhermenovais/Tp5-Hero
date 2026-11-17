"""Offline tests of the goto control law (no ROS needed: rospy and msgs are stubbed).

Run from the kit root:  python tests/test_goto.py
"""
import math
import os
import sys
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros", "psr_hero", "src"))
sys.path.insert(0, os.path.join(ROOT, "exercicios", "goto"))

for name in ("rospy", "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg"):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules["geometry_msgs.msg"].PointStamped = sys.modules["geometry_msgs.msg"].Twist = object
sys.modules["nav_msgs.msg"].Odometry = object

import goto  # noqa: E402
from psr_hero.utils import V_MAX, W_MAX, yaw_from_quaternion  # noqa: E402


class Q:  # minimal quaternion
    def __init__(self, yaw):
        self.x = self.y = 0.0
        self.z, self.w = math.sin(yaw / 2), math.cos(yaw / 2)


class GotoTests(unittest.TestCase):
    def test_yaw_from_quaternion(self):
        for yaw in (-3.0, -1.0, 0.0, 0.5, 3.0):
            self.assertAlmostEqual(yaw_from_quaternion(Q(yaw)), yaw)

    def test_errors_basic(self):
        rho, alpha = goto.goal_errors((0, 0, 0), (1, 0))
        self.assertAlmostEqual(rho, 1.0)
        self.assertAlmostEqual(alpha, 0.0)

    def test_alpha_sign_goal_on_left(self):
        _, alpha = goto.goal_errors((0, 0, 0), (0, 1))
        self.assertAlmostEqual(alpha, math.pi / 2)  # goal to the left -> positive -> w > 0 (CCW)
        _, _, _, w = goto.control_law((0, 0, 0), (0, 1), goto.CONFIGS["A"])
        self.assertGreater(w, 0)

    def test_alpha_wraps_across_pi(self):
        # heading 170 deg, goal at -170 deg: the short way is +20 deg, not -340
        _, alpha = goto.goal_errors((0, 0, math.radians(170)), (math.cos(math.radians(-170)), math.sin(math.radians(-170))))
        self.assertAlmostEqual(alpha, math.radians(20))
        for th in [i * 0.37 for i in range(-20, 21)]:
            _, a = goto.goal_errors((0.1, -0.2, th), (-0.5, 0.6))
            self.assertTrue(-math.pi <= a < math.pi)

    def test_saturation(self):
        for cfg in goto.CONFIGS.values():
            for pose in [(0, 0, 0), (0, 0, 3.0), (0, 0, -2.0)]:
                _, _, v, w = goto.control_law(pose, (5.0, 5.0), cfg)
                self.assertTrue(0.0 <= v <= min(cfg.v_max, V_MAX) + 1e-12)
                self.assertLessEqual(abs(w), min(cfg.w_max, W_MAX) + 1e-12)

    def test_turn_in_place_when_goal_behind(self):
        for cfg in goto.CONFIGS.values():
            _, _, v, w = goto.control_law((0, 0, 0), (-1.0, 0.01), cfg)
            self.assertEqual(v, 0.0)
            self.assertNotEqual(w, 0.0)

    def test_goal_at_robot_position_is_safe(self):
        rho, alpha, v, w = goto.control_law((0.3, 0.3, 1.0), (0.3, 0.3), goto.CONFIGS["B"])
        self.assertEqual((rho, v, w), (0.0, 0.0, 0.0))

    def test_aligned_goal_goes_straight(self):
        _, _, v, w = goto.control_law((0, 0, 0), (0.5, 0), goto.CONFIGS["B"])
        self.assertAlmostEqual(v, V_MAX)
        self.assertAlmostEqual(w, 0.0)

    def test_closed_loop_reaches_goals(self):
        """Unicycle kinematics at 20 Hz: both configs must reach random goals within 6 cm."""
        import random
        rng = random.Random(1)
        for cfg in goto.CONFIGS.values():
            x, y, th = -0.6, -0.6, 0.0
            for _ in range(5):
                goal = (rng.uniform(-0.75, 0.75), rng.uniform(-0.75, 0.75))
                for step in range(20 * 120):
                    rho, _, v, w = goto.control_law((x, y, th), goal, cfg)
                    if rho < goto.REACH_RADIUS:
                        break
                    dt = 0.05
                    x += v * math.cos(th) * dt
                    y += v * math.sin(th) * dt
                    th += w * dt
                else:
                    self.fail("goal %s not reached with %s" % (goal, cfg))


if __name__ == "__main__":
    unittest.main(verbosity=2)
