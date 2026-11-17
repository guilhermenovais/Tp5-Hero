#!/usr/bin/env python3
"""Obstacle avoidance with the 8-beam lidar (TP5, PSR - DCC/UFMG).

Keeps the robot moving in the arena without touching the cylinder or the walls,
using ONLY the lidar (/hero_N/laser). The odometry is subscribed just to log the
trajectory: it never enters the control law.

Lidar: beam k points at -pi + k*pi/4 from the heading (CCW positive): 4 = front,
3/5 = 45 deg right/left, 2/6 = 90 deg right/left, 0 = back. 0.5 m = nothing seen.

Strategy: Braitenberg-style reactive control + a stuck-recovery layer.

  p_k   = clamp((D_INFLUENCE - d_k) / (D_INFLUENCE - 0.037), 0, 1)     proximity of beam k
  w     = K_W * (W_DIAG * (p3 - p5) + W_SIDE * (p2 - p6)) + turn_dir * W_FRONT * p4
          (obstacle on the right -> w > 0 -> turn left; obstacle on the left -> w < 0)
  d_fwd = min(d4, d3*cos45, d5*cos45)                                  clearance ahead
  v     = V_CRUISE * clamp((d_fwd - D_STOP) / (D_SLOW - D_STOP), 0, 1)

  turn_dir is latched when something is straight ahead (towards the freer side,
  random on a tie) and released when the front is clear: avoids dithering.
  Stuck recovery: if d_fwd < D_BLOCK for T_BLOCK seconds, spin in place at
  W_ESCAPE for T_ESCAPE seconds in a random direction.
  Wander: w += wander * (1 - max(p2..p6)), wander uniform in [-W_WANDER, W_WANDER]
  redrawn every 2-4 s. Without it the robot settles into a periodic orbit.

Invalid readings (nan, <= 0) reuse the last valid value of that beam; inf and
values above 0.5 become 0.5; values below 0.037 become 0.037.

Configuration (A = cautious, B = fast), chosen by ROS param ~config
(./run.sh run avoid config:=B), env var CONFIG, default "A".

Log: avoid_log_<CONFIG>.csv next to this script; the node stops by itself after
DURATION seconds (5 min).

Run:  ./run.sh run avoid [config:=A|B]      (see README.md in this folder)
"""
import csv
import math
import os
import random
from collections import namedtuple

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from psr_hero.utils import (V_MAX, W_MAX, LIDAR_BEAMS, LIDAR_RANGE_MAX, LIDAR_RANGE_MIN, clamp,
                            yaw_from_quaternion)

# --------------------------------------------------------------------------- parameters

CONTROL_RATE_HZ = 20.0  # control loop frequency
DURATION = 300.0  # s, length of the trial (the node stops by itself)
TIE_EPS = 0.02  # m, left/right clearance difference below which the turn side is random
COS45 = math.cos(math.pi / 4.0)

Config = namedtuple(
    "Config",
    "v_cruise k_w w_diag w_side w_front d_influence d_slow d_stop d_block t_block t_escape w_escape w_wander",
)
WANDER_PERIOD = (2.0, 4.0)  # s, the wander value is redrawn at random intervals in this range

CONFIGS = {
    # Cautious: slow, reacts early (wide influence zone), gentle steering.
    "A": Config(v_cruise=0.10, k_w=4.0, w_diag=1.0, w_side=0.5, w_front=2.5,
                d_influence=0.35, d_slow=0.30, d_stop=0.10, d_block=0.12,
                t_block=1.5, t_escape=1.0, w_escape=2.5, w_wander=0.3),
    # Fast: full speed, reacts late (narrow influence zone), hard steering.
    "B": Config(v_cruise=0.20, k_w=8.0, w_diag=1.0, w_side=0.5, w_front=4.0,
                d_influence=0.25, d_slow=0.22, d_stop=0.09, d_block=0.10,
                t_block=1.0, t_escape=0.8, w_escape=4.0, w_wander=0.5),
}

CSV_FIELDS = ["t", "x", "y", "theta", "min_range", "v", "w", "mode"]


# --------------------------------------------------------------------------- control


def sanitize(raw, last):
    """Clean 8 raw lidar values. `last` (list of 8) keeps the last valid value per beam
    and is updated in place. Returns a new list of 8 floats in [0.037, 0.5]."""
    out = []
    for k in range(LIDAR_BEAMS):
        d = raw[k] if k < len(raw) else float("nan")
        try:
            d = float(d)
        except (TypeError, ValueError):
            d = float("nan")
        if math.isnan(d) or d <= 0.0:
            d = last[k]  # no reading: keep what we knew
        elif d > LIDAR_RANGE_MAX:  # includes +inf: nothing in range
            d = LIDAR_RANGE_MAX
        else:
            d = max(d, LIDAR_RANGE_MIN)
        last[k] = d
        out.append(d)
    return out


class Controller:
    """Lidar-only controller. Pure Python (no ROS), so it can be tested offline."""

    def __init__(self, cfg, rng=None):
        self.cfg = cfg
        self.rng = rng or random.Random()
        self.last = [LIDAR_RANGE_MAX] * LIDAR_BEAMS
        self.turn_dir = 0  # latched steering side for obstacles straight ahead (+1 = left)
        self.blocked_since = None
        self.escape_until = None
        self.escape_dir = 1
        self.wander = 0.0  # slow random turn rate (rad/s) that breaks periodic orbits
        self.wander_until = None
        self.mode = "cruise"
        self.min_range = LIDAR_RANGE_MAX

    def prox(self, d):
        c = self.cfg
        return clamp((c.d_influence - d) / (c.d_influence - LIDAR_RANGE_MIN), 0.0, 1.0)

    def step(self, raw_ranges, t):
        """raw_ranges: 8 lidar values, t: time (s). Returns (v, w)."""
        c = self.cfg
        d = sanitize(raw_ranges, self.last)
        self.min_range = min(d)
        p = [self.prox(x) for x in d]
        d_fwd = min(d[4], d[3] * COS45, d[5] * COS45)

        # --- stuck recovery: spin in place in a random direction
        if self.escape_until is not None:
            if t < self.escape_until:
                self.mode = "escape"
                return 0.0, self.escape_dir * min(c.w_escape, W_MAX)
            self.escape_until = None
            self.blocked_since = None
        if d_fwd < c.d_block:
            if self.blocked_since is None:
                self.blocked_since = t
            elif t - self.blocked_since >= c.t_block:
                self.escape_until = t + c.t_escape
                self.escape_dir = self.rng.choice((-1, 1))
                self.turn_dir = 0
                self.mode = "escape"
                return 0.0, self.escape_dir * min(c.w_escape, W_MAX)
        else:
            self.blocked_since = None

        # --- steering side for an obstacle straight ahead (latched, released when clear)
        if d_fwd >= c.d_slow:
            self.turn_dir = 0
        elif self.turn_dir == 0 and p[4] > 0.0:
            left, right = d[5] + d[6], d[3] + d[2]
            if abs(left - right) < TIE_EPS:
                self.turn_dir = self.rng.choice((-1, 1))
            else:
                self.turn_dir = 1 if left > right else -1

        # --- Braitenberg steering + speed
        w = c.k_w * (c.w_diag * (p[3] - p[5]) + c.w_side * (p[2] - p[6]))
        w += self.turn_dir * c.w_front * p[4]
        v = c.v_cruise * clamp((d_fwd - c.d_stop) / (c.d_slow - c.d_stop), 0.0, 1.0)
        self.mode = "avoid" if (self.turn_dir != 0 or abs(w) > 1e-6) else "cruise"

        # --- slow random wander, faded out as any front/side beam sees an obstacle
        if self.wander_until is None or t >= self.wander_until:
            self.wander = self.rng.uniform(-c.w_wander, c.w_wander)
            self.wander_until = t + self.rng.uniform(*WANDER_PERIOD)
        w += self.wander * (1.0 - max(p[2:7]))
        return clamp(v, 0.0, V_MAX), clamp(w, -W_MAX, W_MAX)


# --------------------------------------------------------------------------- node


class Avoid:
    def __init__(self):
        rospy.init_node("avoid")
        self.robot = rospy.get_param("~robot", 0)
        name = str(rospy.get_param("~config", "") or os.environ.get("CONFIG", "A")).strip().upper()
        if name not in CONFIGS:
            rospy.logwarn("unknown config '%s', using A", name)
            name = "A"
        self.config_name, self.cfg = name, CONFIGS[name]
        self.duration = float(rospy.get_param("~duration", DURATION))
        seed = rospy.get_param("~seed", None)
        self.controller = Controller(self.cfg, random.Random(seed))

        self.scan = None
        self.odom = None  # (x, y, yaw): ONLY for the log, never used by the controller
        self.t0 = None

        here = os.path.dirname(os.path.realpath(__file__))
        self.log_path = os.path.join(here, "avoid_log_%s.csv" % self.config_name)
        self.log_file = open(self.log_path, "w", newline="")
        self.log = csv.writer(self.log_file)
        self.log.writerow(CSV_FIELDS)

        # Ready: the publisher for the velocity command.
        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)

        # TODO 1 (ROS): subscribers (odometry is for the log only).
        rospy.Subscriber("/hero_%d/laser" % self.robot, LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_odom, queue_size=1)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("avoid: config %s %s, log in %s", self.config_name, self.cfg._asdict(), self.log_path)

    def on_scan(self, scan):
        self.scan = scan

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def compute_command(self, ranges, t=None):
        """TODO 2: the 8 lidar ranges -> (linear_x, angular_z). See Controller.step()."""
        return self.controller.step(ranges, rospy.get_time() if t is None else t)

    def publish(self, v, w):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.pub.publish(cmd)

    def log_row(self, t, v, w):
        x, y, th = self.odom if self.odom is not None else (float("nan"),) * 3
        self.log.writerow(["%.3f" % t, "%.4f" % x, "%.4f" % y, "%.4f" % th,
                           "%.4f" % self.controller.min_range, "%.4f" % v, "%.4f" % w,
                           self.controller.mode])

    def shutdown(self):
        try:
            self.publish(0.0, 0.0)
        except Exception:  # the publisher may already be closed
            pass
        if not self.log_file.closed:
            self.log_file.close()
            rospy.loginfo("avoid: log saved to %s", self.log_path)

    def run(self):
        rate = rospy.Rate(CONTROL_RATE_HZ)
        while not rospy.is_shutdown():
            if self.scan is None:
                rospy.logwarn_throttle(2.0, "no lidar data yet")
            else:
                now = rospy.get_time()
                if self.t0 is None:
                    self.t0 = now
                t = now - self.t0
                if t >= self.duration:
                    rospy.loginfo("avoid: %.0f s trial finished", self.duration)
                    rospy.signal_shutdown("trial finished")
                    break
                v, w = self.compute_command(list(self.scan.ranges), now)
                self.publish(v, w)
                self.log_row(t, v, w)
            rate.sleep()


if __name__ == "__main__":
    Avoid().run()
