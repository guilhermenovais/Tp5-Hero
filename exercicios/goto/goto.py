#!/usr/bin/env python3
"""Go-to-goal navigation (TP5, PSR - DCC/UFMG).

Drives the robot to the goal published in /hero_N/goal, then to the next one the
simulator draws. A goal counts when the robot centre gets within 6 cm of it.

Topics:
  /hero_N/odom     nav_msgs/Odometry          pose in the 'world' frame (x, y, yaw)
  /hero_N/goal     geometry_msgs/PointStamped current goal (world frame)
  /hero_N/cmd_vel  geometry_msgs/Twist        what we publish (linear.x, angular.z)

Control law (polar coordinates of the goal in the robot frame):
  rho   = hypot(xg - x, yg - y)
  alpha = wrap(atan2(yg - y, xg - x) - theta)          in [-pi, pi)
  w     = K_ALPHA * alpha
  v     = K_RHO * rho * max(cos(alpha), 0)             (0 if |alpha| > ALPHA_STOP)
  both saturated at the limits of the chosen configuration.

Configuration (A = conservative, B = aggressive), chosen by, in this order:
  ROS param ~config  (./run.sh run goto config:=B), env var CONFIG, default "A".

Log: goto_log_<CONFIG>.csv next to this script. The node stops by itself after
DURATION seconds (3 min), closes the file and publishes a zero velocity.

Run:  ./run.sh run goto [config:=A|B]      (see README.md in this folder)
"""
import csv
import math
import os
from collections import namedtuple

import rospy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry

from psr_hero.utils import V_MAX, W_MAX, clamp, pose_from_odom, wrap_angle

# --------------------------------------------------------------------------- parameters

CONTROL_RATE_HZ = 20.0  # control loop frequency
DURATION = 180.0  # s, length of the trial (the node stops by itself)
REACH_RADIUS = 0.06  # m, the simulator draws a new goal below this distance
RHO_DEADBAND = 0.005  # m, below this the heading is meaningless: do not turn
GOAL_CHANGE_EPS = 1e-6  # m, a goal moved by more than this is a new goal

Config = namedtuple("Config", "k_rho k_alpha v_max w_max alpha_stop")

CONFIGS = {
    # Conservative: slow, gentle turns, waits until well aligned before advancing.
    "A": Config(k_rho=1.0, k_alpha=2.0, v_max=0.10, w_max=2.0, alpha_stop=math.radians(60.0)),
    # Aggressive: full speed, fast turns, advances while still turning (cos factor only).
    "B": Config(k_rho=2.0, k_alpha=4.0, v_max=V_MAX, w_max=W_MAX, alpha_stop=math.radians(100.0)),
}

CSV_FIELDS = ["t", "x", "y", "theta", "xg", "yg", "rho", "alpha", "v", "w", "goal_reached"]


# --------------------------------------------------------------------------- control law


def goal_errors(pose, goal):
    """Distance rho (m) and heading error alpha (rad, in [-pi, pi)) from pose to goal."""
    x, y, theta = pose
    dx, dy = goal[0] - x, goal[1] - y
    rho = math.hypot(dx, dy)
    alpha = wrap_angle(math.atan2(dy, dx) - theta)
    return rho, alpha


def control_law(pose, goal, cfg):
    """Pure function: (pose, goal, Config) -> (rho, alpha, v, w)."""
    rho, alpha = goal_errors(pose, goal)
    if rho < RHO_DEADBAND:
        return rho, alpha, 0.0, 0.0
    w = cfg.k_alpha * alpha
    if abs(alpha) > cfg.alpha_stop:
        v = 0.0  # pure rotation until the goal is in front of the robot
    else:
        v = cfg.k_rho * rho * max(math.cos(alpha), 0.0)
    v = clamp(v, 0.0, min(cfg.v_max, V_MAX))
    w = clamp(w, -min(cfg.w_max, W_MAX), min(cfg.w_max, W_MAX))
    return rho, alpha, v, w


# --------------------------------------------------------------------------- node


class Goto:
    def __init__(self):
        rospy.init_node("goto")
        self.robot = rospy.get_param("~robot", 0)
        name = str(rospy.get_param("~config", "") or os.environ.get("CONFIG", "A")).strip().upper()
        if name not in CONFIGS:
            rospy.logwarn("unknown config '%s', using A", name)
            name = "A"
        self.config_name, self.cfg = name, CONFIGS[name]
        self.duration = float(rospy.get_param("~duration", DURATION))

        self.pose = None  # (x, y, yaw)
        self.goal = None  # (x, y)
        self.pending_reached = False  # the goal changed since the last logged row
        self.reached = 0
        self.goal_times = []  # seconds taken by each reached goal
        self.t0 = None
        self.last_goal_t = None

        here = os.path.dirname(os.path.realpath(__file__))
        self.log_path = os.path.join(here, "goto_log_%s.csv" % self.config_name)
        self.log_file = open(self.log_path, "w", newline="")
        self.log = csv.writer(self.log_file)
        self.log.writerow(CSV_FIELDS)

        # Ready: the publisher for the velocity command.
        self.pub = rospy.Publisher("/hero_%d/cmd_vel" % self.robot, Twist, queue_size=1)

        # TODO 1 (ROS): subscribers.
        rospy.Subscriber("/hero_%d/odom" % self.robot, Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber("/hero_%d/goal" % self.robot, PointStamped, self.on_goal, queue_size=1)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("goto: config %s %s, log in %s", self.config_name, self.cfg._asdict(), self.log_path)

    def on_odom(self, msg):
        self.pose = pose_from_odom(msg)

    def on_goal(self, msg):
        new = (msg.point.x, msg.point.y)
        if self.goal is not None and math.hypot(new[0] - self.goal[0], new[1] - self.goal[1]) > GOAL_CHANGE_EPS:
            # The simulator only draws a new goal when the previous one was reached.
            self.pending_reached = True
            self.reached += 1
            now = rospy.get_time()
            if self.last_goal_t is not None:
                self.goal_times.append(now - self.last_goal_t)
            self.last_goal_t = now
            rospy.loginfo("goal reached (%d), new goal (%.2f, %.2f)", self.reached, new[0], new[1])
        elif self.goal is None:
            self.last_goal_t = rospy.get_time()
        self.goal = new

    def compute_command(self, pose, goal):
        """TODO 2: control law. Returns (linear_x, angular_z); see control_law()."""
        _, _, v, w = control_law(pose, goal, self.cfg)
        return v, w

    def publish(self, v, w):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.pub.publish(cmd)

    def log_row(self, t, pose, goal, v, w):
        rho, alpha = goal_errors(pose, goal)
        self.log.writerow([
            "%.3f" % t, "%.4f" % pose[0], "%.4f" % pose[1], "%.4f" % pose[2],
            "%.4f" % goal[0], "%.4f" % goal[1], "%.4f" % rho, "%.4f" % alpha,
            "%.4f" % v, "%.4f" % w, int(self.pending_reached),
        ])
        self.pending_reached = False

    def shutdown(self):
        try:
            self.publish(0.0, 0.0)
        except Exception:  # the publisher may already be closed
            pass
        if not self.log_file.closed:
            self.log_file.close()
            mean = sum(self.goal_times) / len(self.goal_times) if self.goal_times else float("nan")
            rospy.loginfo("goto: %d goals reached, mean %.1f s per goal, log saved to %s",
                          self.reached, mean, self.log_path)

    def run(self):
        rate = rospy.Rate(CONTROL_RATE_HZ)
        while not rospy.is_shutdown():
            if self.pose is None or self.goal is None:
                rospy.logwarn_throttle(2.0, "waiting for pose and goal")
            else:
                now = rospy.get_time()
                if self.t0 is None:
                    self.t0 = now
                t = now - self.t0
                if t >= self.duration:
                    rospy.loginfo("goto: %.0f s trial finished", self.duration)
                    rospy.signal_shutdown("trial finished")
                    break
                pose, goal = self.pose, self.goal  # snapshot: callbacks run in other threads
                v, w = self.compute_command(pose, goal)
                self.publish(v, w)
                self.log_row(t, pose, goal, v, w)
            rate.sleep()


if __name__ == "__main__":
    Goto().run()
