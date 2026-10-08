#!/usr/bin/env python3
"""Exercise a running Mobipick simulation via mobipick_api and report navigation metrics.

Run in the sourced simulation overlay, e.g.:
  python3 test_table_navigation.py --route 1 2 3 2 1 3 1 --timeout 120
Route entries are table numbers (base_table_<n>_pose) or other pose names from the running tables demo
(home, handover, ...); an unknown name fails before motion.

A leg fails when move_base does not report success, a recovery behaviour runs, the global plan is replaced
mid-leg, DWB rejects every trajectory in some control cycle, or the commanded velocity jumps by more than
--max-step between two cycles (the controller is supposed to ramp smoothly).
"""
import argparse
import json
import math
import threading
import time

import mobipick_api
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalID, GoalStatusArray
from dwb_msgs.msg import LocalPlanEvaluation
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from move_base_msgs.msg import RecoveryStatus


def distance_to_path(x, y, points):
    best = float("inf")
    for a, b in zip(points, points[1:]):
        dx, dy = b.x - a.x, b.y - a.y
        length_sq = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((x - a.x) * dx + (y - a.y) * dy) / length_sq)) if length_sq else 0.0
        best = min(best, math.hypot(x - a.x - t * dx, y - a.y - t * dy))
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--route", nargs="+", default=["1", "2", "3", "2", "1", "3", "1"])
    parser.add_argument("--timeout", type=float, default=120.0, help="Wall seconds per leg")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--namespace", default="/mobipick")
    parser.add_argument("--max-step", type=float, default=0.2, help="Max allowed cmd_vel change per cycle (m/s, rad/s)")
    args = parser.parse_args()
    if args.repeat < 1 or args.timeout <= 0:
        parser.error("--repeat and --timeout must be positive")
    ns = args.namespace.rstrip("/")
    robot = mobipick_api.Robot(ns)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    def get_pose():
        transform = buffer.lookup_transform("map", ns.strip("/") + "/base_footprint",
                                            rospy.Time(0), rospy.Duration(2)).transform
        p, q = transform.translation, transform.rotation
        return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    poses = rospy.get_param(ns + "/tables_demo_planning/poses")
    goals = [(name, poses[("base_table_%s_pose" if name.isdigit() else "base_%s_pose") % name]) for name in args.route]
    rospy.wait_for_message(ns + "/move_base/status", GoalStatusArray, timeout=30)
    cancel = rospy.Publisher(ns + "/move_base/cancel", GoalID, queue_size=1)
    state = {"path": [], "plans": 0, "cmd": (0.0, 0.0), "steps": [], "recoveries": 0, "no_legal": 0}

    def on_cmd(msg):
        previous = state["cmd"]
        state["cmd"] = (msg.linear.x, msg.angular.z)
        if abs(msg.linear.x - previous[0]) > args.max_step or abs(msg.angular.z - previous[1]) > args.max_step:
            state["steps"].append((round(previous[0], 2), round(msg.linear.x, 2), round(previous[1], 2), round(msg.angular.z, 2)))

    def on_plan(msg):
        state["path"] = [p.pose.position for p in msg.poses]
        state["plans"] += 1

    def on_evaluation(msg):
        if msg.twists and all(t.total < 0 for t in msg.twists):
            state["no_legal"] += 1
    rospy.Subscriber(ns + "/move_base_node/SBPLLatticePlanner/plan", Path, on_plan)
    rospy.Subscriber(ns + "/cmd_vel", Twist, on_cmd)
    rospy.Subscriber(ns + "/move_base_node/DWBLocalPlanner/evaluation", LocalPlanEvaluation, on_evaluation)
    rospy.Subscriber(ns + "/move_base/recovery_status", RecoveryStatus,
                     lambda msg: state.update(recoveries=state["recoveries"] + 1))
    failed = False
    for name, pose in goals * args.repeat:
        done = threading.Event()
        result = {}

        def finished(status, _result):
            result["status"] = status
            done.set()

        state.update(path=[], plans=0, steps=[], recoveries=0, no_legal=0, cmd=(0.0, 0.0))
        start = time.monotonic()
        sim_start = rospy.Time.now().to_sec()
        samples = []
        print(json.dumps({"event": "start", "table": name, "pose": pose}), flush=True)
        robot.base.move(pose=pose, done_cb=finished)
        next_report = start
        try:
            while not done.is_set() and time.monotonic() - start < args.timeout and not rospy.is_shutdown():
                x, y, yaw = get_pose()
                error = distance_to_path(x, y, state["path"])
                samples.append((state["cmd"][0], error))
                if time.monotonic() >= next_report:
                    print(json.dumps({"event": "progress", "table": name, "elapsed": round(time.monotonic() - start, 1),
                                      "pose": [round(x, 3), round(y, 3), round(yaw, 3)],
                                      "speed": round(state["cmd"][0], 3),
                                      "path_error": round(error, 3) if math.isfinite(error) else None}), flush=True)
                    next_report = time.monotonic() + 10
                done.wait(0.1)
        finally:
            if not done.is_set():
                cancel.publish(GoalID())
                done.wait(2)
                result["timeout"] = True
        # Let the final zero command and TF updates arrive before measuring.
        time.sleep(0.3)
        x, y, yaw = get_pose()
        q = pose[1]
        target_yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        errors = sorted(e for _, e in samples if math.isfinite(e))
        result.update(event="result", table=name, recoveries=state["recoveries"], plans=state["plans"],
                      no_legal_cycles=state["no_legal"], velocity_steps=state["steps"][:5],
                      wall_seconds=round(time.monotonic() - start, 2),
                      sim_seconds=round(rospy.Time.now().to_sec() - sim_start, 2),
                      xy_error=round(math.hypot(x - pose[0][0], y - pose[0][1]), 4),
                      yaw_error=round(abs(math.atan2(math.sin(yaw - target_yaw), math.cos(yaw - target_yaw))), 4),
                      peak_speed=round(max((abs(s) for s, _ in samples), default=0), 3),
                      mean_speed=round(sum(abs(s) for s, _ in samples) / max(1, len(samples)), 3),
                      max_path_error=round(max(errors, default=0), 4),
                      p95_path_error=round(errors[int(0.95 * (len(errors) - 1))], 4) if errors else None)
        result["ok"] = (result.get("status") == 3 and not result.get("timeout") and not state["recoveries"]
                        and state["plans"] <= 1 and not state["no_legal"] and not state["steps"])
        print(json.dumps(result), flush=True)
        failed = failed or not result["ok"]
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
