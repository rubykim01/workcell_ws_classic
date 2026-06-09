#!/usr/bin/env python3
"""Spawn multiple entities into Gazebo from a single process.

Each `ros2 run gazebo_ros spawn_entity.py` invocation starts its own Python
interpreter, rclpy node, and ROS 2 discovery cycle (~1-2 s of fixed overhead
per object). This helper pays that cost once: it opens a single rclpy session
and fires all /spawn_entity service calls from it. Objects are still spawned at
runtime via the service — nothing is baked into the world.

Usage:
    batch_spawn.py --obj FILE ENTITY X Y Z R P Y [--obj ...] [--timeout SEC]

R/P/Y are roll/pitch/yaw in radians (fixed-axis XYZ, same convention as
spawn_entity.py's -R/-P/-Y).
"""
import argparse
import math
import sys

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


def rpy_to_quat(r, p, y):
    """Fixed-axis XYZ (roll, pitch, yaw) -> (x, y, z, w). Matches tf2 'sxyz'."""
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main():
    parser = argparse.ArgumentParser(description="Batch-spawn entities into Gazebo.")
    parser.add_argument(
        "--obj", action="append", nargs=8, default=[],
        metavar=("FILE", "ENTITY", "X", "Y", "Z", "R", "P", "Y"),
        help="One object to spawn (repeatable).",
    )
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="Seconds to wait for the spawn service / each response.")
    args = parser.parse_args()

    if not args.obj:
        print("batch_spawn: no --obj given", file=sys.stderr)
        return 1

    rclpy.init()
    node = rclpy.create_node("batch_spawn")
    client = node.create_client(SpawnEntity, "/spawn_entity")
    if not client.wait_for_service(timeout_sec=args.timeout):
        node.get_logger().error("/spawn_entity service unavailable")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # Gazebo's /spawn_entity service is single-threaded server-side: models are
    # inserted one at a time no matter how the client sends requests. Firing them
    # all concurrently overflows the service queue and silently drops some (spawn
    # "stops in the middle"). So send each call and wait for its response before
    # the next. The speedup vs. the old scripts is from staying in ONE process
    # (no per-object interpreter + ROS discovery), not from parallel insertion.
    failed = 0
    for fpath, entity, x, y, z, R, P, Y in args.obj:
        try:
            with open(fpath) as fh:
                xml = fh.read()
        except OSError as e:
            node.get_logger().error(f"cannot read {fpath}: {e}")
            failed += 1
            continue
        req = SpawnEntity.Request()
        req.name = entity
        req.xml = xml
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        qx, qy, qz, qw = rpy_to_quat(float(R), float(P), float(Y))
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        req.initial_pose = pose

        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout)
        result = fut.result() if fut.done() else None
        if result is None:
            print(f"  FAILED {entity}: no response within {args.timeout}s")
            failed += 1
        elif not result.success:
            print(f"  spawn {entity}: {result.status_message}")
        else:
            print(f"  spawned {entity}")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
