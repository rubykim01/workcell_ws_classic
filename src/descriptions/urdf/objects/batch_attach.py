#!/usr/bin/env python3
"""Attach multiple link pairs via the /ATTACHLINK service from one process.

Like batch_spawn.py, this exists to avoid paying the ~1-2 s Python-interpreter +
ROS-discovery startup of a separate `ros2 service call` per attachment. The
linkattacher service is processed serially in Gazebo, so calls are sent one at a
time (wait for each response before the next); the win is staying in one process.

Usage:
    batch_attach.py --attach MODEL1 LINK1 MODEL2 LINK2 [--attach ...] [--timeout SEC]
"""
import argparse
import sys

import rclpy
from linkattacher_msgs.srv import AttachLink


def main():
    parser = argparse.ArgumentParser(description="Batch /ATTACHLINK calls.")
    parser.add_argument(
        "--attach", action="append", nargs=4, default=[],
        metavar=("MODEL1", "LINK1", "MODEL2", "LINK2"),
        help="One link pair to attach (repeatable).",
    )
    parser.add_argument("--timeout", type=float, default=20.0,
                        help="Seconds to wait for the service / each response.")
    args = parser.parse_args()

    if not args.attach:
        print("batch_attach: no --attach given", file=sys.stderr)
        return 1

    rclpy.init()
    node = rclpy.create_node("batch_attach")
    client = node.create_client(AttachLink, "/ATTACHLINK")
    if not client.wait_for_service(timeout_sec=args.timeout):
        node.get_logger().error("/ATTACHLINK service unavailable")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    failed = 0
    for m1, l1, m2, l2 in args.attach:
        req = AttachLink.Request()
        req.model1_name = m1
        req.link1_name = l1
        req.model2_name = m2
        req.link2_name = l2
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout)
        result = fut.result() if fut.done() else None
        if result is None:
            print(f"  FAILED attach {m2} -> {m1}: no response within {args.timeout}s")
            failed += 1
        elif not result.success:
            print(f"  attach {m2} -> {m1}: {result.message}")
            failed += 1
        else:
            print(f"  attached {m2} -> {m1}")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
