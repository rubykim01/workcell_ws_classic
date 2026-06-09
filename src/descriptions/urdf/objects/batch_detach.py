#!/usr/bin/env python3
"""Detach multiple link pairs via the /DETACHLINK service from one process.

Companion to batch_attach.py. Detach failures are non-fatal (a pair may simply
not be attached), so they are reported but never cause a non-zero exit.

Usage:
    batch_detach.py --detach MODEL1 LINK1 MODEL2 LINK2 [--detach ...] [--timeout SEC]
"""
import argparse
import sys

import rclpy
from linkattacher_msgs.srv import DetachLink


def main():
    parser = argparse.ArgumentParser(description="Batch /DETACHLINK calls.")
    parser.add_argument(
        "--detach", action="append", nargs=4, default=[],
        metavar=("MODEL1", "LINK1", "MODEL2", "LINK2"),
        help="One link pair to detach (repeatable).",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    if not args.detach:
        return 0

    rclpy.init()
    node = rclpy.create_node("batch_detach")
    client = node.create_client(DetachLink, "/DETACHLINK")
    if not client.wait_for_service(timeout_sec=args.timeout):
        node.get_logger().error("/DETACHLINK service unavailable")
        node.destroy_node()
        rclpy.shutdown()
        return 0  # detach is best-effort

    for m1, l1, m2, l2 in args.detach:
        req = DetachLink.Request()
        req.model1_name = m1
        req.link1_name = l1
        req.model2_name = m2
        req.link2_name = l2
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout)
        result = fut.result() if fut.done() else None
        if result is None:
            print(f"  detach {m2} from {m1}: no response")
        elif not result.success:
            print(f"  detach {m2} from {m1} (ignored): {result.message}")
        else:
            print(f"  detached {m2} from {m1}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
