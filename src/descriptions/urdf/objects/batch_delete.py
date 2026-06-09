#!/usr/bin/env python3
"""Delete multiple entities from Gazebo via /delete_entity from one process.

Avoids the ~1-2 s Python-interpreter + ROS-discovery startup of a separate
`ros2 service call` per entity. /delete_entity is processed serially in Gazebo,
so calls are sent one at a time; the win is staying in one process.

Usage:
    batch_delete.py NAME [NAME ...] [--timeout SEC]
"""
import sys

import rclpy
from gazebo_msgs.srv import DeleteEntity


def main():
    argv = sys.argv[1:]
    timeout = 15.0
    if "--timeout" in argv:
        i = argv.index("--timeout")
        try:
            timeout = float(argv[i + 1])
        except (IndexError, ValueError):
            pass
        del argv[i:i + 2]
    names = [a for a in argv if a]
    if not names:
        return 0

    rclpy.init()
    node = rclpy.create_node("batch_delete")
    client = node.create_client(DeleteEntity, "/delete_entity")
    if not client.wait_for_service(timeout_sec=timeout):
        node.get_logger().error("/delete_entity service unavailable")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    for name in names:
        req = DeleteEntity.Request()
        req.name = name
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        result = fut.result() if fut.done() else None
        if result is None:
            print(f"  delete {name}: no response within {timeout}s")
        elif not result.success:
            print(f"  delete {name} (may not exist): {result.status_message}")
        else:
            print(f"  deleted {name}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
