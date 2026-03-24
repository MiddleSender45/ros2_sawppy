#!/usr/bin/env python3
"""
waypoint_navigator.py
---------------------
Loads a list of waypoints from a YAML file and sends them sequentially
to Nav2 using the NavigateToPose action server.

Usage (standalone, no Gazebo launch):
    ros2 run <your_pkg> waypoint_navigator.py \
        --waypoints /path/to/waypoints_forest.yaml

Or via the provided launch file:
    ros2 launch <your_pkg> waypoint_mission.launch.py \
        waypoints:=/path/to/waypoints_forest.yaml
"""

import sys
import math
import yaml
import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def yaw_to_quaternion(yaw_deg: float) -> dict:
    """Convert yaw in degrees to a geometry_msgs quaternion (z/w only)."""
    yaw = math.radians(yaw_deg)
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw / 2.0),
        "w": math.cos(yaw / 2.0),
    }


def load_waypoints(path: str) -> list:
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    waypoints = data.get("waypoints", [])
    if not waypoints:
        raise ValueError(f"No waypoints found in {path}")
    return waypoints


class WaypointNavigator(Node):

    def __init__(self, waypoint_file: str):
        super().__init__("waypoint_navigator")

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._waypoints = load_waypoints(waypoint_file)
        self._current_index = 0
        self._total = len(self._waypoints)

        self.get_logger().info(
            f"Loaded {self._total} waypoints from: {waypoint_file}"
        )

        # Wait for Nav2 action server to become available
        self.get_logger().info("Waiting for Nav2 navigate_to_pose action server...")
        self._client.wait_for_server()
        self.get_logger().info("Action server ready. Starting mission.")

        self._send_next_waypoint()

    # ------------------------------------------------------------------
    # Core logic
    # ------------------------------------------------------------------

    def _send_next_waypoint(self):
        if self._current_index >= self._total:
            self.get_logger().info(
                "All waypoints reached. Mission complete — rover idling."
            )
            return

        wp = self._waypoints[self._current_index]
        name = wp.get("name", f"waypoint_{self._current_index + 1}")
        quat = yaw_to_quaternion(wp.get("yaw", 0.0))

        self.get_logger().info(
            f"[{self._current_index + 1}/{self._total}] Navigating to "
            f"'{name}'  x={wp['x']}  y={wp['y']}  yaw={wp.get('yaw', 0.0)}°"
        )

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp["x"])
        goal.pose.pose.position.y = float(wp["y"])
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = quat["x"]
        goal.pose.pose.orientation.y = quat["y"]
        goal.pose.pose.orientation.z = quat["z"]
        goal.pose.pose.orientation.w = quat["w"]

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2 — aborting mission.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        status = result.status

        wp = self._waypoints[self._current_index]
        name = wp.get("name", f"waypoint_{self._current_index + 1}")

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached '{name}' successfully.")
            self._current_index += 1
            self._send_next_waypoint()
        else:
            self.get_logger().warn(
                f"Navigation to '{name}' failed with status {status}. "
                "Skipping to next waypoint."
            )
            self._current_index += 1
            self._send_next_waypoint()

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().info(
            f"  └─ Distance remaining: {dist:.2f} m", throttle_duration_sec=2.0
        )


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="ROS2 Nav2 waypoint mission runner"
    )
    parser.add_argument(
        "--waypoints",
        type=str,
        required=True,
        help="Absolute path to the waypoints YAML file",
    )
    # Strip ROS2 remapping args before parsing
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = WaypointNavigator(waypoint_file=args.waypoints)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()