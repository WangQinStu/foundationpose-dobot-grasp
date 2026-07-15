#!/usr/bin/env python3
"""Republish Dobot's raw TCP feedback as a ROS PoseStamped target template.

Dobot bringup publishes /dobot_msgs_v4/msg/ToolVectorActual as millimeters plus
RPY degrees. This node converts it to /dobot_pick/current_pose in meters plus a
quaternion, matching the format accepted by target_pose_gripper.py.
"""

import math

import rclpy
from dobot_msgs_v4.msg import ToolVectorActual
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node


def rpy_degrees_to_quaternion(rx: float, ry: float, rz: float) -> Quaternion:
    """Convert Dobot Rx/Ry/Rz degrees into a ROS quaternion."""
    roll = math.radians(rx)
    pitch = math.radians(ry)
    yaw = math.radians(rz)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class CurrentPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__('current_pose_publisher')

        self.declare_parameter(
            'tool_vector_topic',
            '/dobot_msgs_v4/msg/ToolVectorActual',
        )
        self.declare_parameter('current_pose_topic', '/dobot_pick/current_pose')
        self.declare_parameter('frame_id', 'dobot_base')
        self.declare_parameter('position_scale', 0.001)

        self.tool_vector_topic = self.get_parameter('tool_vector_topic').value
        self.current_pose_topic = self.get_parameter('current_pose_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.position_scale = float(self.get_parameter('position_scale').value)

        self.pose_pub = self.create_publisher(PoseStamped, self.current_pose_topic, 10)
        self.tool_vector_sub = self.create_subscription(
            ToolVectorActual,
            self.tool_vector_topic,
            self.tool_vector_callback,
            10,
        )

        self.get_logger().info(
            f'Converting {self.tool_vector_topic} to PoseStamped on {self.current_pose_topic}'
        )

    def tool_vector_callback(self, msg: ToolVectorActual) -> None:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id

        # Dobot feedback uses millimeters. The target pose topic used by our
        # controller uses meters, so recorded poses can be replayed directly.
        pose_msg.pose.position.x = float(msg.x) * self.position_scale
        pose_msg.pose.position.y = float(msg.y) * self.position_scale
        pose_msg.pose.position.z = float(msg.z) * self.position_scale
        pose_msg.pose.orientation = rpy_degrees_to_quaternion(msg.rx, msg.ry, msg.rz)
        self.pose_pub.publish(pose_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CurrentPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()