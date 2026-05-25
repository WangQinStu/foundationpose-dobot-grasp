#!/usr/bin/env python3
"""Bridge FoundationPose's conda Python process to ROS 2 via UDP JSON."""

import json
import socket
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def pose_to_dict(msg: PoseStamped) -> dict:
    return {
        'x': float(msg.pose.position.x),
        'y': float(msg.pose.position.y),
        'z': float(msg.pose.position.z),
        'qx': float(msg.pose.orientation.x),
        'qy': float(msg.pose.orientation.y),
        'qz': float(msg.pose.orientation.z),
        'qw': float(msg.pose.orientation.w),
    }


class FoundationPoseUdpBridge(Node):
    def __init__(self) -> None:
        super().__init__('foundationpose_udp_bridge')

        self.declare_parameter('target_pose_topic', '/dobot_pick/target_pose')
        self.declare_parameter('current_pose_topic', '/dobot_pick/current_pose')
        self.declare_parameter('sequence_state_topic', '/dobot_pick/sequence_state')
        self.declare_parameter('udp_target_bind_host', '127.0.0.1')
        self.declare_parameter('udp_target_port', 5005)
        self.declare_parameter('udp_status_host', '127.0.0.1')
        self.declare_parameter('udp_status_port', 5006)
        self.declare_parameter('status_publish_period', 0.1)

        self.target_pose_topic = self.get_parameter('target_pose_topic').value
        self.current_pose_topic = self.get_parameter('current_pose_topic').value
        self.sequence_state_topic = self.get_parameter('sequence_state_topic').value
        self.udp_target_bind_host = self.get_parameter('udp_target_bind_host').value
        self.udp_target_port = int(self.get_parameter('udp_target_port').value)
        self.udp_status_addr = (
            self.get_parameter('udp_status_host').value,
            int(self.get_parameter('udp_status_port').value),
        )

        self.target_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, self.sequence_state_topic, self.state_callback, state_qos)
        self.create_subscription(PoseStamped, self.current_pose_topic, self.current_pose_callback, 10)

        self.latest_state = 'unknown'
        self.latest_current_pose: Optional[PoseStamped] = None

        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.recv_sock.bind((self.udp_target_bind_host, self.udp_target_port))
        self.recv_sock.setblocking(False)
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.create_timer(0.01, self.poll_target_udp)
        self.create_timer(float(self.get_parameter('status_publish_period').value), self.publish_status_udp)

        self.get_logger().info(
            f'UDP target {self.udp_target_bind_host}:{self.udp_target_port} -> {self.target_pose_topic}; '
            f'{self.current_pose_topic}, {self.sequence_state_topic} -> UDP status {self.udp_status_addr}'
        )
        self.publish_status_udp()

    def state_callback(self, msg: String) -> None:
        self.latest_state = str(msg.data)
        self.publish_status_udp()

    def current_pose_callback(self, msg: PoseStamped) -> None:
        self.latest_current_pose = msg

    def poll_target_udp(self) -> None:
        while True:
            try:
                data, _addr = self.recv_sock.recvfrom(65535)
            except BlockingIOError:
                return

            try:
                target = json.loads(data.decode('utf-8'))
            except json.JSONDecodeError as exc:
                self.get_logger().warn(f'忽略非法 UDP JSON: {exc}')
                continue

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = str(target.get('frame_id', 'dobot_base'))
            msg.pose.position.x = float(target['x'])
            msg.pose.position.y = float(target['y'])
            msg.pose.position.z = float(target['z'])
            msg.pose.orientation.x = float(target['qx'])
            msg.pose.orientation.y = float(target['qy'])
            msg.pose.orientation.z = float(target['qz'])
            msg.pose.orientation.w = float(target['qw'])
            self.target_pub.publish(msg)
            self.get_logger().info(
                f'发布视觉目标: x={msg.pose.position.x:.4f}, '
                f'y={msg.pose.position.y:.4f}, z={msg.pose.position.z:.4f}'
            )

    def publish_status_udp(self) -> None:
        payload = {
            'state': self.latest_state,
            'current_pose': pose_to_dict(self.latest_current_pose) if self.latest_current_pose else None,
        }
        data = json.dumps(payload, separators=(',', ':')).encode('utf-8')
        self.send_sock.sendto(data, self.udp_status_addr)

    def destroy_node(self) -> bool:
        self.recv_sock.close()
        self.send_sock.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FoundationPoseUdpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
