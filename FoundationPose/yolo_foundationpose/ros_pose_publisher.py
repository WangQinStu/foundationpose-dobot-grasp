import logging
import sys

import numpy as np
from scipy.spatial.transform import Rotation


class RosPosePublisher:
  def __init__(self, topic, frame_id, node_name='foundationpose_pose_publisher',
               qos_depth=10):
    try:
      import rclpy
      from geometry_msgs.msg import PoseStamped
    except ImportError as exc:
      raise RuntimeError(
        'ROS 2 pose publishing requires rclpy and geometry_msgs. '
        f'Current Python is {sys.version.split()[0]} at {sys.executable}. '
        'If you are running FoundationPose from a conda environment while ROS 2 '
        'Jazzy is installed for system Python 3.12, rclpy cannot be imported in '
        'this process even after sourcing /opt/ros/jazzy/setup.bash. For Dobot '
        'closed-loop grasping, start dobot_pick_place target_pose_gripper.launch.py '
        'on the ROS side and run FoundationPose with --dobot_publish_target '
        'instead of --ros_publish_pose.'
      ) from exc

    self.rclpy = rclpy
    self.PoseStamped = PoseStamped
    if not rclpy.ok():
      rclpy.init(args=None)
      self._owns_rclpy = True
    else:
      self._owns_rclpy = False

    self.node = rclpy.create_node(node_name)
    self.publisher = self.node.create_publisher(PoseStamped, topic, qos_depth)
    self.frame_id = frame_id
    self.topic = topic
    logging.info(f'ROS 2 PoseStamped publisher enabled: topic={topic}, frame_id={frame_id}')

  def publish(self, pose_matrix):
    pose_matrix = np.asarray(pose_matrix, dtype=np.float64).reshape(4, 4)

    msg = self.PoseStamped()
    msg.header.stamp = self.node.get_clock().now().to_msg()
    msg.header.frame_id = self.frame_id
    msg.pose.position.x = float(pose_matrix[0, 3])
    msg.pose.position.y = float(pose_matrix[1, 3])
    msg.pose.position.z = float(pose_matrix[2, 3])

    quat_xyzw = Rotation.from_matrix(pose_matrix[:3, :3]).as_quat()
    msg.pose.orientation.x = float(quat_xyzw[0])
    msg.pose.orientation.y = float(quat_xyzw[1])
    msg.pose.orientation.z = float(quat_xyzw[2])
    msg.pose.orientation.w = float(quat_xyzw[3])

    self.publisher.publish(msg)
    self.rclpy.spin_once(self.node, timeout_sec=0.0)

  def close(self):
    self.node.destroy_node()
    if self._owns_rclpy:
      self.rclpy.shutdown()


def build_ros_pose_publisher(args):
  if not args.ros_publish_pose:
    return None
  return RosPosePublisher(topic=args.ros_pose_topic,
                          frame_id=args.ros_pose_frame_id,
                          node_name=args.ros_pose_node_name,
                          qos_depth=args.ros_pose_qos_depth)
