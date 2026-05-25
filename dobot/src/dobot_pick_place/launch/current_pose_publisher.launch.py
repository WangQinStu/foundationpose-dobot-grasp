from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tool_vector_topic',
            default_value='/dobot_msgs_v4/msg/ToolVectorActual',
            description='Dobot bringup TCP pose topic in millimeters and RPY degrees.',
        ),
        DeclareLaunchArgument(
            'current_pose_topic',
            default_value='/dobot_pick/current_pose',
            description='PoseStamped topic for recording the current TCP pose.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='dobot_base',
            description='Frame id written into the published PoseStamped messages.',
        ),
        Node(
            package='dobot_pick_place',
            executable='current_pose_publisher',
            name='current_pose_publisher',
            output='screen',
            parameters=[{
                'tool_vector_topic': LaunchConfiguration('tool_vector_topic'),
                'current_pose_topic': LaunchConfiguration('current_pose_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
        ),
    ])
