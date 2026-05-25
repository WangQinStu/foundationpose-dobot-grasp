from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    pose_msg = (
        '{header: {frame_id: dobot_base}, '
        'pose: {'
        'position: {'
        'x: -0.6381917724609375, '
        'y: -0.30489663696289065, '
        'z: 0.3185687255859375'
        '}, '
        'orientation: {'
        'x: 0.011411472351837162, '
        'y: 0.0016656594789129023, '
        'z: 0.03427464789428714, '
        'w: 0.99934591227912'
        '}'
        '}}'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2',
                'topic',
                'pub',
                '--once',
                '/dobot_pick/target_pose',
                'geometry_msgs/msg/PoseStamped',
                pose_msg,
            ],
            output='screen',
        ),
    ])
