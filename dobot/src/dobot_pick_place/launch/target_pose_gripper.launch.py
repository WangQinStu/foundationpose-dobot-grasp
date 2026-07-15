from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_pose_topic',
            default_value='/dobot_pick/target_pose',
            description='PoseStamped topic for the robot TCP target pose.',
        ),
        DeclareLaunchArgument(
            'current_pose_topic',
            default_value='/dobot_pick/current_pose',
            description='PoseStamped topic for the current robot TCP pose.',
        ),
        DeclareLaunchArgument(
            'tool_vector_topic',
            default_value='/dobot_msgs_v4/msg/ToolVectorActual',
            description='Dobot bringup TCP pose topic in millimeters and RPY degrees.',
        ),
        DeclareLaunchArgument(
            'grasp_z_offset',
            default_value='0.0',
            description='Z offset in meters applied only to the initial grasp target pose. Negative moves the grasp point lower/deeper.',
        ),
        DeclareLaunchArgument(
            'pre_grasp_height',
            default_value='0.0',
            description='Move this many meters away from the grasp target before descending. Use 0 to go directly to the grasp target.',
        ),
        DeclareLaunchArgument(
            'pre_grasp_along_tcp',
            default_value='true',
            description='If true, pre_grasp_height moves backward along pre_grasp_axis_tcp transformed by the target orientation. If false, it moves along base Z.',
        ),
        DeclareLaunchArgument(
            'pre_grasp_axis_tcp',
            default_value='0,0,-1',
            description='TCP-frame retreat axis. Pre-grasp moves opposite this axis.',
        ),
        DeclareLaunchArgument(
            'fallback_to_pick_when_pre_grasp_rejected',
            default_value='true',
            description='If a pre-grasp pose is rejected by the controller, skip it and try the original pick pose.',
        ),
        DeclareLaunchArgument(
            'fallback_pick_orientation_on_reject',
            default_value='true',
            description='If the pick pose is rejected, retry once at the same xyz with fallback_pick_quat_xyzw.',
        ),
        DeclareLaunchArgument(
            'fallback_pick_quat_xyzw',
            default_value='0.011411472351837162,0.0016656594789129023,0.03427464789428714,0.99934591227912',
            description='Fallback fixed grasp orientation qx,qy,qz,qw used when a 3D pick orientation is rejected.',
        ),
        DeclareLaunchArgument(
            'gripper_close_distance',
            default_value='0.008',
            description='Close the gripper when current TCP is within this distance from target, in meters.',
        ),
        DeclareLaunchArgument(
            'gripper_close_angle',
            default_value='40000',
            description='WHEELTEC gripper close angle. Larger values close harder/farther.',
        ),
        DeclareLaunchArgument(
            'gripper_open_on_start',
            default_value='true',
            description='Open the gripper once when target_pose_gripper starts.',
        ),
        DeclareLaunchArgument(
            'pick_close_timeout',
            default_value='0.0',
            description='Fallback seconds after pick MovL response before closing the gripper if arrival threshold is not reached. Use 0 to disable.',
        ),
        DeclareLaunchArgument(
            'ignore_request_control_failure',
            default_value='true',
            description='Continue when RequestControl fails but the robot is already remotely controllable/enabled.',
        ),
        DeclareLaunchArgument(
            'ignore_speed_factor_failure',
            default_value='true',
            description='Continue when SpeedFactor fails and use the controller current speed setting.',
        ),
        DeclareLaunchArgument(
            'trust_enable_service_response',
            default_value='true',
            description='Continue after EnableRobot/Continue succeeds even if RobotStatus.is_enable remains false.',
        ),
        DeclareLaunchArgument(
            'auto_initialize_robot',
            default_value='true',
            description='Run RequestControl/PowerOn/EnableRobot before the first motion. Set false if the robot is already enabled and these services are slow or unavailable.',
        ),
        DeclareLaunchArgument(
            'wait_for_robot_connection',
            default_value='true',
            description='Hold a received target until RobotStatus reports an active TCP connection.',
        ),
        DeclareLaunchArgument(
            'connection_wait_timeout',
            default_value='15.0',
            description='Maximum seconds to wait for the robot TCP connection before rejecting the target.',
        ),
        DeclareLaunchArgument(
            'movl_service',
            default_value='/dobot_bringup_ros2/srv/MovL',
            description='Dobot MovL service name.',
        ),
        DeclareLaunchArgument(
            'movj_service',
            default_value='/dobot_bringup_ros2/srv/MovJ',
            description='Dobot MovJ service name used as fallback when MovL is rejected.',
        ),
        DeclareLaunchArgument(
            'fallback_to_movj',
            default_value='true',
            description='Fallback to MovJ point-to-point motion when MovL is rejected by the controller.',
        ),
        DeclareLaunchArgument(
            'gripper_topic',
            default_value='/motor_control',
            description='WHEELTEC step motor gripper command topic.',
        ),
        DeclareLaunchArgument(
            'sequence_state_topic',
            default_value='/dobot_pick/sequence_state',
            description='State topic used by the vision loop to know when the robot is idle again.',
        ),
        DeclareLaunchArgument(
            'start_foundationpose_udp_bridge',
            default_value='true',
            description='Whether to start the UDP bridge for FoundationPose conda Python.',
        ),
        DeclareLaunchArgument(
            'udp_target_bind_host',
            default_value='127.0.0.1',
            description='UDP bind host for target poses sent by FoundationPose.',
        ),
        DeclareLaunchArgument(
            'udp_target_port',
            default_value='5005',
            description='UDP port for target poses sent by FoundationPose.',
        ),
        DeclareLaunchArgument(
            'udp_status_host',
            default_value='127.0.0.1',
            description='UDP host where FoundationPose receives robot state/current pose.',
        ),
        DeclareLaunchArgument(
            'udp_status_port',
            default_value='5006',
            description='UDP port where FoundationPose receives robot state/current pose.',
        ),
        DeclareLaunchArgument(
            'start_gripper_motor',
            default_value='true',
            description='Whether to start the WHEELTEC step motor serial node.',
        ),
        DeclareLaunchArgument(
            'gripper_serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port used by the WHEELTEC gripper motor controller.',
        ),
        DeclareLaunchArgument(
            'gripper_serial_baud_rate',
            default_value='115200',
            description='Serial baud rate used by the WHEELTEC gripper motor controller.',
        ),
        DeclareLaunchArgument(
            'gripper_close_dir',
            default_value='1',
            description='Motor direction used to close the gripper. Use 0 if your hardware direction is reversed.',
        ),
        DeclareLaunchArgument(
            'place_after_grasp',
            default_value='true',
            description='Move to intermediate_pose and place_pose after grasping, then open the gripper.',
        ),
        DeclareLaunchArgument(
            'intermediate_pose',
            default_value=(
                '-0.6126588745117187,'
                '0.09267735290527344,'
                '0.06869639587402344,'
                '-0.015204590741187188,'
                '0.016260905108054526,'
                '-0.007558574422311183,'
                '0.9997235974699017'
            ),
            description='Intermediate PoseStamped as x,y,z,qx,qy,qz,qw in meters and quaternion.',
        ),
        DeclareLaunchArgument(
            'place_pose',
            default_value=(
                '-0.6518900146484375,'
                '0.4343290405273438,'
                '0.13612464904785157,'
                '-0.034783284399800544,'
                '0.0016099064028308628,'
                '-0.0031160653596715797,'
                '0.9993887239029721'
            ),
            description='Box/place PoseStamped as x,y,z,qx,qy,qz,qw in meters and quaternion.',
        ),
        DeclareLaunchArgument(
            'return_to_camera_after_place',
            default_value='true',
            description='Return to camera_pose after opening the gripper at place_pose.',
        ),
        DeclareLaunchArgument(
            'camera_pose',
            default_value=(
                '-0.614826416015625,'
                '-0.391370849609375,'
                '0.004824176788330079,'
                '-0.07082562768802304,'
                '0.022282501540186947,'
                '0.028372318712757964,'
                '0.9968361109623641'
            ),
            description='Camera PoseStamped as x,y,z,qx,qy,qz,qw in meters and quaternion.',
        ),
        DeclareLaunchArgument(
            'transfer_arrival_distance',
            default_value='0.03',
            description='Distance threshold in meters for intermediate/place arrival.',
        ),
        DeclareLaunchArgument(
            'after_grasp_delay',
            default_value='0.8',
            description='Seconds to wait after closing the gripper before moving to the intermediate pose.',
        ),
        Node(
            package='dobot_pick_place',
            executable='current_pose_publisher',
            name='current_pose_publisher',
            output='screen',
            parameters=[{
                'tool_vector_topic': LaunchConfiguration('tool_vector_topic'),
                'current_pose_topic': LaunchConfiguration('current_pose_topic'),
            }],
        ),
        Node(
            package='step_motor',
            executable='motor_node',
            name='gripper_motor_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_gripper_motor')),
            parameters=[{
                'usart_port_name': LaunchConfiguration('gripper_serial_port'),
                'serial_baud_rate': LaunchConfiguration('gripper_serial_baud_rate'),
            }],
        ),
        Node(
            package='dobot_pick_place',
            executable='foundationpose_udp_bridge',
            name='foundationpose_udp_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_foundationpose_udp_bridge')),
            parameters=[{
                'target_pose_topic': LaunchConfiguration('target_pose_topic'),
                'current_pose_topic': LaunchConfiguration('current_pose_topic'),
                'sequence_state_topic': LaunchConfiguration('sequence_state_topic'),
                'udp_target_bind_host': LaunchConfiguration('udp_target_bind_host'),
                'udp_target_port': LaunchConfiguration('udp_target_port'),
                'udp_status_host': LaunchConfiguration('udp_status_host'),
                'udp_status_port': LaunchConfiguration('udp_status_port'),
            }],
        ),
        Node(
            package='dobot_pick_place',
            executable='target_pose_gripper',
            name='target_pose_gripper',
            output='screen',
            parameters=[{
                'target_pose_topic': LaunchConfiguration('target_pose_topic'),
                'current_pose_topic': LaunchConfiguration('current_pose_topic'),
                'grasp_z_offset': LaunchConfiguration('grasp_z_offset'),
                'pre_grasp_height': LaunchConfiguration('pre_grasp_height'),
                'pre_grasp_along_tcp': LaunchConfiguration('pre_grasp_along_tcp'),
                'pre_grasp_axis_tcp': LaunchConfiguration('pre_grasp_axis_tcp'),
                'fallback_to_pick_when_pre_grasp_rejected': LaunchConfiguration('fallback_to_pick_when_pre_grasp_rejected'),
                'fallback_pick_orientation_on_reject': LaunchConfiguration('fallback_pick_orientation_on_reject'),
                'fallback_pick_quat_xyzw': LaunchConfiguration('fallback_pick_quat_xyzw'),
                'gripper_close_distance': LaunchConfiguration('gripper_close_distance'),
                'gripper_close_angle': LaunchConfiguration('gripper_close_angle'),
                'gripper_open_on_start': LaunchConfiguration('gripper_open_on_start'),
                'pick_close_timeout': LaunchConfiguration('pick_close_timeout'),
                'ignore_request_control_failure': LaunchConfiguration('ignore_request_control_failure'),
                'ignore_speed_factor_failure': LaunchConfiguration('ignore_speed_factor_failure'),
                'trust_enable_service_response': LaunchConfiguration('trust_enable_service_response'),
                'auto_initialize_robot': LaunchConfiguration('auto_initialize_robot'),
                'wait_for_robot_connection': LaunchConfiguration('wait_for_robot_connection'),
                'connection_wait_timeout': LaunchConfiguration('connection_wait_timeout'),
                'gripper_close_dir': LaunchConfiguration('gripper_close_dir'),
                'place_after_grasp': LaunchConfiguration('place_after_grasp'),
                'intermediate_pose': LaunchConfiguration('intermediate_pose'),
                'place_pose': LaunchConfiguration('place_pose'),
                'return_to_camera_after_place': LaunchConfiguration('return_to_camera_after_place'),
                'camera_pose': LaunchConfiguration('camera_pose'),
                'transfer_arrival_distance': LaunchConfiguration('transfer_arrival_distance'),
                'after_grasp_delay': LaunchConfiguration('after_grasp_delay'),
                'movl_service': LaunchConfiguration('movl_service'),
                'movj_service': LaunchConfiguration('movj_service'),
                'fallback_to_movj': LaunchConfiguration('fallback_to_movj'),
                'gripper_topic': LaunchConfiguration('gripper_topic'),
                'sequence_state_topic': LaunchConfiguration('sequence_state_topic'),
            }],
        ),
    ])
