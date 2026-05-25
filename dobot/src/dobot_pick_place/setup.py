from setuptools import find_packages, setup

package_name = 'dobot_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/current_pose_publisher.launch.py',
            'launch/send_final_place_pose.launch.py',
            'launch/target_pose_gripper.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ptcs',
    maintainer_email='ptcs@todo.todo',
    description='Minimal vision target pose to Dobot MovL and gripper control node.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_pose_gripper = dobot_pick_place.target_pose_gripper:main',
            'current_pose_publisher = dobot_pick_place.current_pose_publisher:main',
            'foundationpose_udp_bridge = dobot_pick_place.foundationpose_udp_bridge:main',
        ],
    },
)
