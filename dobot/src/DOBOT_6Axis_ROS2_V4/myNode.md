窗口A，ros2-机械臂通信：
1. 关闭conda环境 `conda deactivate`
2. 进入工作目录`cd ~/wkSpace/dobot`
3. 移除build的内容`rm -rf build install log`
4. 再次build`colcon build`
5. 更新环境变量`source install/setup.bash`
6. 进入编译好的install目录`cd install/`
7. 运行ros2-机械臂通信的launch文件`ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py`


窗口B：
1. 关闭conda环境 `conda deactivate`
2. 添加环境变量`source install/setup.bash`
3. 运行launch文件`ros2 launch dobot_moveit dobot_moveit.launch.py`


窗口C：
1. 关闭conda环境 `conda deactivate`
2. 查看service的信息`ros2 service info /dobot_bringup_ros2/srv/MovL`
3. 呼叫服务`ros2 service call /dobot_bringup_ros2/srv/MovL dobot_msgs_v4/srv/MovL "{mode: false, a: -700.0, b: -120.0, c: 158.0, d: 0, e: 19.0, f: -4.0, param_value: ['user=0', 'tool=0', 'a=20', 'v=50', 'cp=0']}"`


