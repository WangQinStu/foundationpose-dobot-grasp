ros2 topic pub --once /motor_control step_motor/msg/Motor "{id: 1, speed: 500, dir: 0, mode: 2, angle: 9000, state: 0, sub_divide: 32}"

ros2 topic pub --once /motor_control step_motor/msg/Motor "{id: 1, speed: 500, dir: 1, mode: 2, angle: 30000, state: 0, sub_divide: 32}"