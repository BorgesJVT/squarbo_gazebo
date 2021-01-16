# squarbo_gazebo
Spawns Squarbo into Gazebo world allowing you to control the robot through `/cmd_vel` and get the state with `/odom` or `/model_states` topics.

## Instalation
``` bash
cd squarbo_ws/src
git clone https://github.com/BorgesJVT/squarbo_gazebo/tree/main 
cd ..
colcon build
```

## Run
To spawn the robot into Gazebo just run
``` bash
ros2 launch squarbo_gazebo spawn_squarbo.launch.py
```

To move the robot you can use
``` bash
ros2 topic pub -r 10 cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```
or teleoperate by the keyboard with
``` bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

<img src="/squarbo_gazebo.png" width="800">
