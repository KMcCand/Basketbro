# Basketbro
## Purpose

## Run using pre-installed ROS 2 and RViz
Start rviz with configuration file:
```
ros2 run rviz2 rviz2 -d rviz/viewproject2.rviz
```

(New terminal tab) Run robot state publisher:
```
ros2 run robot_state_publisher robot_state_publisher basketballrobot.urdf
```

(New terminal tab) Run Python Code:
```
python3 play_basketball2.py
```
