# Basketbro
## Purpose
When the user clicks the enter key, the robot will accelerate its joints to shoot the basketball from the green release point into the hoop (see [Demo Video](https://drive.google.com/file/d/1NlR-tVNoZH8qRIMnqBDvmxlS8gX6kUeg/view?usp=sharing) also included in this repo). The release point and the hoop can be moved around using the interactive arrows. To make it realistic, we imposed position and velocity limits on the joints, making some hoop position and release point configurations impossible to score. In that case, the robot makes the closest shot it can. See [Basketbro Final Report](https://github.com/KMcCand/Basketbro/blob/main/Basketbro%20Final%20Report.pdf) for more info!

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
