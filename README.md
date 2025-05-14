# Visualization for ODRI bolt robot
ROS2, rviz2, open-dynamic-robot-initiative

## Required packages
```
sudo apt-get install ros-rolling-ros-gz
sudo apt install ros-rolling-position-controllers
sudo apt install ros-rolling-ros2-controllers
sudo apt install ros-rolling-ros2-control
```

## Visualize with rviz2
`cd` to your ROS2 workspace
```
colcon build
source install/setup.bash
ros2 launch bolt_visu display.launch.py
```

## Visualize with gz (gazebo)
`ros2 launch bolt_visu gazebo.launch.py`
