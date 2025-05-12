# Visualization for ODRI bolt robot
ROS2, rviz2, open-dynamic-robot-initiative

## Visualize with rviz2
`cd` to your ROS2 workspace
```
colcon build
source install/setup.bash
ros2 launch bolt_visu display.launch.py
```

## Visualize with gz (gazebo)
`ros2 launch bolt_visu gazebo.launch.py`
