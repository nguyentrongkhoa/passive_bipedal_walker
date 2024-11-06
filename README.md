# Description
This repository contains the CAD designs of the DIRK passive bipedal walker as well as their simulation in Gazebo. 
ROS humble and Gazebo 11 are used for this purpose. 

## Workflow description

1. The CAD construction of the walker is done using SolidWorks.
2. 

## Usage instructions
Before this ROS2 package can be built and the gazebo simulation can be launched, the following setup steps are neccessary:

1. Copy all gazebo world files in the package into gazebo's default /worlds directory at usr/share/gazebo-11/worlds 
2. Copy the entire /meshes directory into gazebo's default model path at home/.gazebo/models

To build the package, navigate inside the /pasive_bipedal_walker directory and execute the following command:
```bash
colcon build && source install/setup.bash
```
Start gazebo in a separate terminal before launching the launch file:
```bash
gazebo -s libgazebo_ros_init.so -s libgazebo_ros_factory.so worlds/ramp.world
```
Start the gazebo launch file
```bash
ros2 launch walker_design_1 gazebo.launch.py
```

In case only a RVIZ visualization of the robot is desired instead of a Gazebo simulation:
```bash
ros2 launch walker_design_1 display.launch.py
```
