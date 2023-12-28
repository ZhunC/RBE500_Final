
# RBE 500 - Foundations of Robotics 2023 

Taught by Professor Berk Calli at Worcester Polytechnic Institute Robotics Engineering Department

Team 24 Members: Zhun Cheng, Niranjan Kumar Ilampooranan, Chris Nguyen

# OpenManipulatorX_ROS2
ROS2 Humble packages to control Open Manipulator X

## Build
Clone the entire repo into the src/ directory in a new ROS2 workspace.

Run the following to build the packages in your new workspace:
```
colcon build --symlink-install
```
You may get some warning mesages, ignore them for now as long as everything successfully builds.


Don't forget to source your workspace:
```
source install/setup.bash
```

After connecting to the robot over USB, run the following to begin position control:
```
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```

Now run the example code to move the robot:
```
ros2 run rbe500-example basic_robot_control
```
Or for python:
```
ros2 run rbe500_example_py basic_robot_control
```

## Course Packages

omx_24_hw1, omx_24_vel_kin, and omx_24_pd_ctr are the separate packages written for respective modules

To run the packages, the computer needs to be connected to the OpenManipulatorX. The necessary commands are - 
```
ros2 run omx_24_hw1 fk_sub
```
To obtain the current pose of the arm for current joint angles and publish them
```
ros2 run omx_24_vel_kin service
```
To launch the service for conversion between task-space velocities and joint-space velocities
```
ros2 run omx_24_vel_kin qinc
```
To launch the service used to obtain incremental joint angles based on current velocity (which will be used to set as intermediate joint-space goals)
```
ros2 run omx_24_vel_kin client
```
To make the robot arm trace a straight line using previous services and subscribers

```
ros2 run omx_24_pd_ctr client
```
To run the required services for controlling the robot arm to reach desired position based on PD current-based position controller written for Dynamixel actuators


