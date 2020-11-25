
# To find the guide look at the Wiki!

Here below, we'll look at all the different launch files we wrote.

## Mastermind Package
That's the main package, the higher one on the hierarchy.
Its task is to let everything start smoothly.

 0. **main**
 
Just different launch files put together, may change across branches right now.
Usually, it launches the next ones.
 1. **init**
 
It loads both cameras (based on serial number!) and start publishing a static transform from *base_link* (center of the robot, projected on the ground) to *camera1_link* and *camera2_link*

2. **filter_imu**

It run a *imu_filter_madgwick* node over camera1 IMU. It estimates step by step the global orientation based on IMU inputs.

3. **rgbd_conversion**

It takes RGB image topic and Depth topic as input and it outputs an RGBD topic, for both cameras.

4. **rtabmap**

It starts both the Odometry and the SLAM, and even the RVIZ visualization.
It can receive 3 inputs:
*3d* (default = true) it uses more resources, but the map is in 3D and much cooler. Best is false.
*localization* (default = false) if true, it doesn't create a new map but load an old one.
*save_map* (default = false) if true, it creates a new map and saves it

 7. **obstacle_detection**

It starts an rtabmap obstacle detection nodelet, which is able to differentiate objects from the ground.
To do so effectively, we need to convert our pointcloud to a laserscan in order to be able to clean the local map from objects even when the walls are out of our reach.

## State Machine
This package it makes possible to move the robot around and communicate with motors.

 -  **sm_startup**

It launches three things: the joystick controller, the python node which send signals to the motors (UDP_sender) and the actual state machine.

## Navigator
This package contains what's needed to do autonomous navigation.

 - **manual_nav**

Despite the name, it loads all the config files it founds in the config folder and starts the move_base instances we need.

## Joystick to twist
It manages the joystick inputs and it translate them into a Twist message.

## UDP controller
It contains two different python scripts that manage the communication with the motors, one for receiving, the other for sending.

## Localization
It contains two python script that estimate the covariance of a topic, both for IMU and wheels odometry.

 - **localization**

It starts a kalman filter based on every input we give in the config folder.
