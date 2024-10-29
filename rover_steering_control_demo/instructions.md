# Steering Demo on Hiwonder JetAcker arm

This is a second demo in a series using `mocap4ros2_optitrack` package to send commanded velocities to the Hiwonder JetAcker rover using a rigid body tracked by OptiTrack cameras and Motive software.

It is recommended to check out the [first demo's readme](https://github.com/OptiTrack/robot_arm_follower_demo/blob/main/readme.md) with the robot arm before continuing with this one

This demo assumes more working knowledge of ROS2 and Docker as compared to the first demo.

### Outline

1. [About this Demo](#about-this-demo)
2. [Setup Requirements](#setup-requirements)
3. [OptiTrack Configuration](#optitrack-configuration)
3. [Configuring JetAcker Computer](#configuring-jetacker-computer)

## About this Demo

This demo is a continuation on examples of using OptiTrack with robots. In this demo we use an Ackermann-steering style rover to move in directions commanded by the position of a rigid body in space.

## Setup Requirements

See [Setup Requirements from the first demo](https://github.com/OptiTrack/robot_arm_follower_demo/blob/main/readme.md#setup-requirements) as the hardware requirements from the previous demo are the same.

Software Packages:
 - `hiwonder_acker_driver` package on the JetAcker's Jetson nano (ros2 driver derived from the ros1 driver Hiwonder provided)
 - `ackermann_steering_demo` package on the JetAcker that reads in rigid body position in the form of a ROS tf2 message and then publishes a commanded velocity message to the driver.
 - `mocap4ros2_optitrack` on the Ubuntu Companion computer (originally from https://github.com/OptiTrack/mocap4ros2_optitrack)
 - `mocap_msgs` on the Ubuntu Companion computer (originally from https://github.com/OptiTrack/mocap_msgs)


## OptiTrack Configuration

See our [Quick Start Guide](https://docs.optitrack.com/quick-start-guides/quick-start-guide-getting-started) for setting up your OptiTrack system, cameras, and Motive software.

Enable Streaming using [NatNet documentation](https://docs.optitrack.com/developer-tools/natnet-sdk). In the Streaming settings within Motive, make sure to set the local interface to an IP on the same network as the Ubuntu Companion computer.

## Configuring Ubuntu Companion Computer

We found that using Ubuntu 20.04 with ROS2 foxy worked well with sending messages to the JetAcker's Jetson Nano which was also running Ubuntu 20.04 ROS2 foxy.

In our case the companion computer used a Docker image running Ubuntu 20.04 sharing a network with the host computer. See an [example here](https://github.com/dean4ta/moving_to_ROS2/blob/c8917367d130e0d16bdeed657beb9eb7fb105b0d/Makefile#L23). Your own implementation may differ.

Copy the Ubuntu companion packages to the proper place:
```bash
cd ~
git clone https://github.com/OptiTrack/rover_steering_control_demo.git
mkdir -p ~/optitrack_companion_ws/src
cd ~/optitrack_companion_ws/src
# copy mocap_msgs, mocap4ros2_optitrack, and tf_repub to the src directory
cp -r ~/rover_steering_control_demo/packages_on_ubuntu_companion/* ~/optitrack_companion_ws/src
cd ~/optitrack_companion_ws
source /opt/ros/<your-ros-distro>/setup.bash
colcon build
```

## Configuring JetAcker Computer

The JetAcker ships with Ubuntu 18.04. This version of Ubuntu supports ROS2 Dashing and ROS2 dashing does not support ROS2 Transforms library (ROS2 Dashing is also out of support). It is recommended to flash an SD card with Ubuntu 20.04 using [this guide](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image). It is possible to run a Docker container that has Ubuntu 20.04 or 22.04 on the Jetson Nano, but GPIO support within the container would need to be addressed. That is beyond the scope of this guide.

After Flashing (or setting up the Docker container) install ROS2 foxy with https://docs.ros.org/en/foxy/Installation.html

Then move the code to the proper place with these commands:
```bash
cd ~
git clone https://github.com/OptiTrack/rover_steering_control_demo.git
mkdir -p ~/optitrack_robot_ws/src
cd ~/optitrack_robot_ws/src
# copy mocap_msgs, mocap4ros2_optitrack, and tf_repub to the src directory
cp -r ~/rover_steering_control_demo/packages_on_jetacker/* ~/optitrack_robot_ws/src
cd ~/optitrack_robot_ws
source /opt/ros/<your-ros-distro>/setup.bash
colcon build
```

## Bringing Everything Together

#### Launch Motive
Launch Motive and Enable Streaming on the Network the Ubuntu Companion Computer is on!

Create a rigid body:
 - "steering_wheel" - have this nearby the robot (the exact position does not matter)

 #### Launch the Ubuntu Companion Computer
```bash
cd ~/optitrack_companion_ws
source install/setup.bash
ros2 launch mocap_optitrack_driver optitrack2.launch.py
```

#### Launch the JetAcker Rover
```bash
# terminal 1 - start the JetAcker driver
cd ~/optitrack_robot_ws
source install/setup.bash
ros2 run hiwonder_acker_driver acker_controller
```
```bash
# terminal 2 - start the steering application
cd ~/optitrack_robot_ws
source install/setup.bash
ros2 run ackermann_steering_demo steering_demo
```

Ensure the "steering_wheel" rigid body in held in the air in a comfortable position.

```bash
# terminal 3 - publish message
cd ~/optitrack_robot_ws
source install/setup.bash
ros2 topic pub /get_reference_position "data: True"
# IMPORTANT press CTRL-C to stop sending messages
```

Start moving the rigid body up, down, left, and right to move the robot!

### What is going on?
In this demo, the rigid body position is measured by OptiTrack cameras and when the rigid body moves in the YZ plane (with reference to the `map` frame in ROS. See [RViz](https://docs.ros.org/en/iron/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html) for more details `ros2 run rviz2 rviz2`).

The measurement is then passed to the `/cmd_vel` topic in the [steering_demo_node.py](packages_on_jetacker\ackermann_steering_demo\ackermann_steering_demo\steering_demo_node.py). The `/cmd_vel` topic is consumed by the [acker_controller_node.py](packages_on_jetacker\hiwonder_acker_driver\hiwonder_acker_driver\acker_controller_node.py) and the drive controller determines the motor controls to send via GPIO.

