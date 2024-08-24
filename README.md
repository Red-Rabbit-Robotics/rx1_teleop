# rx1_teleop

## Introduction
This repo is a ROS1 package for RX1 humanoid robot teleoperation using exoskeleton style control arms.

## Dependencies
[rx1 ros package](https://github.com/Red-Rabbit-Robotics/rx1)

## Instruction
1. You need a second Feetech servo control board for the control arm servos. 
2. Modify the servo usb port value in the 'rx1_teleop/launch/teleop.launch' file.  
3. Launch the following launch files:  
`roslaunch rx1_bringup bringup.launch`  
`roslaunch rx1_teleop teleop.launch`