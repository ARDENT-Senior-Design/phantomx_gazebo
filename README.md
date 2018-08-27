## phantomx_gazebo

ROS package providing Gazebo simulation of the Phantom X Hexapod robot.
Also provides a Python interface to the joints and some walk capabilities.

These have been tested in simulation and need some work to be used on the real robot, do not use as-is.

![Phantom X model in Gazebo](/phantomx.png?raw=true "Phantom X model in Gazebo")

## Install

Clone in your catkin workspace and catkin_make it.
Make sure you also have the following packages in your workspace
* phantomx_description: https://github.com/ARDENT-Senior-Design/phantomx_description
* phantomx_control: https://github.com/ARDENT-Senior-Design/phantomx_control
    
## Usage

You can launch the simulation with:

    roslaunch phantomx_gazebo phantomx_gazebo.launch
    
PRESS PLAY IN GAZEBO ONLY WHEN EVERYTHING IS LOADED (wait for controllers)

You can run a walk demo with:

    rosrun phantomx_gazebo walker_demo.py
    
The LIDAR can be tuned to move to more specific angles and rates, but the basic sweep code to move the LIDAR up an down can be run with:
    
    rosrun phantomx_gazebo lidar_sweep_node.py
    
By default, the simulation laucnher will run the lidar_sweep_node.

## RVIZ

In order for the robot to appear in RVIZ, the following settings need to be changed:

    General Options:
        Fixed Frame -> base_link

The LaserScan topic can be added by subscribing to the /phantomx/LaserScan topic listed below.

## ROS API

All topics are provided in the /phantomx namespace.

Sensors:

    /phantomx/joint_states
    /phantomx/LaserScan

Actuators (radians for position control, arbitrary normalized speed for cmd_vel):

    /phantomx/cmd_vel
    /phantomx/hokuyo_tilt_position_controller/command
    /phantomx/j_c1_lf_position_controller/command
    /phantomx/j_c1_lm_position_controller/command
    /phantomx/j_c1_lr_position_controller/command
    /phantomx/j_c1_rf_position_controller/command
    /phantomx/j_c1_rm_position_controller/command
    /phantomx/j_c1_rr_position_controller/command
    /phantomx/j_thigh_lf_position_controller/command
    /phantomx/j_thigh_lm_position_controller/command
    /phantomx/j_thigh_lr_position_controller/command
    /phantomx/j_thigh_rf_position_controller/command
    /phantomx/j_thigh_rm_position_controller/command
    /phantomx/j_thigh_rr_position_controller/command
    /phantomx/j_tibia_lf_position_controller/command
    /phantomx/j_tibia_lm_position_controller/command
    /phantomx/j_tibia_lr_position_controller/command
    /phantomx/j_tibia_rf_position_controller/command
    /phantomx/j_tibia_rm_position_controller/command
    /phantomx/j_tibia_rr_position_controller/command


## Python API

Basic usage:
```python
import rospy
from phantomx_gazebo.phantomx import PhantomX

rospy.init_node("walker_demo")

phantomx=PhantomX()
rospy.sleep(1)

phantomx.set_walk_velocity(1,0,0) # Set full speed ahead for 5 secs
rospy.sleep(5)
phantomx.set_walk_velocity(0,0,0) # Stop
```
## Dependencies

The following ROS packages have to be installed:
* gazebo_ros_control
* libgazebo_ros_laser 

## Troubleshooting
If the simulation is not moving correctly, try adding the following line before running the simulation:

    export LIBGL_ALWAYS_SOFTWARE=1

## License

This software is originally provided by Génération Robots http://www.generationrobots.com and HumaRobotics http://www.humarobotics.com under the Simplified BSD license
This software has been modified by Nathan Boyd
