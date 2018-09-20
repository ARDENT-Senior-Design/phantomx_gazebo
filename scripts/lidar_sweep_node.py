#!/usr/bin/env python
# Publishes to the lidar so it moves at the correct speed

import rospy
from phantomx_gazebo.phantomx import PhantomX
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('lidar_sweep_node')
    rospy.loginfo('Instantiating robot Client')
    robot = PhantomX()
    rate = rospy.Rate(50) #hz
    angle = 0
    speed = 0.005
    while not rospy.is_shutdown():
      #  rospy.loginfo(angle)
        robot.set_lidar_theta(angle)
        if((angle)<-0.01 or angle>1.2):
          speed = speed*-1
        angle=angle+speed
        rate.sleep()