#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from open_manipulator_msgs.msg import KinematicsPose


import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

     
if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('target_kinematics_pose_keyboard')
    pub = rospy.Publisher('/om_with_tb3/target_kinematics_pose', KinematicsPose, queue_size=10)

    msg = KinematicsPose()
    once = 0

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg.pose.position.x = 0.20
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.15
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        if once <= 1:
            pub.publish(msg)        
            once = once + 1

        rate.sleep()

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)





