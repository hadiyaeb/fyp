#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import readchar
rospy.init_node("mv_bot")
move = rospy.Publisher("/cmd_vel",String)
while not  rospy.is_shutdown():
    key = readchar.readkey()
    move.publish(key)
