#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from robot import Robot

if __name__ == "__main__":
    rospy.init_node("dumb_bug", anonymous=False)
    msg = rospy.wait_for_message("homing_signal", PoseStamped)
    loc = msg.pose.position
    r = Robot(loc)