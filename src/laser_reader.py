#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist


'''
Basic laser sensor reader which will read the base_scan from laser and
publish out if any obstacles in the vicinity and update the state of the
robot accordingly
'''


def callback(data):
    stop_pub = rospy.Publisher("robo_state", String, queue_size=10)
    laser_perception = tuple(l * r for l, r in zip(data.ranges, data.intensities))

    right = laser_perception[0:71]
    # front_right = laser_perception[72:143]
    front = laser_perception[144:215]
    # front_left = laser_perception[216:287]
    left = laser_perception[288:361]
    front_item_count = 0
    front_total_items = 0
    right_item_count = 0
    right_total_items = 0
    left_item_count = 0
    left_total_items = 0

    for f in front:
        if (f > 0.0) and (f < 2.5):
            front_item_count += 1
        front_total_items += 1

    for r in right:
        if (r>0.0) and (r<2.5):
            right_item_count+=1
        right_total_items+=1

    for l in left:
        if (l>0.0) and (l<2.5):
            left_item_count+=1
        left_total_items+=1

    rospy.loginfo("{}/{} in front are close!".format(
        front_item_count, front_total_items))
    rospy.loginfo("{}/{} in left are close!".format(
        left_item_count, left_total_items))
    rospy.loginfo("{}/{} in right are close!".format(
        right_item_count, right_total_items))

    if float(float(front_item_count)/front_total_items) >= 0.35:
        rospy.loginfo("Publishing WALLFOLLOW")
        stop_pub.publish("WALLFOLLOW")
    elif (float(right_item_count)/right_total_items) >= 0.35:
        rospy.loginfo("Publishing MOVE_LINE")
        stop_pub.publish("MOVE_LINE")
    elif (float(left_item_count)/left_total_items) >= 0.35:
        rospy.loginfo("Publishing MOVE_LINE")
        stop_pub.publish("MOVE_LINE")
    else:
        rospy.loginfo("Publishing GOALSEEK")
        stop_pub.publish("GOALSEEK")
    (rospy.Rate(10)).sleep()


def listener():
    rospy.init_node("bug_monitor", anonymous=False)
    rospy.Subscriber("base_scan", LaserScan, callback)
    rospy.spin()


if __name__ == "__main__":
    listener()
