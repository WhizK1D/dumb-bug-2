#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float64


'''
Basic node that publishes a state for the robot and prints the
current distance from target for easier debugging
'''


def target_distance_callback(data):
    '''
    Callback for homing_signal that also calculates and publishes the distance
    of the target from current location
    '''
    global msg_count
    msg_count += 1
    robo_loc = (rospy.wait_for_message("base_pose_ground_truth",
                      Odometry)).pose.pose.position
    target_loc = data.pose.position

    if (target_loc.x == robo_loc.x):
        target_distance = abs(target_loc.y - robo_loc.y)
        target_dist_pub.publish(target_distance)
        rospy.loginfo("Distance to target: {}".format(
                        str(target_distance)))
    else:
        a = abs(target_loc.x - robo_loc.x)
        b = abs(target_loc.y - robo_loc.y)
        target_distance = math.sqrt(((a**2)+(b**2)))
        target_dist_pub.publish(target_distance)
        rospy.loginfo("Distance to target: {}".format(str(target_distance)))

    if target_distance < 1.5:
        robo_state_pub.publish("TARGET_REACHED")
        rospy.loginfo("Reached charging station")

    if (source_location.x == robo_loc.x):
        home_distance = abs(source_location.x - robo_loc.y)
    else:
        a = abs(source_location.x - robo_loc.x)
        b = abs(source_location.y - robo_loc.y)
        home_distance = math.sqrt(((a**2)+(b**2)))

    if (home_distance <= 0.5) and (msg_count > 1000000):
        robo_state_pub.publish("BUG_FAILED")
        rospy.loginfo("BUG-2 failed, reached starting point")


def listener():
    '''
    Basic listener module to listen
    '''
    rospy.Subscriber("homing_signal", PoseStamped, target_distance_callback)
    rospy.spin()


msg_count = 0
rospy.init_node("overwatch", anonymous=False)
source_location = (rospy.wait_for_message("base_pose_ground_truth",
                   Odometry)).pose.pose.position
robo_state_pub = rospy.Publisher("robo_state", String, queue_size=10)
target_dist_pub = rospy.Publisher("target_distance", Float64, queue_size=10)

if __name__ == "__main__":
    listener()
