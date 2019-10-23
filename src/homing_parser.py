#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def callback(data):
    '''
    Read the homing signal value, calculate the distance from current robot
    and publish the distance on topic target_distance
    '''
    data_str = "[{}] Position: [{}, {}, {}] Orientation: [{}, {}, {}]".format(
                rospy.get_caller_id(), str(data.pose.position.x),
                str(data.pose.position.y), str(data.pose.position.z),
                str(data.pose.orientation.x), str(data.pose.orientation.y),
                str(data.pose.orientation.w), str(data.pose.orientation.w))

    '''
    TODO: Publish actual distance to homing_beacon rather than beacon signal
    '''
    pub.publish(data_str)
    rospy.loginfo(data_str)


def listener():
    '''
    Basic listener module to listen for homing beacon, and parse
    its data
    '''
    rospy.Subscriber("homing_signal", PoseStamped, callback)
    '''Let the function run until termination'''
    rospy.spin()


rospy.init_node("beacon_parser", anonymous=False)
# TODO: Change from String to int when distance calculation is done
pub = rospy.Publisher("target_distance", String, queue_size=10)

if __name__ == "__main__":
    listener()
