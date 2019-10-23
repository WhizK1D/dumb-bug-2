#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
import keyboard


rospy.init_node("manual_robot_controller", anonymous=False)
move_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)


def move_key_press_callback():
    motion_msg = Twist()
    motion_msg.linear.x = 1.0
    rospy.loginfo("Move key pressed")
    move_pub.publish(motion_msg)


def move_key_release_callback():
    motion_msg = Twist()
    motion_msg.linear.x = 0.0
    rospy.loginfo("Move key released")
    move_pub.publish(motion_msg)


def turn_key_press_callback():
    turn_msg = Twist()
    turn_msg.angular.x = 1.8
    rospy.loginfo("Turn key pressed")
    move_pub.publish(turn_msg)


def turn_key_release_callback():
    turn_msg = Twist()
    turn_msg.angular.x = 0.0
    rospy.loginfo("Turn key released")
    move_pub.publish(turn_msg)


if __name__ == "__main__":
    keyboard.on_press_key("up", move_key_press_callback)
    keyboard.on_release_key("up", move_key_release_callback)
    keyboard.on_press_key("right", turn_key_press_callback)
    keyboard.on_release_key("right", turn_key_release_callback)

    rospy.loginfo("Listening to key events")
    rospy.spin()
