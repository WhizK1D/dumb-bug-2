#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math

def approx_Equal(x, y, tolerance=0.01):
    return abs(x-y) <= 0.5 * tolerance * (x + y)

def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('test_pub', anonymous=False)
    rate = rospy.Rate(100) # 10hz
    turn_msg = Twist()
    msg_count = 0
    robo_pose = rospy.wait_for_message("base_pose_ground_truth", Odometry)
    robo_or = tf.transformations.euler_from_quaternion((robo_pose.pose.pose.orientation.x, robo_pose.pose.pose.orientation.y, robo_pose.pose.pose.orientation.z, robo_pose.pose.pose.orientation.w))

    m = 0.5627722208758873
    while not approx_Equal(math.atan(m), robo_or[2]):
        turn_msg.angular.z = 0.1
        pub.publish(turn_msg)
        robo_pose = rospy.wait_for_message("base_pose_ground_truth", Odometry)
        robo_or = tf.transformations.euler_from_quaternion((robo_pose.pose.pose.orientation.x, robo_pose.pose.pose.orientation.y, robo_pose.pose.pose.orientation.z, robo_pose.pose.pose.orientation.w))
        rate.sleep()

    while not rospy.is_shutdown():
        move_msg = Twist()
        move_msg.linear.x = 5
        rospy.loginfo(move_msg.linear.x)
        pub.publish(move_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
