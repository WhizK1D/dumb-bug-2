import math
from enum import Enum
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import numpy
import numpy.linalg


'''
Global enum that defines the possible states of the robot
'''
states = Enum("States",
            "GOALSEEK WALLFOLLOW TARGET_REACHED BUG_FAILED MOVE_LINE")

class Robot:


    '''
    Robot base class that includes the basic intializations and components
    needed to make a generic 2D navigational robot.
    - target_location: Need input of type geometry_msgs.msg.Pose.Position
        to represent the location of the target
    - name: Optional argument for providing a name to the robot
    '''
    #rospy.loginfo("Initialized robot in GOALSEEK mode\n")

    def __approx_equals(self, a, b, tolerance=0.01):
        '''
        Compare 2 values a & b with a default tolerance of 0.01.
        Tells if a is approximately equal to b.
        '''
        return (abs(a-b) <= (0.5 * tolerance * (a + b)))

    def __init__(self, target_location, name="WhizKID"):
        self.name = name
        self.target_position = target_location

        # Print target position
        rospy.loginfo("Target Position: {}, {}".format(
            self.target_position.x,
            self.target_position.y))
        self.pose = Pose()

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Define enum for robot states and set default state to GOALSEEK
        self.state = states.GOALSEEK

        # Get source position and log it
        self.pose.position = self.get_current_position()
        self.source_position = self.pose.position
        data_str = "Source position: {},{}".format(
                    str(self.pose.position.x),
                    str(self.pose.position.y))
        rospy.loginfo(data_str)

        # Get source orientation and log it
        self.default_orientation = self.get_current_orientation()
        self.pose.orientation = self.default_orientation
        data_str = "Source orientation: {},{},{},{}".format(
                    str(self.pose.orientation.x),
                    str(self.pose.orientation.y),
                    str(self.pose.orientation.z),
                    str(self.pose.orientation.w))
        rospy.loginfo(data_str)

        # Get M-line slope and log it
        self.mline_slope = self.get_m_line_slope()
        data_str = "M-line slope: {}".format(str(self.mline_slope))
        rospy.loginfo(data_str)

        # Start aligning & moving robot towards the target
        self.move_towards_target()


    def move_towards_target(self):
        '''
        Continuously move towards the target
        '''
        rospy.loginfo("Aligning robot towards target")
        rospy.logwarn("This could take upto 2-3 minutes in worst case")
        turn_msg = Twist()
        # Align the robot towards the target
        curr_orientation = self.get_current_orientation()
        # temp is the orientation of robot in radians
        temp = ((tf.transformations.euler_from_quaternion((
            curr_orientation.x,
            curr_orientation.y,
            curr_orientation.z,
            curr_orientation.w
        )))[2])
        rospy.loginfo("Current orientation tan: {}".format(
                str(math.tan(temp))))
        # check if atan(m-line_slope) is ~= temp in radians
        while not self.__approx_equals(math.atan(self.mline_slope), temp):
            rospy.loginfo("m_angle: {}, rotation: {}".format(
                math.degrees(math.atan(self.mline_slope)),math.degrees(temp)))
            turn_msg.angular.z = 0.05
            self.vel_pub.publish(turn_msg)
            curr_orientation = self.get_current_orientation()
            temp = ((tf.transformations.euler_from_quaternion((
                curr_orientation.x,
                curr_orientation.y,
                curr_orientation.z,
                curr_orientation.w
                    )))[2])

        rospy.loginfo("Aligned robot successfully with m-line!")
        # Alignment done, now start moving towards target unless WALLFOLLOW
        while not rospy.is_shutdown() and self.state == states.GOALSEEK:

            try:
                robo_state = rospy.wait_for_message("robo_state", String, 5).data
                rospy.loginfo("Received robo_state message: {}".format(
                        robo_state))
                if (robo_state == "TARGET_REACHED"):
                    rospy.loginfo("Target Reached!")
                    break
                elif (robo_state == "WALLFOLLOW"):
                    rospy.loginfo("Going into WALLFOLLOW")
                    self.state = states.WALLFOLLOW
                    self.wallfollow()
                elif (robo_state == "BUG_FAILED"):
                    rospy.loginfo("Reached back home, BUG-2 failed :(")
                    break
                # elif (robo_state == "MOVE_LINE"):
                #     rospy.loginfo("Found obstacles on side, averting...")
                #     self.state = states.MOVE_LINE
                #     self.move_line()
                else:
                    self.move(0.2,0)
                    (rospy.Rate(10)).sleep()
            except rospy.ROSException:
                rospy.loginfo("Timeout expired, assuming GOALSEEK")
                robo_state = "GOALSEEK"
                if (robo_state == "TARGET_REACHED"):
                    rospy.loginfo("Target Reached")
                    break
                elif (robo_state == "WALLFOLLOW"):
                    rospy.loginfo("Going into WALLFOLLOW")
                    self.state = states.WALLFOLLOW
                    self.wallfollow()
                elif (robo_state == "BUG_FAILED"):
                    rospy.loginfo("Reached back home, BUG-2 failed")
                    break
                else:
                    self.move(0.2,0)
                    (rospy.Rate(10)).sleep()


    def wallfollow(self):
        '''
        Movement logic when in WALLFOLLOW mode
        1. Keep rotating till FRONT is clear
        2. Switch robot state to GOALSEEK
        3. Exit
        '''
        raw_data = rospy.wait_for_message("base_scan", LaserScan)
        laser_data = tuple(l * r for l, r in zip(
            raw_data.ranges, raw_data.intensities))
        front = laser_data[144:215]

        item_count = 0
        total_items = 0
        for l in front:
            if (l>0.0) and (l<2.5):
                item_count += 1
            total_items += 1

        rospy.loginfo("{} items out of {} in front are close!".format(
            item_count, total_items))

        while (float(item_count)/total_items) >= 0.35:
            stop_msg = Twist()
            stop_msg.linear.x = 0
            stop_msg.angular.z = 0.4
            self.vel_pub.publish(stop_msg)

            raw_data = rospy.wait_for_message("base_scan", LaserScan)
            laser_data = tuple(l * r for l, r in zip(
                raw_data.ranges, raw_data.intensities))
            front = laser_data[144:215]

            item_count = 0
            total_items = 0
            for l in front:
                if (l>0.0) and (l<2.5):
                    item_count += 1
                total_items += 1
            rospy.loginfo("{} items out of {} in front are close!".format(
                item_count, total_items))


        rospy.loginfo("Safely turned from obstacle for now!")
        self.state = states.GOALSEEK


    def move_line(self):
        rospy.loginfo("Stopping robot to think and figure out...")
        stop_msg = Twist()
        stop_msg.linear.x = 0
        self.vel_pub.publish(stop_msg)

        p1 = (self.source_position.x, self.source_position.y)
        p2 = (self.target_position.x, self.target_position.y)
        cur = self.get_current_position()
        p3 = (cur.x, cur.y)
        x_diff = p2[0] - p1[0]
        y_diff = p2[0] - p1[0]
        num = abs(y_diff*p3[0] - x_diff*p3[1] + p2[0]*p1[1] - p2[1]*p1[0])
        den = math.sqrt(y_diff**2 + x_diff**2)
        distance = num/den
        rospy.loginfo("{} units away from m-line".format(str(distance)))
        if distance < 1:
            self.align_towards_target()
            self.state = states.GOALSEEK
            return

        if self.mline_slope == 0:
            theta_normal = math.atan(-1/float("inf"))
        else:
            m_normal = (-(1.0/self.mline_slope))
            theta_normal = math.atan(m_normal)
        temp = self.get_current_orientation()
        # cur_theta is current angle in radians
        cur_theta = ((tf.transformations.euler_from_quaternion((
            temp.x,
            temp.y,
            temp.z,
            temp.w
        )))[2])
        turn_msg = Twist()
        while not self.__approx_equals(theta_normal, cur_theta):
            turn_msg.angular.z = 0.05
            self.vel_pub.publish(turn_msg)
            curr_orientation = self.get_current_orientation()
            cur_theta = ((tf.transformations.euler_from_quaternion((
                curr_orientation.x,
                curr_orientation.y,
                curr_orientation.z,
                curr_orientation.w
                    )))[2])

        rospy.loginfo("Aligned to move towards the normal")
        self.move_front(distance)
        self.align_towards_target()
        self.state = states.GOALSEEK


    def align_towards_target(self):
        '''
        Rotating the robot towards the target (to be used only when on m-line)
        '''
        turn_msg = Twist()
        # Align the robot towards the target
        curr_orientation = self.get_current_orientation()
        # temp is the orientation of robot in radians
        temp = ((tf.transformations.euler_from_quaternion((
            curr_orientation.x,
            curr_orientation.y,
            curr_orientation.z,
            curr_orientation.w
            )))[2])
        while not self.__approx_equals(math.atan(self.mline_slope), temp):
            turn_msg.angular.z = 0.05
            self.vel_pub.publish(turn_msg)
            curr_orientation = self.get_current_orientation()
            temp = ((tf.transformations.euler_from_quaternion((
                curr_orientation.x,
                curr_orientation.y,
                curr_orientation.z,
                curr_orientation.w
                    )))[2])
        rospy.loginfo("Done aligning towards target")


    def move_front(self, distance):
        '''
        Move to the front for the distance specified
        '''
        start_loc = self.get_current_position()
        dist_traveled = 0
        while dist_traveled < distance:
            self.move(0.1, 0)
            cur_loc = self.get_current_position()
            dist_traveled = math.sqrt(((abs(start_loc.x - cur_loc.x))**2) +
                            ((abs(start_loc.y - cur_loc.y))**2))
        rospy.loginfo("Reached M-Line")


    def move(self, lin, ang):
        '''
        Generic method to publish movement messages
        '''
        move_msg = Twist()
        move_msg.linear.x = lin
        move_msg.angular.z = ang
        self.vel_pub.publish(move_msg)


    def get_current_position(self):
        '''
        Get the current position of the robot by subscribing to the topic
        /base_pose_ground_truth of type nav_msgs/Odometry.pose.position
        '''
        return (rospy.wait_for_message("base_pose_ground_truth",
                                    Odometry).pose.pose.position)


    def get_current_orientation(self):
        '''
        Get the current orientation of the robot by subscribing to the topic
        /base_pose_ground_truth of type nav_msgs/Odometry.pose.orientation
        '''
        return (rospy.wait_for_message("base_pose_ground_truth",
                                    Odometry).pose.pose.orientation)


    def get_m_line_slope(self):
        '''
        Set the m-line slope using m=(y2-y1)/(x2-x1)
        '''
        x1 = self.source_position.x
        y1 = self.source_position.y
        x2 = self.target_position.x
        y2 = self.target_position.y
        if x2 == x1:
            return float("inf")
        return ((y2-y1)/(x2-x1))


# ---------------- Everything TODO from here on ----------------


    def __move_to_position(self, x, y):
        '''
        Move to the specified (x, y) position in a 2D world. Private method to
        implement other abstracted move methods.
        '''
        pass