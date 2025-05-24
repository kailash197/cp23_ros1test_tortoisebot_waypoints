#! /usr/bin/python3

import unittest
import rostest
import rospy
import actionlib
import math
from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyRequest
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal
from tf.transformations import euler_from_quaternion

PKG='tortoisebot_waypoints'
NAME='tortoisebot_as_integration_test'

class TestActionServer(unittest.TestCase):
    def setUp(self):
        rospy.init_node('tortoisebot_as_test')

        self.as_client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        self.as_client.wait_for_server()
        self.success_result = False

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Tolerance for position and yaw checks
        self.pos_tolerance = 0.1
        self.yaw_tolerance = math.pi / 12  # 15 degrees

        self.current_orientation = None
        self.current_position = None
        self.init_yaw = None
        self.final_yaw = None

        # Goals
        self.test_pass = rospy.get_param('test_pass', True)
        self.test_pass_goal = Point(x=0.0, y=0.25, z=0.0)
        self.test_pass_final_yaw = math.pi / 2.0

        self.test_fail_goal = Point(x=0.0, y=-1.0, z=0.0)
        self.test_fail_final_yaw = -math.pi/2.0
        self.goal = WaypointActionGoal()
        self.expected_yaw = None

        # Send goal to action server
        self.action_server_request()


    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
    
    def current_yaw_angle(self):
        """Convert the stored current orientation quaternion to Euler angles and return yaw"""
        if self.current_orientation is None:
            rospy.logwarn("No orientation data available!")
            return 0.0

        orientation_list = [
            self.current_orientation.x,
            self.current_orientation.y,
            self.current_orientation.z,
            self.current_orientation.w
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw
    
    def action_server_request(self):
        # reset robot position to home
        self.robot_reset_request()

        # Wait until we receive the first odometry message
        while not self.current_orientation:
            rospy.sleep(0.5)

        self.init_yaw = self.current_yaw_angle()
        if(self.test_pass):
            self.goal.position = self.test_pass_goal
            self.expected_yaw = self.test_pass_final_yaw
        else:
            self.goal.position = self.test_fail_goal
            self.expected_yaw = self.test_fail_final_yaw
        self.as_client.send_goal(self.goal)

        # Wait for result with timeout
        self.success_result = self.as_client.wait_for_result(rospy.Duration(10))

        if not self.success_result:
            self.as_client.cancel_goal()
            self.stop_robot()
            rospy.logwarn("Goal was cancelled intentionally for test_fail case.")

    def robot_reset_request(self):
        rospy.wait_for_service('/gazebo/reset_world')
        s = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        resp = s.call(EmptyRequest())
    
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def test_goal_position(self):
        # Check if goal was reached
        self.assertTrue(self.success_result)

        # Check position (with tolerance)
        rospy.loginfo("Goal position: %s" % str(self.goal.position))
        rospy.loginfo("Current position: %s" % str(self.current_position))
        self.assertAlmostEqual(self.goal.position.x, self.current_position.x, delta=self.pos_tolerance)
        self.assertAlmostEqual(self.goal.position.y, self.current_position.y, delta=self.pos_tolerance)
        
    def test_yaw_alignment(self):
        # Check if goal was reached
        self.assertTrue(self.success_result)

        self.final_yaw = self.current_yaw_angle()
        rospy.loginfo("Final Yaw:  %s" % str(self.final_yaw))
        
        # Check yaw alignment (with tolerance)
        self.assertAlmostEqual(self.final_yaw, self.expected_yaw, delta=self.yaw_tolerance)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestActionServer)
