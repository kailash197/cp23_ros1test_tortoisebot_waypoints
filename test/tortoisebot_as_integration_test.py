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
        self.yaw_tolerance = math.pi / 18  # 10 degrees

        self.current_orientation = Quaternion()
        self.current_position = Point()
        self.init_yaw = 0
        self.final_yaw = 0

        # Goals
        self.test_pass = rospy.get_param('test_pass', True)
        self.test_pass_goal = Point(x=0.0, y=0.25, z=0.0)
        self.test_fail_goal = Point(x=2.0, y=0.0, z=0.0)
        self.goal = WaypointActionGoal()

        # Reset the robot position to before the test begins
        self.robot_reset_request()

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
    
    def euler_to_quaternion(self, msg):
        orientation_list = [msg.x, msg.y, msg.z, msg.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw
    
    def action_server_request(self):
        if(self.test_pass):
            self.goal.position = self.test_pass_goal
        else:
            self.goal.position = self.test_fail_goal
        self.as_client.send_goal(self.goal)

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
        # Send goal to action server
        self.action_server_request()
    
        # Wait for result with timeout
        self.success_result = self.as_client.wait_for_result(rospy.Duration(10))

        if not self.success_result:
            self.as_client.cancel_goal()            
            self.stop_robot()
            rospy.logwarn("Goal was cancelled intentionally for test_fail case.")
        
        # Check if goal was reached
        self.assertTrue(self.success_result)
        
        # Check position (with tolerance)
        rospy.loginfo("Goal position: %s" % str(self.goal.position))
        rospy.loginfo("Current position: %s" % str(self.current_position))
        self.assertAlmostEqual(self.goal.position.x, self.current_position.x, delta=self.pos_tolerance)
        self.assertAlmostEqual(self.goal.position.y, self.current_position.y, delta=self.pos_tolerance)
        
    # def test_yaw_alignment(self):
    #     rospy.loginfo("Current Orientation: %s" % str(self.current_orientation))
    #     time.sleep(2)
    #     #rospy.wait_for_message("/odom", Odometry, timeout=2)
    #     self.final_yaw = self.euler_to_quaternion(self.current_orientation)
    #     rospy.loginfo("Final Yaw:  %s" % str(self.final_yaw))

    #     yaw_diff = self.init_yaw - self.final_yaw
    #     rospy.loginfo("Yaw Diff:  %s" % str(yaw_diff ))
    #     self.assertTrue((1.3 <= yaw_diff <= 2.1), "Integration error. Rotation was not between the expected values.")

    #     # Set initial position
    #     self.publish_odometry(0, 0, 0)
        
    #     # Send goal
    #     goal = WaypointActionGoal()
    #     goal.position = Point(1.0, 1.0, 0.0)
    #     self.client.send_goal(goal)
        
    #     # Wait for result with timeout
    #     self.client.wait_for_result(rospy.Duration(20))
        
    #     # Check if goal was reached
    #     self.assertTrue(self.client.get_result().success)
        
    #     # Calculate desired yaw (should be π/4 for (1,1) goal)
    #     desired_yaw = math.atan2(goal.position.y, goal.position.x)
        
    #     # Check yaw alignment (with tolerance)
    #     # Note: In a real test, you'd get the actual yaw from odometry
    #     self.assertAlmostEqual(desired_yaw, math.pi/4, delta=self.yaw_tolerance)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestActionServer)
