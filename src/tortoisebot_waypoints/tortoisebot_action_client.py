#! /usr/bin/python3

import rospy
import actionlib
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal
from geometry_msgs.msg import Point

if __name__ == '__main__':
    rospy.init_node('tortoisebot_as_client')
    client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
    client.wait_for_server()
    
    goal = WaypointActionGoal(position=Point(x=0.0, y=-0.25, z=0.0))
    
    client.send_goal(goal)
    print("Goal sent to (0, -0.25, 0)")
    
    client.wait_for_result()
    print("Result:", client.get_result())
