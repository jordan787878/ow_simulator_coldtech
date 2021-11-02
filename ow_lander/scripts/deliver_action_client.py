#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import ow_lander.msg
import argparse

def deliver_client():
    
    parser = argparse.ArgumentParser()
    parser.add_argument('x_start', type=float,
                        help='x coordinates of sample delivery location', nargs='?', default=0.55, const=0)
    parser.add_argument('y_start', type=float,
                        help='y coordinates of sample delivery location', nargs='?', default=-0.3, const=0)
    parser.add_argument('z_start', type=float,
                        help='z coordinates of sample delivery location', nargs='?', default=0.82, const=0)
    args = parser.parse_args()
    rospy.loginfo("Requested x_start: %s", args.x_start)
    rospy.loginfo("Requested y_start: %s", args.y_start)
    rospy.loginfo("Requested z_start: %s", args.z_start)

 
    client = actionlib.SimpleActionClient('Deliver', ow_lander.msg.DeliverAction)

    client.wait_for_server()

    goal = ow_lander.msg.DeliverGoal()
    
    goal.delivery.x = args.x_start
    goal.delivery.y = args.y_start
    goal.delivery.z = args.z_start
    # Default trenching values
    # goal.delivery.x = 0.55
    # goal.delivery.y = -0.3
    # goal.delivery.z = 0.82 
    
    # Remove Tailings values
    #goal.delivery.x = 1.5
    #goal.delivery.y = 0.8
    #goal.delivery.z = 0.65 

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # 

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the UnstowActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('deliver_client_py')
        result = deliver_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
