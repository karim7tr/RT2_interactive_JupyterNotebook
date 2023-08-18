#! /usr/bin/env python

import rospy
from assignment_2_2022.srv import count, goal_srvResponse
import actionlib
import actionlib.msg
import assignment_2_2022.msg

def goal_callback(goal):
    # create a new action client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)
    client.wait_for_server()

    # send the goal to the action server
    client.send_goal(goal)

    # wait for the result
    client.wait_for_result()

    # get the result
    result = client.get_result()

    # print the result
    rospy.loginfo("Result: {}".format(result.result))

def info_callback(req):
    rospy.loginfo("Received request with goal: {}".format(req.goal))
    return goal_srvResponse()

if __name__ == "__main__":
    rospy.init_node('goal_client')
    rospy.Subscriber('/goal', assignment_2_2022.msg.PlanningGoal, goal_callback)
    srv = rospy.Service('goal_info', goal_srv, info_callback)
    rospy.spin()
