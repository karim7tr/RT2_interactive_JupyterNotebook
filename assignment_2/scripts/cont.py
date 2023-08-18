#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from assignment_2_2022.msg import relxy_posxy
import assignment_2_2022.msg
from std_srvs.srv import *
import sys
import select
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import time
import math

def update_current_robot_state(msg):
    global current_state_publisher
    # extract the current position and velocity from the message
    position = msg.pose.pose.position
    velocity = msg.twist.twist.linear
    # create a custom message to store and publish the current state
    current_state = relxy_posxy()
    current_state.cor_x = position.x
    current_state.cor_y = position.y
    current_state.vel_x = velocity.x
    current_state.vel_y = velocity.y
    current_state_publisher.publish(current_state)

def main():
    global current_state_publisher
    # create a publisher to send the current state of the robot
    current_state_publisher = rospy.Publisher("/pos_vel", relxy_posxy, queue_size = 1)
    # subscribe to the odometry topic to receive updates on the robot's position and velocity
    odom_subscriber = rospy.Subscriber("/odom", Odometry, update_current_robot_state)
    # create a client for the reaching_goal action server
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)
    rospy.loginfo('Waiting for server to start...')
    client.wait_for_server()
    while not rospy.is_shutdown():
        # get the desired x and y position from the user
        x_des = float(input("Please enter the desired X position: "))
        y_des = float(input("Please enter the desired Y position: "))
        rospy.loginfo("Target position is set!")
        goal = assignment_2_2022.msg.PlanningGoal()
        goal.target_pose.pose.position.x = x_des
        goal.target_pose.pose.position.y = y_des
        client.send_goal(goal)
        # check if the user wants to cancel the goal
        c_input = input("Please type 'cancel' to cancel the goal: ")
        if (c_input == "cancel"):
            print("Goal canceled!")
            client.cancel_goal()
        else:
            continue

if __name__ == '__main__':
    rospy.init_node('user_input')
    main()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
