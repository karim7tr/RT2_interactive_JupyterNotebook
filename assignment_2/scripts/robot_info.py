#! /usr/bin/env python

import rospy
import math
import time
from assignment_2_2022.msg import relxy_posxy

previous_print_time = 0
frequency_param = 1.0

def print_robot_info(msg):
    global previous_print_time, frequency_param
    time_period = (1.0/frequency_param) * 1000
    current_time = time.time() * 1000

    if current_time - previous_print_time > time_period:
        target_x = rospy.get_param("des_pos_x")
        target_y = rospy.get_param("des_pos_y")
        current_x = msg.cor_x
        current_y = msg.cor_y
        distance = math.dist([target_x, target_y], [current_x, current_y])
        average_speed = math.sqrt(msg.vel_x**2 + msg.vel_y**2)
        print(' \n We are {:.3f}m far from our final postion'.format(float(distance)))
        print(' \n The average speed is {:.3f} m\s'.format(float(average_speed)))
        previous_print_time = current_time

if __name__ == "__main__":
    rospy.init_node('rob_info')
    frequency_param = rospy.get_param("frequency")
    pos_vel_subscriber = rospy.Subscriber("/pos_vel", relxy_posxy, print_robot_info)
    rospy.spin()
