#! /usr/bin/env python3

import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from rosbot_bath.srv import distanceHandler
from std_msgs.msg import Float32

odom = None
distance = 0
total_distance = 0
fwd_distance = 0
total_fwd_distance = 0
yaw_error = 0

def yaw_callback(msg):
    global yaw_error
    yaw_error = msg.data
    print("Type of yaw_error:", type(yaw_error))
def odom_callback(msg):
    global odom, total_distance, distance,fwd_distance, total_fwd_distance, yaw_error
    if odom is not None:

        dx = msg.pose.pose.position.x - odom.pose.pose.position.x
        dy = msg.pose.pose.position.y - odom.pose.pose.position.y

        distance = np.sqrt(dx**2+dy**2)
        fwd_distance = distance * math.cos(yaw_error)
        total_distance += distance
        total_fwd_distance += fwd_distance
    
    odom = msg

# Service handler function
def handle_service(request):
    # Process the request and generate a response
    global total_distance, total_fwd_distance
    return total_distance,total_fwd_distance


def main():
    rospy.init_node('odomHandler')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('yawHandler', Float32, yaw_callback)
    rospy.Service('distanceHandler', distanceHandler, handle_service)
    rospy.spin()

if __name__ == '__main__':
    main()
