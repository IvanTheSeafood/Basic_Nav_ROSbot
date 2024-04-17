#!/usr/bin/env python3

import rospy
import signal
import sys
from rosbot_bath.srv import distanceHandler

total_distance, total_fwd_distance = 0, 0

def termination_handler(signal, frame):
    global total_distance, total_fwd_distance

    # Ask for distance when the program is terminated
    rospy.wait_for_service('distanceHandler')  
    
    # Create a handle to the service
    odom_service = rospy.ServiceProxy('distanceHandler', distanceHandler)
        
    # Call the service with the provided arguments
    response = odom_service()

    # Process the response
    total_distance, total_fwd_distance = response.total_distance, response.total_fwd_distance
    rospy.loginfo("Program Terminated")
    rospy.loginfo("Total Distance: %s, Total FWD Distance: %s", total_distance, total_fwd_distance)
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, termination_handler)
    rospy.init_node("terminate")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        # Catch KeyboardInterrupt (Ctrl+C) and call termination handler
        termination_handler(signal.SIGINT, None)
