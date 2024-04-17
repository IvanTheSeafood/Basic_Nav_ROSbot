#! /usr/bin/env python3

import time
import numpy as np
import rospy
import tf
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from rosbot_bath.srv import distanceHandler
from rosbot_bath.msg import lidarHandler


# States
current_state = "Forward"
last_state = None
command = None

# Global variables
last_print_time = None
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#pub = rospy.Publisher('/processor', String, queue_size =2)
odom = None
target_yaw = 0
current_yaw = None
total_distance = 0
total_fwd_distance = 0
prev_distance = 0
distance_in_fwd = 0
yaw_error = 0

# Range sensors
range_fl = 0.7 
range_fr = 0.7

# Global variable to track the last turn time
last_turn_time = 0
turn_delay = 1.0  # Delay 


# State definitions
FORWARD = "Forward"
STOP = "Stop"
LEFT = "Left"
RIGHT = "Right"
AVOID_OBSTACLE_LEFT = "AvoidLeft"
AVOID_OBSTACLE_RIGHT = "AvoidRight"
CORRECTION = "Correction"

# Range sensors clbks
def Rangefl(msg):
    global range_fl
    range_fl = msg.range

def Rangefr(msg):
    global range_fr
    range_fr = msg.range                  

def yaw_callback(msg):
    global yaw_error
    yaw_error=msg.data

def laser_callback(msg):
    global current_state, last_print_time, distance_in_fwd, yaw_error, range_fl, range_fr

    regions = {
        'front': msg.front,
        'right': msg.right,
        'left': msg.left,
    }

    # State transition logic based on LiDAR data
    safe_distance = 0.6  # Safe distance threshold for laser
    safe_dist_rng = 0.5
    if regions['front'] < safe_distance or range_fl < safe_dist_rng or range_fr < safe_dist_rng :
        if regions['left'] < regions['right']:
            current_state = AVOID_OBSTACLE_RIGHT
        else:
            current_state = AVOID_OBSTACLE_LEFT
    
    elif distance_in_fwd >= 1:
            
            if abs(yaw_error) > 0.1:
                current_state = CORRECTION
            else:
                current_state = FORWARD
    
    else:
        current_state = FORWARD
    print(current_state, distance_in_fwd)
    odom_request()
    fsm_action() 

def fsm_action():
    global command, current_state, last_turn_time, last_state, distance_in_fwd, yaw_error,prev_distance
    current_time = time.time()

    if current_state == FORWARD:
        if last_state != FORWARD:
            #rospy.loginfo("New FORWARD state detected. Resetting distance.")
            prev_distance = total_distance
        distance_in_fwd = total_distance-prev_distance    
        move_forward()
    elif current_state == CORRECTION:

        # Check if the robot is close enough to the target orientation
        if abs(yaw_error) <= 0.1:  # Threshold to exit correction state
            current_state = FORWARD

        else:
            turn_correction(yaw_error)
            
    elif current_state == AVOID_OBSTACLE_LEFT and (current_time - last_turn_time) > turn_delay:
        turn_right()
        last_turn_time = current_time
    elif current_state == AVOID_OBSTACLE_RIGHT and (current_time - last_turn_time) > turn_delay:
        turn_left()
        last_turn_time = current_time
    
    last_state = current_state

def turn_correction(yaw_error):
    Kp = 1.5  # Gain factor, adjust as needed
    angular_velocity = Kp * yaw_error

    # Limit the angular velocity to avoid overcorrection
    max_angular_velocity = 1
    angular_velocity = max(min(angular_velocity, max_angular_velocity), -max_angular_velocity)
    msg = Twist()
    msg.angular.z = angular_velocity
    pub.publish(msg) # control publish, not mode publish, del later

def turn_left():
    msg = Twist()
    msg.linear.x = -0.15  # Slight backward movement
    msg.angular.z = 1.0  # Left turn
    pub.publish(msg)
    rospy.sleep(0.5)  # Back up for a short duration before turning

def turn_right():
    msg = Twist()
    msg.linear.x = -0.15  # Slight backward movement
    msg.angular.z = -1.0  # Right turn
    pub.publish(msg)
    rospy.sleep(0.5) 

def move_forward():
    msg = Twist()
    msg.linear.x = 0.5  # Forward speed
    msg.angular.z = 0   # No rotation
    pub.publish(msg)

def odom_request():
    global last_state, current_state,total_distance, total_fwd_distance

    if current_state!=last_state or current_state == FORWARD:   #ask for distance when state is diff
        rospy.wait_for_service('distanceHandler')  
    
        # Create a handle to the service
        odom_service = rospy.ServiceProxy('distanceHandler', distanceHandler)
        
        # Call the service with the provided arguments
        response = odom_service()

        # Process the response
        
        total_distance, total_fwd_distance = response.total_distance, response.total_fwd_distance
        rospy.loginfo("Total Distance: %s, Total FWD Distance: %s", total_distance, total_fwd_distance)

def main():
    global pub
    rospy.init_node('objectAvoid')

    # Subscribers
    rospy.Subscriber('/range_fl', Range, Rangefl)
    rospy.Subscriber('/range_fr', Range, Rangefr)
    rospy.Subscriber('lidarHandler', lidarHandler, laser_callback)
    rospy.Subscriber('yawHandler', Float32, yaw_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
