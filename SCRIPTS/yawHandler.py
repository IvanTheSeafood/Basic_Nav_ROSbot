#! /usr/bin/env python3

import rospy
import tf
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

target_yaw = 0
current_yaw = 0

pub = rospy.Publisher('yawHandler', Float32, queue_size =2)

def imu_clbk(msg):
    global target_yaw, current_yaw

    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
        )

    # convert to Euler 
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    current_yaw = euler[2]

    angle = target_yaw-current_yaw
    
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi

    #rospy.loginfo("Current Yaw: %f, Angle: %f", current_yaw, angle)
    msg=Float32()
    msg.data = angle
    pub.publish(msg) 

def main():

    global pub
    rospy.init_node('yawHandler')

    # Subscribers
    rospy.Subscriber('/imu', Imu, imu_clbk)

    rospy.spin()

if __name__ == '__main__':
    main()