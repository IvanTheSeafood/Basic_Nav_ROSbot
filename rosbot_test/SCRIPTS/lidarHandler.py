#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rosbot_bath.msg import lidarHandler

pub = rospy.Publisher('lidarHandler',lidarHandler, queue_size = 3)
last_print_time = 0

def clbk_laser(msg):
    global last_print_time

    if last_print_time is None:
        last_print_time = rospy.Time.now()
    
    step = 2
    
    region =lidarHandler()

    lfront = slice(0*step,18*step)
    rfront = slice(342*step,360*step)
    right = slice(252*step, 288*step)
    left = slice(72*step, 108*step)
    
    region.front= min(min(msg.ranges[lfront] + msg.ranges[rfront]), 10)
    region.right= min(min(msg.ranges[right]), 10)
    region.left= min(min(msg.ranges[left]), 10)

    pub.publish(region)
 

def main():
    global pub
    rospy.init_node('lidarHandler')

    # Subscribers
    rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()