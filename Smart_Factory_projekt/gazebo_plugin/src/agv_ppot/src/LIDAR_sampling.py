#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

rospy.init_node('LIDAR_scan', anonymous=True) #Node Path planning "Pola Potencjalne" (eng. potential field)


sampling_rate = 100

pub1 = rospy.Publisher('LIDAR_scan', LaserScan, queue_size=sampling_rate) #Publisher "pub1" to publish at topic "LIDAR_scan" to send message with laser scan data

rate = rospy.Rate(sampling_rate)

data_acq = 0

scan_data = LaserScan()
def LIDAR_scan_fun(data): 
    global scan_data
    global sub_scan
    global data_acq
    
    scan_data = data
    
    scan_data.ranges = np.nan_to_num(scan_data.ranges, posinf=40)
    
    data_acq = 1
    
    
sub_scan = rospy.Subscriber('/scan', LaserScan, LIDAR_scan_fun)      #Identify the subscriber "sub_scan" to subscribe topic containing laser scan data


iterator = 1

while 1:
    
    if data_acq == 1:
        print("ranges nr " + str(iterator) + "\n\n")
        print(scan_data.ranges)
        pub1.publish(scan_data)
        data_acq = 0
        rate.sleep()
    
    
    