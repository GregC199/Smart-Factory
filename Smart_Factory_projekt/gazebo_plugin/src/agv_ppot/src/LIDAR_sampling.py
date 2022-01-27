#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

rospy.init_node('LIDAR_scan', anonymous=True) #Node Path planning "Pola Potencjalne" (eng. potential field)


sampling_rate = int(1.0/rospy.get_param("~sampling_time"))
d_nonlimit = rospy.get_param("~d_nonlimit")

pub1 = rospy.Publisher('LIDAR_scan', LaserScan, queue_size=500) #Publisher "pub1" to publish at topic "LIDAR_scan" to send message with laser scan data

rate = rospy.Rate(sampling_rate)

data_acq = 0

scan_data = LaserScan()
def LIDAR_scan_fun(data): 
    global scan_data
    global sub_scan
    global data_acq
    
    scan_data = data
    
    scan_data.ranges = np.nan_to_num(scan_data.ranges, posinf=40)
    
    scan_data.ranges = np.where(scan_data.ranges > (2.5), 40, scan_data.ranges)
    
    data_acq = 1
    
    
sub_scan = rospy.Subscriber('/scan', LaserScan, LIDAR_scan_fun)      #Identify the subscriber "sub_scan" to subscribe topic containing laser scan data

#iterator = 1


while 1 and not rospy.is_shutdown():
    
    if data_acq == 1:
        #print("\n range " + str(iterator) + "\n\n")
        #iterator = iterator + 1
        #print(scan_data.ranges)
        pub1.publish(scan_data)
        data_acq = 0
        rate.sleep()
    
    
    