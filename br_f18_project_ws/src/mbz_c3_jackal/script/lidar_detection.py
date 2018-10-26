#!/usr/bin/env python

import rospy, math, random
import numpy as np
from sensor_msgs.msg import LaserScan
#from laser_geometry import LaserProjection

class Lidar:
    def __init__(self, scan_topic="/scan"):
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.on_scan)
        self.scan_pub = rospy.Publisher("/scan_filtered", LaserScan, queue_size=10)
        #self.laser_projector = LaserProjection()

    def on_scan(self, scan):
        rospy.loginfo("Got scan")

        bearing = 0
        bearing_offset = 0.3

        scan_filtered = self.GetScanInRange(scan, bearing-bearing_offset, bearing+bearing_offset)
        self.scan_pub.publish(scan_filtered)

        """
        cloud = self.laser_projector.projectLaser(scan)
        """

    def GetScanInRange(self, scan, angle_min, angle_max):
        scan_filtered = LaserScan()
        scan_filtered.header = scan.header
        scan_filtered.angle_increment = scan.angle_increment
        scan_filtered.range_max = scan.range_max
        scan_filtered.range_min = scan.range_min
        scan_filtered.scan_time = scan.scan_time
        scan_filtered.time_increment = scan.time_increment
        
        ## Round the input range to the closest valid angle
        num_pts = len(scan.ranges)-1

        min_idx = int(round((angle_min-scan.angle_min)/scan.angle_increment))
        if min_idx < 0:
            print("Warning: angle_min is less than minimum scan angle.")
            min_idx = 0
        elif min_idx > num_pts:
            print("Warning: angle_min is less than maximum scan angle.")
            min_idx = num_pts
        angle_min = scan.angle_min + min_idx*scan.angle_increment

        max_idx = int(round((angle_max-scan.angle_min)/scan.angle_increment))
        if max_idx < 0:
            print("Warning: angle_max is less than minimum scan angle.")
            max_idx = 0
        elif max_idx > num_pts:
            print("Warning: angle_max is less than maximum scan angle.")
            max_idx = num_pts
        angle_max = scan.angle_min + max_idx*scan.angle_increment
        
        ## Output the final values
        scan_filtered.angle_min = angle_min
        scan_filtered.angle_max = angle_max
        scan_filtered.ranges = scan.ranges[min_idx:max_idx]

        """
        for i in range(len(scan.ranges)):
            angle = angle_min + angle_inc*i
            angle_deg = math.degrees(angle)
            print("Angle: {:3.2f} deg, Range: {:2.2f}".format(angle_deg, scan.ranges[i]))
        """
        return scan_filtered

if __name__=="__main__":
    rospy.init_node("lidar_detection", anonymous=True)

    Lidar()
    rospy.spin()