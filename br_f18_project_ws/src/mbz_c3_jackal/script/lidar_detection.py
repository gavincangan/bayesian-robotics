#!/usr/bin/env python

import rospy, math, random
import numpy as np
from sensor_msgs.msg import LaserScan
#from laser_geometry import LaserProjection

from mbz_c3_jackal.msg import PositionPolar, Vector
from visualization_msgs.msg import Marker

class Lidar:
    def __init__(self, scan_topic="/scan"):
        self.bearing = 0.0
        self.bearing_offset = 0.1

        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.on_scan)
        self.cam_pos_sub = rospy.Subscriber("target/cam_position",PositionPolar, self.on_cam_pos)
        self.marker_pub = rospy.Publisher("target/lidar_marker",Marker, queue_size=32)
        self.out_pub = rospy.Publisher("target/lidar_position",PositionPolar, queue_size=32)
        self.scan_pub = rospy.Publisher("/scan/filtered", LaserScan, queue_size=10)
        
        #self.laser_projector = LaserProjection()

    
    def on_cam_pos(self, msg):
        rospy.loginfo("Got cam_position")

        self.bearing = math.radians(msg.heading)
        #self.bearing_offset = msg.distance/10


    def on_scan(self, scan):
        rospy.loginfo("Got scan")

        scan_filtered = self.GetScanInRange(scan, self.bearing-self.bearing_offset, self.bearing+self.bearing_offset)
        self.scan_pub.publish(scan_filtered)

        ## Similar to median, but get the 30% closest point rather than the 50%
        dist = np.percentile(scan_filtered.ranges, 30)

        out_msg = PositionPolar()
        out_msg.distance = dist
        out_msg.heading = self.bearing
        out_msg.cov_size = 2
        out_msg.covariance = np.array([
            [0.2, 0],   \
            [0, 9999],   \
        ]).flatten().tolist()

        self.out_pub.publish(out_msg)
        
        

        ## Create a marker for visualization
        x = dist * math.cos(self.bearing)
        y = dist * math.sin(self.bearing)

        marker_msg = Marker()
        marker_msg.header.frame_id = "laser"
        marker_msg.id = 0
        marker_msg.type = 2 #Sphere
        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = 0

        marker_msg.scale.x = 0.24
        marker_msg.scale.y = 0.24
        marker_msg.scale.z = 0.24

        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 1.0
        marker_msg.color.a = 1.0

        self.marker_pub.publish(marker_msg)


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