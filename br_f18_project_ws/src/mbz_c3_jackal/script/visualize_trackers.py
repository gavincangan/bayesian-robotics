#!/usr/bin/env python

import numpy as np
from math import pi, sin, cos, atan, tan, radians, degrees

import rospy

from visualization_msgs.msg import Marker

from mbz_c3_jackal.msg import PositionPolar, Vector

class AllMarkers:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber("/target/lidar_position",PositionPolar, self.lidar_cb)
        self.cam_sub = rospy.Subscriber("/target/cam_position", PositionPolar, self.camera_cb)
        self.fused_sub = rospy.Subscriber("/target/fused_position", PositionPolar, self.fused_cb)

        self.fused_marker_pub = rospy.Publisher("target/fused_marker2", Marker, queue_size=32)
        self.lidar_marker_pub = rospy.Publisher("target/lidar_marker2", Marker, queue_size=32)
        self.cam_marker_pub = rospy.Publisher("target/cam_marker2", Marker, queue_size=32)


    def publisher( self, data, pub=None, color=(1.0, 1.0, 1.0, 1.0) ):
        ## Create a marker for visualization
        angle = radians(data.heading)
        x = data.distance * cos(angle)
        y = data.distance * sin(angle)

        marker_msg = Marker()
        marker_msg.header.frame_id = "base_link"
        marker_msg.id = 0
        marker_msg.type = 2 #Sphere
        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = 0

	size = 0.22
	marker_msg.scale.x = size
	marker_msg.scale.y = size
	marker_msg.scale.z = size

        marker_msg.color.r = color[0]
        marker_msg.color.g = color[1]
        marker_msg.color.b = color[2]
        marker_msg.color.a = color[3]

        if(pub is None):
            pub = self.cam_marker_pub

        pub.publish(marker_msg)

    def fused_cb( self, data ):
        self.publisher( data, self.fused_marker_pub, color=(1.0, 1.0, 0.5, 0.85) )

    def lidar_cb( self, data ):
        self.publisher( data, self.lidar_marker_pub, color=(1.0, 0.0, 1.0, 0.75) )

    def camera_cb( self, data ):
        self.publisher( data, self.cam_marker_pub, color=(0.0, 0.0, 1.0, 0.75) )

def main():
    rospy.init_node('visualize_trackers', anonymous=True)

    markers = AllMarkers()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
