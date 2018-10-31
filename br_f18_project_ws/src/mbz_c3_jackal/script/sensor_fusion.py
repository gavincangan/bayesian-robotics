#!/usr/bin/env python
from __future__ import print_function

import sys
import cv2
import numpy as np
from math import pi, sin, cos, atan, tan, radians, degrees
from matplotlib import pyplot as plt
import os

import threading, time

import roslib
roslib.load_manifest('mbz_c3_jackal')
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovariance
from visualization_msgs.msg import Marker

from mbz_c3_jackal.msg import PositionPolar, Vector

from kalman_filter import *

class SensorFusion(KalmanFilter):
    
    def __init__(self, _dist, _theta):
        self.A = np.array([  \
            [1, 0, 1, 0 ], \
            [0, 1, 0, 1 ], \
            [0, 0, 1, 0 ], \
            [0, 0, 0, 1 ], \
        ])

        self.C = np.array([  \
            [1, 0, 0, 0 ],   \
            [0, 1, 0, 0 ],   \
        ])

        self.B = np.array([0])
        self.D = np.array([0])

        self.w = Gaussian.diagonal( [0, 0, 0, 0], [1e-5, 1e-4, 1e-6, 1e-4 ] )
        self.v = Gaussian.diagonal( [0, 0], [1e-2, 5e-5] )

        self.x = Gaussian.diagonal( [_dist, _theta, 0, 0], [1e-3, 1e-5, 1e-4, 1e-3] )

        self.yold = [_dist, _theta]

        # self.v_lidar = Gaussian.diagonal( [0, 0], [1e-8, 5e-1] )
        # self.v_cam = Gaussian.diagonal( [0, 0], [1e-2, 5e-7] )

        self.lidar_sub = rospy.Subscriber("/target/lidar_pos",PositionPolar, self.lidar_cb)
        self.cam_sub = rospy.Subscriber("/target/cam_pos", PositionPolar, self.camera_cb)

        self.marker_pub = rospy.Publisher("target/fused_marker", Marker, queue_size=32)
        self.out_pub = rospy.Publisher("/target/fused_pos", PositionPolar, queue_size=32)

    def correct(self, y, v=None):
        ty = np.append(y, [ y[ix]-self.yold[ix] for ix in range(len(y)) ] )
        KalmanFilter.correct(self, ty, v)
        self.yold = y

    def lidar_cb(self, data):
        data.covariance = np.array(data.covariance).reshape( (-1, data.cov_size) )

        y = np.array([ data.distance, data.heading ])
        self.correct( y, data.covariance[:2, :2] )

        # print('\nLIDAR:')
        # print(data.distance)
        # print(data.heading)
        # print(data.cov_size)
        # print(data.covariance)

    def camera_cb(self, data):
        data.covariance = np.array(data.covariance).reshape( (-1, data.cov_size) )

        y = np.array([ data.distance, data.heading ])
        self.correct( y, data.covariance[:2, :2] )

        # print('\nCamera:')
        # print(data.distance)
        # print(data.heading)
        # print(data.cov_size)
        # print(data.covariance)

    def predict(self, u=np.array([0]), w=None):
        self.publish()
        KalmanFilter.predict(self, u, w)

    def publish(self):
        next_pub = time.time()

        while True:
            x_mu, x_var = self.get_state()

            out_msg = PositionPolar()
            out_msg.distance = x_mu[0]
            out_msg.heading = x_mu[1]
            out_msg.cov_size = x_var.shape()[0]
            out_msg.covariance = x_var.flatten().tolist()

            self.out_pub.publish(out_msg)

            ## Create a marker for visualization
            angle = radians(x_mu[1])
            x = x_mu[0] * cos(angle)
            y = x_mu[0] * sin(angle)

            marker_msg = Marker()
            marker_msg.header.frame_id = "fused"
            marker_msg.id = 0
            marker_msg.type = 2 #Sphere
            marker_msg.pose.position.x = x
            marker_msg.pose.position.y = y
            marker_msg.pose.position.z = 0

            marker_msg.scale.x = 0.24
            marker_msg.scale.y = 0.24
            marker_msg.scale.z = 0.24

            marker_msg.color.r = 1.0
            marker_msg.color.g = 1.0
            marker_msg.color.b = 0.0
            marker_msg.color.a = 1.0

            self.marker_pub.publish(marker_msg)

            next_pub = next_pub + 0.1
            time.sleep( next_pub - time.time() )


def main():
    rospy.init_node('sensor_fusion', anonymous=True)
    sf = SensorFusion(0, 0)

    timerThread = threading.Thread(target=sf.publish)
    timerThread.daemon = True
    timerThread.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()