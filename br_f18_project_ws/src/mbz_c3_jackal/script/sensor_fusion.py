#!/usr/bin/env python
from __future__ import print_function

import sys
import cv2
import numpy as np
from math import pi
from matplotlib import pyplot as plt
import os

import time, threading

import roslib
roslib.load_manifest('mbz_c3_jackal')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovariance

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
        
        self.out_pub = rospy.Publisher("/target/lidar_pos", PositionPolar, queue_size=32)

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

def main():
    rospy.init_node('sensor_fusion', anonymous=True)
    sf = SensorFusion(0, 0)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()