#!/usr/bin/env python
from __future__ import print_function

import sys
import cv2
import numpy as np
from math import pi
from matplotlib import pyplot as plt
import os

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
            [1, 0, 0, 0, 1, 0, 0, 0 ], \
            [0, 1, 0, 0, 0, 1, 0, 0 ], \
            [0, 0, 1, 0, 0, 0, 1, 0 ], \
            [0, 0, 0, 1, 0, 0, 0, 1 ], \
            [0, 0, 0, 0, 1, 0, 0, 0 ], \
            [0, 0, 0, 0, 0, 1, 0, 0 ], \
            [0, 0, 0, 0, 0, 0, 1, 0 ], \
            [0, 0, 0, 0, 0, 0, 0, 1 ], \
        ])

        self.C = np.array([  \
            [1, 0, 0, 0, 0, 0, 0, 0 ],   \
            [0, 1, 0, 0, 0, 0, 0, 0 ],   \
            [0, 0, 1, 0, 0, 0, 0, 0 ],   \
            [0, 0, 0, 1, 0, 0, 0, 0 ],   \
        ])

        self.B = np.array([0])
        self.D = np.array([0])

        self.w = Gaussian.diagonal( [0, 0, 0, 0, 0, 0, 0, 0 ], [1e-5, 1e-4, 1e-6, 1e-4, 1e-5, 1e-4, 1e-6, 1e-4 ] )
        self.v = Gaussian.diagonal( [0, 0, 0, 0], [1e-2, 5e-5, 1e-3, 1e-4] )

        self.x = Gaussian.diagonal( [_dist_cam, _theta_cam, _dist_lidar, _theta_lidar, 0, 0, 0, 0], [1e-3, 1e-3, 1e-5, 1e-4, 1e-3, 1e-3, 1e-5, 1e-4] )

        self.yold = [_dist, _theta]

        self.lidar_sub = rospy.Subscriber("/target/lidar_pos",PositionPolar, self.lidar_cb)
        self.cam_sub = rospy.Subscriber("/target/cam_pos", PositionPolar, self.cam_cb)
        
        self.out_pub = rospy.Publisher("/target/lidar_pos", PositionPolar, queue_size=32)

    def correct(self, y):
        ty = np.append(y, [ y[ix]-self.yold[ix] for ix in range(len(y)) ] )
        # pdb.set_trace()
        KalmanFilter.correct(self, ty)
        self.yold = y

    def lidar_cb(self, data):
        data.covariance = np.array(data.covariance).reshape( (-1, data.cov_size) )

        print('\nLIDAR:')
        print(data.distance)
        print(data.heading)
        print(data.cov_size)
        print(data.covariance)
        pass

    def cam_cb(self, data):
        data.covariance = np.array(data.covariance).reshape( (-1, data.cov_size) )

        print('\nCamera:')
        print(data.distance)
        print(data.heading)
        print(data.cov_size)
        print(data.covariance)
        pass

def main():
    rospy.init_node('sensor_fusion', anonymous=True)

    sf = SensorFusion(0, 0)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()