#!/usr/bin/env python
from __future__ import print_function

import sys
import cv2
import numpy as np
from math import pi, sin, cos, atan, tan, radians, degrees
from matplotlib import pyplot as plt
import os

# import pdb

import threading, time

import roslib
roslib.load_manifest('mbz_c3_jackal')
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovariance, PoseWithCovarianceStamped
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
            [0, 0, 1, 0 ], \
            [0, 0, 0, 1 ], \
        ])

        self.B = np.array([0])
        self.D = np.array([0])

        self.w = Gaussian.diagonal( [0, 0, 0, 0], [3e-1, 3e-1, 5e-2, 5e-2 ] )
        self.v = Gaussian.diagonal( [0, 0, 0, 0], [1e-2, 5e-2, 1e-2, 5e-2] )

        self.x = Gaussian.diagonal( [_dist, _theta, 0, 0], [9999, 9999, 1e+2, 1e+2] )

        self.yold = { 'camera':[_dist, _theta], 'lidar':[_dist, _theta] }

        # self.v_lidar = Gaussian.diagonal( [0, 0], [1e-8, 5e-1] )
        # self.v_cam = Gaussian.diagonal( [0, 0], [1e-2, 5e-7] )

        self.lidar_sub = rospy.Subscriber("/target/lidar_position",PositionPolar, self.lidar_cb)
        self.cam_sub = rospy.Subscriber("/target/cam_position", PositionPolar, self.camera_cb)

        self.marker_pub = rospy.Publisher("target/fused_marker", Marker, queue_size=32)
        self.out_pub = rospy.Publisher("/target/fused_position", PositionPolar, queue_size=32)
        self.out_pose = rospy.Publisher("/target/pose_with_cov", PoseWithCovarianceStamped, queue_size=32)

    def correct(self, y, v=None, sensor='camera'):
        self.predict()

        ty = np.append(y, [ y[ix]-self.yold[sensor][ix] for ix in range(len(y)) ] )

        # print('y: ', ty)
        # print('cov:\n', v)

        KalmanFilter.correct(self, ty, v)
        self.yold[sensor] = y

    def lidar_cb(self, data):
        # rospy.loginfo("SF: Got LIDAR position")

        data.covariance = np.array(data.covariance).reshape( (-1, data.cov_size) )

        y = np.array([ data.distance, data.heading ])
        self.correct( y, data.covariance[:4, :4], sensor='lidar' )

        # print('\nLIDAR:')
        # print(data.distance)
        # print(data.heading)
        # print(data.cov_size)
        # print(data.covariance)

    def camera_cb(self, data):
        # rospy.loginfo("SF: Got camera position")

        data.covariance = np.array(data.covariance).reshape( (-1, data.cov_size) )

        y = np.array([ data.distance, data.heading ])
        self.correct( y, data.covariance[:4, :4], sensor='camera' )

        # print('\nCamera:')
        # print(data.distance)
        # print(data.heading)
        # print(data.cov_size)
        # print(data.covariance)

    def predict(self, u=np.array([0]), w=None):
        KalmanFilter.predict(self, u, w)

    def publish(self):
        next_pub = time.time()

        count = 0
        msg_seq = 1
        while True:
            # count += 1
            # if count%1 == 0:

            x_mu, x_var = self.get_state()

            out_msg = PositionPolar()
            out_msg.distance = x_mu[0]
            out_msg.heading = x_mu[1]
            out_msg.cov_size = x_var.shape[0]
            out_msg.covariance = x_var.flatten().tolist()

            self.out_pub.publish(out_msg)

            ## Create a marker for visualization
            angle = radians(x_mu[1])
            x = x_mu[0] * cos(angle)
            y = x_mu[0] * sin(angle)

            marker_msg = Marker()
            marker_msg.header.frame_id = "laser"
            marker_msg.id = 0
            marker_msg.type = 2 #Sphere
            marker_msg.pose.position.x = x
            marker_msg.pose.position.y = y
            marker_msg.pose.position.z = 0

            marker_msg.scale.x = 0.27
            marker_msg.scale.y = 0.27
            marker_msg.scale.z = 0.27

            marker_msg.color.r = 1.0
            marker_msg.color.g = 1.0
            marker_msg.color.b = 0.5
            marker_msg.color.a = 0.85

            self.marker_pub.publish(marker_msg)

            #Point, Quaternion, Pose, PoseWithCovariance
            pose_with_cov = PoseWithCovarianceStamped()


            pose_with_cov.header.seq = msg_seq
            msg_seq += 1
            pose_with_cov.header.stamp = rospy.get_rostime()
            pose_with_cov.header.frame_id = "laser"

            pose_with_cov.pose.pose.position.x = x
            pose_with_cov.pose.pose.position.y = y
            pose_with_cov.pose.pose.position.z = 0
            
            pose_with_cov.pose.pose.orientation.w = 1

            jac = np.array([    [ np.cos(angle) , np.sin(angle) ],  \
                                [ -np.sin(angle), np.cos(angle) ]   \
                            ])
            cov_rt = self.x.var[:2, :2]
            cov_xy = np.eye(6)
            cov_xy[:2, :2] = np.dot( np.dot( jac, cov_rt ), jac.T )
            # cov_xy = np.pad(cov_xy, ((0, 4), (0, 4)), 'constant', constant_values=(1, 1) )
            pose_with_cov.pose.covariance = cov_xy.flatten().tolist()

            self.out_pose.publish(pose_with_cov)

            # next_pub = next_pub + 0.1          
            # time.sleep( next_pub - time.time() )

                # if(count > 1000):
                #     count %= 1000

            rate = rospy.Rate(10)
            rate.sleep()


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