#!/usr/bin/env python
from __future__ import print_function

import sys
import cv2
import numpy as np
from math import pi, sin, cos, atan, tan, radians, degrees
# from matplotlib import pyplot as plt
import os, threading, time


import roslib
roslib.load_manifest('mbz_c3_jackal')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from visualization_msgs.msg import Marker
from mbz_c3_jackal.msg import PositionPolar, Vector
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

from kalman_filter import KalmanFilter, Gaussian
from ext_kalman_filter import ExtKalmanFilter

class DataType(object):
    INPUT=1
    SLAM=1
    IMU=2
    FUSED=3
    CAM=4
    LIDAR=5

class StateTracker(ExtKalmanFilter):
    
    def __init__(self):

        self.w = Gaussian.diagonal( [0, 0, 0, 0, 0], [1e-4, 1e-4, 5e-3, 1e-1, 1e-1] )
        self.v = Gaussian.diagonal( [0, 0, 0], [5e-5, 5e-5, 1e-3] )

        self.v_SLAM = Gaussian.diagonal( [0, 0, 0], [5e-5, 5e-5, 1e-3] )
        self.v_FUSED = Gaussian.diagonal( [0, 0], [5e-3, 5e-3] )
        self.v_LIDAR = Gaussian.diagonal( [0, 0], [5e-3, 5e-1] )
        self.v_CAM = Gaussian.diagonal( [0, 0], [1e0, 5e-3] )

        self.x = Gaussian.diagonal( [0, 0, 0, 0, 0], [1e-3, 1e-3, 1e-3, 1e1, 1e1] )

        self.mahalonobis_threshold = 1.0
    
        # u --> linear vel, angular vel
        self.u = np.array([0.0, 0.0], dtype=float)

        self.input_sub = rospy.Subscriber("/cmd_vel", Twist, self.input_callback)
        # self.imu_sub = rospy.Subscriber("/imu/data_raw", Imu, self.imu_callback)
        self.slam_sub = rospy.Subscriber("/slam_out_pose", PoseStamped, self.slam_callback)
        self.cam_sub = rospy.Subscriber("/target/cam_position", PositionPolar, self.fused_callback)
        self.lidar_sub = rospy.Subscriber("/target/lidar_position", PositionPolar, self.fused_callback)

        self.marker_pub = rospy.Publisher("target/mm_fused_marker", Marker, queue_size=32)
        self.out_pose = rospy.Publisher("/robot/fused_pose_with_cov", PoseWithCovarianceStamped, queue_size=32)

        self.updated_flags = np.array([ False, False, False ], dtype=bool)

        self.time_last = np.array([ 0.0, 0.0, 0.0 ], dtype=float)
        self.time_diff = np.array([ 0.0, 0.0, 0.0 ], dtype=float)

        self.z_slam = np.array([0.0, 0.0, 0.0])
        self.z_fused = np.array([0.0, 0.0])
        self.z_cam = np.array([0.0, 0.0])
        self.z_lidar = np.array([0.0, 0.0])

        self.jacA = np.eye(5)

        self.jacC_SLAM = np.zeros((3, 5))
        self.jacC_SLAM[0, 0] = 1
        self.jacC_SLAM[1, 1] = 1
        self.jacC_SLAM[2, 2] = 1

        self.jacC_FUSED = np.zeros((2,5))
        self.jacC_FUSED[0, 3] = 1
        self.jacC_FUSED[1, 4] = 1

        self.jacC_FUSED = np.zeros((2,5))
        self.jacC_FUSED[0, 3] = 1
        self.jacC_FUSED[1, 4] = 1

        # ([
        #         [   1,  0,  0,  0,  0   ], \
        #         [   0,  1,  0,  0,  0   ], \
        #         [   0,  0,  1,  0,  0   ],  \
        #         [   0,  0,  0,  0,  0   ],  \
        #         [   0,  0,  0,  0,  0   ]  \
        # ])

        self.input_timestamp_last = time.time()

    def nextState(self):
        # x.mu --> Xrobot, Yr, Theta_r, Xball, Yb
        dt = 0.5

        dXr = self.u[0] * np.cos( self.x.mu[2] ) * dt
        dYr = self.u[0] * np.sin( self.x.mu[2] ) * dt

        self.x.mu[0] = self.x.mu[0] + dXr
        self.x.mu[1] = self.x.mu[1] + dYr
        self.x.mu[2] += self.u[1] * dt

        self.x.mu[3] = self.x.mu[3] + dXr
        self.x.mu[4] = self.x.mu[4] + dYr


        self.u[0] = 0.0
        self.u[1] = 0.0
        # print(dt, dXr, dYr, '\n', self.u , '\n', self.x.mu)

    def jacobianA(self):
        # u[0] --> linear velocity
        # x[2] --> theta
        self.jacA[0, 2] = -self.u[0] * np.cos(self.x.mu[2])
        self.jacA[1, 2] = self.u[0] * np.sin(self.x.mu[2])
        return self.jacA

    def expMeasurement(self, DATATYPE=None):
        if DATATYPE==DataType.SLAM:
            return self.x.mu[0:3]
        elif DATATYPE==DataType.FUSED:
            return self.x.mu[3:]
        elif DATATYPE==DataType.IMU:
            raise NotImplementedError

    def jacobianC(self, DATATYPE=None):
        if DATATYPE==DataType.SLAM:
            return self.jacC_SLAM
        elif DATATYPE==DataType.FUSED:
            return self.jacC_FUSED

    def slam_callback(self, data):
        DATATYPE = DataType.SLAM
        # rospy.loginfo("Received position from SLAM")
        # self.update_time_diff( data.header.stamp.secs + 1e-9* data.header.stamp.nsecs, DATATYPE=DATATYPE )

        # if self.is_updated(DATATYPE):
        self.z_slam[0] = data.pose.position.x
        self.z_slam[1] = data.pose.position.y
        self.z_slam[2] = data.pose.orientation.z
        ExtKalmanFilter.correct(self, self.z_slam, self.v_SLAM.var, DATATYPE=DataType.SLAM)

    def imu_callback(self, data):
        raise NotImplementedError
        # DATATYPE = DataType.IMU
        # self.update_time_diff( data.header.stamp.secs + 1e-9* data.header.stamp.nsecs, DATATYPE=DATATYPE )
        # pass

    def input_callback(self, data):
        # u --> linear vel, angular vel
        log_str = "Received cmd_vel {}, {}".format(data.linear.x, data.angular.z)
        rospy.loginfo(log_str)

        self.u[0] = data.linear.x
        self.u[1] = data.angular.z

    def fused_callback(self, data):
        self.z_lidar[0] = data.distance * np.cos( data.heading )
        self.z_lidar[1] = data.distance * np.sin( data.heading )

        data.covariance = np.array(data.covariance[:4]).reshape( (-1, 2) )

        jac = np.array([    [ np.cos(data.heading) , np.sin(data.heading) ],  \
                            [ -np.sin(data.heading), np.cos(data.heading) ]   \
                        ])
        self.v_FUSED.var = np.dot( np.dot( jac, data.covariance ), jac.T )
        if abs( np.linalg.det( self.v_FUSED.var ) ) < 1e-10:
            return
        ExtKalmanFilter.correct(self, self.z_lidar, self.v_FUSED.var, DATATYPE=DataType.FUSED)

    def is_updated( self, DATATYPE=DataType.SLAM ):
        return self.updated_flags[ DATATYPE ]

    # def update_time_diff( self, time_now, DATATYPE=DataType.SLAM ):
    #     if self.time_last[ DATATYPE ] > 0.0:
    #         self.time_diff[ DATATYPE ] = time_now - self.time_last[ DATATYPE ]
    #         self.updated_flags[ DATATYPE ] = True
    #     self.time_last[ DATATYPE ] = time_now


    def publish(self):
        next_pub = time.time()

        count = 0
        msg_seq = 1
        while True:
            # count += 1
            # if count%1 == 0:

            self.predict()

            marker_msg = Marker()
            marker_msg.header.frame_id = "base_link"
            marker_msg.id = 0
            marker_msg.type = 2 #Sphere
            marker_msg.pose.position.x = self.x.mu[0]
            marker_msg.pose.position.y = self.x.mu[1]
            marker_msg.pose.position.z = 0

            marker_msg.scale.x = 0.27
            marker_msg.scale.y = 0.27
            marker_msg.scale.z = 0.27

            marker_msg.color.r = 0.75
            marker_msg.color.g = 1
            marker_msg.color.b = 0.5
            marker_msg.color.a = 0.85

            self.marker_pub.publish(marker_msg)


            #Point, Quaternion, Pose, PoseWithCovariance
            pose_with_cov = PoseWithCovarianceStamped()

            pose_with_cov.header.seq = msg_seq
            msg_seq += 1
            pose_with_cov.header.stamp = rospy.get_rostime()
            pose_with_cov.header.frame_id = "laser"

            pose_with_cov.pose.pose.position.x = self.x.mu[0]
            pose_with_cov.pose.pose.position.y = self.x.mu[1]
            pose_with_cov.pose.pose.position.z = 0
            
            pose_with_cov.pose.pose.orientation.w = self.x.mu[2]

            pose_with_cov.pose.covariance = self.x.var[0:2, 0:2].flatten().tolist()

            self.out_pose.publish(pose_with_cov)

            # next_pub = next_pub + 0.1          
            # time.sleep( next_pub - time.time() )

                # if(count > 1000):
                #     count %= 1000

            rate = rospy.Rate(2)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('state_tracker', anonymous=True)
    strack = StateTracker()
    
    timerThread = threading.Thread(target=strack.publish)
    timerThread.daemon = True
    timerThread.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")