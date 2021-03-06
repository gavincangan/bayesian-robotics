#!/usr/bin/env python
from __future__ import print_function

import sys
import cv2
import numpy as np
from math import pi, sin, cos, atan, tan, radians, degrees
# from matplotlib import pyplot as plt
import os

import roslib
roslib.load_manifest('mbz_c3_jackal')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from visualization_msgs.msg import Marker
from mbz_c3_jackal.msg import PositionPolar, Vector

from kalman_filter import KalmanFilter, Gaussian
from ext_kalman_filter import ExtKalmanFilter

class image_converter:
    def __init__(self):
        # self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=32)
        self.out_pub = rospy.Publisher("target/cam_position",PositionPolar, queue_size=32)
        # self.marker_pub = rospy.Publisher("target/cam_marker",Marker, queue_size=32)
        self.out_raw_pub = rospy.Publisher("target/raw_cam_position",PositionPolar, queue_size=32)

        self.bridge = CvBridge()

        # ## Manually set exposure for camera
        # os.system("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1")
        # os.system("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=170")
    
        self.tracker = None
        self.gui = False

        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback, queue_size=5)


    def callback(self, data):
        # rospy.loginfo("Received usb cam data")


        # print(data.header)
        dt = rospy.get_rostime() - data.header.stamp
        if dt.to_sec() > 0.1:
            return

        try:
            # img_np_arr = np.fromstring(data.data, np.uint8)
            # img = cv2.imdecode(img_np_arr, cv2.IMREAD_COLOR)
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        except Exception as ex:
            print(ex)
        
        img_threshold = self.colorThreshold(img)
        circles = self.detectCircles(img_threshold)
        
        ## Draw circles on final image
        img_final = img.copy()

        if self.tracker:
            self.tracker.predict( np.array([0]) )

        for c in circles:
            center = c[0]
            radius = c[1]

            # if self.gui:
            #     cv2.circle(img_final, center, int(radius), (0, 255, 255), 2)
            #     cv2.circle(img_final, center, 5, (255, 0, 255), -1)

            ## Display some extra information about the object on-screen
            ## Get the estimated distance
            ## This is the number of pixels we see at 1m for a ball of radius 10cm
            # pixels_at_1m = 142.0 ## Radius = 10cm
            #pixels_at_1m = 180.3 ## Radius = 12.7cm
            pixels_at_1m = 120 ## Radius = 12.7cm
            
            z_est = pixels_at_1m/radius
            
            #self.drawText(img_final, "Radius: {:3.1f} px".format(radius), center[0], center[1])
            # self.drawText(img_final, "Distance: {:1.2f}m".format(z_est), center[0]+30, center[1]-int(radius)-30)
            
            ## Get estimated bearing, knowing the FOV is 90 degrees
            bearing = (float(center[0])/(img_final.shape[1]/2)-1)*20.0

            out_raw_msg = PositionPolar()
            out_raw_msg.distance = z_est
            out_raw_msg.heading = bearing
            out_raw_msg.cov_size = 0
            out_raw_msg.covariance = []
            self.out_raw_pub.publish(out_raw_msg)

            """
            w = (img_final.shape[1]/2)
            x = float(center[0]) - w
            angle_fov = 45.0
            bearing = -degrees(atan(x/w*tan(radians(angle_fov))))
            """
            # if self.gui:
            #     self.drawText(img_final, "Bearing:  {:2.1f} deg".format(bearing), center[0]+30, center[1]-int(radius)-0)
           
            if(not self.tracker):
                self.tracker = BearingTracker(z_est, bearing)
            else:
                self.tracker.correct( np.array([ z_est, bearing ]) )

        if self.tracker and circles:
            # self.tracker.predict( np.array([0]) )
            # self.tracker.correct( np.array([ center[0], center[1], radius ]) )

            x_mu, x_var = self.tracker.get_state()

            # cv2.circle(img_final, ( int(x_mu[0]), int(x_mu[1]) ), int(x_var[2][2] * 7e3 + 3 ), (255, 0, 0), -1) #5e4
            # cv2.circle( img_final, ( int(x_mu[0]), int(x_mu[1]) ), int(x_mu[2]), (255, 255, 255), int((x_var[0][0]**2 + x_var[1][1]**2) + 2 ) ) #1e7

            out_msg = PositionPolar()
            out_msg.distance = x_mu[0]
            out_msg.heading = x_mu[1]
            out_msg.cov_size = x_var.shape[0]

            out_msg.covariance = x_var.flatten().tolist()
            # out_msg.covariance = np.array([
            #     [9999, 0, 0, 0],   \
            #     [0, 1e-10, 0, 0],   \
            #     [0, 0, 9999, 0],   \
            #     [0, 0, 0, 1e-10],   \
            # ]).flatten().tolist()

            self.out_pub.publish(out_msg)

            # ## Create a marker for visualization
            # angle = radians(x_mu[1])
            # x = x_mu[0] * cos(angle)
            # y = x_mu[0] * sin(angle)

            # marker_msg = Marker()
            # marker_msg.header.frame_id = "laser"
            # marker_msg.id = 0
            # marker_msg.type = 2 #Sphere
            # marker_msg.pose.position.x = x
            # marker_msg.pose.position.y = y
            # marker_msg.pose.position.z = 0

            # marker_msg.scale.x = 0.24
            # marker_msg.scale.y = 0.24
            # marker_msg.scale.z = 0.24

            # marker_msg.color.r = 0.0
            # marker_msg.color.g = 0.0
            # marker_msg.color.b = 1.0
            # marker_msg.color.a = 0.75

            # self.marker_pub.publish(marker_msg)
            
            # if self.gui:        
            #     cv2.imshow("Threshold", img_threshold)
            #     cv2.imshow("Image window", img_final)
            #     cv2.waitKey(1)
            
                # try:
                #     # self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_final, "bgr8"))
                # except CvBridgeError as e:
                #     print(e)


    def drawText(self, img, text, x, y):
        if self.gui:
            cv2.putText(img, text, (x, y), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(  0,  0,  0), lineType=cv2.LINE_AA, thickness=5, bottomLeftOrigin=False)
            cv2.putText(img, text, (x, y), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255,255), lineType=cv2.LINE_AA, thickness=2, bottomLeftOrigin=False)
    
    
    def colorThreshold(self, img):  
        ## Get HSV image
        ## NOTE: We purposefully did RGB and not BGR. This maps red to the B channel
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        
        ## Max out the saturation (not very important) and brightness (not important)
        h, s, v = cv2.split(hsv)
        #s.fill(255)
        v.fill(255)

        ## Merge HSV channels back to a single image
        hsv = cv2.merge([h, s, v])
        
        ## Go back to RGB space
        img = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        
        ## Let's try to find red objects
        center = 115 #110-120
        bandwidth = 10
        
        _, mask1 = cv2.threshold(h,thresh=center-bandwidth, maxval=1, type=cv2.THRESH_BINARY)
        _, mask2 = cv2.threshold(h,thresh=center+bandwidth, maxval=1, type=cv2.THRESH_BINARY_INV)
        mask_hue = cv2.bitwise_and(mask1, mask2)
        
        ## And make sure that they're saturated
        _, mask_sat = cv2.threshold(s,thresh=80, maxval=1, type=cv2.THRESH_BINARY)
        
        ## Make the final mask
        mask = cv2.bitwise_and(mask_hue, mask_sat)
        
        ## Apply the mask
        img_masked = cv2.bitwise_and(img, img, mask = mask)
        
        return img_masked
    
    
    def detectCircles(self, img):
        ## Get a greyscale version of the image
        img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        _, mask = cv2.threshold(img_gray,thresh=20, maxval=1, type=cv2.THRESH_BINARY)
        
        min_radius = 10
        min_area = 400
        
        # find contours in the mask and initialize the current (x, y) center of the ball
        #cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

        circles_out = []
        ## Process all contours
        for c in cnts:
            if cv2.contourArea(c) < min_area:
                continue
            
            ## Find bounding circle
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            center = (int(x), int(y))
            
            ## Draw contour
            # if self.gui:
            #     cv2.drawContours(img, [c], 0, (0, 255, 0), 2)
            
            ## Only proceed if the radius meets a minimum size
            if radius < min_radius:
                continue
            
            ## Make sure this is actually a circle (ie. fills up some minimum percentage of bounding circle)
            area_bounding = pi*(radius**2)
            area_ratio = cv2.contourArea(c)/area_bounding
            
            if (area_ratio < 0.7):
                continue
            
            ## Save this value
            circles_out.append([center, radius])
        
        return circles_out

class BallTracker(ExtKalmanFilter):
    
    def __init__(self, _X, _Y, _R):

        self.w = Gaussian.diagonal( [0, 0, 0, 0, 0, 0], [1e-4, 1e-4, 5e-0, 1e-4, 1e-4, 5e-0] )
        self.v = Gaussian.diagonal( [0, 0, 0, 0, 0, 0], [5e-5, 5e-5, 1e0, 1e-4, 1e-4, 5e-1] )

        self.x = Gaussian.diagonal( [_X, _Y, _R, 0, 0, 0], [1e-3, 1e-3, 1e-3, 1e-5, 1e-5, 1e-4] )

        self.mahalonobis_threshold = 1.0
    
        # u --> linear vel, angular vel
        self.u = None

    def nextState(self):
        # x.mu --> Xrobot, Yr, Theta_r, Xball, Yb

        dXr = self.u[0] * np.cos( x.mu[2] )
        dYr = self.u[0] * np.sin( x.mu[2] )

        self.x.mu[0] = self.x.mu[0] + dXr
        self.x.mu[1] = self.x.mu[1] + dYr
        self.x.mu[2] = self.u[1]

        self.x.mu[3] = self.x.mu[3] + dXr
        self.x.mu[4] = self.x.mu[4] + dYr        


    def correct(self, z, measureType='camera'):

        if measureType == 'camera':
            pass


class BearingTracker(KalmanFilter):
    
    def __init__(self, _dist, _theta):
        self.A = np.eye(2)
        self.C = np.eye(2)

        self.B = np.array([0])
        self.D = np.array([0])

        self.w = Gaussian.diagonal( [0, 0], [1e-1, 1e-4] )
        self.v = Gaussian.diagonal( [0, 0], [5e0, 1e-2] )

        self.x = Gaussian.diagonal( [_dist, _theta], [1e1, 1e-1] )

        # self.yold = [_dist, _theta]

        self.mahalonobis_threshold = 1.0

    def correct(self, y):
        KalmanFilter.correct(self, y, self.v.var)

    #     # ty = np.append(y, [ y[ix]-self.yold[ix] for ix in range(len(y)) ] )
    #     # pdb.set_trace()
    #     KalmanFilter.correct(self, y)
    #     self.yold = y

def main(args):
    rospy.init_node('ball_detection', anonymous=True)
    
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    if ic.gui:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
