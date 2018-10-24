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
from cv_bridge import CvBridge, CvBridgeError

from kalman_filter import *

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback)

        # ## Manually set exposure for camera
        # os.system("v4l2-ctl -d /dev/video1 --set-ctrl=exposure_auto=1")
        # os.system("v4l2-ctl -d /dev/video1 --set-ctrl=exposure_absolute=170")
    
        self.tracker = None

    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        img_threshold = self.colorThreshold(img)
        circles = self.detectCircles(img_threshold)
        
        ## Draw circles on final image
        img_final = img.copy()

        if self.tracker:
            self.tracker.predict( np.array([0]) )

        for c in circles:
            center = c[0]
            radius = c[1]

            cv2.circle(img_final, center, int(radius), (0, 255, 255), 2)

            cv2.circle(img_final, center, 5, (255, 0, 255), -1)
            

            if(not self.tracker):
                self.tracker = BallTracker(center[0], center[1], radius)
            else:
                self.tracker.correct( np.array([ center[0], center[1], radius ]) )

            ## Display some extra information about the object on-screen
            ## Get the estimated distance
            ## This is the number of pixels we see at 1m for a ball of radius 10cm
            # pixels_at_1m = 142.0 ## Radius = 10cm
            pixels_at_1m = 163.3 ## Radius = 11.5cm
            
            z_est = pixels_at_1m/radius
            
            #self.drawText(img_final, "Radius: {:3.1f} px".format(radius), center[0], center[1])
            self.drawText(img_final, "Distance: {:1.2f}m".format(z_est), center[0]+30, center[1]-int(radius)-30)
            
            ## Get estimated bearing, knowing the FOV is 90 degrees
            bearing = (float(center[0])/(img_final.shape[1]/2)-1)*45.0
            self.drawText(img_final, "Bearing:  {:2.1f} deg".format(bearing), center[0]+30, center[1]-int(radius)-0)
           
        if self.tracker:
            # self.tracker.predict( np.array([0]) )
            # self.tracker.correct( np.array([ center[0], center[1], radius ]) )

            x_mu, x_var = self.tracker.get_state()

            cv2.circle(img_final, ( int(x_mu[0]), int(x_mu[1]) ), int(x_var[2][2] * 7e3 + 3 ), (255, 0, 0), -1) #5e4
            cv2.circle( img_final, ( int(x_mu[0]), int(x_mu[1]) ), int(x_mu[2]), (0, 0, 0), int((x_var[0][0]**2 + x_var[1][1]**2) ) ) #1e7

            center = (int(x_mu[0]), int(x_mu[1]))
            radius = int(x_mu[3])

            self.drawText(img_final, "X: {}, {}".format(x_mu[0], x_var[0][0]), center[0]+30, center[1]-int(radius)+150)
            self.drawText(img_final, "Y: {}, {}".format(x_mu[1], x_var[1][1]), center[0]+30, center[1]-int(radius)+180)
            self.drawText(img_final, "R: {}, {}".format(x_mu[2], x_var[2][2]), center[0]+30, center[1]-int(radius)+210)

        cv2.imshow("Image window", img_final)
        cv2.waitKey(1)
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)


    def drawText(self, img, text, x, y):
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
        bandwidth = 5
        
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
            cv2.drawContours(img, [c], 0, (0, 255, 0), 2)
            
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

def main(args):
    ic = image_converter()
    rospy.init_node('ball_detection', anonymous=True)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)