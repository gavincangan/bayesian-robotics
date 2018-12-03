#!/usr/bin/env python
import rospy
import roslib

from geometry_msgs.msg import TransformStamped, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import tf
import math
from math import sin, cos, pi,tan, atan2, log, exp
import numpy as np
from pylab import *
import matplotlib.pyplot as plt

pose=[0.0, 0.0, 0.0]

class localmap:
    def __init__(self, height, width, resolution, morigin):
        self.height = height
        self.width = width
        self.resolution = resolution
        self.punknown = -1
        self.localmap = [self.punknown]*int(self.width/self.resolution)*int(self.height/self.resolution)
        self.logodds = [0.0]*int(self.width/self.resolution)*int(self.height/self.resolution)
        self.morigin = morigin
        self.origin = int(math.ceil(morigin[0]/resolution))+int(math.ceil(width/resolution)*math.ceil(morigin[1]/resolution))
        self.pfree = self.prob2logodds(0.44)
        self.pocc  = self.prob2logodds(0.8)
        self.prior = self.prob2logodds(0.5)
        self.ball_detection_probability = 0.9
        self.max_logodd = 4
        self.max_scan_range = 1.0
        self.map_origin = morigin

        ball_range = 1.0
        ball_idx_range = int(ball_range/resolution)
        if (ball_idx_range%2 == 0):
            ball_mid_idx = ball_idx_range/2
            ball_idx_range += 1
        else:
            ball_mid_idx = (ball_idx_range-1)/2

        self.ball_conv = np.zeros((ball_idx_range, ball_idx_range))
        cov = 1.5
        if self.ball_detection_probability > 1 or self.ball_detection_probability < 0.5:
            raise Exception("ball_detection_probability must be between 0.5 and 1")

        for y in range(ball_idx_range):
            for x in range(ball_idx_range):
                r = (x-ball_mid_idx)**2 + (y-ball_mid_idx)**2
                g = exp(-0.5 * r/cov**2)
                ## Rescale so that g is between 0.5 and peak
                g = (self.ball_detection_probability-0.5)*g + 0.5
                self.ball_conv[x][y] = self.prob2logodds(g)

        #np.set_printoptions(precision=3, linewidth=150)
        #print(self.ball_conv)


    def prob2logodds(self, p):
        return log(p/(1-p))

    def logodds2prob(self, l):
        e = exp(self.logodds[index])
        return e / (1 + e)

    def updatemap(self,scandata,angle_min,angle_max,angle_increment,range_min,range_max,pose):

        robot_origin=int(pose[0])+int(math.ceil(self.width/self.resolution)*pose[1])
        centreray=len(scandata)/2+1

        for i in range(len(scandata)):
            if not math.isnan(scandata[i]):
                beta=(i-centreray)*angle_increment

                if (scandata[i] == float('inf')):
                    continue

                px = int(float(scandata[i])*cos(beta-pose[2])/self.resolution)
                py = int(float(scandata[i])*sin(beta-pose[2])/self.resolution)

                ## Perform ray tracing
                l = bresenham([0,0],[px,py])
                for j in range(len(l.path)):
                    lpx = self.map_origin[0] + pose[0] + l.path[j][0]*self.resolution
                    lpy = self.map_origin[1] + pose[1] + l.path[j][1]*self.resolution

                    if (0<=lpx<self.width and 0<=lpy<self.height):
                        #index = self.origin + int(l.path[j][0]+math.ceil(self.width/self.resolution)*l.path[j][1])
                        index = int(math.ceil(lpx/resolution))+int(math.ceil(width/resolution)*math.ceil(lpy/resolution))

                        if scandata[i]<self.max_scan_range*range_max:
                            if(j<len(l.path)-1):
                                self.logodds[index]+=self.pfree
                            else:
                                self.logodds[index]+=self.pocc
                        else:
                            self.logodds[index]+=self.pfree

                        ## Threshold this
                        if self.logodds[index]>self.max_logodd:
                            self.logodds[index]=self.max_logodd
                        elif self.logodds[index]<-self.max_logodd:
                            self.logodds[index]=-self.max_logodd

                        e = exp(self.logodds[index])
                        self.localmap[index] = 100*e/(1+e)

    def updateballmap(self, ball_x, ball_y):
        x_center = self.ball_conv.shape[1]//2 + 1
        y_center = self.ball_conv.shape[0]//2 + 1

        for y in range(self.ball_conv.shape[1]):
            for x in range(self.ball_conv.shape[0]):
                x_idx = math.ceil((self.map_origin[0] + ball_x)/self.resolution) + x - x_center
                y_idx = math.ceil((self.map_origin[1] + ball_y)/self.resolution) + y - y_center

                index = int(x_idx + math.ceil(self.width/self.resolution)*y_idx)
                self.logodds[index] += self.ball_conv[x][y]

                ## Threshold this
                if self.logodds[index] > self.max_logodd:
                    self.logodds[index] = self.max_logodd
                elif self.logodds[index] < -self.max_logodd:
                    self.logodds[index] = -self.max_logodd

                e = exp(self.logodds[index])
                self.localmap[index] = 100*e/(1+e)





class bresenham:
    def __init__(self, start, end):
        self.start = list(start)
        self.end = list(end)
        self.path = []
        self.toggle=0

        if start[0]-end[0]+start[1]-end[1]==0:
            return None

        self.steep = abs(self.end[1]-self.start[1]) > abs(self.end[0]-self.start[0])

        if self.steep:
            self.start = self.swap(self.start[0],self.start[1])
            self.end = self.swap(self.end[0],self.end[1])

        if self.start[0] > self.end[0]:
            self.toggle=1
            #print 'flippin and floppin'
            _x0 = int(self.start[0])
            _x1 = int(self.end[0])
            self.start[0] = _x1
            self.end[0] = _x0

            _y0 = int(self.start[1])
            _y1 = int(self.end[1])
            self.start[1] = _y1
            self.end[1] = _y0

        dx = self.end[0] - self.start[0]
        dy = abs(self.end[1] - self.start[1])
        error = 0
        derr = dy/float(dx)

        ystep = 0
        y = self.start[1]

        if self.start[1] < self.end[1]: ystep = 1
        else: ystep = -1

        for x in range(self.start[0],self.end[0]+1):
            if self.steep:
                self.path.append((y,x))
            else:
                self.path.append((x,y))

            error += derr

            if error >= 0.5:
                y += ystep
                error -= 1.0

        if self.toggle==1:
            self.path.reverse()

    def swap(self,n1,n2):
        return [n2,n1]


















def handle_robot_pose(parent, child, pose):
    br = tf.TransformBroadcaster()
    br.sendTransform((pose[0], pose[1], 0), tf.transformations.quaternion_from_euler(0, 0, pose[2]), rospy.Time.now(), child,parent)

def poseCb(msg):
    global pose
    x = msg.pose.position.x
    y = msg.pose.position.y
    q0 = msg.pose.orientation.w
    q1 = msg.pose.orientation.x
    q2 = msg.pose.orientation.y
    q3 = msg.pose.orientation.z
    theta = -atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
    pose = [x,y,theta]


    ## Generate an empty sector
    num_points = 80
    angle_max = pi/6 #30 degrees
    angle_min = -angle_max  # 30 degrees
    angle_increment = (angle_max - angle_min)/num_points
    range_min = 0.2
    range_max = 3
    ranges = [4]*num_points

    m.updatemap(ranges, angle_min, angle_max, angle_increment, range_min, range_max, pose)

    ball_x = pose[0] + 2.0*cos(-pose[2])
    ball_y = pose[1] + 2.0*sin(-pose[2])
    m.updateballmap(ball_x, ball_y)

    handle_robot_pose("map", "odom", pose)
    print("Updated map. Pose {}".format(pose))

def scanCb(msg):
    print(pose)
    py,px=[],[]
    scandata=msg.ranges
    angle_min=msg.angle_min
    angle_max=msg.angle_max
    angle_increment=msg.angle_increment
    range_min=msg.range_min
    range_max=msg.range_max
    m.updatemap(scandata,angle_min,angle_max,angle_increment,range_min,range_max,pose)
    handle_robot_pose("map", "odom", pose)

def mappublisher(m, height, width, resolution, morigin):
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = resolution
    msg.info.width      = math.ceil(width/resolution)
    msg.info.height     = math.ceil(height/resolution)
    msg.info.origin.position.x = -morigin[0]
    msg.info.origin.position.y = -morigin[1]
    msg.data = m
    mappub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('ball_map_publisher', anonymous=True) #make node
    rospy.Subscriber('/slam_out_pose', PoseStamped, poseCb)
    #rospy.Subscriber("/scan", LaserScan, scanCb)
    mappub =  rospy.Publisher('/map_ball', OccupancyGrid,queue_size=1)

    rate = rospy.Rate(10) # 100hz

    height, width, resolution = 20, 20, 0.1
    morigin = [0.5*width, 0.5*height]
    m = localmap(height, width, resolution, morigin)

    while not rospy.is_shutdown():
        mappublisher(m.localmap, height, width, resolution, morigin)
        rate.sleep()
