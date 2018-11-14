#!/usr/bin/env python

import numpy as np
from math import log
import matplotlib.pyplot as plt

import rospy
from nav_msgs.msg import OccupancyGrid

class InformationLossCalculator:
    def __init__(self):
        self.entropy_history = []
        self.lidar_sub = rospy.Subscriber("/scanmatcher_map", OccupancyGrid, self.map_cb)



    def map_cb( self, msg):
        entropy = 0
        for x in msg.data:
            if (x == -1):
                entropy += 1
            elif (x == 0):
                continue
            else:
                x /= 100
                entropy += x*log(x)

        self.entropy_history.append(entropy)
        #plt.axis([0, 10, 0, 1])
        plt.plot(self.entropy_history, 'ro-')
        plt.title('Total map entropy')
        plt.ylabel('Entropy')
        plt.xlabel('Sample')
        plt.pause(0.01)

if __name__ == "__main__":
    rospy.init_node('visualize_trackers', anonymous=True)

    InformationLossCalculator()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")