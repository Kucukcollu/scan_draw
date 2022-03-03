#!/usr/bin/env python

from math import radians
from turtle import distance
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon

# class Draw:
#     def __init__(self):
#         pass
    
#     def draw(self,X,Y):
#         self.x = X
#         self.y = Y
#         plt.plot(self.x,self.y)

class Robot:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.laserscan = rospy.Subscriber("/scan", LaserScan, self.laserscan_callback)
        
        # odom callback
        self.x=0
        self.y=0
        self.yaw=0
        self.quaternion=0
        self.rpy=0
        
        # scan callback
        self.min_angle = 0
        self.max_angle = 0
        self.field_of_view = 0
        self.min_range = 0
        self.max_range = 0
        self.angle = 0
        self.distances = list()
        self.dx = list()
        self.dy = list()

        # draw
        # self.d = Draw()

    def odom_callback(self,message):

        self.x = message.pose.pose.position.x
        self.y = message.pose.pose.position.y
        
        self.quaternion = (message.pose.pose.orientation.x,
                    message.pose.pose.orientation.y,
                    message.pose.pose.orientation.z,
                    message.pose.pose.orientation.w)
        
        self.rpy = tf.transformations.euler_from_quaternion(self.quaternion)

        self.yaw = self.rpy[2]
        
        # robot pose
        #rospy.loginfo("X: {} Y: {} Yaw: {}".format(self.x,self.y,self.yaw))

    def laserscan_callback(self,message):

        self.min_angle = message.angle_min
        self.max_angle = message.angle_max

        self.angle_diff = self.max_angle - self.min_angle

        self.field_of_view = self.max_angle-self.min_angle
        # rospy.logwarn("Total angle: {}".format(np.rad2deg(self.field_of_view)))

        self.min_range = message.range_min
        self.max_range = message.range_max
        
        self.angle = message.angle_increment
        
        self.distances = list(message.ranges)

        #print(type(self.distances))

        iteration = 0
        for i in np.arange(self.min_angle,self.max_angle,self.angle):
            self.dx.append(self.distances[iteration]*(np.cos(i)))
            self.dy.append(self.distances[iteration]*(np.sin(i)))
            iteration = iteration + 1
        
        plt.plot(self.dx,self.dy)
        plt.show()

def main():
    rospy.init_node('plot_map', anonymous=True)
    tb3 = Robot()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass