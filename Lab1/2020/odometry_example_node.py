#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 28 00:13:45 2020

@author: andres
"""

import rospy
from nav_msgs.msg import Odometry
import math
import tf
from tf.transformations import quaternion_from_euler




pub_odom = rospy.Publisher('odom', Odometry, queue_size=1)#the Publisher is initialized in this line 


class listenerNode():
   
   
    def __init__(self):
        self.loop_hertz = 5.0#loop frequency
        self.x=0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.6
        self.yaw_rate = 0.2
        
    def run(self):
        self.rate = rospy.Rate(self.loop_hertz)#this line is used to declare the time loop
        self.br = tf.TransformBroadcaster()#Initialize the object to be used in the frame transformation
        while not rospy.is_shutdown():
            
            self.linearvelocity()#call the function used to create the Odometry ROS message
            self.br.sendTransform((self.x, self.y, 0.0), self.q, rospy.Time.now(),"/base_link" , "/map")#this line is used to transform from local frame to global frame and it is necessary to plot the trajectory in RVIZ
            print(self.x)
            pub_odom.publish(self.odom)
            
############Replace this part with the x and y positions determined using the information from the rosbag##################
            if self.x < 10.0 and self.y == 0.0:
                self.x+=0.5
                self.y == 0.0
                if self.x == 10.0:
                    self.theta = -math.pi/2
        
            elif self.x == 10.0 and self.y == 0.0:
                self.y -= 0.5
                self.theta = -math.pi/2
            elif self.x == 10.0 and self.y < 0 and self.y > -10.0:
                self.y -= 0.5
                self.x = 10.0
                if self.y == -10.0:
                    self.theta -=math.pi/2
            elif self.x > 0.0 and self.y == -10.0:
                self.x-=0.5
                self.y = -10.0
                if self.x == 0.0:
                    self.theta -=math.pi/2
            elif self.x == 0.0 and self.y < 0.0:
                self.y += 0.5
                self.x = 0.0
                if self.y == 0.0:
                    self.theta -=math.pi/2
                
###########################################################################################################################
            self.rate.sleep()
       
        
    def linearvelocity(self):
        self.q = quaternion_from_euler(0.0, 0.0, self.theta)#function used to convert euler angles to quaternions
        self.odom = Odometry()
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation.x = self.q[0]
        self.odom.pose.pose.orientation.y = self.q[1]
        self.odom.pose.pose.orientation.z = self.q[2]
        self.odom.pose.pose.orientation.w = self.q[3]
        self.odom.twist.twist.linear.x = self.vx
        self.odom.twist.twist.angular.z = self.yaw_rate
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = "/map"
        self.odom.child_frame_id = "/base_link"

 
# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('odom_example_node', anonymous = True)
    # Go to the main loop.
    ne = listenerNode()
    ne.run()

