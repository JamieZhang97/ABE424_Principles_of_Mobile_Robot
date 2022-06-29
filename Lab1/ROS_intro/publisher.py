#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 10 09:35:20 2021

@author: andres
"""


import rospy
from std_msgs.msg import Int32

class PublisherNode():
    
    def __init__(self):
        pub = rospy.Publisher('publisher_node_example',Int32,queue_size=1)
        rate = 10.0
        A = 0
        while not rospy.is_shutdown():
            pub.publish(A)
            A+=1#this is equal to A = A+1
            if rate:
                rospy.sleep(1/rate)
            else:
                rospy.sleep(1.0)

if __name__ == '__main__':
    #Initialize the node and name it
    rospy.init_node('rospy_publisher', anonymous = True)
    #go to the init function
    try:
        ne = PublisherNode()
    except rospy.ROSInterruptException: pass