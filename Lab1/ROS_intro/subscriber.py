#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 10 09:43:46 2021

@author: andres
"""

import rospy
from std_msgs.msg import Int32

class listenerNode():
   
    
    number = 0.0
    A=0.0
    def __init__(self):
        
        rate = 10.0
        rospy.Subscriber("publisher_node_example", Int32, self.callback)
        while not rospy.is_shutdown():
            print("The number is: ",listenerNode.number)
            if rate:
                rospy.sleep(1/rate)
            else:
                rospy.sleep(1.0)

   
    def callback(self,msg):
        listenerNode.number = msg.data
        rospy.loginfo("number %s", self.number)

if __name__ == '__main__':
    #Initialize the node and name it
    rospy.init_node('rospy_subscriber', anonymous = True)
    #go to the init function
    try:
        ne = listenerNode()
    except rospy.ROSInterruptException: pass