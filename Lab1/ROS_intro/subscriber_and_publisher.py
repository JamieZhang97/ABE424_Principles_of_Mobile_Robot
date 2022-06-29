#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 10 09:51:44 2021

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
        pub = rospy.Publisher('publisher_subscriber',Int32,queue_size=1)
        while not rospy.is_shutdown():
            A = listenerNode.number * 2
            print("The number is: ",listenerNode.number)
            pub.publish(A)
            #rospy.loginfo("yaw_rate: %s", yawrate)
           
            if rate:
                rospy.sleep(1/rate)
            else:
                rospy.sleep(1.0)

   
    def callback(self,msg):
        listenerNode.number = msg.data
        rospy.loginfo("number %s", self.number)
        

    
   
# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('rospy_listener_example', anonymous = True)
    # Go to the main loop.
    #try:
    ne = listenerNode()