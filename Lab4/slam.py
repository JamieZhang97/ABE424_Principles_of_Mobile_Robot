# -*- coding: utf-8 -*-
"""
Created on Fri Apr 30 21:51:34 2021

@author: Jamie
"""

import numpy as np
import rospy
import time
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

pubx = rospy.Publisher('x_data', Float32MultiArray, queue_size = 1)
puby = rospy.Publisher('y_data', Float32MultiArray, queue_size = 1)

class listenerNode():
    
    def _init_(self):
        '''
        Lidar Model: Hokuyo UST-10LX
        Accuracy: 40mm
        Scan Angle: 270deg
        Number of measurements: 1080
        Angular resolution: 0.25deg
        Max detection distance: 30m
        '''
        self.sacn_angle = np.deg2rad(270)
        self.min_angle = np.deg2rad(-45)
        self.max_angle = np.deg2rad(225)
        self.liadr_info = []
        self.x_lidar = []
        self.y_lidar = []
        self.angle = []
        self.x = []
        self.y = []
        self.thres = 0.001
        self.range_t_l = []
        self.angle_t_l = []
        self.range_t_r = []
        self.angle_t_r = []
        
    def run(self):
        rospy.Suscriber("/terrasentia/scan", LaserScan, self.callback)
        self.rate = rospy.Rate(self.loop_hertz)
        while not rospy.is_shutdown():
            if len(self.lidar_info) > 0:
                print('message_ok')
                self.polar2xy()
                print('polartransformation_ok')
                self.splitLidar()
                print('fitlines_ok')
                self.plot_lines()
                msg = Float32MultiArray(data = self.x_lidar)
                msg2 = Float32MultiArray(data = self.y_lidar)
                pubx.publish(msg)
                puby.publish(msg2)
                print(len(self.lidar_info[0]))
            self.rate.sleep()
        plt.close('all')
        
    def polar2xy(self):
        self.x_lidar = []
        self.y_lidar = []
        self.angle = []
        self.angle = np.linspace(self.min_angle, self.max_angle, 1081, endpoint = False)
        
        for i in range(0, len(self.liadr_info[0])):
            self.x_lidar.append((self.liadr_info[0])[i]*np.cos(self.angle[i]))
            self.y_lidar.append((self.liadr_info[0])[i]*np.sin(self.angle[i]))
            
    def splitLidar(self):
        self.range_t_l = []
        self.angle_t_l = []
        self.range_t_r = []
        self.angle_t_r = []
        self.x_t_l = []
        self.y_t_l = []
        self.x_t_r = []
        self.y_t_r = []
            
        for t in range(len(self.x_lidar)):
            if (self.liadr_info[0])[t] > 0:
                if t >= 540:
                    if self.x_lidar[t] < 8.0 and self.x_lidar[t] > -8.0 and self.y_lidar[t] < 10.0 and self.y_lidar[t] > -10.0:
                        self.range_t_l.append((self.liadr_info[0])[t])
                        self.angle_t_l.append(self.angle[t])
                        self.x_t_l.append(self.x_lidar[t])
                        self.y_t_l.append(self.y_lidar[t])
            
                if t < 540:
                    if self.x_lidar[t] < 8.0 and self.x_lidar[t] > -8.0 and self.y_lidar[t] < 10.0 and self.y_lidar[t] > -10.0:
                        self.range_t_r.append((self.liadr_info[0])[t])
                        self.angle_t_r.append(self.angle[t])
                        self.x_t_r.append(self.x_lidar[t])
                        self.y_t_r.append(self.y_lidar[t])
        
    def fitLines(self):
        j = 0
        self.m_l = []
        self.m_r = []
        self.c_l = []
        self.c_r = []
        self.x_all_l = []
        self.x_all_el = []
        self.x_all_r = []
        self.x_all_er = []
        self.range_l = []
        self.angle_l = []
        self.range_r = []
        self.angle_r = []
        
        self.range_r = [self.range_t_r[j], self.range_t_r[j+1]]
        self.angle_r = [self.angle_t_r[j], self.angle_t_r[j+1]]
        xr_1 = self.range_r[0]*np.cos(self.angle_r[0])
        yr_1 = self.range_r[0]*np.sin(self.angle_r[0])
        xr_2 = self.range_r[1]*np.cos(self.angle_r[1])
        yr_2 = self.range_r[1]*np.sin(self.angle_r[1])
        m = np.tan((yr_2 - yr_1)/(xr_2 - xr_1))
        c = yr_1 - m*xr_1
        
        for i in range(j+2, len(self.range_t_r) - 1):
            self.range_r.append(self.range_t_r[i])
            self.angle_r.append(self.angle_t_r[i])
            m, c, r, alpha = self.line_calc(self.range_r, self.angle_r)
        
            temp = (np.array(self.range_r)*np.array([np.cos(x - alpha) for x in self.angle_r]))
            b = temp - r
            d = np.amax(b)
            
            if d > 0.001:
                self.m_r.append(m)
                self.c_r.append(c)
                #not finished
                self.x_all_r.append(np.amin((np.array(self.range_r))*(np.array([np.cos(x) for i in self]))))
                self.x_all_er.append(np.amax((np.array(self.range_r))*(np.array([np.cos(x) for i in self]))))
        
                self.range_r = [self.range_t_r[i], self.range_t_r[i+1]]
                self.angle_r = [self.angle_t_r[i], self.angle_t_r[i+1]]
                xr_1 = self.range_r[0]*np.cos(self.angle_r[0])
                yr_1 = self.range_r[0]*np.sin(self.angle_r[0])
                xr_2 = self.range_r[1]*np.cos(self.angle_r[1])
                yr_2 = self.range_r[1]*np.sin(self.angle_r[1])
                m = np.tan((yr_2 - yr_1)/(xr_2 - xr_1))
                c = yr_1 - m*xr_1
            
            if i == len(self.range_t_r) - 1:
                self.m_r.append(m)
                self.c_r.append(c)
                #not finished
                self.x_all_r.append(np.amin((np.array(self.range_r))*(np.array([np.cos(x) for i in self]))))
                self.x_all_er.append(np.amax((np.array(self.range_r))*(np.array([np.cos(x) for i in self]))))
        
        for i in range(j+2, len(self.range_t_l) - 1):
            self.range_l.append(self.range_t_l[i])
            self.angle_l.append(self.angle_t_l[i])
            m, c, r, alpha = self.line_calc(self.range_l, self.angle_l)
        
            temp = (np.array(self.range_l)*np.array([np.cos(x - alpha) for x in self.angle_l]))
            b = temp - r
            d = np.amax(b)
            
            if d > 0.001:
                self.m_l.append(m)
                self.c_l.append(c)
                #not finished
                self.x_all_l.append(np.amin((np.array(self.range_l))*(np.array([np.cos(x) for i in self]))))
                self.x_all_el.append(np.amax((np.array(self.range_l))*(np.array([np.cos(x) for i in self]))))
        
                self.range_l = [self.range_t_l[i], self.range_t_l[i+1]]
                self.angle_l = [self.angle_t_l[i], self.angle_t_l[i+1]]
                xl_1 = self.range_l[0]*np.cos(self.angle_l[0])
                yl_1 = self.range_l[0]*np.sin(self.angle_l[0])
                xl_2 = self.range_l[1]*np.cos(self.angle_l[1])
                yl_2 = self.range_l[1]*np.sin(self.angle_l[1])
                m = np.tan((yl_2 - yl_1)/(xl_2 - xl_1))
                c = yl_1 - m*xl_1
            
            if i == len(self.range_t_l) - 1:
                self.m_l.append(m)
                self.c_l.append(c)
                #not finished
                self.x_all_l.append(np.amin((np.array(self.range_l))*(np.array([np.cos(x) for i in self]))))
                self.x_all_el.append(np.amax((np.array(self.range_l))*(np.array([np.cos(x) for i in self]))))
        
    def callback(self, msg):
        self.lidar_info = []
        self.liadr_info.append(msg.ranges)
        
    def line_calc(self, rang, angle):
        add1 = 0
        add2 = 0
        
        for i in range(0, len(rang)):
            for j in range(0, len(rang)):
                add1 += rang[i]*rang[j]*np.cos(angle[i])*np.sin(angle[j])
                add2 += rang[i]*rang[j]*np.cos(angle[i] + angle[j])
        #not finished
        alpha = np.atan2( - (np.sum((np.array([x*x for x in rang]))*(np.array([np.sin(2*x)]) for x in ))))
        
        r = np.sum(np.array(rang)*bp.array([np.cos(x - alpha) for x in angle]))/(len(rang))
        m = -1/np.tan(alpha)
        if alpha > 0:
            c = r*np.sqrt(1 + m*m)
        else:
            c = -r*np.sqrt(1 + m*m)
        
        return m, c, r, alpha
    
    def plot_lines(self):
        plt.cla()
        for i in range(len(self.m_l)):
            if self.x_all_l[i] < self.x_all_el[i]
                x_plot = np.arange(self.x_all_l[i], self.x_all_el[i], 0.001)
            else:
                x_plot = np.arange(self.x_all_el[i], self.x_all_l[i], 0.001)
            y_plot = np.array([self.m_l[i]*x + self.c_l[i] for x in x_plot])
            plt.plot(x_plot, y_plot, 'b')
    
        for i in range(len(self.m_r)):
            if self.x_all_r[i] < self.x_all_er[i]
                x_plot = np.arange(self.x_all_r[i], self.x_all_er[i], 0.001)
            else:
                x_plot = np.arange(self.x_all_er[i], self.x_all_r[i], 0.001)
            y_plot = np.array([self.m_r[i]*x + self.c_r[i] for x in x_plot])
            plt.plot(x_plot, y_plot, 'b')
        
        plt.pause(0.001)
    
    
    
        
        
        
        
        
        
        
        
        
        
        
        
        