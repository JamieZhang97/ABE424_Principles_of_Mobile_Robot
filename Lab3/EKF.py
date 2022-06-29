#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 18:12:52 2021

@author: andres
"""

import numpy as np
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Accel 
from numpy.linalg import inv
from numpy import linalg as LA
import matplotlib.pyplot as plt


class listenerNode():
   
    
    number = 0.0
    A=0.0
    def __init__(self):
        self.loop_hertz = 50.0#loop frequency
        self.A = np.zeros(16)#array to save the information from the rosbag
        #Initialization of the variables used to generate the plots
        self.PHI = []
        self.PSI = []
        self.THETA = []
        self.PHI_raw = []
        self.PSI_raw = []
        self.THETA_raw = []
        self.P_R = []
        self.P_R1 = []
        self.P_R2 = []
        self.Pos = []
        self.Vel = []
        self.Quater = []
        self.A_PosX = []
        self.A_PosY = []
        self.A_PosZ = []
        self.AVelX = []
        self.AVelY = []
        self.AVelZ = []
        self.AQuater1 = []
        self.AQuater2 = []
        self.AQuater3 = []
        self.AQuater4 = []
        self.P_angular = []
        self.Q_angular = []
        self.R_angular = []
        self.P_raw_angular = []
        self.Q_raw_angular = []
        self.R_raw_angular = []
        self.Fx_RAW = []
        self.Fy_RAW = []
        self.Fz_RAW = []
        self.Bias =[]
        
        #Initialization of the variables used in the EKF
        
        
        # initial position in x y and z
        self.x = np.array([0, 0, 0])

        # bias values, these are accelerometer and gyroscope biases
        self.bp= 0;#.54*math.pi/180;
        self.bq=-12*math.pi/180;
        self.br=-.1*math.pi/180;
        self.bfx = 0;
        self.bfy = 0;
        self.bfz = 0;


        #IMU location specifier
        self.r_imu=np.array([[-0.5/12, -3/12, 1/12]]).T*0.0 #I have set this to zero, for Bonus, you can include the effect of this
        self.rgps=np.array([1.5, 0 ,0]) #This is the location of the GPS wrt CG, this is very important
        #rotation matrix ------------------------------------------------------
        self.phi= self.x[0]
        self.theta= self.x[1]
        self.psi = self.x[2]

        #rotation matrix body to inertial
        self.L_bi = np.array([[math.cos(self.theta)*math.cos(self.psi), math.cos(self.theta)*math.sin(self.psi), -math.sin(self.theta)],
                          [math.sin(self.phi)*math.sin(self.theta)*math.cos(self.psi)-math.cos(self.phi)*math.sin(self.psi),  math.sin(self.phi)*math.sin(self.theta)*math.sin(self.psi)+math.cos(self.phi)*math.cos(self.psi), math.sin(self.phi)*math.cos(self.theta)],
                          [math.cos(self.phi)*math.sin(self.theta)*math.cos(self.psi)+math.sin(self.phi)*math.sin(self.psi),  math.cos(self.phi)*math.sin(self.theta)*math.sin(self.psi)-math.sin(self.phi)*math.cos(self.psi), math.cos(self.phi)*math.cos(self.theta)]])


        self.Rt2b=self.L_bi

        [U,S,V]=LA.svd(self.Rt2b)
        self.R = U@V.T
        self.b = np.zeros((4,1))

        if 1+self.R[0,0]+self.R[1,1]+self.R[2,2] > 0:
            self.b[0,0]    = 0.5*math.sqrt(1+self.R[0,0]+self.R[1,1]+self.R[2,2])
            self.b[1,0]    = (self.R[2,1]-self.R[1,2])/4/self.b[0,0]
            self.b[2,0]    = (self.R[0,2]-self.R[2,0])/4/self.b[0,0]
            self.b[3,0]    = (self.R[1,0]-self.R[0,1])/4/self.b[0,0]
            self.b       /= LA.norm(self.b)    # renormalize
        else:
            print(self.R)
            print('R diagonal too negative.')
            self.b = np.zeros((4,1))

        #set quats
        #-----------------------------------------------------------------
        self.q1=self.b[0,0] #the quaternions are called b1-b4 in the data file that you loaded
        self.q2=self.b[1,0]
        self.q3=self.b[2,0]
        self.q4=self.b[3,0]

        #initialize velocity
        self.vx = 0
        self.vy = 0
        self.vz = 0
        
        #set sample time
        self.dt = 0.02
        
        #initialize x hat
        #Note carefull the order the states appear in, this can be arbitrary, but
        #we must stick to it along the entire code
        #      [x y z vx vy vz          quat    gyro-bias accl-bias]
        self.xhat = np.array([[0, 0, 0, 0, 0, 0, self.b[0,0], self.b[1,0], self.b[2,0], self.b[3,0], self.bp, self.bq, self.br, self.bfx, self.bfy, self.bfz]]).T
        #x_tilde = xhat
        #x = np.zeros(16)


        #noise params process noise (my gift to you :))
        self.Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.8, 0.8, 0.8, 0.8, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001])

        #noise params, measurement noise
        #measurements are GPS position and velocity and mag
        self.R = np.diag([9, 9, 9, 3, 3, 3])

        #Initialize P, the covariance matrix
        self.P = np.diag([30, 30, 30, 3, 3, 3, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.Pdot=self.P*0.0
        
        self.time = []
        self.loop_t = 0
        self.Z = np.zeros((3,3))
        self.I = np.eye(3,3)
        self.Z34 = np.zeros((3,4))
        self.Z43 = np.zeros((4,3))
        self.Z36 = np.zeros((3,6))
        
        
    def run(self):
        self.rate = rospy.Rate(self.loop_hertz)#this line is used to declare the time loop
        rospy.Subscriber("A", Odometry, self.callback)
        rospy.Subscriber("Accel", Accel, self.callback1)
        while not rospy.is_shutdown():
            self.EKF()
            self.rate.sleep()
        self.Print_data()

    def Print_data(self):
        plt.close('all')
        plt.figure(1)
        plt.plot(self.time,self.P_R)
        plt.title('Covariance of Position')
        plt.legend(['px','py','pz'])
        plt.figure(2)
        plt.plot(self.time,self.P_R1)
        plt.legend(['pxdot','pydot','pzdot'])
        plt.title('Covariance of Velocities')
        plt.figure(3)
        plt.plot(self.time,self.P_R2)
        plt.title('Covariance of Quaternions')
        plt.figure(4)
        plt.plot(self.time,self.Pos,self.time,self.A_PosX,'r:', self.time,self.A_PosY,'r:', self.time,self.A_PosZ,'r:')
        plt.title('Position')
        plt.figure(5)
        plt.plot(self.time,self.Vel,self.time,self.AVelX,'r:',self.time,self.AVelY,'r:',self.time,self.AVelZ,'r:')
        plt.title('vel x y z')
        plt.figure(6)
        plt.plot(self.time,self.Quater,self.time,self.AQuater1,'r:',self.time,self.AQuater2,'r:',self.time,self.AQuater3,'r:',self.time,self.AQuater4,'r:')
        plt.title('Quat')
        plt.figure(7)
        plt.plot(self.time,self.P_angular,self.time,self.Q_angular,self.time,self.R_angular)
        plt.title('OMEGA with Bias')
        plt.legend(['p','q','r'])
        plt.figure(8)
        plt.plot(self.time,self.PHI,'b', self.time, self.THETA, 'g', self.time,self.PSI, 'r', self.time,self.PHI_raw,'b:',self.time,self.THETA_raw,'g:',self.time,self.PSI_raw,'r:')
        plt.legend(['phi','theta','psi','phiraw', 'thetaraw', 'psiraw'])
        plt.title('Phi, Theta, Psi')
        plt.figure(9)
        plt.plot(self.time,self.Bias)
        plt.title('Bias')
        plt.legend(['bp','bq','br','bfx','bfy','bfz'])
        plt.figure(10)
        plt.plot(self.time,self.P_raw_angular,self.time,self.Q_raw_angular,self.time,self.R_raw_angular)
        plt.title('OMEGA without Bias')
        plt.legend(['p','q','r'])
        plt.figure(11)
        plt.plot(self.time,self.Fx_RAW,self.time,self.Fy_RAW,self.time,self.Fz_RAW)
        plt.title('accelerometer')
        plt.legend(['ax','ay','az'])
        plt.show()
    
    def callback(self,msg):
        #A = [p, q, r, fx, fy, fz, x, y, z, vx, vy, vz,q1,q2,q3,q4] 
        self.A[0] = msg.twist.twist.angular.x
        self.A[6] = msg.pose.pose.position.x
        self.A[9] = msg.twist.twist.linear.x
        self.A[12] = msg.pose.pose.orientation.x
        
    def callback1(self,msg):
        self.A[3] = msg.linear.x
        
    def quat2euler(self, q):
        q_size = q.shape
        n=q_size[0]
        m=np.eye(3,3)
        phi=np.zeros((n,1))
        psi=np.zeros((n,1))
        theta=np.zeros((n,1))
        for i in range(0,n):
            q0 = q[i,0]
            q1 = q[i,1]
            q2 = q[i,2]
            q3 = q[i,3]
            m[0,0] = 1.0 - 2.0*(q2*q2 + q3*q3)
            m[0,1] = 2.0*(q1*q2 - q0*q3)
            m[0,2] = 2.0*(q1*q3 + q0*q2)
            m[1,0] = 2.0*(q1*q2 + q0*q3)
            m[1,1] = 1.0 - 2.0*(q1*q1 + q3*q3)
            m[1,2] = 2.0*(q2*q3 - q0*q1)
            m[2,0] = 2.0*(q1*q3 - q0*q2)
            m[2,1] = 2.0*(q2*q3 + q0*q1)
            m[2,2] = 1.0 - 2.0*(q1*q1 + q2*q2)
            phi[i,0] = math.atan2(m[2,1], m[2,2])
            theta[i,0] = -math.asin(m[2,0])
            psi[i,0] = math.atan2(m[1,0], m[0,0])
        return phi, theta, psi

    def EKF(self):
    
        #Streaming sensor measurements and adjust for bias
        # these come from the file that is loaded in the begining
        self.p = (self.A[0]*math.pi/180-self.xhat[10,0])
        self.q = (self.A[1]*math.pi/180-self.xhat[11,0])
        self.r = self.A[2]*math.pi/180-self.xhat[12,0]
        self.fx = (self.A[3]-self.xhat[13,0])
        self.fy = (self.A[4]-self.xhat[14,0])
        self.fz = -self.A[5]-self.xhat[15,0]
    
        #Raw sensor measurments for plotting
        self.p_raw = self.A[0]*math.pi/180
        self.q_raw = self.A[1]*math.pi/180
        self.r_raw = self.A[2]*math.pi/180
        self.fx_raw = self.A[3]
        self.fy_raw = self.A[4]
        self.fz_raw = self.A[5]
   
        self.quat = np.array([[self.xhat[6,0], self.xhat[7,0], self.xhat[8,0], self.xhat[9,0]]]).T
    
        self.q1 = self.quat[0,0]
        self.q2 = self.quat[1,0]
        self.q3 = self.quat[2,0]
        self.q4 = self.quat[3,0]
        self.qv = np.array([self.q1, self.q2, self.q3, self.q4]).T
    
   
        self.L_bl = np.array([[pow(self.q1,2)+pow(self.q2,2)-pow(self.q3,2)-pow(self.q4,2), 2*(self.q2*self.q3-self.q1*self.q4), 2*(self.q2*self.q4+self.q1*self.q3)],
                          [2*(self.q2*self.q3+self.q1*self.q4), pow(self.q1,2)-pow(self.q2,2)+pow(self.q3,2)-pow(self.q4,2), 2*(self.q3*self.q4-self.q1*self.q2)],
                          [2*(self.q2*self.q4-self.q1*self.q3), 2*(self.q3*self.q4-self.q1*self.q2), pow(self.q1,2)-pow(self.q2,2)-pow(self.q3,2)+pow(self.q4,2)]])
        self.L_lb = self.L_bl.transpose()
    
    
        #Implement your code here: 
        
        #Prediction step
        #First write out all the dots, e.g. pxdot, pydot, q1dot etc
        
        print('your code here')
        pxdot=self.xhat[3,0]
        pydot=self.xhat[4,0]
        pzdot=self.xhat[5,0]
        vdot=np.dot(self.L_lb,np.array([[self.fx],[self.fy],[self.fz]]))
        vxdot=vdot[0,0]
        vydot=vdot[1,0]
        vzdot=vdot[2,0]
        q1dot= -0.5*(self.p*self.q2+self.q*self.q3+self.r*self.q4)
        q2dot= -0.5*(-self.p*self.q1-self.r*self.q3+self.q*self.q4)
        q3dot= -0.5*(-self.q*self.q1+self.r*self.q2-self.p*self.q4)
        q4dot= -0.5*(-self.r*self.q1-self.q*self.q2+self.p*self.q3)
        
        
        
        #Now integrate Euler Integration for Process Updates and Covariance Updates
        # Euler works fine
        self.xhatdot=np.array([[pxdot, pydot, pzdot, vxdot, vydot, vzdot, q1dot, q2dot, q3dot, q4dot, 0, 0, 0, 0, 0, 0]]).transpose()
        self.xhat=self.xhat+self.xhatdot*self.dt
        
        # Extract and normalize the quat    
        self.quat = np.array([[self.xhat[6,0], self.xhat[7,0], self.xhat[8,0], self.xhat[9,0]]]).T
        self.quatmag= LA.norm(self.quat)#math.sqrt(pow(q1,2)+pow(q2,2)+pow(q3,2)+pow(q4,2))
        
        #Renormalize quaternion if needed
        if abs(self.quatmag-1) > 0.01:
            self.quat = self.quat/LA.norm(self.quat)
    
        #re-assign quat
        self.xhat[6,0] = self.quat[0,0]
        self.xhat[7,0] = self.quat[1,0]
        self.xhat[8,0] = self.quat[2,0]
        self.xhat[9,0] = self.quat[3,0]
    
        #Remember again the state vector [ px py pz vx vy vz q1 q2 q3 q4 bp bq br bx by bz]
        
        # Now write out all the partials to compute the transition matrix Phi
        #delV/delQ
        vqdot=np.array([[2*(self.q1*self.fx-self.q4*self.fy+self.q3*self.fz), 2*(self.q2*self.fx+self.q3*self.fy+self.q4*self.fz), 2*(-self.q3*self.fx+self.q2*self.fy+self.q1*self.fz), 2*(-self.q4*self.fx-self.q1*self.fy+self.q2*self.fz)], [-2*(-self.q4*self.fx-self.q1*self.fy+self.q2*self.fz), -2*(-self.q3*self.fx+self.q2*self.fy+self.q1*self.fz), 2*(self.q2*self.fx+self.q3*self.fy+self.q4*self.fz), 2*(self.q1*self.fx-self.q4*self.fy+self.q3*self.fz)], [2*(-self.q3*self.fx+self.q2*self.fy+self.q1*self.fz), -2*(-self.q4*self.fx-self.q1*self.fy+self.q2*self.fz), -2*(self.q1*self.fx-self.q4*self.fy+self.q3*self.fz), 2*(self.q2*self.fx+self.q3*self.fy+self.q4*self.fz)]])
        #delV/del_abias
        vbadot=-self.L_lb
        #delQ/delQ
        qqdot=-0.5*np.array([[0, self.p, self.q, self.r],
        [-self.p, 0, -self.r, self.q],
        [-self.q, self.r, 0, -self.p],
        [-self.r, -self.q, self.p, 0]])
        #delQ/del_gyrobias
        qbwdot=0.5*np.array([[ self.q2, self.q3, self.q4],\
             [-self.q1, self.q4, -self.q3],\
             [-self.q4, -self.q1, self.q2],\
             [self.q3, -self.q2, -self.q1]])
        # Now assemble the Transition matrix
        Phi=np.block([[np.zeros([3,3]), np.eye(3), np.zeros([3,4]), np.zeros([3,3]), np.zeros([3,3])],\
         [np.zeros([3,3]), np.zeros([3,3]), vqdot, np.zeros([3,3]), vbadot],\
         [np.zeros([4,3]), np.zeros([4,3]), qqdot, qbwdot, np.zeros([4,3])],\
         [np.zeros([3,3]), np.zeros([3,3]), np.zeros([3,4]), np.zeros([3,3]), np.zeros([3,3])],\
         [np.zeros([3,3]), np.zeros([3,3]), np.zeros([3,4]), np.zeros([3,3]), np.zeros([3,3])]])
        #Propagate the error covariance matrix, I suggest using the continuous integration since Q, R are not discretized 
        Pdot = Phi@self.P+self.P@Phi.transpose() + self.Q
        #P1 = Pdot*dt
        self.P = self.P +Pdot*self.dt
    
        #Correction step
        #Get your measurements, 3 positions and 3 velocities from GPS
        self.z = np.array([[self.A[6], self.A[7], self.A[8], self.A[9], self.A[10], self.A[11]]]).T #x y z vx vy vz
    
        #Write out the measurement matrix linearization to get H
        rgps=np.array([[1.5,0,0]])
        # del v/del q
        Hxq=np.array([[-rgps[0,0]*2*self.q1, -rgps[0,0]*2*self.q2, rgps[0,0]*2*self.q3, rgps[0,0]*2*self.q4],
         [-rgps[0,0]*2*self.q4, -rgps[0,0]*2*self.q3, -rgps[0,0]*2*self.q2, -rgps[0,0]*2*self.q1],
         [rgps[0,0]*2*self.q3, -rgps[0,0]*2*self.q4, rgps[0,0]*2*self.q1, -rgps[0,0]*2*self.q2]])
#         Hxq=np.zeros([3,4])
        #del P/del q
        Hvq=np.array([[rgps[0,0]*2*self.q3*self.q+rgps[0,0]*2*self.q4*self.r,  rgps[0,0]*2*self.q4*self.q-rgps[0,0]*2*self.q3*self.r, rgps[0,0]*2*self.q1*self.q-rgps[0,0]*2*self.q2*self.r, rgps[0,0]*2*self.q2*self.q+rgps[0,0]*2*self.q1*self.r], [-rgps[0,0]*2*self.q2*self.q+rgps[0,0]*2*self.q1*self.r,  rgps[0,0]*2*self.q2*self.r-rgps[0,0]*2*self.q1*self.q, rgps[0,0]*2*self.q4*self.q-rgps[0,0]*2*self.q3*self.r, rgps[0,0]*2*self.q3*self.q+rgps[0,0]*2*self.q4*self.r], [rgps[0,0]*2*self.q1*self.q-rgps[0,0]*2*self.q2*self.r,  -rgps[0,0]*2*self.q2*self.q+rgps[0,0]*2*self.q1*self.r, -rgps[0,0]*2*self.q3*self.q-rgps[0,0]*2*self.q4*self.r, rgps[0,0]*2*self.q4*self.q-rgps[0,0]*2*self.q3*self.r]])
#         Hvq=np.zeros([3,4])
        
        # Assemble H
        self.H=np.block([[np.eye(3), np.zeros([3,3]), Hxq, np.zeros([3,6])],\
                         [np.zeros([3,3]), np.eye(3), Hvq, np.zeros([3,6])]])
        #Compute Kalman gain
        e=self.z-np.dot(self.H,self.xhat)
        S=np.dot(np.dot(self.H,self.P),self.H.transpose())+self.R
        K=np.dot(np.dot(self.P,self.H.transpose()),inv(S))
        #Perform xhat correction    xhat = xhat + K@(z-H@xhat)
        self.xhat=self.xhat+np.dot(K,e);
        #propagate error covariance approximation P = (np.eye(16,16)-K@H)@P
        self.P=np.dot(np.eye(16)-np.dot(K,self.H),self.P)
        #end
        
        #Now let us do some book-keeping 
        # Get some Euler angles
        phi, theta, psi = self.quat2euler(self.quat.T)

        self.PHI.append(phi[0][0]*(180/math.pi))
        self.THETA.append(theta[0][0]*(180/math.pi))
        self.PSI.append(psi[0][0]*(180/math.pi))
    
    
        quat1 = np.array([self.A[12:16]])
        
        phi_raw, theta_raw, psi_raw = self.quat2euler(quat1)
        
        #####Saving the information for the plots
        self.PHI_raw.append(phi_raw[0][0]*(180/math.pi))
        self.THETA_raw.append(theta_raw[0][0]*(180/math.pi))
        self.PSI_raw.append(psi_raw[0][0]*(180/math.pi))
        DP = np.diag(self.P)
        self.P_R.append(DP[0:3])
        self.P_R1.append(DP[3:6])
        self.P_R2.append(DP[6:10])
        self.Pos.append(self.xhat[0:3].T[0])
        self.Vel.append(self.xhat[3:6].T[0])
        self.Quater.append(self.xhat[6:10].T[0])
        self.Bias.append(self.xhat[10:16].T[0])
        B = self.A[6:9].T
        self.A_PosX.append(B[0])
        self.A_PosY.append(B[1])
        self.A_PosZ.append(B[2])
        C = self.A[9:12].T
        self.AVelX.append(C[0])
        self.AVelY.append(C[1])
        self.AVelZ.append(C[2])
        D = self.A[12:16].T
        self.AQuater1.append(D[0])
        self.AQuater2.append(D[1])
        self.AQuater3.append(D[2])
        self.AQuater4.append(D[3])
        self.P_angular.append(self.p)
        self.Q_angular.append(self.q)
        self.R_angular.append(self.r)
        self.P_raw_angular.append(self.p_raw)
        self.Q_raw_angular.append(self.q_raw)
        self.R_raw_angular.append(self.r_raw)
        self.Fx_RAW.append(self.fx_raw)
        self.Fy_RAW.append(self.fy_raw)
        self.Fz_RAW.append(self.fz_raw)
        self.loop_t += 1
        rospy.loginfo(self.loop_t)
        self.time.append(self.loop_t)
        
    

# Main function.
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('EKF', anonymous = True)
    # Go to the main loop.
    ne = listenerNode()
    ne.run()