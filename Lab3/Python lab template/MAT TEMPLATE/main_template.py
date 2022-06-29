#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov  4 00:42:12 2020

@author: andres
"""


import math
import numpy as np
from numpy import linalg as LA
from scipy.io import loadmat
import matplotlib.pyplot as plt

#6 DOF EKF GPS-INS Fusion
# This code was developed by Girish Chowdhary
# To learn more about the filter, please read:
# 1. A Compact Guidance, Navigation, and Control System for Unmanned Aerial Vehicles (2006)
# by Henrik B. Christophersen , R. Wayne Pickell , James C. Neidhoefer , Adrian A. Koller , K. Kannan , Eric N. Johnson
# 2. (More advanced) GPS-Denied Indoor and Outdoor Monocular Vision Aided
# Navigation and Control of Unmanned Aircraft, Chowdhary, Magree, Johnson,
# Shein, Wu
# 3. (very detailed) (late) Nimrod Rooz's thesis proposal


# load a check file with the data
data = loadmat('check.mat')
#This loads a matrix called A (arbitrary name, nothing to do with the real
# A) which contains the data that you need

# initial position in x y and z
x = np.array([0, 0, 0])

# bias values, these are accelerometer and gyroscope biases
bp= 0;#.54*math.pi/180;
bq=-12*math.pi/180;
br=-.1*math.pi/180;
bfx = 0;
bfy = 0;
bfz = 0;


#IMU location specifier
r_imu=np.array([[-0.5/12, -3/12, 1/12]]).T*0 #I have set this to zero, for Bonus, you can include the effect of this
r_GPS=np.array([1.5, 0 ,0]) #This is the location of the GPS wrt CG, this is very important
#rotation matrix ------------------------------------------------------
phi= x[0]
theta= x[1]
psi = x[2]

#rotation matrix body to inertial
L_bi = np.array([[math.cos(theta)*math.cos(psi), math.cos(theta)*math.sin(psi), -math.sin(theta)],
    [math.sin(phi)*math.sin(theta)*math.cos(psi)-math.cos(phi)*math.sin(psi),  math.sin(phi)*math.sin(theta)*math.sin(psi)+math.cos(phi)*math.cos(psi), math.sin(phi)*math.cos(theta)],
    [math.cos(phi)*math.sin(theta)*math.cos(psi)+math.sin(phi)*math.sin(psi),  math.cos(phi)*math.sin(theta)*math.sin(psi)-math.sin(phi)*math.cos(psi), math.cos(phi)*math.cos(theta)]])


Rt2b=L_bi

[U,S,V]=LA.svd(Rt2b)
R = U@V.T
b = np.zeros((4,1))

if 1+R[0,0]+R[1,1]+R[2,2] > 0:
    b[0,0]    = 0.5*math.sqrt(1+R[0,0]+R[1,1]+R[2,2])
    b[1,0]    = (R[2,1]-R[1,2])/4/b[0,0]
    b[2,0]    = (R[0,2]-R[2,0])/4/b[0,0]
    b[3,0]    = (R[1,0]-R[0,1])/4/b[0,0]
    b       /= LA.norm(b)    # renormalize
else:
    print(R)
    print('R diagonal too negative.')
    b = np.zeros((4,1))


#b =np.zeros((4,1))

#set quats
#-----------------------------------------------------------------
q1=b[0,0] #the quaternions are called b1-b4 in the data file that you loaded
q2=b[1,0]
q3=b[2,0]
q4=b[3,0]

#initialize velocity
vx = 0
vy = 0
vz = 0

#set sample time
dt = 0.02
A = data['A']
tf=A.shape[1]


#initialize x hat
#Note carefull the order the states appear in, this can be arbitrary, but
#we must stick to it along the entire code
#      [x y z vx vy vz          quat    gyro-bias accl-bias]
xhat = np.array([[0, 0, 0, 0, 0, 0, b[0,0], b[1,0], b[2,0], b[3,0], bp, bq, br, bfx, bfy, bfz]]).T


#noise params process noise (my gift to you :))
Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.8, 0.8, 0.8, 0.8, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001])

#noise params, measurement noise
#measurements are GPS position and velocity and mag
R = np.diag([9, 9, 9, 3, 3, 3])

#Initialize P, the covariance matrix
P = np.diag([30, 30, 30, 3, 3, 3, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
Pdot=P*0

def quat2euler(q):
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

time = np.zeros(A.shape[1])

PHI = np.zeros(A.shape[1])
THETA = np.zeros(A.shape[1])
PSI = np.zeros(A.shape[1])
phi_raw = 0
theta_raw = 0
psi_raw = 0
PHI_RAW = np.zeros(A.shape[1])
THETA_RAW = np.zeros(A.shape[1])
PSI_RAW = np.zeros(A.shape[1])
xhatR = np.zeros((A.shape[1],xhat.shape[0]))
P_R = np.zeros((A.shape[1],P.shape[0]))
OMEGA = np.zeros((A.shape[1],3))
OMEGA_RAW = np.zeros((A.shape[1],3))
FX = np.zeros((A.shape[1],3))

for k in range(0,tf):
    time[k] = k+1
    
    #Streaming sensor measurements and adjust for bias
    # these come from the file that is loaded in the begining
    p = (A[0,k]*math.pi/180-xhat[10,0])
    q = (A[1,k]*math.pi/180-xhat[11,0])
    r = A[2,k]*math.pi/180-xhat[12,0]
    fx = (A[3,k]-xhat[13,0])
    fy = (A[4,k]-xhat[14,0])
    fz = -A[5,k]-xhat[15,0]
    
    #Raw sensor measurments for plotting
    p_raw = A[0,k]*math.pi/180
    q_raw = A[1,k]*math.pi/180
    r_raw = A[2,k]*math.pi/180
    fx_raw = A[3,k]
    fy_raw = A[4,k]
    fz_raw = A[5,k]
   
    quat = np.array([[xhat[6,0], xhat[7,0], xhat[8,0], xhat[9,0]]]).T
    
    q1 = quat[0,0]
    q2 = quat[1,0]
    q3 = quat[2,0]
    q4 = quat[3,0]
    
   
    L_bl = np.array([[pow(q1,2)+pow(q2,2)-pow(q3,2)-pow(q4,2), 2*(q2*q3+q1*q4), 2*(q2*q4-q1*q3)],
        [2*(q2*q3-q1*q4), pow(q1,2)-pow(q2,2)+pow(q3,2)-pow(q4,2), 2*(q3*q4+q1*q2)],
        [2*(q2*q4+q1*q3), 2*(q3*q4-q1*q2), pow(q1,2)-pow(q2,2)-pow(q3,2)+pow(q4,2)]])
    L_lb = L_bl.transpose()
    
    
    #Implement your code here: 
    
    #Prediction step
    #First write out all the dots, e.g. pxdot, pydot, q1dot etc
    
    print('your code here')
    
    #Now integrate Euler Integration for Process Updates and Covariance Updates
    # Euler works fine
    
    
    # Extract and normalize the quat    
    quat = np.array([[xhat[6,0], xhat[7,0], xhat[8,0], xhat[9,0]]]).T
    quatmag= LA.norm(quat)#math.sqrt(pow(q1,2)+pow(q2,2)+pow(q3,2)+pow(q4,2))
    
    #Renormalize quaternion if needed
    if abs(quatmag-1) > 0.01:
        quat = quat/LA.norm(quat)
    
    #re-assign quat
    xhat[6,0] = quat[0,0]
    xhat[7,0] = quat[1,0]
    xhat[8,0] = quat[2,0]
    xhat[9,0] = quat[3,0]

    #Remember again the state vector [ px py pz vx vy vz q1 q2 q3 q4 bp bq br bx by bz]
    
    # Now write out all the partials to compute the transition matrix Phi
    #delV/delQ
   
    
    #delV/del_abias
    
    #delV/del_gyro_bias
    
    #delQ/delQ
 
    #delQ/del_gyrobias
     
      
    # Now assemble the Transition matrix
    
    #Propagate the error covariance matrix, I suggest using the continuous integration since Q, R are not discretized 
    #Pdot = Phi@P+P@Phi.transpose() + Q
    #P1 = Pdot*dt
    #P = P +Pdot*dt
    
    #Correction step
    #Get your measurements, 3 positions and 3 velocities from GPS
    z = np.array([[A[6,k], A[7,k], A[8,k], A[9,k], A[10,k], A[11,k]]]).T #x y z vx vy vz
    
    #Write out the measurement matrix linearization to get H
    
    #del P/del q
    
    # del v/del q
    
    # Assemble H
    
  
 
    #Compute Kalman gain
    
    #Perform xhat correction    xhat = xhat + K@(z-H@xhat)
    
    
    #propagate error covariance approximation P = (np.eye(16,16)-K@H)@P
    #end
    
    
    #Now let us do some book-keeping 
    # Get some Euler angles
    phi, theta, psi = quat2euler(quat.T)

    PHI[k] = phi*(180/math.pi)
    THETA[k] = theta*(180/math.pi)
    PSI[k] = psi*(180/math.pi)
    
    
    quat1 = np.array([A[12:16,k]])
    phi_raw, theta_raw, psi_raw = quat2euler(quat1)
    PHI_RAW[k] = phi_raw*(180/math.pi)
    THETA_RAW[k] = theta_raw*(180/math.pi)
    PSI_RAW[k] = psi_raw*(180/math.pi)
    
    #Recording data for plots

    xhatR[k,:]= xhat.T
    P_R[k,:] = np.diag(P)
    OMEGA[k,:]=np.array([p,q,r])
    OMEGA_RAW[k,:] = np.array([p_raw,q_raw,r_raw])
    FX[k,:] = np.array([fx_raw,fy_raw,fz_raw])


plt.close('all')

plt.figure(1)
plt.plot(time,P_R[:,0:3])
plt.title('Covariance of Position')
plt.legend(['px','py','pz'])
plt.figure(2)
plt.plot(time,P_R[:,3:6])
plt.legend(['pxdot','pydot','pzdot'])
plt.title('Covariance of Velocities')
plt.figure(3)
plt.plot(time,P_R[:,6:10])
plt.title('Covariance of Quaternions')
plt.figure(4)
plt.plot(time,xhatR[:,0:3],time,A[6:9,:].T,'r:')
plt.title('Position')
plt.figure(5)
plt.plot(time,xhatR[:,3:6],time,A[9:12,:].T,'r:')
plt.title('vel x y z')
plt.figure(6)
plt.plot(time,xhatR[:,6:10],time,A[12:16,:].T,'r:')
plt.title('Quat')
plt.figure(7)
plt.plot(time,OMEGA[:,0],time,OMEGA[:,1],time,OMEGA[:,2])
plt.title('OMEGA with Bias')
plt.legend(['p','q','r'])
plt.figure(8)
plt.plot(time,PHI,'b', time, THETA, 'g', time,PSI, 'r', time,PHI_RAW,'b:',time,THETA_RAW,'g:',time,PSI_RAW,'r:')
plt.legend(['phi','theta','psi','phiraw', 'thetaraw', 'psiraw'])
plt.title('Phi, Theta, Psi')
plt.figure(9)
plt.plot(time,xhatR[:,10:16])
plt.title('Bias')
plt.legend(['bp','bq','br','bfx','bfy','bfz'])
plt.figure(10)
plt.plot(time,OMEGA_RAW[:,0],time,OMEGA_RAW[:,1],time,OMEGA_RAW[:,2])
plt.title('OMEGA without Bias')
plt.legend(['p','q','r'])
plt.figure(11)
plt.plot(time,FX[:,0],time,FX[:,1],time,FX[:,2])
plt.title('accelerometer')
plt.legend(['ax','ay','az'])