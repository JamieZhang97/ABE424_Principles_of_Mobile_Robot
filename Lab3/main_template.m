

%% 6 DOF EKF GPS-INS Fusion
% This code was developed by Girish Chowdhary
% To learn more about the filter, please read:
% 1. A Compact Guidance, Navigation, and Control System for Unmanned Aerial Vehicles (2006)
% by Henrik B. Christophersen , R. Wayne Pickell , James C. Neidhoefer , Adrian A. Koller , K. Kannan , Eric N. Johnson
% 2. (More advanced) GPS-Denied Indoor and Outdoor Monocular Vision Aided
% Navigation and Control of Unmanned Aircraft, Chowdhary, Magree, Johnson,
% Shein, Wu
% 3. (very detailed) (late) Nimrod Rooz's thesis proposal



clear all
close all

% load a check file with the data
load check
% This loads a matrix called A (arbitrary name, nothing to do with the real
% A) which contains the data that you need

% initial position in x y and z
x=[0 0 0];

% bias values, these are accelerometer and gyroscope biases
bp= 0;%.54*pi/180;
bq=-12*pi/180;
br=-.1*pi/180;
bfx = 0;
bfy = 0;
bfz = 0;


% IMU location specifier
r_imu=[-.5/12 -3/12 1/12]'*0; %% I have set this to zero, for Bonus, you can include the effect of this
r_GPS=[1.5, 0 ,0 ]; % This is the location of the GPS wrt CG, this is very important
%rotation matrix ------------------------------------------------------
phi= x(1);
theta= x(2);
psi = x(3);

%roation matrix body to inertial
L_bi = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta); ...
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)  sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta); ...
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];


Rt2b=L_bi;

[U,S,V]=svd(Rt2b);
R = U*V';
if 1+R(1,1)+R(2,2)+R(3,3) > 0
    b(1,1)    = 0.5*sqrt(1+R(1,1)+R(2,2)+R(3,3));
    b(2,1)    = (R(3,2)-R(2,3))/4/b(1);
    b(3,1)    = (R(1,3)-R(3,1))/4/b(1);
    b(4,1)    = (R(2,1)-R(1,2))/4/b(1);
    b = b/norm(b);    % renormalize
else
    R; 
    error('R diagonal too negative.')
    b=zeros(4,1);
end

%b =[0 0 0 0]';

% set quats
%-----------------------------------------------------------------
q1=b(1);%the quaternions are called b1-b4 in the data file that you loaded
q2=b(2);
q3=b(3);
q4=b(4);

%initialize velocity
vx = 0;
vy = 0;
vz = 0;

%set sample time
dt = .02;
tf=size(A,2);


% initialize x hat
% Note carefull the order the states appear in, this can be arbitrary, but
% we must stick to it along the entire code
%      [x y z vx vy vz          quat    gyro-bias accl-bias]
xhat = [0 0 0 0 0 0 b(1) b(2) b(3) b(4) bp bq br bfx bfy bfz]';

% noise params process noise (my gift to you :))
Q = diag([.1 .1 .1 .1 .1 .1 .8 .8 .8 .8 .0001 .0001 .0001 .0001 .0001 .0001]);

% noise params, measurement noise
% measurements are GPS position and velocity and mag
R = diag([9 9 9 3 3 3]);

% Initialize P, the covariance matrix
P = diag([30 30 30 3 3 3 .1 .1 .1 .1 .1 .1 .1 .1 .1 .1]);
Pdot=P*0;
tic
for k = 1:tf
    time= (k-1)*dt;
    
    %  Streaming sensor measurements and adjust for bias
    % these come from the file that is loaded in the begining
    p = (A(1,k)*pi/180-xhat(11));
    q = (A(2,k)*pi/180-xhat(12));
    r = (A(3,k)*pi/180-xhat(13));
    fx = (A(4,k)-xhat(14));
    fy = (A(5,k)-xhat(15));
    fz = -A(6,k)-xhat(16);
    
    % Raw sensor measurments for plotting
    p_raw = A(1,k)*pi/180;
    q_raw = A(2,k)*pi/180;
    r_raw = A(3,k)*pi/180;
    fx_raw = A(4,k);
    fy_raw = A(5,k);
    fz_raw = A(6,k);
    
    quat = [xhat(7) xhat(8) xhat(9) xhat(10)]';
    
    q1 = quat(1);
    q2 = quat(2);
    q3 = quat(3);
    q4 = quat(4);
    
    L_bl = [q1^2+q2^2-q3^2-q4^2    2*(q2*q3+q1*q4)      2*(q2*q4-q1*q3);
        2*(q2*q3-q1*q4)    q1^2-q2^2+q3^2-q4^2    2*(q3*q4+q1*q2);
        2*(q2*q4+q1*q3)      2*(q3*q4-q1*q2)    q1^2-q2^2-q3^2+q4^2];
    L_lb = L_bl';
    
    
    %% Implement your code here: 
    
    %% Prediction step
    %First write out all the dots, e.g. pxdot, pydot, q1dot etc
    pxdot=xhat(4);
    pydot=xhat(5); 
    pzdot=xhat(6);
    vdot=L_lb*[fx;fy;fz];
    vxdot=vdot(1);
    vydot=vdot(2); 
    vzdot=vdot(3)+32.2; 
    q1dot= -0.5*(p*q2+q*q3+r*q4);
    q2dot= -0.5*(-p*q1-r*q3+q*q4);
    q3dot= -0.5*(-q*q1+r*q2-p*q4);
    q4dot= -0.5*(-r*q1-q*q2+p*q3);
    
    %Now integrate Euler Integration for Process Updates and Covariance Updates
    % Euler works fine
    xhatdot=[pxdot pydot pzdot vxdot vydot vzdot q1dot q2dot q3dot q4dot 0 0 0 0 0 0]';
    xhat=xhat+xhatdot*dt;
    % Extract and normalize the quat
    quat = [xhat(7) xhat(8) xhat(9) xhat(10)]';
    quatmag= norm(quat);%sqrt(q1^2+q2^2+q3^2+q4^2)
    %Renormalize quaternion if needed
    if abs(quatmag-1)>0.01
        quat = quat/norm(quat);
    end
    %re-assign quat
    xhat(7) = quat(1);
    xhat(8) = quat(2);
    xhat(9) = quat(3);
    xhat(10) = quat(4);


    %Remember again the state vector [ px py pz vx vy vz q1 q2 q3 q4 bp bq br bx by bz]
    
    % Now write out all the partials to compute the transition matrix Phi
    %delV/delQ
    vqdot=[2*(q1*fx-q4*fy+q3*fz) 2*(q2*fx+q3*fy+q4*fz) 2*(-q3*fx+q2*fy+q1*fz) 2*(-q4*fx-q1*fy+q2*fz);
        -2*(-q4*fx-q1*fy+q2*fz) -2*(-q3*fx+q2*fy+q1*fz) 2*(q2*fx+q3*fy+q4*fz) 2*(q1*fx-q4*fy+q3*fz);
        2*(-q3*fx+q2*fy+q1*fz) -2*(-q4*fx-q1*fy+q2*fz) -2*(q1*fx-q4*fy+q3*fz) 2*(q2*fx+q3*fy+q4*fz) ];
    
    %delV/del_abias
    vbadot=-L_lb;
    %delV/del_gyro_bias
    
    %delQ/delQ
    qqdot=-0.5*[0 p q r;
        -p 0 -r q;
        -q r 0 -p;
        -r -q p 0];
        
    %delQ/del_gyrobias
     qbwdot=0.5*[ q2 q3 q4; 
             -q1 q4 -q3;
             -q4 -q1 q2;
             q3 -q2 -q1];
      
    % Now assemble the Transition matrix
    A1=[ zeros(3,3) eye(3) zeros(3,4) zeros(3,3) zeros(3,3);
         zeros(3,3) zeros(3,3) vqdot zeros(3,3) vbadot;
         zeros(4,3) zeros(4,3) qqdot qbwdot zeros(4,3);
         zeros(3,3) zeros(3,3) zeros(3,4) zeros(3,3) zeros(3,3);
         zeros(3,3) zeros(3,3) zeros(3,4) zeros(3,3) zeros(3,3)];
   % Propagate the error covariance matrix, I suggest using the continuous integration since Q, R are not discretized 
   Pdot = A1*P+P*A1'+Q;
   P = P +Pdot*dt;
    
    %% Correction step
    % Get your measurements, 3 positions and 3 velocities from GPS
    z =[ A(7,k) A(8,k) A(9,k) A(10,k) A(11,k) A(12,k)]'; % x y z vx vy vz

    % Write out the measurement matrix linearization to get H
    rgps=[0,0,0];
    %del P/del q
    Hxq=[-rgps(1)*2*q1 -rgps(1)*2*q2 rgps(1)*2*q3 rgps(1)*2*q4;
         -rgps(1)*2*q4 -rgps(1)*2*q3 -rgps(1)*2*q2 -rgps(1)*2*q1;
         rgps(1)*2*q3 -rgps(1)*2*q4 rgps(1)*2*q1 -rgps(1)*2*q2];
    % del v/del q
    Hvq=[rgps(1)*2*q3*q+rgps(1)*2*q4*r  rgps(1)*2*q4*q-rgps(1)*2*q3*r rgps(1)*2*q1*q-rgps(1)*2*q2*r rgps(1)*2*q2*q+rgps(1)*2*q1*r;
        -rgps(1)*2*q2*q+rgps(1)*2*q1*r  rgps(1)*2*q2*r-rgps(1)*2*q1*q rgps(1)*2*q4*q-rgps(1)*2*q3*r rgps(1)*2*q3*q+rgps(1)*2*q4*r;
        rgps(1)*2*q1*q-rgps(1)*2*q2*r  -rgps(1)*2*q2*q+rgps(1)*2*q1*r -rgps(1)*2*q3*q-rgps(1)*2*q4*r rgps(1)*2*q4*q-rgps(1)*2*q3*r];
    % Assemble H
    H=[eye(3) zeros(3,3) Hxq zeros(3,6);
       zeros(3,3) eye(3) Hvq zeros(3,6)];
    %Compute Kalman gain
    e=z-H*xhat;
    S=H*P*H'+R;
    K=P*H'/S;
    % Perform xhat correction    xhat = xhat + K*(z - H*xhat);
    xhat=xhat+K*e;
    % propagate error covariance approximation P = (eye(16)-K*H)*P;
    P=(eye(16)-K*H)*P;
    %  end
    
    
    %% Now let us do some book-keeping 
    % Get some Euler angles
    [phi(k),theta(k),psi(k)]=quat2euler(quat');
    phi(k)=phi(k)*180/pi;
    theta(k)=theta(k)*180/pi;
    psi(k)=psi(k)*180/pi;
    
    quat1 = A(13:16,k);
    [phi_raw(k),theta_raw(k),psi_raw(k)]=quat2euler(quat1');
    phi_raw(k)=phi_raw(k)*180/pi;
    theta_raw(k)=theta_raw(k)*180/pi;
    psi_raw(k)=psi_raw(k)*180/pi;
    
    %  Recording data for plots
    xhatR(k,:)= xhat;
    P_R(k,:) = diag(P);
    z_R(k,:) = z;
    OMEGA_raw(k,:)=[p_raw,q_raw,r_raw]';
    OMEGA(k,:)=[p,q,r]';
    FX(k,:)=[fx_raw,fy_raw,fz_raw]';
    
end
toc
t = 1:k;
figure(1)
plot(t,P_R(:,1:3))
title('Covariance of Position')
legend('px','py','pz')
figure(2)
plot(t,P_R(:,4:6))
legend('pxdot','pydot','pzdot')
title('Covariance of Velocities')
figure(3)
plot(t,P_R(:,7:10))
title('Covariance of Quaternions')

figure(8)
plot(t,phi,t,theta,t,psi,t,phi_raw,'b:',t,theta_raw,'g:',t,psi_raw,'r:')
title('Phi, Theta, Psi')
legend('phi','theta','psi','phiraw', 'thetaraw', 'psiraw')

figure(4)
hold on
plot(t,xhatR(:,1:3),t,A(7:9,:),'r:')
%plot(t,z_R(:,1),'r')
title('Position')
figure(5)
plot(t,xhatR(:,4:6),t,A(10:12,:),'r:')
title('vel x y z')
figure(6)
plot(t,xhatR(:,7:10),t,A(13:16,:),'r:')
title('Quat')
figure(9)
plot(t,xhatR(:,11:16))
title('Bias')
legend('bp','bq','br','bfx','bfy','bfz')

figure(7)
plot(t,OMEGA(:,1),t,OMEGA(:,2),t,OMEGA(:,3))
title('OMEGA with Bias')
legend('p','q','r')

figure(10)
plot(t,OMEGA_raw(:,1),t,OMEGA_raw(:,2),t,OMEGA_raw(:,3))
title('OMEGA raw without Bias')
legend('p','q','r')

figure(11)
plot(t,FX(:,1),t,FX(:,2),t,FX(:,3))
title('accelerometer')
legend('ax','ay','az')