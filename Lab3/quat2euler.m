function [phi, theta, psi] = quat2euler(q)
[n, junk] = size(q);
m=eye(3);
phi=zeros(n,1);
theta=phi;
psi = theta;

for i=1:n,...
      q0 = q(i,1);
      q1 = q(i,2);
      q2 = q(i,3);
      q3 = q(i,4);
  m(1,1) = 1.0 - 2.0*( q2*q2 + q3*q3 );
  m(1,2) = 2.0*( q1*q2 - q0*q3 );
  m(1,3) = 2.0*( q1*q3 + q0*q2 );
  m(2,1) = 2.0*( q1*q2 + q0*q3 );
  m(2,2) = 1.0 - 2.0*( q1*q1 + q3*q3 );
  m(2,3) = 2.0*( q2*q3 - q0*q1 );
  m(3,1) = 2.0*( q1*q3 - q0*q2 );
  m(3,2) = 2.0*( q2*q3 + q0*q1 );
  m(3,3) = 1.0 - 2.0*( q1*q1 + q2*q2 );
    phi(i,1)   = atan2( m(3,2), m(3,3) );
    theta(i,1) = -asin( m(3,1) );
    psi(i,1)   = atan2( m(2,1), m(1,1) );
end
 
