clear all

%% input parameters

R1 = 0.2;                         % lower radius(m)
R2 = 0.1;                         % upper radius
L1 = 0.2;                         % length of lower legs(m)
L2 = 0.3;                         % length of upper legs
thetalist = [pi/6,pi/6,pi/6];          % value of theta(rad)
p_guess = [0,0,0.5];              % initial guess of position of upper platform

%% inverse kinematics and visualization

p = DeltaFkin(R1,R2,L1,L2,thetalist,p_guess);  % p is the position vector of upper platform
DeltaVisualization( R1,R2,L1,L2,p,thetalist );