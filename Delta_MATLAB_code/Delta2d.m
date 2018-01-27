clear all

%% parameters of Delta
R1 = 0.15;                      % lower radius(m)
R2 = 0;                         % upper radius
L1 = 0.17;                      % length of lower legs(m)
L2 = 0.33;                      % length of upper legs

%% x z to theta
y = 0;                        % y always equals 0 in this case
x0 = 0;
z0 = 0.4;                     % the value of z at home position
x = x0;                       % x,z indicates the position of end-effector   
z = z0;
p = [x,y,z];                 % home position of Delta
thetalist = DeltaIkin(R1,R2,L1,L2,p);  % inverse kinematics x,z --> theta
theta1 = thetalist(1);
% thetalist contains the value of angles of three joints, thetalist(1), thetalist(2),
% thetalist(3) are angles of joint at the position of 0, 120, 240deg in x-y plane respectively
% Maybe thetalist(0) is the angle you want

%% thetadot to xdot zdot
% the values of x z are equal to previous part
J = DeltaJacobian( R1,R2,L1,L2,p );   % Jacobian at position of p
% thetadotlist = [1,0,0];             % thetadotlist contains the angular velocities of joints
% pdot = J*thetadotlist';
% xdot = pdot(1);
% zdot = pdot(3);

%% xdot zdot to thetadot
pdot = [1,0,0];
thetadot = inv(J)*pdot';
theta1dot = thetadot(1);






