function p = DeltaFkin(R1,R2,L1,L2,thetalist,p0)
% Forward kinematics of Delta mechanism
% Taking the six parameters 
% R1 = radius of the base
% R2 = radius of the top platform 
% L1 = length of the lower leg 
% L2 = length of the upper leg 
% thetalist = given theta1,theta2 and theta3 angles
% p0 = initial guess of endeffect x,y,z position *will be critical for the
% the calculation.
% Returns a 1x3 p vector, containing x,y,z position.

%   Example input:
%{ 
R1 = 1;
R2 = 0.9;
L1 = 1;
L2 = 1;
thetalist = [0,0,0.2];
p0 = [0,0,1];
p = DeltaFkin(R1,R2,L1,L2,thetalist,p0)
%}

% Output:
%p =

 %  -0.0849   -0.1470    1.9717
 
 
blist = [[R1,0,0];[R1*cosd(120),R1*sind(120),0];[R1*cosd(240),R1*sind(240),0]];
plist = [[R2,0,0];[R2*cosd(120),R2*sind(120),0];[R2*cosd(240),R2*sind(240),0]];
Llist = [[L1*sin(thetalist(1)),0,L1*cos(thetalist(1))];
    [L1*cosd(120)*sin(thetalist(2)),L1*sind(120)*sin(thetalist(2)),L1*cos(thetalist(2))];
    [L1*cosd(240)*sin(thetalist(3)),L1*sind(240)*sin(thetalist(3)),L1*cos(thetalist(3))]];

xlist = [p0(1),p0(2),p0(3)];   % 3-dimensional initial guess vector

%Solve equation with multi-newton method
y = MultiNewtonMethod(@DeltaoFunction,@DeltadFunction,xlist,blist,plist,Llist,L2);
p = y(1:3);

end










