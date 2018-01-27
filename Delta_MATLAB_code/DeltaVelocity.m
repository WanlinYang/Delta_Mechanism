function thetadotlist = DeltaVelocity( R1,R2,L1,L2,p,pdot )
% Inverse kinematics of Delta mechanism
% Taking the five parameters 
% R1 = radius of the base
% R2 = radius of the top platform 
% L1 = length of the lower leg 
% L2 = length of the upper leg 
% p = desired x,y,z position(1x3)
% pdot = desired x,y,z velocity(1x3)
% Returns a 1x3 thetadotlist
% Example input
%{
    thetadotlist = DeltaVelocity(1,0.5,1,1,[0,0,1.5],[0,0,2])
%}

[thetalist,S] = DeltaIkin(R1,R2,L1,L2,p);

if S==0
    msg = 'Invalid inputs.';
    error(msg);
end

blist = [[R1,0,0];[R1*cosd(120),R1*sind(120),0];[R1*cosd(240),R1*sind(240),0]];
plist = [[R2,0,0];[R2*cosd(120),R2*sind(120),0];[R2*cosd(240),R2*sind(240),0]];
Llist = [[L1*sin(thetalist(1)),0,L1*cos(thetalist(1))];
    [L1*cosd(120)*sin(thetalist(2)),L1*sind(120)*sin(thetalist(2)),L1*cos(thetalist(2))];
    [L1*cosd(240)*sin(thetalist(3)),L1*sind(240)*sin(thetalist(3)),L1*cos(thetalist(3))]];
pmbmLlist = plist-blist-Llist;
x = p(1);
y = p(2);
z = p(3);

A = [[x+pmbmLlist(1,1),y+pmbmLlist(1,2),z+pmbmLlist(1,3)];
    [x+pmbmLlist(2,1),y+pmbmLlist(2,2),z+pmbmLlist(2,3)];
    [x+pmbmLlist(3,1),y+pmbmLlist(3,2),z+pmbmLlist(3,3)]];
b1 = L1*cosd(0)*cos(thetalist(1))*(x+pmbmLlist(1,1))+L1*sind(0)*cos(thetalist(1))*...
    (y+pmbmLlist(1,2))-L1*sin(thetalist(1))*(z+pmbmLlist(1,3));
b2 = L1*cosd(120)*cos(thetalist(2))*(x+pmbmLlist(2,1))+L1*sind(120)*cos(thetalist(2))*...
    (y+pmbmLlist(2,2))-L1*sin(thetalist(2))*(z+pmbmLlist(2,3));
b3 = L1*cosd(240)*cos(thetalist(3))*(x+pmbmLlist(3,1))+L1*sind(240)*cos(thetalist(3))*...
    (y+pmbmLlist(3,2))-L1*sin(thetalist(3))*(z+pmbmLlist(3,3));
B = [[b1,0,0];[0,b2,0];[0,0,b3]];
thetadotlist = inv(B)*A*pdot';        
thetadotlist = thetadotlist';

end

