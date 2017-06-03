function [thetalist,S] = DeltaIkin(R1,R2,L1,L2,p)
% Inverse kinematics of Delta mechanism
% Taking the five parameters 
% R1 = radius of the base
% R2 = radius of the top platform 
% L1 = length of the lower leg 
% L2 = length of the upper leg 
% p = desired x,y,z position
% Returns a 1x3 thetalist, containing values of each leg.
%         a number S, 1 = valid input, 0 = invalid input
%         when S = 1, set thetalist to be [0,0,0]
%   Example input:
%{ 
R1 = 1;
R2 = 1;
L1 = 1;
L2 = 1;
p = [0,0,1.5];
[thetalist,S] = DeltaIkin(R1,R2,L1,L2,p)
%}

% Output:
%thetalist =
%     0     0     0
%S =
%    1


blist = [[R1,0,0];[R1*cosd(120),R1*sind(120),0];[R1*cosd(240),R1*sind(240),0]];
plist = [[R2,0,0];[R2*cosd(120),R2*sind(120),0];[R2*cosd(240),R2*sind(240),0]];
pmblist = plist - blist;
bmplist = blist - plist;

J = [0,0,0];
G = zeros(1,3);
E = zeros(1,3);
F = zeros(1,3);
tp = zeros(1,3);
tm = zeros(1,3);
thetap = zeros(1,3);
thetam = zeros(1,3);
theta = zeros(1,3);
for i=1:3
    G(i) = L2^2 - L1^2 - Distance(p,bmplist(i,:))^2;
    E(i) = 2*p(3)*L1 + 2*pmblist(i,3)*L1;
    F(i) = 2*L1*((p(1)+pmblist(i,1))*cosd(i*120-120)+(p(2)+pmblist(i,2))*sind(i*120-120));
    tp(i) = (-F(i)+sqrt(E(i)^2+F(i)^2-G(i)^2))/(G(i)-E(i));
    tm(i) = (-F(i)-sqrt(E(i)^2+F(i)^2-G(i)^2))/(G(i)-E(i));
    thetap(i) = 2*atan(tp(i));
    thetam(i) = 2*atan(tm(i));
    if -pi/4 <= thetap(i) && thetap(i)<= pi/2 
        if isreal(thetap(i)) == 1
            theta(i) = thetap(i);
            J(i) = 1;
        end
    end
    if -pi/4 <= thetam(i) && thetam(i)<= pi/2 
        if isreal(thetap(i)) == 1
            theta(i) = thetam(i);
            J(i) = 1;
        end
    end
end

if J(1) == 1&& J(2)==1&& J(3)==1
    thetalist = [theta(1),theta(2),theta(3)];
    S = 1;  % success
else
    thetalist = [0,0,0];
    S = 0;
end

end

