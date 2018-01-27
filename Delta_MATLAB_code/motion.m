clear all

R1 = 0.18;                      % lower radius(m)
R2 = 0.062;                     % upper radius
L1 = 0.2;                       % length of lower legs(m)
L2 = 0.31;                      % length of upper legs
z_height = 0.3;                 % height of platform
motion_radius = 0.1;            % radius of the motion circle
total_time = 10;                % seconds
dtime = 0.01;                   % time interval


time = linspace(0,2*pi,total_time/dtime)';
x_position = motion_radius*sin(time);    % position as function of time
y_position = motion_radius*cos(time);
z_position = z_height*ones(length(time),1);
uc = [x_position, y_position, z_position];
thetalist = zeros(length(time),3);

for i=1:length(time)
    ptemp = uc(i,:);
    thetalist(i,:) = DeltaIkin(R1,R2,L1,L2,ptemp);
end

thetalist_deg = rad2deg(thetalist);
% plot frames varing with time

figure;
home_p = [0,0,z_height];
[home_theta,S] = DeltaIkin(R1,R2,L1,L2,home_p);
DeltaVisualization( R1,R2,L1,L2,home_p,home_theta )
 axeslength = 0.1;
 for i = 1:10:length(time)
     p = [x_position(i),y_position(i),z_position(i)]';
     eframe(:,1) = p + [axeslength,0,0]';  % x axis
     eframe(:,2) = p + [0,axeslength,0]';  % y axis
     eframe(:,3) = p + [0,0,axeslength]';  % z axis
     plot3([p(1),eframe(1,1)],[p(2),eframe(2,1)],[p(3),eframe(3,1)],'LineWidth',1.5,'Color',[1,0,0])  % x axis
     hold on;
     plot3([p(1),eframe(1,2)],[p(2),eframe(2,2)],[p(3),eframe(3,2)],'LineWidth',1.5,'Color',[0,1,0])  % y axis
     hold on;
     plot3([p(1),eframe(1,3)],[p(2),eframe(2,3)],[p(3),eframe(3,3)],'LineWidth',1.5,'Color',[0,0,1])  % z axis
     hold on;
 end
 axis equal
 grid on
 
 