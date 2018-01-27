clear all

%% user input
R1 = 0.18;                      % lower radius(m)
R2 = 0.062;                     % upper radius
L1 = 0.2;                       % length of lower legs(m)
L2 = 0.31;                      % length of upper legs
theta_home = pi/6;              % home angle (rad) of torsion spring
thetaLL = 10*(pi/180);          % lower limit (rad) of lower leg angles
thetaUL = 80*(pi/180);          % upper limit (rad) of lower leg angles
phiLL = 25*(pi/180);            % lower limit (rad) of upper leg angles
phiUL = 50*(pi/180);            % upper limit (rad) of upper leg angles
p_guess = [0,0,0.3];            % initial guess of end-effector position
tau_limit = 12;                 % torque limit of torque (Nm)
                                % eg: tau_limit=5 means torque range is [-5,5]
ve_limit = 10;                  % angular velocity limit of motor (rad/s)
                                % eg: ve_limit=5 means velocity range is [-5,5]
relative_force = 50;            % relative force(N) that used for comparing with force limit
relative_velocity = 1;          % relative velocity(m/s)that used for comparing with velocity limit
force_scale = 0.001;            % plot force scale of force limit and relative force
velocity_scale = 0.03;          % plot force scale of force limit and relative force
resolution = 0.005;             % resolution of workspace, in order to be efficient
                                % smaller value is denser

%% 2D workspace
figure

thetalist_home = theta_home*ones(1,3);
p = DeltaFkin(R1,R2,L1,L2,thetalist_home,p_guess);
%figure
yz_coord = DeltaWS2D_data( R1,R2,L1,L2,thetaUL,thetaLL,phiUL,phiLL,resolution );
hold on
DeltaVisualization( R1,R2,L1,L2,p,thetalist_home );
hold on

view(90,0);

%set(gcf,'Position',get(0,'ScreenSize'));

%% plot force limits at representative points

% y max
x = 0;
[y,y_position] = max(yz_coord(:,1));
z = yz_coord(y_position,2);
p_y_max = [x,y,z];
%ForceSphere( R1,R2,L1,L2,p_y_max,tau_limit,relative_force,force_scale )
hold on

% y min
[y,y_position] = min(yz_coord(:,1));
z = yz_coord(y_position,2);
p_y_min = [x,y,z];
%ForceSphere( R1,R2,L1,L2,p_y_min,tau_limit,relative_force,force_scale )
hold on

% z max
[z,z_position] = max(yz_coord(:,2));
y = yz_coord(z_position,1);
p_z_max = [x,y,z];
%ForceSphere( R1,R2,L1,L2,p_z_max,tau_limit,relative_force,force_scale )
hold on

% z min
z = 1;
for i=1:length(yz_coord)
    test = yz_coord(i,2);
    if test ~= 0 && test <= z
        z = test;
        z_position = i;
    end
end
y = yz_coord(z_position,1);
p_z_min = [x,y,z];


% ForceSphere( R1,R2,L1,L2,p_z_min,tau_limit,relative_force,force_scale )
% 
% axeslength = force_scale*relative_force;
% p_scale = p+[0,2.5*R2,0];
% v_scale = p_scale+[0,axeslength,0];
% plot3([p_scale(1),v_scale(1)],[p_scale(2),v_scale(2)],[p_scale(3),v_scale(3)],'LineWidth',1.5,'Color',[1,0,0])
% text(v_scale(1),v_scale(2),v_scale(3),[num2str(relative_force),'N'],'color',[1 0 0])
% 
% view(90,0)

%% plot velocity limits at representative points

% y max
VelocitySphere( R1,R2,L1,L2,p_y_max,ve_limit,relative_velocity,velocity_scale )
hold on

% y min
VelocitySphere( R1,R2,L1,L2,p_y_min,ve_limit,relative_velocity,velocity_scale )
hold on

% z max
VelocitySphere( R1,R2,L1,L2,p_z_max,ve_limit,relative_velocity,velocity_scale )
hold on

% z min
VelocitySphere( R1,R2,L1,L2,p_z_min,ve_limit,relative_velocity,velocity_scale )

axeslength = velocity_scale*relative_velocity;
p_scale = p+[0,2.5*R2,0];
v_scale = p_scale+[0,axeslength,0];
plot3([p_scale(1),v_scale(1)],[p_scale(2),v_scale(2)],[p_scale(3),v_scale(3)],'LineWidth',1.5,'Color',[1,0,0])
text(v_scale(1),v_scale(2),v_scale(3),[num2str(relative_velocity),'m/s'],'color',[1 0 0])

view(90,0)
