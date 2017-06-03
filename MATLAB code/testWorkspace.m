clear all
% testWorkspace

%% input parameters

R1 = 0.18;                      % lower radius(m)
R2 = 0.062;                     % upper radius
L1 = 0.2;                       % length of lower legs(m)
L2 = 0.31;                      % length of upper legs
theta_home = pi/6;              % home angle (rad) of torsion spring
thetaUL = 80*(pi/180);          % upper limit (rad) of spring(-pi/4 to pi/2)
thetaLL = 10*(pi/180);          % lower limit (rad) of spring(-pi/4 to pi/2)
phiLL = 25*(pi/180);            % lower limit (rad) of upper leg angles
phiUL = 50*(pi/180);            % upper limit (rad) of upper leg angles
p_guess = [0,0,0.3];            % initial guess of end-effector position
res2d = 0.005;                  % resolution of 2d plot, less value with denser dots
res3d = 0.015;                   % resolution of 3d plot, less value with denser dots

%%
thetalist_home = theta_home*ones(1,3);
p = DeltaFkin(R1,R2,L1,L2,thetalist_home,p_guess);

%% plot 3D workspace

% figure
% DeltaWS3D( R1,R2,L1,L2,thetaUL,thetaLL,phiUL,phiLL,res3d );
% hold on
% DeltaVisualization( R1,R2,L1,L2,p,thetalist_home );
% view(3)

%% plot 2D workspace

figure
DeltaWS2D( R1,R2,L1,L2,thetaUL,thetaLL,phiUL,phiLL,res2d );
hold on
DeltaVisualization( R1,R2,L1,L2,p,thetalist_home );
view(90,0)


