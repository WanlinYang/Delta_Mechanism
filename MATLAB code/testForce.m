clear all

%% input parameters

R1 = 0.18;                       % lower radius(m)
R2 = 0.062;                       % upper radius
L1 = 0.2;                       % length of lower legs(m)
L2 = 0.31;                       % length of upper legs
p = [0,0,0.4];                 % position of upper platform
K = 10;                          % stifness of torsion spring(Nm/rad)
force_scale = 0.01;             % plot scale of force
restangle = pi/12;               % rest angle of torsion spring, between 0 and pi/2
                                % this angle should be same as theta_home in testWorkspace.m

%% plot 3D force vector 
% forcelist is the output force vector of upper platform

forcelist = DeltaForce( R1,R2,L1,L2,p,K,restangle );    % force vector

% plot out the 3D force vector ( the red line without arrow) 
thetalist = DeltaIkin(R1,R2,L1,L2,p);
figure
DeltaVisualization(R1,R2,L1,L2,p,thetalist);
hold on
f_vector = p+forcelist*force_scale;
plot3([p(1),f_vector(1)],[p(2),f_vector(2)],[p(3),f_vector(3)],'LineWidth',1.5,'Color',[1,0,0])
text(f_vector(1),f_vector(2),f_vector(3),['(',num2str(forcelist(1)),',',...
    num2str(forcelist(2)),',',num2str(forcelist(3)),')'],'color',[1 0 0])



