clear all

%% input parameters

R1 = 0.18;                      % lower radius(m)
R2 = 0.062;                     % upper radius
L1 = 0.2;                       % length of lower legs(m)
L2 = 0.31;                      % length of upper legs
p = [0,0,0.4];                  % position of upper platform
K = 10;                         % stifness of torsion spring(Nm/rad)
theta_home = pi/2;              % rest angle of torsion spring, between 0 and pi/2
                                % this angle should be same as theta_home in testWorkspace.m
displacement = 0.001;           % displacement distance
stiffness_scale = 0.0005;       % plot scale of ellipse
relative_stiffness = 500;       % relative stiffness (N/m)that used for comparasion

%% calculate delta stiffness matrix and plot out

[thetalist,S] = DeltaIkin(R1,R2,L1,L2,p);
figure
k_matrix = DeltaStiffnessEllipse( R1,R2,L1,L2,p,K,theta_home,displacement,stiffness_scale );
hold on
DeltaVisualization( R1,R2,L1,L2,p,thetalist );

axeslength = stiffness_scale*relative_stiffness;
p_scale = p+[0,2.5*R2,0];
v_scale = p_scale+[0,axeslength,0];
plot3([p_scale(1),v_scale(1)],[p_scale(2),v_scale(2)],[p_scale(3),v_scale(3)],'LineWidth',1.5,'Color',[1,0,0])
text(v_scale(1),v_scale(2),v_scale(3),[num2str(relative_stiffness),'N/m'],'color',[1 0 0])

view(90,0)



