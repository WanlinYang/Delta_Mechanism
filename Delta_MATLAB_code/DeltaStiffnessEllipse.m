function k_matrix = DeltaStiffnessEllipse( R1,R2,L1,L2,p,K,restangle,displacement,scale )
%input parameters and small displacemet, output stiffness matrix
%   example input
%{

R1 = 0.5;                       % lower radius(m)
R2 = 0.3;                       % upper radius
L1 = 0.5;                       % length of lower legs(m)
L2 = 0.5;                       % length of upper legs
p = [0,0,0.7];                 % position of upper platform
K = 1;                          % stifness of torsion spring(Nm/rad)
restangle = pi/6;               % rest angle of torsion spring, between 0 and pi/2
                                % this angle should be same as theta_home in testWorkspace.m
displacement = 0.001;            % perturbation distance
scale = 0.1                     % plot scale of ellipse
k_m = DeltaStiffnessEllipse(R1,R2,L1,L2,p,K,restangle,displacement,scale);

%}

k_matrix = DeltaStiffnessYZ( R1,R2,L1,L2,p,K,restangle,displacement );

[V,D] = eig(k_matrix); 
if V(1,1)==0
    theta = pi/2;
else
    theta = atan(V(2,1)/V(1,1));
end
i= 0:pi/50:2*pi;
ytemp = D(1,1)*cos(i);
ztemp = D(2,2)*sin(i);
y = (ytemp*cos(theta)-ztemp*sin(theta))*scale+p(2);
z = (ytemp*sin(theta)+ztemp*cos(theta))*scale+p(3);
x = zeros(size(y))+p(1);

plot3(x,y,z)
axis equal

end

