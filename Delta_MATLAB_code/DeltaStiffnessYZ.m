function k_matrix = DeltaStiffnessYZ( R1,R2,L1,L2,p,K,restangle,displacement )
%input parameters and small displacemet, output 2x2(lower right corner) stiffness matrix
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
k_m = DeltaStiffnessYZ(R1,R2,L1,L2,p,K,restangle,displacement);

%}

p_xp = p + [displacement,0,0];
p_yp = p + [0,displacement,0];
p_zp = p + [0,0,displacement];

f = DeltaForce( R1,R2,L1,L2,p,K,restangle );
f_xp = DeltaForce( R1,R2,L1,L2,p_xp,K,restangle );    % force plus in x direction
f_yp = DeltaForce( R1,R2,L1,L2,p_yp,K,restangle );
f_zp = DeltaForce( R1,R2,L1,L2,p_zp,K,restangle );
df_x = f_xp - f;
df_y = f_yp - f;
df_z = f_zp - f;
df = [df_x',df_y',df_z'];

k_matrix3 = zeros(3,3);
for i=1:3
    k_matrix3(:,i) = df(:,i)/displacement;
end
k_matrix = k_matrix3(2:3,2:3);

end

