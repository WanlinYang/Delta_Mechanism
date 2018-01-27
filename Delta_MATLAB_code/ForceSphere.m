function  ForceSphere( R1,R2,L1,L2,p,tau_limit,relative_force,scale )
%Given the position of Delta robot, output the 3D plot of force limit
%   Example
%{
R1 = 1;
R2 = 0.5;
L1 = 1;
L2 = 1;
p = [0,0.3,1.3];
tau_limit = 5;
relative_force = 50;
scale = 0.3;
ForceSphere( R1,R2,L1,L2,p,tau_limit,relative_force,scale )
%}

% construct unit sphere
r = tau_limit*scale;
nphi = 30;
ntheta = 40;
phi = linspace(0,pi,nphi);
theta = linspace(0,2*pi,ntheta);
[phi,theta] = meshgrid(phi,theta);
x = r*sin(phi).*cos(theta);
y = r*sin(phi).*sin(theta);
z = r*cos(phi); 

% Jacobian
J = DeltaJacobian( R1,R2,L1,L2,p );
xj = x;
yj = y;
zj = z;
for i=1:ntheta
    for j=1:nphi
        xyzj = inv(J')*[x(i,j);y(i,j);z(i,j)];
        xj(i,j) = xyzj(1);
        yj(i,j) = xyzj(2);
        zj(i,j) = xyzj(3);
    end
end
xj = p(1)*ones(ntheta,nphi)+xj;
yj = p(2)*ones(ntheta,nphi)+yj;
zj = p(3)*ones(ntheta,nphi)+zj;

mesh(xj,yj,zj,'edgecolor','c')
alpha(0.1)
hold on

% plot unit force
axeslength = scale*relative_force;
vx = p+[axeslength,0,0];
vy = p+[0,axeslength,0];
vz = p+[0,0,axeslength];
plot3([p(1),vx(1)],[p(2),vx(2)],[p(3),vx(3)],'LineWidth',1.5,'Color',[1,0,0])
plot3([p(1),vy(1)],[p(2),vy(2)],[p(3),vy(3)],'LineWidth',1.5,'Color',[0,1,0])
plot3([p(1),vz(1)],[p(2),vz(2)],[p(3),vz(3)],'LineWidth',1.5,'Color',[0,0,1])

axis equal

end

