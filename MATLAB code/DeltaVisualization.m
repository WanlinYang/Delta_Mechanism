function DeltaVisualization( R1,R2,L1,L2,p,thetalist )
% given all the position variables, output plot of the whole platform in 3D-Space.
%   Example
%{
R1 = 1;
R2 = 0.5;
L1 = 1;
L2 = 1;
p = [0,0,1.5];
[thetalist,S] = DeltaIkin(R1,R2,L1,L2,p);
 DeltaVisualization(R1,R2,L1,L2,p,thetalist);
%}

blist = [[R1,0,0];[R1*cosd(120),R1*sind(120),0];[R1*cosd(240),R1*sind(240),0]];
plist = [[R2,0,0];[R2*cosd(120),R2*sind(120),0];[R2*cosd(240),R2*sind(240),0]];

% plots
% plot base platform
t = 0:pi/50:2*pi;
xt = R1*cos(t);
yt = R1*sin(t);
zt = zeros(length(t));
plot3(xt,yt,zt,'LineWidth',2.5,'Color',[1,0,0])
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on
hold on
% plot lower legs
for i=1:3
    plot3([blist(i,1),blist(i,1)+L1*cosd(i*120-120)*sin(thetalist(i))],...
    [blist(i,2),blist(i,2)+L1*sind(i*120-120)*sin(thetalist(i))],...
    [blist(i,3),blist(i,3)+L1*cos(thetalist(i))],'LineWidth',2.3,'Color',[0,0,1]);
    hold on
end
% plot upper legs
for i=1:3
    plot3([blist(i,1)+L1*cosd(i*120-120)*sin(thetalist(i)),plist(i,1)+p(1)],...
    [blist(i,2)+L1*sind(i*120-120)*sin(thetalist(i)),plist(i,2)+p(2)],...
    [blist(i,3)+L1*cos(thetalist(i)),p(3)],'LineWidth',2.3,'Color',[0,0.8,0.5]);
    hold on
end
% plot top platform
t = 0:pi/50:2*pi;
xtt = R2*cos(t)+p(1);
ytt = R2*sin(t)+p(2);
ztt = zeros(length(t))+p(3);
plot3(xtt,ytt,ztt,'LineWidth',2.5,'Color',[0,0,0])
hold on
% plot two points
plot3(0,0,0,'.')
hold on 
text(0,0,0,['(',num2str(0),',',num2str(0),',',num2str(0),')'],'color',[1 0 0])
plot3(p(1),p(2),p(3),'.')
hold on
%text(p(1),p(2),p(3),['(',num2str(p(1),2),',',num2str(p(2),2),',',num2str(p(3),2),')'],'color',[1 0 0])
axis equal


end

