function  DeltaForcePlot( R1,R2,L1,L2,p,torque,scale )
%{
R1 = 0.17;                         % lower radius(m)
R2 = 0.07;                       % upper radius
L1 = 0.14;                         % length of lower legs(m)
L2 = 0.24;                         % length of upper legs
p = [0,0,0.3];
torque = 1;
scale = 1;
DeltaForcePlot( R1,R2,L1,L2,p,torque,scale );
%}

for fy = -10:0.025:10
   for fz = -25:0.025:25
       torquelist = DeltaJacobian( R1,R2,L1,L2,p )'*[0,fy,fz]';
       torquelist = torquelist';
       if torquelist(1) >= -torque && torquelist(1) <= torque &&...
           torquelist(2) >= -torque && torquelist(2) <= torque &&...
           torquelist(3) >= -torque && torquelist(3) <= torque 
           
                plot3(0+p(1),(fy/scale)+p(2),(fz/scale)+p(3),'.','color',[0 1 0])
                hold on
       
       end
   end
end



end

