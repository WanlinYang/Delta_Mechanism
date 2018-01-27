function DeltaWS3D(R1,R2,L1,L2,thetaUL,thetaLL,phiUL,phiLL,res )
% Plot the workspace of Delta mechanism in 2D(x = 0)
% Taking the six parameters 
% R1 = radius of the base
% R2 = radius of the top platform 
% L1 = length of the lower leg 
% L2 = length of the upper leg 
% thetaUL = upper limit of joint angles
% thetaLL = lower limit of joint angles
% Returns a number F, 1 = plot finished
%   Example input:
%{ 
R1 = 1;
R2 = 0.5;
L1 = 1;
L2 = 1;
thetaUL = pi/3;
thetaLL = pi/12;
phiUL = pi/3;
phiLL = pi/4;
res = 0.01
DeltaWS2D(R1,R2,L1,L2,thetaUL,thetaLL,phiUL,phiLL,res )
%}

blist = [[R1,0,0];[R1*cosd(120),R1*sind(120),0];[R1*cosd(240),R1*sind(240),0]];
plist = [[R2,0,0];[R2*cosd(120),R2*sind(120),0];[R2*cosd(240),R2*sind(240),0]];



plot(0,0,'.')
hold on 
text(0,0,['(',num2str(0),',',num2str(0),')'],'color',[1 0 0])
xlabel('Y');
ylabel('Z');
grid on
axis on
    for x = -R1-L1-L2:res:R1+L1+L2
    for y = -R1-L1-L2:res:R1+L1+L2
        for z = 0:res:R2+L1+L2
            p = [x,y,z];
            [thetalist,S] = DeltaIkin(R1,R2,L1,L2,p);
            if thetaLL <= thetalist(1) && thetalist(1)<= thetaUL ...
                    && thetaLL <= thetalist(2)&& thetalist(2)<= thetaUL...
                    &&thetaLL <= thetalist(3) &&thetalist(3)<= thetaUL && S==1
                philist = [-1,-1,-1];
                Llist = [[L1*sin(thetalist(1)),0,L1*cos(thetalist(1))];
                    [L1*cosd(120)*sin(thetalist(2)),L1*sind(120)*sin(thetalist(2)),L1*cos(thetalist(2))]; 
                    [L1*cosd(240)*sin(thetalist(3)),L1*sind(240)*sin(thetalist(3)),L1*cos(thetalist(3))]];
                L2list = plist-blist-Llist+[p;p;p];
                if L2list(1,3) > 0 && L2list(2,3) > 0 && L2list(3,3) > 0 && L2list(1,1) <= 0 && ...
                        L2list(2,1) >= 0 && L2list(2,1) >= 0 && L2list(2,2) <= 0 && L2list(3,1) >= 0 && L2list(3,2) >= 0
                    for i = 1:3
                        philist(i) = atan(sqrt(L2list(i,1)^2+L2list(i,2)^2)/L2list(i,3));
                    end
                end
                if philist(1) <= phiUL && philist(2) <= phiUL && ...
                        philist(3) <= phiUL && philist(1) >= phiLL &&...
                        philist(2) >= phiLL && philist(3) >= phiLL
                    plot3(x,y,z,'.','color',[1 0 0])
                    hold on
                end
            end
        end
    end
    end
end

