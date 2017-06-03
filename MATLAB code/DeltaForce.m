function  forcelist = DeltaForce( R1,R2,L1,L2,p,K,restangle )
%Given the position of Delta robot, output the the end-effect force in
%x,y,z directions 
%   Example
%{
R1 = 1;
R2 = 0.5;
L1 = 1;
L2 = 1;
p = [0,0.3,1.3];
K = 1;
restangle = pi/6;
forcelist = DeltaForce( R1,R2,L1,L2,p,K,restangle )
%}

[thetalist,S] = DeltaIkin(R1,R2,L1,L2,p);

if S==0
      msg = 'Invalid inputs.';
      error(msg);
end

taulist = -K*(thetalist-[restangle,restangle,restangle]);

forcelist = inv(DeltaJacobian( R1,R2,L1,L2,p)')*taulist';
forcelist = forcelist';

end

