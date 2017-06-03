clear all

%% input parameters

R1 = 0.2;                         % lower radius(m)
R2 = 0.1;                       % upper radius
L1 = 0.2;                         % length of lower legs(m)
L2 = 0.3;                         % length of upper legs
p = [0,0,0.3];                  % position of upper platform

%% inverse kinematics and visualization

[thetalist,S] = DeltaIkin( R1,R2,L1,L2,p );   % thetalist is the output angles of R joint
if S==0
      msg = 'Invalid inputs.';    % this configuration cannot be reached
      error(msg);
end
DeltaVisualization( R1,R2,L1,L2,p,thetalist );