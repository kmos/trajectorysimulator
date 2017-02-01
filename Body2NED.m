function [xned, yned, zned] = Body2NED(xb, yb, zb, Phi, Theta, Psi)
% %#codegen
% clc
% xb = 0;
% yb = 1;
% zb = 0;
% 
% Phi = 0/180*pi;
% Theta = 0/180*pi;
% Psi = 45/180*pi;

cPsi = cos(Psi);
sPsi = sin(Psi);
MPsi = [ cPsi   -sPsi   0;
         sPsi    cPsi   0;
         0       0      1];

cTheta = cos(Theta);
sTheta = sin(Theta);   
MTheta = [ cTheta  0   sTheta;
           0       1   0;
          -sTheta  0   cTheta];

cPhi = cos(Phi);
sPhi = sin(Phi);
MPhi = [ 1    0     0;
         0    cPhi -sPhi;
         0    sPhi  cPhi];
     
M = MPsi*MTheta*MPhi;
     
a = M*[xb; yb; zb];

xned = a(1);
yned = a(2);
zned = a(3);