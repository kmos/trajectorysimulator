function [xb, yb, zb] = NED2Body(xned, yned, zned, Phi, Theta, Psi)
% %#codegen
% clc
% xned = 0;
% yned = 1;
% zned = 0;
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
     
a = M'*[xned; yned; zned];

xb = a(1);
yb = a(2);
zb = a(3);