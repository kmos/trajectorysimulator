function [xdot, ydot, zdot] = SPH2CAR(V, Vz, Track)
%#codegen

xdot  = V*cos(Track);
ydot  = V*sin(Track);
zdot  = Vz;