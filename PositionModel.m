function [x, y, z] = PositionModel(x, y, z, xdot, ydot, zdot, Tc)
%#codegen

x = x + Tc*xdot;
y = y + Tc*ydot;
z = z + Tc*zdot;