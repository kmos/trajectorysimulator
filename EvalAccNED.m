function [xdotdot, ydotdot, zdotdot] = EvalAccNED(t, Tc, xdot, ydot, zdot)
%#codegen

xdotdot = (xdot(t) - xdot(t-1))/Tc;
ydotdot = (ydot(t) - ydot(t-1))/Tc;
zdotdot = (zdot(t) - zdot(t-1))/Tc;
