function [V_d, Altitude_d, Track_d] = EvaluateCommand(NextTarget, TargetAltitude, SelectAircraftHorSpeed, NextTargetEast, NextTargetNorth)
%#codegen

V_d        = SelectAircraftHorSpeed;
Altitude_d = TargetAltitude(NextTarget);
Track_d    = atan2(NextTargetNorth, NextTargetEast)*180/pi;