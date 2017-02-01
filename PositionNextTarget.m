function [NextTargetEast, NextTargetNorth, NextTargetUp] = PositionNextTarget(NextTarget, TargetEast, TargetNorth, TargetUp, StopExploration, BarycentreLat, BarycentreLon, OwnLat, OwnLon, ELLIPSOID, x0, y0)
%#codegen

NextTargetEast  = TargetEast(NextTarget,1);
NextTargetNorth = TargetNorth(NextTarget,1);
NextTargetUp    = TargetUp(NextTarget,1);