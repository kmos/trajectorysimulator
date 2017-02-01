function [TargetEast, TargetNorth, TargetUp] = TargetLLH2ENU(TargetNumber, TargetLat, TargetLon, TargetAlt, ELLIPSOID, OwnLat, OwnLon, OwnAlt)
%#codegen

TargetEast  = single(zeros(60,1));
TargetNorth = single(zeros(60,1));
TargetUp    = single(zeros(60,1));

for t = 1:TargetNumber
    [x, y, z]                           = geodetic2ecef(TargetLat(t,1), TargetLon(t,1), TargetAlt(t,1), ELLIPSOID);
    [TargetEast(t,1), TargetNorth(t,1)] = ecef2lv(x, y, z, OwnLat, OwnLon, OwnAlt, ELLIPSOID);
    TargetUp(t,1)                       = TargetAlt(t,1) - OwnAlt;
end