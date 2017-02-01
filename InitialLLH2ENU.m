function [x0, y0, z0] = InitialLLH2ENU(BarycentreLat, BarycentreLon, BarycentreAltitude, TakeoffAircraftAlt, TakeoffAircraftLat, TakeoffAircraftLon, ELLIPSOID)
%#codegen

[x, y, z]    = RTgeodetic2ecef(TakeoffAircraftLat, TakeoffAircraftLon, TakeoffAircraftAlt, ELLIPSOID);
[x0, y0, z0] = ecef2lv(x, y, z, BarycentreLat/180*pi, BarycentreLon/180*pi, BarycentreAltitude, ELLIPSOID);
z0           = 0;