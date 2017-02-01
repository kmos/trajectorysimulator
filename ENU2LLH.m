function [OwnLat, OwnLon, OwnAlt]= ENU2LLH(x, y, z, BarycentreLat, BarycentreLon, BarycentreAltitude, ELLIPSOID)
%#codegen

[x_ecef, y_ecef, z_ecef]                   = lv2ecef(x, y ,z, BarycentreLat/180*pi, BarycentreLon/180*pi, BarycentreAltitude, ELLIPSOID);
[OwnLat, OwnLon, OwnAlt]                   = ecef2geodetic(x_ecef, y_ecef, z_ecef, ELLIPSOID);
OwnAlt                                     = BarycentreAltitude + z;