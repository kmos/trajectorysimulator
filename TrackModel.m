function [Track, TrackErr] = TrackModel(Track, TrackErr, V, Track_d, g, PhiMax, PhiDotMax, TauChi, Tc)
%#codegen

chi_dot_dot_max = abs( g*(PhiDotMax/180*pi) / ( max(V,0.1)*(cos(PhiMax/180*pi)^2)) );
chi_dot_dot_min = -chi_dot_dot_max;
chi_dot_max = abs( g*tan(PhiMax/180*pi) / max(V,0.1) );
chi_dot_min = -chi_dot_max;

Err = Calc_rif(Track_d, Track*180/pi);
Err = Err/180*pi;
Err = Err/TauChi;

if Err < chi_dot_min
    Err = chi_dot_min;
else
    if Err > chi_dot_max
        Err = chi_dot_max;
    end
end

if (Err - TrackErr)/Tc < chi_dot_dot_min
    Err = TrackErr + Tc*chi_dot_dot_min;
else
    if (Err - TrackErr)/Tc > chi_dot_dot_max
        Err = TrackErr + Tc*chi_dot_dot_max;
    end
end

TrackErr = Err;
Track = Track + Err;

% Si porta il track tra +-2pi, sottraendo gli interi di 2pi
Div = Track/(2*pi);
Mol = sign(Div)*floor(abs(Div))*(2*pi);
Track = Track - Mol;

% Si porta il track tra +-pi
if abs(Track) > pi
    if Track > 0
        Track = Track - 2*pi;
    else
        Track = Track + 2*pi;
    end
end


