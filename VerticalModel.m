function Vz = VerticalModel(z, Vz, Altitude_d, VzMin, VzMax, VzDotMin, VzDotMax, TauVz, GainAltitudeControl)
%#codegen

ErrAlt = Altitude_d - z;
ErrAlt = ErrAlt*GainAltitudeControl;

if ErrAlt < VzMin
    ErrAlt = VzMin;
else
    if ErrAlt > VzMax
        ErrAlt = VzMax;
    end
end

Err = ErrAlt - Vz;
Err = Err/TauVz;

if Err < VzDotMin
    Err = VzDotMin;
else
    if Err > VzDotMax
        Err = VzDotMax;
    end
end

Vz = Vz + Err;

if Vz < VzMin
    Vz = VzMin;
else
    if Vz > VzMax
        Vz = VzMax;
    end
end