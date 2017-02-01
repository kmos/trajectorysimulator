function V = SpeedModel(V, V_d, Vmin, Vmax, VdotMin, VdotMax, TauV)
%#codegen

if V_d < Vmin
    V_d = Vmin;
else
    if V_d > Vmax
        V_d = Vmax;
    end
end

Err = V_d - V;
Err = Err/TauV;

if Err < VdotMin
    Err = VdotMin;
else
    if Err > VdotMax
        Err = VdotMax;
    end
end

V = V + Err;

if V < Vmin
    V = Vmin;
else
    if V > Vmax
        V = Vmax;
    end
end