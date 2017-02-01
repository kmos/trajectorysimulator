function [Phi, Theta, Psi] = EulerEval(Vo, Vz, Track, TrackDot)
%#codegen

V = sqrt(Vo^2 + Vz^2);
Phi = -V*TrackDot/9.81;
if abs(Phi*180/pi) < 1
    Phi = 0;
end
Theta = atan2(Vz,Vo);
Psi = -Track + pi/2;
if abs(Psi) > pi
    if Psi > 0
        Psi = Psi - 2*pi;
    else
        Psi = Psi + 2*pi;
    end
end