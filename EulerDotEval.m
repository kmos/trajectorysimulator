function [PhiDot, ThetaDot, PsiDot] = EulerDotEval(t, Tc, Phi, Theta, Psi)
%#codegen

PhiDot   = (Phi(t) - Phi(t-1))/Tc;
ThetaDot = (Theta(t) - Theta(t-1))/Tc;
DeltaPsi = Psi(t) - Psi(t-1);
if abs(DeltaPsi) > pi
    if DeltaPsi > 0
        DeltaPsi = DeltaPsi - 2*pi;
    else
        DeltaPsi = DeltaPsi + 2*pi;
    end
end
PsiDot = DeltaPsi/Tc;
if abs(PsiDot*180/pi) < 5
    PsiDot = 0;
end