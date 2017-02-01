function [x, y, z, V, Vz, Track, TrackErr, OwnLat, OwnLon, OwnAlt, V_d, Altitude_d, Track_d, Phi, Theta, Psi, PhiDot, ThetaDot, PsiDot, Vxb, Vyb, Vzb, xdotdot, ydotdot, zdotdot, xdot, ydot, zdot, Axb, Ayb, Azb, PhiErr, ThetaErr, PsiErr, xdotdotErr, ydotdotErr, zdotdotErr, xdotErr, ydotErr, zdotErr, xErr, yErr, zErr, PhiDotErr, ThetaDotErr, PsiDotErr] = AircraftModel(FalsePos, NextTarget, t, x, y, z, xdot, ydot, zdot, V, Vz, Track, TrackErr, OwnLat, OwnLon, OwnAlt, TargetAltitude, AircraftHorSpeed, TargetNumber, TargetLat, TargetLon, TargetAlt, BarycentreLat, BarycentreLon, BarycentreAltitude, ELLIPSOID, g, Tc, Vmin, Vmax, VdotMin, VdotMax, TauV, VzMin, VzMax, VzDotMin, VzDotMax, TauVz, GainAltitudeControl, PhiMax, PhiDotMax, TauChi, StopExploration, x0, y0, Phi, Theta, Psi, PhiDot, ThetaDot, PsiDot, Vxb, Vyb, Vzb, xdotdot, ydotdot, zdotdot, Axb, Ayb, Azb, GyroErrRoll, GyroErrPitch, GyroErrYaw, AccErrXb, AccErrYb, AccErrZb, PhiErr, ThetaErr, PsiErr, xdotdotErr, ydotdotErr, zdotdotErr, xdotErr, ydotErr, zdotErr, xErr, yErr, zErr, PhiDotErr, ThetaDotErr, PsiDotErr, OwnLatErr, OwnLonErr, OwnAltErr)
%#codegen

% Calcolo posizioni relative dei target
if t > 1
    if FalsePos == 1 && NextTarget >= TargetNumber - 1
        [TargetEast, TargetNorth, TargetUp] = TargetLLH2ENU(TargetNumber, TargetLat, TargetLon, TargetAlt, ELLIPSOID, OwnLatErr(t-1,1), OwnLonErr(t-1,1), OwnAltErr(t-1,1));
    else
        [TargetEast, TargetNorth, TargetUp] = TargetLLH2ENU(TargetNumber, TargetLat, TargetLon, TargetAlt, ELLIPSOID, OwnLat(t-1), OwnLon(t-1), OwnAlt(t-1));
    end
        % Selezione della posizione relativa del target che si insegue
    [NextTargetEast, NextTargetNorth, NextTargetUp] = PositionNextTarget(NextTarget, TargetEast, TargetNorth, TargetUp, StopExploration, BarycentreLat, BarycentreLon, OwnLat(t-1), OwnLon(t-1), ELLIPSOID, x0, y0);
else
    [TargetEast, TargetNorth, TargetUp] = TargetLLH2ENU(TargetNumber, TargetLat, TargetLon, TargetAlt, ELLIPSOID, OwnLat, OwnLon, OwnAlt);   
    % Selezione della posizione relativa del target che si insegue
    [NextTargetEast, NextTargetNorth, NextTargetUp] = PositionNextTarget(NextTarget, TargetEast, TargetNorth, TargetUp, StopExploration, BarycentreLat, BarycentreLon, OwnLat, OwnLon, ELLIPSOID, x0, y0);
end

% Riferimenti da inseguire
[V_d, Altitude_d, Track_d] = EvaluateCommand(NextTarget, TargetAltitude, AircraftHorSpeed, NextTargetEast, NextTargetNorth);

if t > 1
    % Modello di evoluzione del modulo della velocità orizzontale del velivolo
    V(t) = SpeedModel(V(t-1), V_d, Vmin, Vmax, VdotMin, VdotMax, TauV);
    % (t-1) va messo anche su z
    % Modello di evoluzione della velocità verticale del velivolo
    if FalsePos == 1 && NextTarget >= TargetNumber - 1
        Vz(t) = VerticalModel(OwnAltErr(t-1), Vz(t-1), Altitude_d, VzMin, VzMax, VzDotMin, VzDotMax, TauVz, GainAltitudeControl);
    else
        Vz(t) = VerticalModel(z(t-1), Vz(t-1), Altitude_d, VzMin, VzMax, VzDotMin, VzDotMax, TauVz, GainAltitudeControl);
    end
        % Modello di evoluzione dell'angolo di track della velocità orizzontale del velivolo
    [Track(t), TrackErr(t)] = TrackModel(Track(t-1), TrackErr(t-1), V(t-1), Track_d, g, PhiMax, PhiDotMax, TauChi, Tc);
    % Conversione delle velocità da riferimento sferico a cartesiano
    [xdot(t), ydot(t), zdot(t)] = SPH2CAR(V(t), Vz(t), Track(t));
    % Modello di evoluzione delle posizioni
    [x(t), y(t), z(t)] = PositionModel(x(t-1), y(t-1), z(t-1), xdot(t), ydot(t), zdot(t), Tc);
    % Calcolo posizione LLH dell'ownship
    [OwnLat(t), OwnLon(t), OwnAlt(t)] = ENU2LLH(x(t), y(t), z(t), BarycentreLat, BarycentreLon, BarycentreAltitude, ELLIPSOID);
    % Calcolo angoli di Eulero [rad rad rad]
    [Phi(t), Theta(t), Psi(t)] = EulerEval(V(t), Vz(t), Track(t), TrackErr(t));
    % Calcolo derivate angoli di Eulero [rad/s rad/s rad/s]
    [PhiDot(t), ThetaDot(t), PsiDot(t)] = EulerDotEval(t, Tc, Phi, Theta, Psi);
    % Calcolo derivate angoli di Eulero con errore [rad/s rad/s rad/s]
    PhiDotErr(t)   = PhiDot(t) + GyroErrRoll(t);
    ThetaDotErr(t) = ThetaDot(t) + GyroErrPitch(t);
    PsiDotErr(t)   = PsiDot(t) + GyroErrYaw(t);
    
    % Calcolo velocità body [m/s m/s m/s]
    [Vxb(t), Vyb(t), Vzb(t)] = NED2Body(ydot(t),xdot(t),-zdot(t),Phi(t), Theta(t), Psi(t));
    if t > 2
        % Calcolo accelerazioni NED [m/s2 m/s2 m/s2]
        [xdotdot(t), ydotdot(t), zdotdot(t)] = EvalAccNED(t, Tc, xdot, ydot, zdot);
        % Calcolo accelerazioni body [m/s2 m/s2 m/s2]
        [Axb(t), Ayb(t), Azb(t)] = NED2Body(ydotdot(t),xdotdot(t),-zdotdot(t),Phi(t), Theta(t), Psi(t));
        % Calcolo accelerazioni body con errore [m/s2 m/s2 m/s2]
        AxbErr(t) = Axb(t) + AccErrXb(t);
        AybErr(t) = Ayb(t) + AccErrYb(t);
        AzbErr(t) = Azb(t) + AccErrZb(t);
        
        % Calcolo angoli di Eulero con errore [rad rad rad]
        PhiErr(t)   = PhiErr(t-1) + PhiDotErr(t)*Tc;
        ThetaErr(t) = ThetaErr(t-1) + ThetaDotErr(t)*Tc;
        PsiErr(t)   = PsiErr(t-1) + PsiDotErr(t)*Tc;
        if abs(PsiErr) > pi
            if PsiErr > 0
                PsiErr = PsiErr - 2*pi;
            else
                PsiErr = PsiErr + 2*pi;
            end
        end

        
        % Calcolo accelerazioni NED con errore [m/s2 m/s2 m/s2]
        [AxNEDerr(t), AyNEDerr(t), AzNEDerr(t)] = Body2NED(AxbErr(t),AybErr(t),AzbErr(t),PhiErr(t), ThetaErr(t), PsiErr(t));

        % Calcolo accelerazioni ENU con errore [m/s2 m/s2 m/s2]
        xdotdotErr(t) = AyNEDerr(t);
        ydotdotErr(t) = AxNEDerr(t);
        zdotdotErr(t) = -AzNEDerr(t);
    else
        %%XXX: commentato da giovanni
        %AxbErr = Axb(t);
        %AybErr = Ayb(t);
        %AzbErr = Azb(t);
        PhiErr(t)   = Phi(t);
        ThetaErr(t) = Theta(t);
        PsiErr(t)   = Psi(t);
        xdotdotErr(t) = xdotdot(t);
        ydotdotErr(t) = ydotdot(t);
        zdotdotErr(t) = zdotdot(t);
        xdotErr(t) = xdot(t);
        ydotErr(t) = ydot(t);
        zdotErr(t) = zdot(t);
        xErr(t) = x(t);
        yErr(t) = y(t);
        zErr(t) = z(t);
    end
else
    PhiDotErr(t)   = PhiDot(t);
    ThetaDotErr(t) = ThetaDot(t);
    PsiDotErr(t)   = PsiDot(t);
    PhiErr(t)   = Phi(t);
    ThetaErr(t) = Theta(t);
    PsiErr(t)   = Psi(t);
    xdotdotErr(t) = xdotdot(t);
    ydotdotErr(t) = ydotdot(t);
    zdotdotErr(t) = zdotdot(t);
    xdotErr(t) = xdot(t);
    ydotErr(t) = ydot(t);
    zdotErr(t) = zdot(t);
    xErr(t) = x(t);
    yErr(t) = y(t);
    zErr(t) = z(t);
end
