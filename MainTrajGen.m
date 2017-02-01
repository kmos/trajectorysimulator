close all
clear all
%clc

%%Simulation Start
disp(['Simulation is starting...']);

ScenarioParameters
%% Definizione di alcuni parametri.

NextTarget = 1;
NextTargetg = 1;
StopExploration = 0;
AircraftNumber = 1;
Init

%% Inizializzazione.

[GlobalTime, TrackErr, TrackErrg] = InitFunction;

%% LLH del punto di decollo degli aircraft
[TakeoffAircraftEcefX, TakeoffAircraftEcefY, TakeoffAircraftEcefZ] = lv2ecef(XTakeoffAircraft, YTakeoffAircraft, 0, BarycentreLat/180*pi, BarycentreLon/180*pi, BarycentreAltitude, ELLIPSOID);
[TakeoffAircraftLat, TakeoffAircraftLon]       = ecef2geodetic(TakeoffAircraftEcefX, TakeoffAircraftEcefY, TakeoffAircraftEcefZ, ELLIPSOID);
TakeoffAircraftAlt = single(AircraftAltitude);

%% Elaborazioni iniziali.

%% [[m], [m], [m]] Calcolo posizioni relative di partenza dei velivoli rispetto alla
% posizione baricetrica assoluta definita.
[x0, y0, z0] = InitialLLH2ENU(BarycentreLat, BarycentreLon, BarycentreAltitude, TakeoffAircraftAlt, TakeoffAircraftLat, TakeoffAircraftLon, ELLIPSOID);
z0 = TargetAltitude(1);
%% Posizioni iniziali di tutti i velivoli rispetto alla posizione
% baricetrica assoluta definita.
x = x0; % [m]
y = y0; % [m]
z = z0; % [m]
xg = x0; % [m]
yg = y0; % [m]
zg = z0; % [m]

%% Velocità iniziali di tutti i velivoli in coordinate sferiche.
V       = AircraftHorSpeed;     % [m/s]
Vz      = Vz0;                  % [m/s]
Track   = Chi0/180*pi;          % [rad]
Vg      = AircraftHorSpeed;     % [m/s]
Vzg     = Vz0;                  % [m/s]
Trackg  = Chi0/180*pi;          % [rad]

%% [[rad], [rad], [m]] Posizioni iniziali in coordinate LLH.
[OwnLat, OwnLon, OwnAlt] = ENU2LLH(x, y, z, BarycentreLat, BarycentreLon, BarycentreAltitude, ELLIPSOID);
[OwnLatg, OwnLong, OwnAltg] = ENU2LLH(xg, yg, zg, BarycentreLat, BarycentreLon, BarycentreAltitude, ELLIPSOID);
OwnLatErr = OwnLat;
OwnLongErr = OwnLon;
OwnAltErr = OwnAlt;

%% [[m/s], [m/s], [m/s]] Velocità iniziali di tutti i velivoli in coordinate cartesiane.
[xdot, ydot, zdot] = SPH2CAR(V, Vz, Track);
[xdotg, ydotg, zdotg] = SPH2CAR(Vg, Vzg, Trackg);


%% attacco var
attack(t) = 0;

%% safe var
safe = 0;

%% Ciclo principale di simulazione sul numero di step assegnato.
t = 1;

%% velocità GPS in ENU con errore [m/s m/s m/s]
GPSxdotErr(1) = AircraftHorSpeed;
GPSydotErr(1) = 0;
GPSzdotErr(1) = 0;
%% posizione GPS in ENU con errore [m m m]
GPSxErr(1) = XTakeoffAircraft;
GPSyErr(1) = YTakeoffAircraft;
GPSzErr(1) = OwnAlt(1);

ContStepUpdateGPS = 1;
ContStepGPSoutput = 1;
%% velocità iniziale ENU con errore [m/s m/s m/s]
xdotErr(1)  = AircraftHorSpeed;
ydotErr(1)  = 0;
zdotErr(1)  = 0;
%% posizione iniziali ENU con errore [m m m]
xErr(1)     = XTakeoffAircraft;
yErr(1)     = YTakeoffAircraft;
zErr(1)     = OwnAlt(1);

%% distanza euclidea errori
E(1)    = 0;
Ev(1)   = 0;
ehoriz(1) = 0;
ez(1)   = 0;
evhoriz(1) = 0;
evz(1)  = 0;

while ((t*Tc) < MaxSimTime)
    
    time(t,:)=t*Tc;
    t = t + 1; % t [], indice del ciclo principale.
    if mod(t*Tc, 10) < 0.001
        disp(['Simulation time = ' num2str(t*Tc) 's']) % [s] Tempo di simulazione visualizzato a schermo.
    end

    
    %% Aggiornamento del GlobalTime uguale per tutti i target.
    GlobalTime = TimeUpdate(GlobalTime,Tc); % [s]
       
 

%% [[m], [m], [m], [m/s], [m/s], [rad], [rad], [rad], [rad], [m]] Aircraft
% Model, modello di evoluzione di posizione e velocità dei velivoli, con
% relativo autopilota (modello da non implementare a bordo).
    [x, y, z, V, Vz, Track, TrackErr, OwnLat, OwnLon, OwnAlt, V_d(t), Altitude_d(t), Track_d(t), Phi, Theta, Psi, PhiDot, ThetaDot, PsiDot, Vxb, Vyb, Vzb, xdotdot, ydotdot, zdotdot, xdot, ydot, zdot, Axb, Ayb, Azb, PhiErr, ThetaErr, PsiErr, xdotdotErr, ydotdotErr, zdotdotErr, xdotErr, ydotErr, zdotErr, xErr, yErr, zErr, PhiDotErr, ThetaDotErr, PsiDotErr] = AircraftModel(1, NextTarget, t, x, y, z, xdot, ydot, zdot, V, Vz, Track, TrackErr, OwnLat, OwnLon, OwnAlt, TargetAltitude, AircraftHorSpeed, TargetNumber, TargetLat, TargetLon, TargetAlt, BarycentreLat, BarycentreLon, BarycentreAltitude, ELLIPSOID, g, Tc, Vmin, Vmax, VdotMin, VdotMax, TauV, VzMin, VzMax, VzDotMin, VzDotMax, TauVz, GainAltitudeControl, PhiMax, PhiDotMax, TauChi, StopExploration, x0, y0, Phi, Theta, Psi, PhiDot, ThetaDot, PsiDot, Vxb, Vyb, Vzb, xdotdot, ydotdot, zdotdot, Axb, Ayb, Azb, GyroErrRoll, GyroErrPitch, GyroErrYaw, AccErrXb, AccErrYb, AccErrZb, PhiErr, ThetaErr, PsiErr, xdotdotErr, ydotdotErr, zdotdotErr, xdotErr, ydotErr, zdotErr, xErr, yErr, zErr, PhiDotErr, ThetaDotErr, PsiDotErr, OwnLatErr, OwnLongErr, OwnAltErr);
    [xg, yg, zg, Vg, Vzg, Trackg, TrackErrg, OwnLatg, OwnLong, OwnAltg, V_dg(t), Altitude_dg(t), Track_dg(t), Phig, Thetag, Psig, PhiDotg, ThetaDotg, PsiDotg, Vxbg, Vybg, Vzbg, xdotdotg, ydotdotg, zdotdotg, xdotg, ydotg, zdotg, Axbg, Aybg, Azbg, PhiErrg, ThetaErrg, PsiErrg, xdotdotErrg, ydotdotErrg, zdotdotErrg, xdotErrg, ydotErrg, zdotErrg, xErrg, yErrg, zErrg, PhiDotErrg, ThetaDotErrg, PsiDotErrg] = AircraftModel(0, NextTargetg, t, xg, yg, zg, xdotg, ydotg, zdotg, Vg, Vzg, Trackg, TrackErrg, OwnLatg, OwnLong, OwnAltg, TargetAltitudeg, AircraftHorSpeed, TargetNumber, TargetLatg, TargetLong, TargetAltg, BarycentreLat, BarycentreLon, BarycentreAltitude, ELLIPSOID, g, Tc, Vmin, Vmax, VdotMin, VdotMax, TauV, VzMin, VzMax, VzDotMin, VzDotMax, TauVz, GainAltitudeControl, PhiMax, PhiDotMax, TauChi, StopExploration, x0, y0, Phi, Thetag, Psig, PhiDotg, ThetaDotg, PsiDotg, Vxbg, Vybg, Vzbg, xdotdotg, ydotdotg, zdotdotg, Axbg, Aybg, Azbg, GyroErrRoll, GyroErrPitch, GyroErrYaw, AccErrXb, AccErrYb, AccErrZb, PhiErrg, ThetaErrg, PsiErrg, xdotdotErrg, ydotdotErrg, zdotdotErrg, xdotErrg, ydotErrg, zdotErrg, xErrg, yErrg, zErrg, PhiDotErrg, ThetaDotErrg, PsiDotErrg, OwnLatErr, OwnLongErr, OwnAltErr);
    
%% Onboard computer software, parte del SW che dovrà essere implementata a
% bordo.

%% [[m], [m], [m]] Distanze relative dei target dalla posizione attuale del
% velivolo ownship.
    [TargetEast, TargetNorth, TargetUp] = TargetLLH2ENU(TargetNumber, TargetLat, TargetLon, TargetAlt, ELLIPSOID, OwnLat(t), OwnLon(t), OwnAlt(t));
    [TargetEastg, TargetNorthg, TargetUpg] = TargetLLH2ENU(TargetNumber, TargetLatg, TargetLong, TargetAltg, ELLIPSOID, OwnLatg(t), OwnLong(t), OwnAltg(t));

    %% [[s], [], [], [], [], [], [], []] Funzione di cattura del target
% committed, con aggiornamento delle matrici di negoziazione e del relativo
% vettori di riempimento. TimeTargetCoverage in uscita è il tempo al
% passaggio sul target committed.
    [NextTarget]      = CaptureTargetEval(NextTarget, TargetNumber, TargetEast, TargetNorth, TargetUp, CaptureTargetThreshold);
    [NextTargetg]     = CaptureTargetEval(NextTargetg, TargetNumber, TargetEastg, TargetNorthg, TargetUpg, CaptureTargetThreshold);
    NextTargetVet(t)  = NextTarget;
    NextTargetVetg(t) = NextTargetg;
    

    %fine zona sicura
    if NextTarget == 2
        if safe == 0;
            fprintf ('Safe zone is ended from the Waypoint %s, at time %s\r', num2str(NextTarget), num2str(t*Tc));
            timeSafe = t*Tc;
        end
        safe = 1;
    end
    
    %% Attacco
    if NextTarget == 3
        if attack == 0;
        fprintf ('Under Attack from the Waypoint %s, at time %s\r', num2str(NextTarget), num2str(t*Tc));
        timeAttack = t*Tc;
        end
        attack(t,:) = 1;
    end
    
    %%         
   % if t > 1
        if ContStepUpdateGPS == StepUpdateGPS
            %%la frequenza della condizione è pari a quella della finestra
            %%di tolleranza 
            %% Calcolo velocità ENU con errore [m/s m/s m/s]
            xdotErr(t) = xdotg(t) + GPSerrVx(t);
            ydotErr(t) = ydotg(t) + GPSerrVy(t);
            zdotErr(t) = zdotg(t) + GPSerrVz(t);
            %% Calcolo posizioni ENU con errore [m m m]
            xErr(t) = xg(t) + GPSerrX(t);
            yErr(t) = yg(t) + GPSerrY(t);
            zErr(t) = zg(t) + GPSerrZ(t);
            ContStepUpdateGPS = 1;
            %ContStepGPSoutput = 0;
            E(t) = E(t-1);
            Ev(t) = Ev(t-1);
            
            ehoriz(t,:) = ehoriz(t-1,:);
            ez(t,:) = ez(t-1,:);
                
            evhoriz(t,:) = evhoriz(t-1,:);
            evz(t,:) = evz(t-1,:);
            
        else
            ContStepUpdateGPS = ContStepUpdateGPS + 1;
            %% Calcolo velocità GPS in ENU con errore [m/s m/s m/s]
            GPSxdotErr(t) = xdotg(t) + GPSerrVx(t);
            GPSydotErr(t) = ydotg(t) + GPSerrVy(t);
            GPSzdotErr(t) = zdotg(t) + GPSerrVz(t);
            %% Calcolo posizioni GPS in ENU con errore [m m m]
            GPSxErr(t) = xg(t) + GPSerrX(t);
            GPSyErr(t) = yg(t) + GPSerrY(t);
            GPSzErr(t) = zg(t) + GPSerrZ(t);
            %% Calcolo velocità ENU con errore [m/s m/s m/s]
            xdotErr(t) = xdotErr(t-1) + xdotdotErr(t)*Tc;
            ydotErr(t) = ydotErr(t-1) + ydotdotErr(t)*Tc;
            zdotErr(t) = zdotErr(t-1) + zdotdotErr(t)*Tc;
            %% Calcolo posizioni ENU con errore [m m m]
            xErr(t) = xErr(t-1) + xdotErr(t)*Tc;
            yErr(t) = yErr(t-1) + ydotErr(t)*Tc;
            zErr(t) = zErr(t-1) + zdotErr(t)*Tc;
            
            
            if ContStepGPSoutput == StepGPSoutput
                %la frequenza della condizione è pari a quella del gps
                ContStepGPSoutput = 1;
                E(t) = sqrt((GPSxErr(t) - xErr(t))^2 + (GPSyErr(t) - yErr(t))^2 + (GPSzErr(t) - zErr(t))^2); % [m]
                Ev(t) = sqrt((GPSxdotErr(t) - xdotErr(t))^2 + (GPSydotErr(t) - ydotErr(t))^2 + (GPSzdotErr(t) - zdotErr(t))^2); % [m/s]
                
                ehoriz(t,:) = sqrt((GPSxErr(t) - xErr(t))^2 + (GPSyErr(t) - yErr(t))^2);
                ez(t,:) = sqrt((GPSzErr(t) - zErr(t))^2); 
                evhoriz(t,:) = sqrt((GPSxdotErr(t) - xdotErr(t))^2 + (GPSydotErr(t) - ydotErr(t))^2);
                evz(t,:) = sqrt((GPSzdotErr(t) - zdotErr(t))^2);
                
            else
                ContStepGPSoutput = ContStepGPSoutput + 1;
                E(t) = E(t-1);
                Ev(t) = Ev(t-1);
                
                ehoriz(t,:) = ehoriz(t-1,:);
                ez(t,:) = ez(t-1,:);
                
                evhoriz(t,:) = evhoriz(t-1,:);
                evz(t,:) = evz(t-1,:);
            end
        end
        [OwnEcefXg, OwnEcefYg, OwnEcefZg] = lv2ecef(xErr(t), yErr(t), 0, BarycentreLat/180*pi, BarycentreLon/180*pi, BarycentreAltitude, ELLIPSOID);
        [OwnLatErr(t,1), OwnLongErr(t,1)] = ecef2geodetic(OwnEcefXg, OwnEcefYg, OwnEcefZg, ELLIPSOID);
        OwnAltErr(t,1)                    = zErr(t);
%    else
        OwnLatErr(t,1)  = OwnLat(1,t);
        OwnLongErr(t,1) = OwnLong(1,t);
        OwnAltErr(t,1)  = OwnAlt(1,t);

%    end
        
end


%%Simulation terminated
disp('Simulation terminated...');
%%

%%Dati traiettoria senza errori
TrajectoryData.LocPos   = [x',y',z'];
TrajectoryData.Ang      = [Phi',Theta',Psi'];       
TrajectoryData.Vel      = [xdotdot', ydotdot', zdotdot'];
TrajectoryData.AngVel   = [PhiDot', ThetaDot', PsiDot'];
TrajectoryData.GeoPos   = [OwnLat',OwnLon',OwnAlt'];
TrajectoryData.Time     = time;
%%

%%Dati traiettoria indotta senza errori
GhostTrajectoryData.LocPos   = [xg',yg',zg'];
GhostTrajectoryData.Ang      = [Phig',Thetag',Psig'];       
GhostTrajectoryData.Vel      = [xdotdotg', ydotdotg', zdotdotg'];
GhostTrajectoryData.AngVel   = [PhiDotg', ThetaDotg', PsiDotg'];
GhostTrajectoryData.GeoPos   = [OwnLatg',OwnLong',OwnAltg'];
GhostTrajectoryData.Time     = time;
%%

%%Dati traiettoria indotta con errori
RealGhostTrajectoryData.LocPos   = [GPSxErr',GPSyErr',GPSzErr'];     
RealGhostTrajectoryData.Vel      = [GPSxdotErr', GPSydotErr', GPSzdotErr'];
RealGhostTrajectoryData.Time     = time;
%%

%%Dati traiettoria finale con errori
RealTrajectoryData.LocPos   = [xErr',yErr',zErr'];     
RealtTrajectoryData.Vel     = [xdotErr', ydotErr', zdotErr'];
RealTrajectoryData.Time     = time;
%%

Report.ins.pos      = [xErr',yErr',zErr'];
Report.ins.vel      = [xdotErr', ydotErr', zdotErr'];
Report.gps.pos      = [GPSxErr',GPSyErr',GPSzErr'];
Report.gps.vel      = [GPSxdotErr', GPSydotErr', GPSzdotErr'];
Report.error.pos    = [ehoriz,ez];
Report.error.vel    = [evhoriz,evz];
Report.error.E      = E';
Report.error.Ev     = Ev';
Report.attack       = attack;
Report.time         = time;

save report.mat Report
