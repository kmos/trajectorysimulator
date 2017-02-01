classdef Scenario
    %scenario Summary of this class goes here
    %   class scenario
    
    properties
        
        velocity;
        deltaX;
        deltaY;
        frequency;
        accurancy;
        time;
                
        %% Name                        Value                 Unit    Type     Comment
        Tc                          = 0.01;                 % [s]   - double - Tempo di campionamento della simulazione
        MaxSimulationStep           = uint32(36000);        % []    - uint32 - Massimo numero di step di simulazione
        MaxSimTime                  = min(300);             % [s] Assegnazione del massimo tempo di simulazione.
        CaptureTargetThreshold      = 10;                   % [m]   - double - Distanza orizzontale minima dal target per considerarlo catturato.
        ECIcostVet                  = [0.1 0.4 0.7 0.9];    % []    - double - ECI associato a tutti i target.
        BetaVet                     = [0.1 0.4 0.7 0.9];    % []    - double - Fattore beta.
        AircraftCollDist            = 30;                   % [m]   - double - Distanza minima al di sotto della quale si considera una collisione anche lontano dai target
        TargetCollDist              = 30;                   % [m]   - double - Distanza minima dal target al di sotto della quale si può considera una eventuale collisione sul target
        TargetCoverTime             = single(5);            % [s]   - single - Tempo di stazionamento sui target
        KT2MS                       = 0.514444;             % knot to m/s
        
        %Imu
        imu = Imu(1,1);
 
        %GPS         
        gps = GPS(1,1);
        fGPS          = 1/10;         % [Hz]
        StepGPSoutput = 10;
        StepUpdateGPS;

        %%
        area = Area;
        
        %% Fixed target parameters
        % Name                        Value                 Unit    Type     Comment
        TargetNumber                = uint8(5);             % []    - uint8  - Quantizzazione numero di target
        MinTargetAltitude           = single(0);            % [m]   - single - Numero minimo di target
        MaxTargetAltitude           = single(10);           % [m]   - single - Numero massimo di target
        MinDistAmongTarget          = single(20);           % [m]   - single - Distanza minima tra i target
        MinTargetSurveillanceTime   = single(0);            % [s]   - single - Minimo tempo di sorveglianza dei target
        MaxTargetSurveillanceTime   = single(0);            % [s]   - single - Massimo tempo di sorveglianza dei target
        MaxECIcost                  = single(1);            % []    - single - Massimo valore dell'ECI cost dei target
        
        %% Aircraft
        aircraft = Aircraft(1,1);
                
        %% Earth parameters
        ELLIPSOID = double([6378137.000000000 0.082094438]);
        
        TargetEcefX;
        TargetEcefY;
        TargetEcefZ;
        TargetLat;
        TargetLon;
        TargetAlt;
        
        x_target =       [ 500;  -500;   0; 1000; 2000];
        y_target =       [-600;  -300;   0; 1000; 2000];
        TargetAltitude = [ 500;   500; 500;  500;  500];
        
        TargetEcefXg;
        TargetEcefYg;
        TargetEcefZg;
        TargetLatg;
        TargetLong;
        TargetAltg;
        
        x_targetg =       [ 500;  -500;   0;  900; 1800];
        y_targetg =       [-600;  -300;   0; 1000; 2000];
        TargetAltitudeg = [ 500;   500; 500;  600;  700];

        %% A/C parameters
        
        g        = single(9.81);                                          % [m/s^2] - single - Accelerazione di gravità
        
        globalTime = 0; % [s] Istante di inizio simulazione.
        
        
        % [rad] Inizializzazione stato interno modello di evoluzione dell'angolo di
        % track.
        trackErr = 0;
        trackErrg = 0;
        
        trajectory  = Trajectory(1);
        trajectoryg = Trajectory(1);

    end
    
    methods
        %%init function
        function obj = Scenario(tim)
            
            obj.MaxSimTime = min(tim);
            obj.Tc = 0.01;
            obj.StepUpdateGPS = 1/obj.Tc/obj.fGPS;    % []
            %%set accurancy
            obj.imu.accurancy = obj.accurancy;
            
            %% init IMU e GPS
            obj.imu = Imu(obj.Tc,obj.MaxSimTime);
            obj.gps = GPS(obj.Tc,obj.MaxSimTime);
            %% init aircraft
            obj.aircraft = Aircraft(obj.area.width,obj.area.lenght);
            %% init trajectories
            obj.trajectory = Trajectory(tim/obj.Tc);
            obj.trajectoryg = Trajectory(tim/obj.Tc);
          
            %%Preallocate data            
            obj.TargetEcefX=zeros([obj.TargetNumber 1]);
            obj.TargetEcefY=zeros([obj.TargetNumber 1]);
            obj.TargetEcefZ=zeros([obj.TargetNumber 1]);
            obj.TargetLat=zeros([obj.TargetNumber 1]);
            obj.TargetLon=zeros([obj.TargetNumber 1]);
            obj.TargetAlt=zeros([obj.TargetNumber 1]);
            
            obj.TargetEcefXg=zeros([obj.TargetNumber 1]);
            obj.TargetEcefYg=zeros([obj.TargetNumber 1]);
            obj.TargetEcefZg=zeros([obj.TargetNumber 1]);
            obj.TargetLatg=zeros([obj.TargetNumber 1]);
            obj.TargetLong=zeros([obj.TargetNumber 1]);
            obj.TargetAltg=zeros([obj.TargetNumber 1]);
            
            for t = 1:obj.TargetNumber
                [obj.TargetEcefX(t,1), obj.TargetEcefY(t,1), obj.TargetEcefZ(t,1)]  = lv2ecef(obj.x_target(t,1), obj.y_target(t,1), 0, obj.area.barycentreLat/180*pi, obj.area.barycentreLon/180*pi, obj.area.barycentreAlt, obj.ELLIPSOID);
                [obj.TargetLat(t,1), obj.TargetLon(t,1)]                            = ecef2geodetic(obj.TargetEcefX(t,1), obj.TargetEcefY(t,1), obj.TargetEcefZ(t,1), obj.ELLIPSOID);
                obj.TargetAlt(t,1)                                                  = obj.TargetAltitude(t,1);
            end

            for t = 1:obj.TargetNumber
                [obj.TargetEcefXg(t,1), obj.TargetEcefYg(t,1), obj.TargetEcefZg(t,1)]  = lv2ecef(obj.x_targetg(t,1), obj.y_targetg(t,1), 0, obj.area.barycentreLat/180*pi, obj.area.barycentreLon/180*pi, obj.area.barycentreAlt, obj.ELLIPSOID);
                [obj.TargetLatg(t,1), obj.TargetLong(t,1)]                            = ecef2geodetic(obj.TargetEcefXg(t,1), obj.TargetEcefYg(t,1), obj.TargetEcefZg(t,1), obj.ELLIPSOID);
                obj.TargetAltg(t,1)                                                  = obj.TargetAltitudeg(t,1);
            end
            
            % Tempo di simulazione
            obj.globalTime = 0; % [s] Istante di inizio simulazione.
            
            
            % [rad] Inizializzazione stato interno modello di evoluzione dell'angolo di
            % track.
            obj.trackErr = 0;
            obj.trackErrg = 0;      
            
        end
        %%       
        %%generate filename
        function name = fileName(obj)
            %%implementation
            if obj.accurancy == 1
                type = 'high';
            else
                type = 'low';
            end
            
            t = datetime('now','TimeZone','local','Format','d-MMM-y HH.mm');
     
            name = strcat('scenario-',type,'.accurancy-time.',num2str(obj.time),   ...
                's-freq.', num2str(obj.frequency), 'hz-vel.',     ...
                num2str(obj.velocity),'-date.', datestr(t));
        end
        %%
        function result = runScenario(obj)
            %
            nextTarget = 1;
            nextTargetg = 1;
            stopExploration = 0;
            
            %% LLH del punto di decollo degli aircraft
            [TakeoffAircraftEcefX, TakeoffAircraftEcefY, TakeoffAircraftEcefZ] = lv2ecef(obj.aircraft.XTakeoff, obj.aircraft.YTakeoff, 0, obj.area.barycentreLat/180*pi,obj.area.barycentreLon/180*pi, obj.area.barycentreAlt, obj.ELLIPSOID);
            %TakeoffAircraftEcefX=0, TakeoffAircraftEcefY=0, TakeoffAircraftEcefZ=0;
            [TakeoffAircraftLat, TakeoffAircraftLon]       = ecef2geodetic(TakeoffAircraftEcefX, TakeoffAircraftEcefY, TakeoffAircraftEcefZ, obj.ELLIPSOID);
            TakeoffAircraftAlt = single(obj.aircraft.altitude);
            
            %% [[m], [m], [m]] Calcolo posizioni relative di partenza dei velivoli rispetto alla
            % posizione baricetrica assoluta definita.
            [x0, y0, z0] = InitialLLH2ENU(obj.area.barycentreLat, obj.area.barycentreLon, obj.area.barycentreAlt, TakeoffAircraftAlt, TakeoffAircraftLat, TakeoffAircraftLon, obj.ELLIPSOID);
            z0 = obj.TargetAltitude(1);
            %% Posizioni iniziali di tutti i velivoli rispetto alla posizione
            % baricetrica assoluta definita.
            x = x0; % [m]
            y = y0; % [m]
            z = z0; % [m]
            xg = x0; % [m]
            yg = y0; % [m]
            zg = z0; % [m]            
            %% Velocità iniziali di tutti i velivoli in coordinate sferiche.
            V       = obj.aircraft.HorSpeed;             % [m/s]
            Vz      = obj.aircraft.Vz0;                  % [m/s]
            Track   = 0/180*pi;                       % [rad]
            Vg      = obj.aircraft.HorSpeed;             % [m/s]
            Vzg     = obj.aircraft.Vz0;                  % [m/s]
            Trackg  = 0/180*pi;                       % [rad]
            
            %% [[rad], [rad], [m]] Posizioni iniziali in coordinate LLH.
            [obj.trajectory.lat,obj.trajectory.lon, obj.trajectory.alt] = ENU2LLH(x, y, z, obj.area.barycentreLat, obj.area.barycentreLon, obj.area.barycentreAlt, obj.ELLIPSOID);
            [obj.trajectoryg.lat, obj.trajectoryg.lon, obj.trajectoryg.alt] = ENU2LLH(xg, yg, zg, obj.area.barycentreLat, obj.area.barycentreLon, obj.area.barycentreAlt, obj.ELLIPSOID);
            obj.trajectory.latErr(1)   = obj.trajectory.lat;
            obj.trajectory.lonErr(1)  = obj.trajectory.lon;
            obj.trajectory.altErr(1)   = obj.trajectory.alt;
            
            %% [[m/s], [m/s], [m/s]] Velocità iniziali di tutti i velivoli in coordinate cartesiane.
            [obj.trajectory.ned.xV,obj.trajectory.ned.yV,obj.trajectory.ned.zV] = SPH2CAR(V, Vz, Track);
            [obj.trajectoryg.ned.xV,obj.trajectoryg.ned.yV,obj.trajectoryg.ned.zV] = SPH2CAR(Vg, Vzg, Trackg);
            

            
            %% Ciclo principale di simulazione sul numero di step assegnato.
            t = 1;
            
            %% attacco
            attack(t) = 0;
            
            %% velocità GPS in ENU con errore [m/s m/s m/s]
            GPSxdotErr = zeros([1 obj.MaxSimTime/obj.Tc]);
            GPSydotErr = zeros([1 obj.MaxSimTime/obj.Tc]);
            GPSzdotErr = zeros([1 obj.MaxSimTime/obj.Tc]);
            %% posizione GPS in ENU con errore [m m m]
            GPSxErr = zeros([1 obj.MaxSimTime/obj.Tc]);
            GPSyErr = zeros([1 obj.MaxSimTime/obj.Tc]);
            GPSzErr = zeros([1 obj.MaxSimTime/obj.Tc]);            
            
            
            %% velocità GPS in ENU con errore [m/s m/s m/s]
            GPSxdotErr(1) = obj.aircraft.HorSpeed;
            GPSydotErr(1) = 0;
            GPSzdotErr(1) = 0;
            %% posizione GPS in ENU con errore [m m m]
            GPSxErr(1) = obj.aircraft.XTakeoff;
            GPSyErr(1) = obj.aircraft.YTakeoff;
            GPSzErr(1) = obj.trajectory.alt(1);
            
            ContStepUpdateGPS = 1;
            ContStepGPSoutput = 1;
            %% velocità iniziale ENU con errore [m/s m/s m/s]
            obj.trajectory.ned.xVErr(1)  = obj.aircraft.HorSpeed;
            obj.trajectory.ned.yVErr(1)  = 0;
            obj.trajectory.ned.zVErr(1)  = 0;
            %% posizione iniziali ENU con errore [m m m]
            obj.trajectory.ned.xErr(1)     = obj.aircraft.XTakeoff;
            obj.trajectory.ned.yErr(1)     = obj.aircraft.YTakeoff;
            obj.trajectory.ned.zErr(1)     = obj.trajectory.alt(1);
            
            %% distanza euclidea errori

            E    = zeros([obj.MaxSimTime/obj.Tc 1]);
            Ev   = zeros([obj.MaxSimTime/obj.Tc 1]);
            
            ehoriz = zeros([obj.MaxSimTime/obj.Tc 1]);
            evhoriz = zeros([obj.MaxSimTime/obj.Tc 1]);
            
            ez   = zeros([obj.MaxSimTime/obj.Tc 1]);           
            evz  = zeros([obj.MaxSimTime/obj.Tc 1]);            

%             E(1)    = 0;
%             Ev(1)   = 0;
%             
%             ehoriz(1) = 0;
%             evhoriz(1) = 0;
%             
%             ez(1)   = 0;            
%             evz(1)  = 0;

            while ((t*obj.Tc) < obj.MaxSimTime)
                
                t = t + 1; % t [], indice del ciclo principale.
                if mod(t*obj.Tc, 10) < 0.001
                    disp(['Simulation time = ' num2str(t*obj.Tc) 's']) % [s] Tempo di simulazione visualizzato a schermo.
                end
                %disp(num2str(t));
                %XXX: è utile? Aggiornamento del GlobalTime uguale per tutti i target. 
                obj.globalTime = obj.globalTime + obj.Tc; % [s]
                
                %% [[m], [m], [m], [m/s], [m/s], [rad], [rad], [rad], [rad], [m]] Aircraft
                % Model, modello di evoluzione di posizione e velocità del velivolo, con
                % relativo autopilota (modello da non implementare a bordo).
                [obj.trajectory.ned.x, obj.trajectory.ned.y,obj.trajectory.ned.z,           ... 
                    V, Vz, Track, obj.trackErr, obj.trajectory.lat                          ...
                    ,obj.trajectory.lon, obj.trajectory.alt,V_d(t),                         ...
                    Altitude_d(t), Track_d(t), obj.trajectory.angles.phi,                   ... 
                    obj.trajectory.angles.theta, obj.trajectory.angles.psi,                 ...
                    obj.trajectory.angles.phiV,obj.trajectory.angles.thetaV,                ...
                    obj.trajectory.angles.psiV,obj.trajectory.body.xV,                      ... 
                    obj.trajectory.body.yV,obj.trajectory.body.zV,                          ...
                    obj.trajectory.ned.xA,obj.trajectory.ned.yA,                            ...
                    obj.trajectory.ned.zA,obj.trajectory.ned.xV,                            ...
                    obj.trajectory.ned.yV,obj.trajectory.ned.zV,                            ...
                    obj.trajectory.body.xA,obj.trajectory.body.yA,                          ...
                    obj.trajectory.body.zA,obj.trajectory.angles.phiErr,                    ...
                    obj.trajectory.angles.thetaErr,                                         ...
                    obj.trajectory.angles.psiErr,obj.trajectory.ned.xAErr,                  ...
                    obj.trajectory.ned.yAErr,obj.trajectory.ned.zAErr,                     ...
                    obj.trajectory.ned.xVErr,obj.trajectory.ned.yVErr,                      ...
                    obj.trajectory.ned.zVErr,obj.trajectory.ned.xErr,                       ...
                    obj.trajectory.ned.yErr,obj.trajectory.ned.zErr,                        ...
                    obj.trajectory.angles.phiVErr,obj.trajectory.angles.thetaVErr,          ...
                    obj.trajectory.angles.psiVErr] =                                        ...
                 AircraftModel(1, nextTarget, t,obj.trajectory.ned.x,                       ...
                    obj.trajectory.ned.y,obj.trajectory.ned.z,obj.trajectory.ned.xV,        ...
                    obj.trajectory.ned.yV,obj.trajectory.ned.zV,V, Vz, Track,               ... 
                    obj.trackErr,obj.trajectory.lat,obj.trajectory.lon,                     ...
                    obj.trajectory.alt, obj.TargetAltitude,                                 ...
                    obj.aircraft.HorSpeed, obj.TargetNumber,obj.TargetLat,                  ...
                    obj.TargetLon,obj.TargetAlt,obj.area.barycentreLat,                     ...
                    obj.area.barycentreLon,obj.area.barycentreAlt,                          ...
                    obj.ELLIPSOID, obj.g, obj.Tc, obj.aircraft.Vmin,                        ...
                    obj.aircraft.Vmax,obj.aircraft.VdotMin,obj.aircraft.VdotMax,            ...
                    obj.aircraft.TauV,obj.aircraft.VzMin,obj.aircraft.VzMax,                ...
                    obj.aircraft.VzDotMin,obj.aircraft.VzDotMax,obj.aircraft.TauVz,         ...
                    obj.aircraft.GainAltitudeControl,obj.aircraft.PhiMax,                   ...
                    obj.aircraft.PhiDotMax,obj.aircraft.TauChi,                             ...
                    stopExploration,x0, y0, obj.trajectory.angles.phi,                      ...
                    obj.trajectory.angles.theta,obj.trajectory.angles.psi,                  ...
                    obj.trajectory.angles.phiV,obj.trajectory.angles.thetaV,                ...
                    obj.trajectory.angles.psiV,obj.trajectory.body.xV,                      ... 
                    obj.trajectory.body.yV,obj.trajectory.body.zV,                          ... 
                    obj.trajectory.ned.xA,obj.trajectory.ned.yA,                            ...
                    obj.trajectory.ned.zA,obj.trajectory.body.xA,obj.trajectory.body.yA,    ...
                    obj.trajectory.body.zA, obj.imu.gyroErrRoll,obj.imu.gyroErrPitch,       ...
                    obj.imu.gyroErrYaw,obj.imu.accErrXb,obj.imu.accErrYb,                   ...
                    obj.imu.accErrZb,obj.trajectory.angles.phiErr,                          ...
                    obj.trajectory.angles.thetaErr,obj.trajectory.angles.psiErr,            ...
                    obj.trajectory.ned.xAErr,                                               ...
                    obj.trajectory.ned.yAErr,obj.trajectory.ned.zAErr,                      ...
                    obj.trajectory.ned.xVErr,obj.trajectory.ned.yVErr,                      ...
                    obj.trajectory.ned.zVErr,obj.trajectory.ned.xErr,                       ...
                    obj.trajectory.ned.yErr,obj.trajectory.ned.zErr,                        ...
                    obj.trajectory.angles.phiVErr,obj.trajectory.angles.thetaVErr,          ... 
                    obj.trajectory.angles.psiVErr,obj.trajectory.latErr,                    ...
                    obj.trajectory.lonErr,obj.trajectory.altErr);
                
                
                %% [[m], [m], [m], [m/s], [m/s], [rad], [rad], [rad], [rad], [m]] Aircraft
                % Model, modello di evoluzione di posizione e velocità del velivolo ghost, con
                % relativo autopilota (modello da non implementare a bordo).
                [obj.trajectoryg.ned.x, obj.trajectoryg.ned.y,obj.trajectoryg.ned.z,            ... 
                    V, Vz, Track, obj.trackErrg, obj.trajectoryg.lat                            ...
                    ,obj.trajectoryg.lon, obj.trajectoryg.alt,V_dg(t),                          ...
                    Altitude_dg(t), Track_dg(t), obj.trajectoryg.angles.phi,                    ... 
                    obj.trajectoryg.angles.theta, obj.trajectoryg.angles.psi,                   ...
                    obj.trajectoryg.angles.phiV,obj.trajectoryg.angles.thetaV,              ...
                    obj.trajectoryg.angles.psiV,obj.trajectoryg.body.xV,                      ... 
                    obj.trajectoryg.body.yV,obj.trajectoryg.body.zV,                            ...
                    obj.trajectoryg.ned.xA,obj.trajectoryg.ned.yA,                              ...
                    obj.trajectoryg.ned.zA,obj.trajectoryg.ned.xV,                              ...
                    obj.trajectoryg.ned.yV,obj.trajectoryg.ned.zV,                              ...
                    obj.trajectoryg.body.xA,obj.trajectoryg.body.yA,                            ...
                    obj.trajectoryg.body.zA,obj.trajectoryg.angles.phiErr,                      ...
                    obj.trajectoryg.angles.thetaErr,obj.trajectoryg.angles.psiErr,              ...
                    obj.trajectoryg.ned.xAErr,                    ...
                    obj.trajectoryg.ned.yAErr,obj.trajectoryg.ned.zAErr,                       ...
                    obj.trajectoryg.ned.xVErr,obj.trajectoryg.ned.yVErr,                        ...
                    obj.trajectoryg.ned.zVErr,obj.trajectoryg.ned.xErr,                         ...
                    obj.trajectoryg.ned.yErr,obj.trajectoryg.ned.zErr,                          ...
                    obj.trajectoryg.angles.phiVErr,obj.trajectoryg.angles.thetaVErr,            ...
                    obj.trajectoryg.angles.psiVErr] =                                           ...
                 AircraftModel(1, nextTargetg, t,obj.trajectoryg.ned.x,                         ...
                    obj.trajectoryg.ned.y,obj.trajectoryg.ned.z,obj.trajectoryg.ned.xV,         ...
                    obj.trajectoryg.ned.yV,obj.trajectoryg.ned.zV,V, Vz, Track,                 ... 
                    obj.trackErrg,obj.trajectoryg.lat,obj.trajectoryg.lon,                      ...
                    obj.trajectoryg.alt, obj.TargetAltitudeg,               ...
                    obj.aircraft.HorSpeed, obj.TargetNumber,obj.TargetLatg,                     ...
                    obj.TargetLong,obj.TargetAltg,obj.area.barycentreLat,                       ...
                    obj.area.barycentreLon,obj.area.barycentreAlt,                              ...
                    obj.ELLIPSOID, obj.g, obj.Tc, obj.aircraft.Vmin,                            ...
                    obj.aircraft.Vmax,obj.aircraft.VdotMin,obj.aircraft.VdotMax,                ...
                    obj.aircraft.TauV,obj.aircraft.VzMin,obj.aircraft.VzMax,                    ...
                    obj.aircraft.VzDotMin,obj.aircraft.VzDotMax,obj.aircraft.TauVz,             ...
                    obj.aircraft.GainAltitudeControl,obj.aircraft.PhiMax,                       ...
                    obj.aircraft.PhiDotMax,obj.aircraft.TauChi,                                 ...
                    stopExploration,x0, y0, obj.trajectoryg.angles.phi,                         ...
                    obj.trajectoryg.angles.theta,obj.trajectoryg.angles.psi,                    ...
                    obj.trajectoryg.angles.phiV,obj.trajectoryg.angles.thetaV,              ...
                    obj.trajectoryg.angles.psiV,obj.trajectoryg.body.xV,                      ... 
                    obj.trajectoryg.body.yV,obj.trajectoryg.body.zV,                            ... 
                    obj.trajectoryg.ned.xA,obj.trajectoryg.ned.yA,                              ...
                    obj.trajectoryg.ned.zA,obj.trajectoryg.body.xA,obj.trajectoryg.body.yA,     ...
                    obj.trajectoryg.body.zA, obj.imu.gyroErrRoll,obj.imu.gyroErrPitch,          ...
                    obj.imu.gyroErrYaw,obj.imu.accErrXb,obj.imu.accErrYb,                       ...
                    obj.imu.accErrZb,obj.trajectoryg.angles.phiErr,                             ...
                    obj.trajectoryg.angles.thetaErr,obj.trajectoryg.angles.psiErr,              ...
                    obj.trajectoryg.ned.xAErr,                     ...
                    obj.trajectoryg.ned.yAErr,obj.trajectoryg.ned.zAErr,                       ...
                    obj.trajectoryg.ned.xVErr,obj.trajectoryg.ned.yVErr,                        ...
                    obj.trajectoryg.ned.zVErr,obj.trajectoryg.ned.xErr,                         ...
                    obj.trajectoryg.ned.yErr,obj.trajectoryg.ned.zErr,                          ...
                    obj.trajectoryg.angles.phiVErr,obj.trajectoryg.angles.thetaVErr,        ... 
                    obj.trajectoryg.angles.psiVErr,obj.trajectoryg.latErr,                    ...
                    obj.trajectoryg.lonErr,obj.trajectoryg.altErr);

                %% Onboard computer software, parte del SW che dovrà essere implementata a
                % bordo.
                
                %% [[m], [m], [m]] Distanze relative dei target dalla posizione attuale del
                % velivolo ownship.
                [TargetEast, TargetNorth, TargetUp] = TargetLLH2ENU(obj.TargetNumber, obj.TargetLat, obj.TargetLon, obj.TargetAlt, obj.ELLIPSOID, obj.trajectory.lat(t), obj.trajectory.lon(t), obj.trajectory.alt(t));
                [TargetEastg, TargetNorthg, TargetUpg] = TargetLLH2ENU(obj.TargetNumber, obj.TargetLatg, obj.TargetLong, obj.TargetAltg, obj.ELLIPSOID, obj.trajectoryg.lat(t), obj.trajectoryg.lon(t), obj.trajectoryg.alt(t));
                
                %% [[s], [], [], [], [], [], [], []] Funzione di cattura del target
                % committed, con aggiornamento delle matrici di negoziazione e del relativo
                % vettori di riempimento. TimeTargetCoverage in uscita è il tempo al
                % passaggio sul target committed.
                [nextTarget]      = CaptureTargetEval(nextTarget, obj.TargetNumber, TargetEast, TargetNorth, TargetUp, obj.CaptureTargetThreshold);
                [nextTargetg]     = CaptureTargetEval(nextTargetg, obj.TargetNumber, TargetEastg, TargetNorthg, TargetUpg, obj.CaptureTargetThreshold);
                nextTargetVet(t)  = nextTarget;
                nextTargetVetg(t) = nextTargetg;
                
                if nextTarget > 5
                    fprintf('errore: %s', num2str(nextTarget));
                end
                %% Attacco
                
                if nextTarget == 3
                    if attack == 0;
                        fprintf ('Under Attack from the Waypoint %s, at time %s\r', num2str(nextTarget), num2str(t*obj.Tc));
                    end
                    attack(t,:) = 1;
                end
                
                if ContStepUpdateGPS == obj.StepUpdateGPS
                    %%la frequenza della condizione è pari a quella della finestra
                    %%di tolleranza
                    %% Calcolo velocità ENU con errore [m/s m/s m/s]
                    obj.trajectory.ned.xVErr(t) = obj.trajectoryg.ned.xV(t) + obj.gps.GPSerrVx(t);
                    obj.trajectory.ned.yVErr(t) = obj.trajectoryg.ned.yV(t) + obj.gps.GPSerrVy(t);
                    obj.trajectory.ned.zVErr(t) = obj.trajectoryg.ned.zV(t) + obj.gps.GPSerrVz(t);
                    %% Calcolo posizioni ENU con errore [m m m]
                    obj.trajectory.ned.xErr(t) = obj.trajectoryg.ned.x(t) + obj.gps.GPSerrX(t);
                    obj.trajectory.ned.yErr(t) = obj.trajectoryg.ned.y(t) + obj.gps.GPSerrY(t);
                    obj.trajectory.ned.zErr(t) = obj.trajectoryg.ned.z(t) + obj.gps.GPSerrZ(t);
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
                    GPSxdotErr(t) = obj.trajectoryg.ned.xV(t) + obj.gps.GPSerrVx(t);
                    GPSydotErr(t) = obj.trajectoryg.ned.yV(t) + obj.gps.GPSerrVy(t);
                    GPSzdotErr(t) = obj.trajectoryg.ned.zV(t) + obj.gps.GPSerrVz(t);
                    %% Calcolo posizioni GPS in ENU con errore [m m m]
                    GPSxErr(t) = obj.trajectoryg.ned.x(t) + obj.gps.GPSerrX(t);
                    GPSyErr(t) = obj.trajectoryg.ned.y(t) + obj.gps.GPSerrY(t);
                    GPSzErr(t) = obj.trajectoryg.ned.z(t) + obj.gps.GPSerrZ(t);
                    %% Calcolo velocità ENU con errore [m/s m/s m/s]
                    obj.trajectory.ned.xVErr(t) = obj.trajectory.ned.xVErr(t-1) + obj.trajectory.ned.xAErr(t)*obj.Tc;
                    obj.trajectory.ned.yVErr(t) = obj.trajectory.ned.yVErr(t-1) + obj.trajectory.ned.yAErr(t)*obj.Tc;
                    obj.trajectory.ned.zVErr(t) = obj.trajectory.ned.zVErr(t-1) + obj.trajectory.ned.zAErr(t)*obj.Tc;
                    %% Calcolo posizioni ENU con errore [m m m]
                    obj.trajectory.ned.xErr(t) = obj.trajectory.ned.xErr(t-1) + obj.trajectory.ned.xVErr(t)*obj.Tc;
                    obj.trajectory.ned.yErr(t) = obj.trajectory.ned.yErr(t-1) + obj.trajectory.ned.yVErr(t)*obj.Tc;
                    obj.trajectory.ned.zErr(t) = obj.trajectory.ned.zErr(t-1) + obj.trajectory.ned.zVErr(t)*obj.Tc;                    
                end
                if ContStepGPSoutput == obj.StepGPSoutput
                    %la frequenza della condizione è pari a quella del gps
                    ContStepGPSoutput = 1;
                    E(t, :)  =     sqrt((GPSxErr(t) - obj.trajectory.ned.xErr(t))^2 + (GPSyErr(t) - obj.trajectory.ned.yErr(t))^2 + (GPSzErr(t) - obj.trajectory.ned.zErr(t))^2); % [m]
                    Ev(t, :) =     sqrt((GPSxdotErr(t) - obj.trajectory.ned.xVErr(t))^2 + (GPSydotErr(t) - obj.trajectory.ned.yVErr(t))^2 + (GPSzdotErr(t) - obj.trajectory.ned.zVErr(t))^2); % [m/s]
                    
                    ehoriz(t,:)  = sqrt((GPSxErr(t) - obj.trajectory.ned.xErr(t))^2 + (GPSyErr(t) - obj.trajectory.ned.yErr(t))^2);
                    ez(t,:)      = sqrt((GPSzErr(t) - obj.trajectory.ned.zErr(t))^2);
                    evhoriz(t,:) = sqrt((GPSxdotErr(t) - obj.trajectory.ned.xVErr(t))^2 + (GPSydotErr(t) - obj.trajectory.ned.yVErr(t))^2);
                    evz(t,:)     = sqrt((GPSzdotErr(t) - obj.trajectory.ned.zVErr(t))^2);                    
                else
                    ContStepGPSoutput = ContStepGPSoutput + 1;
                    E(t) = E(t-1);
                    Ev(t) = Ev(t-1);                    
                    ehoriz(t,:) = ehoriz(t-1,:);
                    ez(t,:) = ez(t-1,:);                    
                    evhoriz(t,:) = evhoriz(t-1,:);
                    evz(t,:) = evz(t-1,:);
                end
                [OwnEcefXg, OwnEcefYg, OwnEcefZg] = lv2ecef(obj.trajectory.ned.xErr(t), obj.trajectory.ned.yErr(t), 0, obj.area.barycentreLat/180*pi, obj.area.barycentreLon/180*pi, obj.area.barycentreAlt, obj.ELLIPSOID);
                [obj.trajectory.latErr(t,1), obj.trajectory.lonErr(t,1)] = ecef2geodetic(OwnEcefXg, OwnEcefYg, OwnEcefZg, obj.ELLIPSOID);
                obj.trajectory.altErr(t,1)                    = obj.trajectory.ned.zErr(t);
                %    else
                obj.trajectory.latErr(t,1)  = obj.trajectory.lat(1,t);
                obj.trajectory.lonErr(t,1) = obj.trajectory.lon(1,t);
                obj.trajectory.altErr(t,1)  = obj.trajectory.alt(1,t);
                
                
            end
            
            
            result.E = E;
            result.Ev = Ev;
            result.ehoriz = ehoriz;
            result.evhoriz = evhoriz;
            result.ez = ez;
            result.evz = evz;
            
            result.nextTargetVet =  nextTargetVet;
            result.nextTargetVet =  nextTargetVetg;
        end
    end
    
end