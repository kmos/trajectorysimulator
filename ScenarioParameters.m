% Scenario parameters

% Name                        Value                 Unit    Type     Comment
Tc                          = 0.01;                 % [s]   - double - Tempo di campionamento della simulazione
MaxSimulationStep           = uint32(36000);        % []    - uint32 - Massimo numero di step di simulazione
CaptureTargetThreshold      = 10;                   % [m]   - double - Distanza orizzontale minima dal target per considerarlo catturato. 
ECIcostVet                  = [0.1 0.4 0.7 0.9];    % []    - double - ECI associato a tutti i target.
BetaVet                     = [0.1 0.4 0.7 0.9];    % []    - double - Fattore beta.
AircraftCollDist            = 30;                   % [m]   - double - Distanza minima al di sotto della quale si considera una collisione anche lontano dai target
TargetCollDist              = 30;                   % [m]   - double - Distanza minima dal target al di sotto della quale si può considera una eventuale collisione sul target
TargetCoverTime             = single(5);            % [s]   - single - Tempo di stazionamento sui target

%% INS error model
INSQuality = 1;

switch INSQuality
    case 0 % Low
        GyroBias = (10/(180*pi))/3600;                  % [rad/s]
        GyroWhiteNoise = ((0.001)/(180*pi))/sqrt(Tc);   % [rad/s]
        AccBias = 1000*(10^-6)*9.81;                    % [m/s^2]
        AccWhiteNoise = (50*(10^-6)*9.81)/sqrt(Tc);     % [m/s^2]
    case 1 % High
        GyroBias = (0.01/(180*pi))/3600;                % [rad/s]
        GyroWhiteNoise = ((3*10^-5)/(180*pi))/sqrt(Tc); % [rad/s]
        AccBias = 10*(10^-6)*9.81;                      % [m/s^2]
        AccWhiteNoise = (10*(10^-6)*9.81)/sqrt(Tc);     % [m/s^2]
end

%% GPS error model
KT2MS         = 0.514444;     % knot to m/s
GPSstdPosHor  = 5;            % [m] Deviazione Standard GPS horizontal
GPSstdPosVer  = 10;           % [m] Deviazione Standard GPS vertical
GPSstdVel     = 0.1 * KT2MS ; % [m/s] Deviazione Standard Velocità
fGPS          = 1/10;         % [Hz]
StepGPSoutput = 10;
StepUpdateGPS = 1/Tc/fGPS;    % []
%%

MaxSimTime = min([300]);  % [s] Assegnazione del massimo tempo di simulazione.

for i = 1:MaxSimTime/Tc
    GyroErrRoll(i)  = normrnd(GyroBias,GyroWhiteNoise); % [rad/s]
    GyroErrPitch(i) = normrnd(GyroBias,GyroWhiteNoise); % [rad/s]
    GyroErrYaw(i)   = normrnd(GyroBias,GyroWhiteNoise); % [rad/s]
    AccErrXb(i)     = normrnd(AccBias,AccWhiteNoise);   % [m/s^2]
    AccErrYb(i)     = normrnd(AccBias,AccWhiteNoise);   % [m/s^2]
    AccErrZb(i)     = normrnd(AccBias,AccWhiteNoise);   % [m/s^2]
    GPSerrX(i)      = normrnd(0,GPSstdPosHor);          % [m]
    GPSerrY(i)      = normrnd(0,GPSstdPosHor);          % [m]
    GPSerrZ(i)      = normrnd(0,GPSstdPosVer);          % [m]
    GPSerrVx(i)     = normrnd(0,GPSstdVel);             % [m/s]
    GPSerrVy(i)     = normrnd(0,GPSstdVel);             % [m/s]
    GPSerrVz(i)     = normrnd(0,GPSstdVel);             % [m/s]
end

%% Area parameters
% Name                        Value                 Unit    Type     Comment
AreaWidth                   = single(2000);         % [m]   - single - Larghezza area target
AreaLenght                  = single(2000);         % [m]   - single - Lunghezza area target
BarycentreLat               = single(41.122533);    % [deg] - single - Latitudine del baricentro dell'area target
BarycentreLon               = single(14.169928);    % [deg] - single - Longitudine del baricentro dell'area target
BarycentreAltitude          = single(0);            % [m]   - single - Quota del baricentro dell'area target

%% Fixed target parameters
% Name                        Value                 Unit    Type     Comment
TargetNumber                = uint8(5);             % []    - uint8  - Quantizzazione numero di target
MinTargetAltitude           = single(0);            % [m]   - single - Numero minimo di target
MaxTargetAltitude           = single(10);           % [m]   - single - Numero massimo di target
MinDistAmongTarget          = single(20);           % [m]   - single - Distanza minima tra i target
MinTargetSurveillanceTime   = single(0);            % [s]   - single - Minimo tempo di sorveglianza dei target
MaxTargetSurveillanceTime   = single(0);            % [s]   - single - Massimo tempo di sorveglianza dei target
MaxECIcost                  = single(1);            % []    - single - Massimo valore dell'ECI cost dei target

%% Aircraft parameters
% Name                        Value                         Unit    Type     Comment
AircraftHorSpeed            = single(15);                   % [m/s] - single - Velocità orizzontale minima degli aircraft
AircraftAltitude            = single(50);                   % [m]   - single - Distanza minima in quota tra le traiettorie degli aircraft
XTakeoffAircraft            = single(-AreaWidth/2);         % [m]   - uint8  - Ascissa degli aircraft al decollo
YTakeoffAircraft            = single(-AreaLenght/2);        % [m]   - uint8  - Ordinata degli aircraft al decollo

%% Earth parameters
ELLIPSOID = double([6378137.000000000 0.082094438]);

x_target =       [ 500;  -500;   0; 1000; 2000];
y_target =       [-600;  -300;   0; 1000; 2000];
TargetAltitude = [ 500;   500; 500;  500;  500];

for t = 1:TargetNumber
    [TargetEcefX(t,1), TargetEcefY(t,1), TargetEcefZ(t,1)] = lv2ecef(x_target(t,1), y_target(t,1), 0, BarycentreLat/180*pi, BarycentreLon/180*pi, BarycentreAltitude, ELLIPSOID);
    [TargetLat(t,1), TargetLon(t,1)]                       = ecef2geodetic(TargetEcefX(t,1), TargetEcefY(t,1), TargetEcefZ(t,1), ELLIPSOID);
    TargetAlt(t,1)                                         = TargetAltitude(t,1);
end

x_targetg =       [ 500;  -500;   0;  900; 1800];
y_targetg =       [-600;  -300;   0; 1000; 2000];
TargetAltitudeg = [ 500;   500; 500;  600;  700];

for t = 1:TargetNumber
    [TargetEcefXg(t,1), TargetEcefYg(t,1), TargetEcefZg(t,1)] = lv2ecef(x_targetg(t,1), y_targetg(t,1), 0, BarycentreLat/180*pi, BarycentreLon/180*pi, BarycentreAltitude, ELLIPSOID);
    [TargetLatg(t,1), TargetLong(t,1)]                       = ecef2geodetic(TargetEcefXg(t,1), TargetEcefYg(t,1), TargetEcefZg(t,1), ELLIPSOID);
    TargetAltg(t,1)                                         = TargetAltitudeg(t,1);
end

%% A/C parameters

g        = single(9.81);                                          % [m/s^2] - single - Accelerazione di gravità

% Orizzontale
V0       = single(0);                    % [m/s]   - single - Velocità orizzontale iniziale degli aircraft
Vmin     = single(10);                   % [m/s]   - single - Velocità orizzontale minima degli aircraft
Vmax     = single(20);                   % [m/s]   - single - Velocità orizzontale massima degli aircraft
VdotMin  = single(-1.8);                 % [m/s^2] - single - Decelerazione orizzontale massima degli aircraft
VdotMax  = single(1.8);                  % [m/s^2] - single - Accelerazione orizzontale massima degli aircraft
TauV     = single(5);                    % [s]     - single - Costante di tempo del controllore di velocità orizzontale

% Verticale
GainAltitudeControl = single(0.05);                                % [1/s]   - single - Guadagno del controllore di quota
Vz0      = single(0);                   % [m/s]   - single - Velocità verticale iniziale degli aircraft
VzMin    = single(-5);                  % [m/s]   - single - Velocità verticale minima degli aircraft
VzMax    = single(5);                   % [m/s]   - single - Velocità verticale massima degli aircraft
VzDotMin = single(-0.2);                % [m/s^2] - single - Accelerazione verticale minima degli aircraft
VzDotMax = single(0.2);                 % [m/s^2] - single - Accelerazione verticale massima degli aircraft
TauVz    = single(2.5);                 % [s]     - single - Costante di tempo del controllore di velocità verticale

% Laterale
Chi0      = single(0);                  % [deg]   - single - Track iniziale degli aircraft
PhiMax    = single(3);                  % [deg]   - single - Massimo angolo di roll degli aircraft
PhiDotMax = single(20);                 % [deg/s] - single - Massimo roll rate degli aircraft
TauChi    = single(5);                  % [s]     - single - Costante di tempo del controllore laterale


