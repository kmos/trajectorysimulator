classdef Aircraft
    %aircraft Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %% Aircraft parameters
        % Name                        Value                         Unit    Type     Comment
        HorSpeed            = single(15);                   % [m/s] - single - Velocità orizzontale minima degli aircraft
        altitude            = single(50);                   % [m]   - single - Distanza minima in quota tra le traiettorie degli aircraft
        XTakeoff;                                           % [m]   - uint8  - Ascissa degli aircraft al decollo
        YTakeoff;                                           % [m]   - uint8  - Ordinata degli aircraft al decollo
        
        % Orizzontale
        V0       = single(0);                               % [m/s]   - single - Velocità orizzontale iniziale degli aircraft
        Vmin     = single(10);                              % [m/s]   - single - Velocità orizzontale minima degli aircraft
        Vmax     = single(20);                              % [m/s]   - single - Velocità orizzontale massima degli aircraft
        VdotMin  = single(-1.8);                            % [m/s^2] - single - Decelerazione orizzontale massima degli aircraft
        VdotMax  = single(1.8);                             % [m/s^2] - single - Accelerazione orizzontale massima degli aircraft
        TauV     = single(5);                               % [s]     - single - Costante di tempo del controllore di velocità orizzontale
        
        % Verticale
        GainAltitudeControl = single(0.05);                 % [1/s]   - single - Guadagno del controllore di quota
        Vz0      = single(0);                               % [m/s]   - single - Velocità verticale iniziale degli aircraft
        VzMin    = single(-5);                              % [m/s]   - single - Velocità verticale minima degli aircraft
        VzMax    = single(5);                               % [m/s]   - single - Velocità verticale massima degli aircraft
        VzDotMin = single(-0.2);                            % [m/s^2] - single - Accelerazione verticale minima degli aircraft
        VzDotMax = single(0.2);                             % [m/s^2] - single - Accelerazione verticale massima degli aircraft
        TauVz    = single(2.5);                             % [s]     - single - Costante di tempo del controllore di velocità verticale
        
        % Laterale
        Chi0      = single(0);                              % [deg]   - single - Track iniziale degli aircraft
        PhiMax    = single(3);                              % [deg]   - single - Massimo angolo di roll degli aircraft
        PhiDotMax = single(20);                             % [deg/s] - single - Massimo roll rate degli aircraft
        TauChi    = single(5);                              % [s]     - single - Costante di tempo del controllore laterale
    end
    
    methods
        
        function obj = Aircraft(width,lenght)
            obj.XTakeoff            = single(-width/2);         % [m]   - uint8  - Ascissa degli aircraft al decollo
            obj.YTakeoff            = single(-lenght/2);        % [m]   - uint8  - Ordinata degli aircraft al decollo
        
        end
    end
    
end

