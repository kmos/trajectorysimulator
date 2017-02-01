classdef GPS
    %GPS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %% GPS error model
        GPSstdPosHor  = 5;              % [m] Deviazione Standard GPS horizontal
        GPSstdPosVer  = 10;             % [m] Deviazione Standard GPS vertical
        GPSstdVel     = 0.1 * 0.514444; % [m/s] Deviazione Standard Velocità
        fGPS          = 1/10;           % [Hz]
        
        GPSerrX;
        GPSerrY;
        GPSerrZ;
        GPSerrVx;
        GPSerrVy;
        GPSerrVz;
    end
    
    methods        
        function obj = GPS(Tc,time)
            %%Preallocate data
            obj.GPSerrX =   zeros([1 time/Tc]);
            obj.GPSerrY =   zeros([1 time/Tc]);
            obj.GPSerrZ =   zeros([1 time/Tc]);
            obj.GPSerrVx =  zeros([1 time/Tc]);
            obj.GPSerrVy =  zeros([1 time/Tc]);
            obj.GPSerrVz =  zeros([1 time/Tc]);
            
            for i = 1:time/Tc
                obj.GPSerrX(i)      = normrnd(0,obj.GPSstdPosHor);          % [m]
                obj.GPSerrY(i)      = normrnd(0,obj.GPSstdPosHor);          % [m]
                obj.GPSerrZ(i)      = normrnd(0,obj.GPSstdPosVer);          % [m]
                obj.GPSerrVx(i)     = normrnd(0,obj.GPSstdVel);             % [m/s]
                obj.GPSerrVy(i)     = normrnd(0,obj.GPSstdVel);             % [m/s]
                obj.GPSerrVz(i)     = normrnd(0,obj.GPSstdVel);             % [m/s]
            end
        end
    end
    
end

