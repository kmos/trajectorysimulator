classdef Imu
    %Imu Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        accurancy;
        
        gyroBias;                       % [rad/s]
        gyroWhiteNoise;                 % [rad/s]
        accBias;                        % [m/s^2]
        accWhiteNoise;                  % [m/s^2]
        
        
        %error
        gyroErrRoll;
        gyroErrPitch;
        gyroErrYaw;
        accErrXb;
        accErrYb;
        accErrZb;
        
    end
    
    methods       
        function obj = Imu(Tc,time)
            
            %
            if obj.accurancy == 0
                obj.gyroBias = (10/(180*pi))/3600;                  % [rad/s]
                obj.gyroWhiteNoise = ((0.001)/(180*pi))/sqrt(Tc);   % [rad/s]
                obj.accBias = 1000*(10^-6)*9.81;                    % [m/s^2]
                obj.accWhiteNoise = (50*(10^-6)*9.81)/sqrt(Tc);     % [m/s^2]
                
            else
                obj.gyroBias = (0.01/(180*pi))/3600;                % [rad/s]
                obj.gyroWhiteNoise = ((3*10^-5)/(180*pi))/sqrt(Tc); % [rad/s]
                obj.accBias = 10*(10^-6)*9.81;                      % [m/s^2]
                obj.accWhiteNoise = (10*(10^-6)*9.81)/sqrt(Tc);     % [m/s^2]
            end
            
            %Preallocate data
            obj.gyroErrRoll = zeros([1 time/Tc]);
            obj.gyroErrPitch = zeros([1 time/Tc]);
            obj.gyroErrYaw = zeros([1 time/Tc]);
            obj.accErrXb = zeros([1 time/Tc]);
            obj.accErrYb = zeros([1 time/Tc]);
            obj.accErrZb = zeros([1 time/Tc]);
            %%
            
            for i = 1:time/Tc
                obj.gyroErrRoll(i)  = normrnd(obj.gyroBias,obj.gyroWhiteNoise);     % [rad/s]
                obj.gyroErrPitch(i) = normrnd(obj.gyroBias,obj.gyroWhiteNoise);     % [rad/s]
                obj.gyroErrYaw(i)   = normrnd(obj.gyroBias,obj.gyroWhiteNoise);     % [rad/s]
                obj.accErrXb(i)     = normrnd(obj.accBias,obj.accWhiteNoise);       % [m/s^2]
                obj.accErrYb(i)     = normrnd(obj.accBias,obj.accWhiteNoise);       % [m/s^2]
                obj.accErrZb(i)     = normrnd(obj.accBias,obj.accWhiteNoise);       % [m/s^2]
            end
            
        end
        
    end
    
end

