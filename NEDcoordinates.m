classdef NEDcoordinates
    %Axes in NED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        %axes
        x;
        y;
        z;
        
        %NED velocities
        xV;
        yV;
        zV;

%       xdotdot(2) = 0;
%       ydotdot(2) = 0;
%       zdotdot(2) = 0;
        
        %NED accelerations
        xA;
        yA;
        zA;

        %positional errors
        xErr;
        yErr;
        zErr;
        %velocity errors
        xVErr;
        yVErr;
        zVErr;
        %accelerations errors
        xAErr;
        yAErr;
        zAErr;

    end
    
    methods
        function obj = NEDcoordinates(t)
            
            %axes
            obj.x = zeros([1 t]);
            obj.y = zeros([1 t]);
            obj.z = zeros([1 t]);
            
            %NED velocities
            obj.xV = zeros([1 t]);
            obj.yV = zeros([1 t]);
            obj.zV = zeros([1 t]);
            
            %       xdotdot(2) = 0;
            %       ydotdot(2) = 0;
            %       zdotdot(2) = 0;
            
            %NED accelerations
            obj.xA = zeros([1 t]);
            obj.yA = zeros([1 t]);
            obj.zA = zeros([1 t]);
            
            %positional errors
            obj.xErr = zeros([1 t]);
            obj.yErr = zeros([1 t]);
            obj.zErr = zeros([1 t]);
            %velocity errors
            obj.xVErr = zeros([1 t]);
            obj.yVErr = zeros([1 t]);
            obj.zVErr = zeros([1 t]);
            %accelerations errors
            obj.xAErr = zeros([1 t]);
            obj.yAErr = zeros([1 t]);
            obj.zAErr = zeros([1 t]);
            
        end
    end
    
end

