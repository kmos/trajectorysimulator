classdef Trajectory
    %Trajectory Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lat;
        lon;
        alt;
        
        body = Body(1);
        ned = NEDcoordinates(1);
        angles = Angles(1);
        
        latErr;
        lonErr;
        altErr;
    end
    
    methods
        function  obj = Trajectory(t)
            obj.lat = zeros([1 t]);
            obj.lon = zeros([1 t]);
            obj.alt = zeros([1 t]);
            
            obj.latErr = zeros([t 1]);
            obj.lonErr = zeros([t 1]);
            obj.altErr = zeros([t 1]);
            
            obj.body = Body(t);
            obj.ned = NEDcoordinates(t);
            obj.angles = Angles(t);
        end
    end
    
end

