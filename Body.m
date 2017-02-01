classdef Body
    %Body coordinates Summary of this class goes here
    %   Detailed explanation goes here
    
    properties      
        %body velocities
        xV;
        yV;
        zV;
        
        %body accelerations
        xA;
        yA;
        zA;
        
        %accelerations errors
        xAErr;
        yAErr;
        zAErr;
    end
    
    methods
        function obj = Body(t)
            obj.xV = zeros([1 t]);
            obj.yV = zeros([1 t]);
            obj.zV = zeros([1 t]);
            
            %body accelerations
            obj.xA = zeros([1 t]);
            obj.yA = zeros([1 t]);
            obj.zA = zeros([1 t]);
            
            %accelerations errors
            obj.xAErr = zeros([1 t]);
            obj.yAErr = zeros([1 t]);
            obj.zAErr = zeros([1 t]);
        end
    end
    
end

