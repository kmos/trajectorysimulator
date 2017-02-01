classdef Angles
    %Angles Summary of this class goes here
    %   Detailed explanation goes here
    
    properties        
        %angles
        phi = 0;
        theta = 0;
        psi= 0/180*pi+pi/2; %Chi0/180*pi+pi/2;
        
        %angular velocities
        phiV    = 0;
        thetaV  = 0;
        psiV    = 0;

        %angles errors
        phiErr = 0;
        thetaErr = 0;
        psiErr = 0;
        
        %angular velocities error
        phiVErr = 0;
        thetaVErr = 0;
        psiVErr = 0;
    end
    
    methods
        function obj = Angles(t)
            %angles
            obj.phi = zeros([1 t]);
            obj.theta = zeros([1 t]);
            obj.psi= zeros([1 t]);
            
            %angular velocities
            obj.phiV = zeros([1 t]);
            obj.thetaV = zeros([1 t]);
            obj.psiV = zeros([1 t]);
            
            %angles errors
            obj.phiErr = zeros([1 t]);
            obj.thetaErr = zeros([1 t]);
            obj.psiErr = zeros([1 t]);
            
            %angular velocities error
            obj.phiVErr = zeros([1 t]);
            obj.thetaVErr = zeros([1 t]);
            obj.psiVErr = zeros([1 t]);
        end
    end
    
end

