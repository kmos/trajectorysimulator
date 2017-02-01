classdef Area
    %Area Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %% Area parameters
        % Name                        Value                 Unit    Type     Comment
        width                       = single(2000);         % [m]   - single - Larghezza area target
        lenght                      = single(2000);         % [m]   - single - Lunghezza area target
        barycentreLat               = single(41.122533);    % [deg] - single - Latitudine del baricentro dell'area target
        barycentreLon               = single(14.169928);    % [deg] - single - Longitudine del baricentro dell'area target
        barycentreAlt               = single(0);            % [m]   - single - Quota del baricentro dell'area target
    end
    
    methods
    end
    
end

