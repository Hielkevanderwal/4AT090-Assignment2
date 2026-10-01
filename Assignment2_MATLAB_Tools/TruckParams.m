classdef TruckParams
    properties (Constant)
        % Tractor geometry
        L1 = 0.263;                  % wheelbase of tractor [m]
        L1c = 0.055;                % COM to front axle distance [m]
        L2 = 0.56;                  % trailer wheelbase [m]
        
        % Limits
        delta_max = 38*pi/180;      % max steering angle [rad]
        gamma_max = 110*pi/180;     % max articulation angle [rad]
    end
end
