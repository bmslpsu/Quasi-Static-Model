function [metrics, fly_body, fly_wing] = get_metrics()
    % Returns a structure with common parameters used in the program.
    
    %% Standard parameters
    metrics.gravity = 9810;       % Acceleration due to gravity (mm/s^2)
    metrics.airDensity = 1.225e-6;  % Density of air at sea level (g/mm^3)
    
    %% Body parameters
    fly_body.density = 1.1e-3; %g/mm^3

    %% Wing parameters
    fly_wing.density = 0.8e-3; %g/mm^3
    fly_wing.thickness = 5 *10^-3; %mm
    
end
