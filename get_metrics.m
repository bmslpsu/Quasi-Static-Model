function [metrics, fly_body, fly_wing] = get_metrics()
    % Returns a structure with common parameters used in the program.
    
    %% Standard parameters
    metrics.gravity = 9810;       % Acceleration due to gravity (mm/s^2)
    metrics.airDensity = 1.225e-6;  % Density of air at sea level (g/mm^3)
    
    %% Body parameters
    %fly_body.density = 1.2e-3; %g/mm^3 % This is from the MPC simulation.
    fly_body.density = 1e-3; %g/mm^3 % https://arc.aiaa.org/doi/epdf/10.2514/6.2006-34

    %% Wing parameters
    fly_wing.density = 1.1e-3 ; %g/mm^3 %1990_Zanker_The Wing Beat of Drosophila Melanogaster. II. Dynamics
    fly_wing.thickness = 2.6 *10^-3; %mm  %1990_Zanker_The Wing Beat of Drosophila Melanogaster. II. Dynamics
    
end
