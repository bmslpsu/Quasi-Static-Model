function [element, Inertial_torque] = Torque_Forces(element,  Kinematics, inertia)
    %% Preamble
    % This function will find both the magnitude and the direction.

    %% Starting message
    disp('Inertial Torque Calculation - Start') 

    %% Array sizing
    N = length(Kinematics.psi_d);

    %% Array initialization
    Inertial_torque = zeros(3, N);

    %% Entire Time Calculation
    for j = 1:N
        %% Torques over entire kinematics (time)
        Inertial_torque(:,j) = inertia*Kinematics.alpha(:,j)+cross(Kinematics.omega(:,j),inertia*Kinematics.omega(:,j)); 
    end
        
    %% Ending message
    disp('Inertial Torque Calculation - End') 

end