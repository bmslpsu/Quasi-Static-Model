function [element, Inertial_torque] = Torque_Inertia(element, Kinematics, inertia, R_inv)
    %% Preamble
    % Calculates inertial torques for a flapping wing model.
    % The torque accounts for both angular acceleration and gyroscopic effects.

    % Inputs:
    % - element: Structure array for wing elements
    % - Kinematics: Structure containing angular velocity and acceleration
    % - inertia: Inertia matrix (3x3)

    % Outputs:
    % - element: Updated structure (if needed)
    % - Inertial_torque: Inertial torque at each time step (3xN)

    %% Starting Message
    disp('Inertial Torque Calculation - Start');

    %% Initialize Parameters
    N = size(Kinematics.omega, 2);  % Number of time steps
    Inertial_torque = zeros(3, N);  % Preallocate inertial torque array

    %% Compute Inertial Torques
    for j = 1:N

    inertia_Rotated = R_inv(:, :, j) * inertia * R_inv(:, :, j)';

    torque_angular_accel(:, j) = inertia_Rotated * Kinematics.alpha(:, j);                                % Torque due to angular acceleration
    torque_gyroscopic(:, j) = cross(Kinematics.omega(:, j), inertia_Rotated * Kinematics.omega(:, j));    % Gyroscopic (Coriolis) torque
    
    Inertial_torque(:, j) = torque_angular_accel(:, j) + torque_gyroscopic(:, j);                         % Total inertial torque
    end
    

    %% Ending Message
    disp('Inertial Torque Calculation - End');
end
