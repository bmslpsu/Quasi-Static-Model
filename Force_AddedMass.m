function [element, AM_wing_force, AM_wing_torque] = Force_AddedMass(element, Kinematics, del_r, c, rho, weight)
    %% Preamble
    % Calculates added mass forces and torques for a flapping wing model.
    % Forces and torques are computed for each wing element over time.

    % Units:
    % - del_r (mm): Length of each wing segment
    % - c (mm): Chord length of each segment

    %% Starting Message
    disp('Added Mass Force Calculation - Start');

    %% Initialize Parameters
    N = length(Kinematics.psi);           % Number of time steps
    num_elements = length(element);       % Number of wing elements

    % Preallocate force and torque arrays
    AM_wing_force = zeros(3, N);
    AM_wing_torque = zeros(3, N);

    %% Main Computation
    for j = 1:N % Loop over time steps
        % Temporary accumulators for the entire wing at time step j
        AM_wing_temp = zeros(3, 1);
        AM_wing_torque_temp = zeros(3, 1);

        for i = 1:num_elements % Loop over each wing element
            %% Added Mass Force Calculation
            % Added mass force magnitude
            part1 = (rho * pi * c(i)^2 / 4) * del_r;
            part2 = (dot(element(i).linear_vel(:, j), element(i).linear_acc(:, j)) * sin(abs(Kinematics.psi(j)))) / element(i).linear_vel_norm(j);
            part3 = element(i).linear_vel_norm(j) * deg2rad(Kinematics.psi_d(j)) * cos(abs(Kinematics.psi(j)));

            element(i).force_AddedMass(j) = part1 * (part2 + part3);

            % Force direction (opposite to angle of attack direction)
            Added_Mass_Direction = -[0; 1; 0]; % Fixed direction in Zafar's model

            % Compute added mass force vector for the element
            Force_element = element(i).force_AddedMass(j) * Added_Mass_Direction;

            % Accumulate added mass force and torque
            AM_wing_temp = AM_wing_temp + Force_element;
            AM_wing_torque_temp = AM_wing_torque_temp + cross(element(i).locationInMovingFrame(:, j), Force_element);
        end

        %% Store Total Forces and Torques for Time Step
        AM_wing_force(:, j) = AM_wing_temp;
        AM_wing_torque(:, j) = AM_wing_torque_temp;
    end

    %% Ending Message
    disp('Added Mass Force Calculation - End');
end
