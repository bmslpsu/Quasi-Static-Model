function [element, Rot_wing_force, Rot_wing_torque] = Force_Rotational(element, Kinematics, del_r, c, rho, weight)
    %% Preamble
    % Calculates rotational forces and torques for a flapping wing model.
    % Forces and torques are computed for each wing element over time.

    % Units:
    % - del_r (mm): Length of each wing segment
    % - c (mm): Chord length of each segment

    %% Starting Message
    disp('Rotation Force Calculation - Start');

    %% Initialize Parameters
    N = length(Kinematics.psi_d);           % Number of time steps
    num_elements = length(element);         % Number of wing elements

    % Preallocate force and torque arrays
    Rot_wing_force = zeros(3, N);
    Rot_wing_torque = zeros(3, N);

    % Rotational force coefficient
    C_r = 1.55;

    %% Main Computation
    for j = 1:N % Loop over time steps
        % Temporary accumulators for the entire wing at time step j
        Rot_wing_temp = zeros(3, 1);
        Rot_wing_torque_temp = zeros(3, 1);

        for i = 1:num_elements % Loop over each wing element
            %% Rotational Force Calculation
            % Element-wise rotational force magnitude
            element(i).force_Rotation(j) = C_r * rho * c(i)^2 * del_r * element(i).linear_vel_norm(j) * Kinematics.psi_d(j);

            % Force direction (normal to wing surface, in velocity direction)
            Rotation_Direction = [0; 1; 0]; % Consistent with Zafar's model and 2008 Dickinson

            % Compute rotational force vector for the element
            Force_element = element(i).force_Rotation(j) * Rotation_Direction;

            % Accumulate rotational force and torque
            Rot_wing_temp = Rot_wing_temp + Force_element;
            Rot_wing_torque_temp = Rot_wing_torque_temp + cross(element(i).locationInMovingFrame(:, j), Force_element);
        end

        %% Store Total Forces and Torques for Time Step
        Rot_wing_force(:, j) = Rot_wing_temp;
        Rot_wing_torque(:, j) = Rot_wing_torque_temp;
    end

    %% Ending Message
    disp('Rotation Force Calculation - End');
end
