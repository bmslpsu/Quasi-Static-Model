function [element, force_components] = Force_Components(element, Kinematics, del_r, rho, c)
    %% Preamble
    % Calculates lift, drag, rotational, and added mass forces and their corresponding torques for a flapping wing.
    % Forces and torques are calculated for each wing element across time steps.
    % Only the magnitudes of forces are computed, not their directions.

    % Units:
    % - del_r (mm): Length of each wing segment
    % - c (mm): Chord length of each segment

    %% Starting Message
    disp('Force Calculation - Start');

    %% Initialize Parameters
    N = length(Kinematics.psi); % Number of time steps
    num_elements = length(element); % Number of wing elements

    % Preallocate force and torque arrays
    Lift_force = zeros(3, N);
    Drag_force = zeros(3, N);
    Lift_torque = zeros(3, N);
    Drag_torque = zeros(3, N);
    Rot_wing_force = zeros(3, N);
    Rot_wing_torque = zeros(3, N);
    AM_wing_force = zeros(3, N);
    AM_wing_torque = zeros(3, N);

    %% Compute Coefficients
    % Based on angle of attack using Dickinson's 2002 equations
    angle_of_attack_deg = rad2deg(Kinematics.psi);
    C_L = 0.225 + 1.58 * sin(deg2rad(2.13 * abs(angle_of_attack_deg) - 7.2));
    C_D = 1.92 - 1.55 * cos(deg2rad(2.04 * angle_of_attack_deg - 9.82));
    C_r = 1.55;

    %% Compute Forces and Torques
    for j = 1:N % Loop over each time step
        % Temporary accumulators for the entire wing at time step j
        Lift_wing_temp = zeros(3, 1);
        Drag_wing_temp = zeros(3, 1);
        Lift_wing_torque_temp = zeros(3, 1);
        Drag_wing_torque_temp = zeros(3, 1);
        Rot_wing_temp = zeros(3, 1);
        Rot_wing_torque_temp = zeros(3, 1);
        AM_wing_temp = zeros(3, 1);
        AM_wing_torque_temp = zeros(3, 1);

        for i = 1:num_elements % Loop over each wing element
            %% Lift
            % Element-wise Calculation
            element(i).force_Lift(j) = 0.5 * rho * del_r * c(i) * C_L(j) * element(i).linear_vel_norm(j).^2;
            
            % Force Direction
            Lift_Direction = rotx(sign(element(i).linear_vel(3))*pi/2)*element(i).linear_vel_direction(:,j)*sign(Kinematics.phi_d(j)); % Force is always normal (up) to the velocity direction % Same as Zafar's model and 2008 Dickson
            
            % Element force
            Force_element = element(i).force_Lift(j).*Lift_Direction;
            
            %Force for entire Element
            Lift_wing_temp = Lift_wing_temp + Force_element;
            
            %Torque for entire Element
            Lift_wing_torque_temp = Lift_wing_torque_temp + cross(element(i).locationInMovingFrame(1:3,j),Force_element);

            %% Drag
            % Element-wise Calculation
            element(i).force_Drag(j) = 0.5 * rho * del_r * c(i) * C_D(j) * element(i).linear_vel_norm(j).^2;
            
            % Force Direction % Force is always opposite the velocity direction
            Drag_Direction = element(i).linear_vel_direction(:,j)*sign(Kinematics.phi_d(j)); % Force is always opposite the velocity direction % Same as Zafar's model and 2008 Dickson
            
            % Element force
            Force_element = element(i).force_Drag(j).*Drag_Direction;
            
            %Force for entire Element
            Drag_wing_temp = Drag_wing_temp + Force_element;
            
            %Torque for entire Element
            Drag_wing_torque_temp = Drag_wing_torque_temp + cross(element(i).locationInMovingFrame(1:3,j),Force_element);

            %% Rotational
            % Element-wise rotational force magnitude
            element(i).force_Rotation(j) = C_r * rho * c(i)^2 * del_r * element(i).linear_vel_norm(j) * Kinematics.psi_d(j);

            % Force direction (normal to wing surface, in velocity direction)
            Rotation_Direction = [0; 1; 0]; % Consistent with Zafar's model and 2008 Dickinson

            % Compute rotational force vector for the element
            Force_element = element(i).force_Rotation(j) * Rotation_Direction;

            % Accumulate rotational force and torque
            Rot_wing_temp = Rot_wing_temp + Force_element;
            Rot_wing_torque_temp = Rot_wing_torque_temp + cross(element(i).locationInMovingFrame(:, j), Force_element);

            %% Added Mass
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
        Lift_force(:, j) = Lift_wing_temp;
        Drag_force(:, j) = Drag_wing_temp;
        Lift_torque(:, j) = Lift_wing_torque_temp;
        Drag_torque(:, j) = Drag_wing_torque_temp;
        Rot_wing_force(:, j) = Rot_wing_temp;
        Rot_wing_torque(:, j) = Rot_wing_torque_temp;
        AM_wing_force(:, j) = AM_wing_temp;
        AM_wing_torque(:, j) = AM_wing_torque_temp;
    end
    %% Store Total Forces and Torques in Component Structure
    force_components.Lift_force = Lift_force;
    force_components.Drag_force = Drag_force;
    force_components.Lift_torque = Lift_torque;
    force_components.Drag_torque = Drag_torque;
    force_components.Rot_force = Rot_wing_force;
    force_components.Rot_torque = Rot_wing_torque;
    force_components.AM_force = AM_wing_force;
    force_components.AM_torque = AM_wing_torque;

    %% Ending Message
    disp('Force Calculation - End');
end
