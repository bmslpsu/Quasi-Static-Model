function [element, Lift_force, Drag_force, Lift_torque, Drag_torque] = Force_LiftAndDrag(element, Kinematics, del_r, rho, c, weight)
    %% Preamble
    % Calculates lift and drag forces and their corresponding torques for a flapping wing.
    % Forces and torques are calculated for each wing element across time steps.
    % Only the magnitudes of forces are computed, not their directions.

    % Units:
    % - del_r (mm): Length of each wing segment
    % - c (mm): Chord length of each segment

    %% Starting Message
    disp('Lift and Drag Force Calculation - Start');

    %% Initialize Parameters
    N = length(Kinematics.psi); % Number of time steps
    num_elements = length(element); % Number of wing elements

    % Preallocate force and torque arrays
    Lift_force = zeros(3, N);
    Drag_force = zeros(3, N);
    Lift_torque = zeros(3, N);
    Drag_torque = zeros(3, N);

    %% Compute Lift and Drag Coefficients
    % Based on angle of attack using Dickinson's 2002 equations
    angle_of_attack_deg = rad2deg(Kinematics.psi);
    C_L = 0.225 + 1.58 * sin(deg2rad(2.13 * abs(angle_of_attack_deg) - 7.2));
    C_D = 1.92 - 1.55 * cos(deg2rad(2.04 * angle_of_attack_deg - 9.82));

    %% Compute Forces and Torques
    for j = 1:N % Loop over each time step
        % Temporary accumulators for the entire wing at time step j
        Lift_wing_temp = zeros(3, 1);
        Drag_wing_temp = zeros(3, 1);
        Lift_wing_torque_temp = zeros(3, 1);
        Drag_wing_torque_temp = zeros(3, 1);

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
        end

        %% Store Total Forces and Torques for Time Step
        Lift_force(:, j) = Lift_wing_temp;
        Drag_force(:, j) = Drag_wing_temp;
        Lift_torque(:, j) = Lift_wing_torque_temp;
        Drag_torque(:, j) = Drag_wing_torque_temp;
    end

    %% Ending Message
    disp('Lift and Drag Force Calculation - End');
end
