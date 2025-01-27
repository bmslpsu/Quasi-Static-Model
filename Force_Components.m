function [element, Components] = Force_Components(element, Kinematics, del_r, rho, c)
    %% Preamble
    % Calculates lift, drag, rotational, and added mass forces and their corresponding torques for a flapping wing.
    % Forces and torques are calculated for each wing element across time steps.

    %% Starting Message
    disp('Component Calculation - Start');

    %% Initialize Parameters
    N = length(Kinematics.psi); % Number of time steps
    N_elements = length(element); % Number of wing elements

    % Preallocate force and torque arrays
    Force_Lift      = zeros(3, N);
    Force_Drag      = zeros(3, N);
    Force_Rotation  = zeros(3, N);
    Force_AM        = zeros(3, N);
    Torque_Lift     = zeros(3, N);
    Torque_Drag     = zeros(3, N);
    Torque_Rotation = zeros(3, N);
    Torque_AM       = zeros(3, N);

    %% Compute Coefficients
    % Based on angle of attack using Dickinson's 1999 equations
    AoA_deg = rad2deg(Kinematics.psi);
    C_L = 0.225 + 1.58 * sin(deg2rad(2.13 * abs(AoA_deg) - 7.2));
    C_D = 1.92 - 1.55 * cos(deg2rad(2.04 * AoA_deg - 9.82));
    C_r = 1.55;

    %% Compute Forces and Torques
    for j = 1:N % Loop over each time step
        % Temporary accumulators for the entire wing at time step j
        Force_Lift_temp         = zeros(3, 1);
        Force_Drag_temp         = zeros(3, 1);
        Force_Rotation_temp     = zeros(3, 1);
        Force_AM_temp           = zeros(3, 1);
        Torque_Lift_temp        = zeros(3, 1);
        Torque_Drag_temp        = zeros(3, 1);
        Torque_Rotation_temp    = zeros(3, 1);
        Torque_AM_temp          = zeros(3, 1);

        for i = 1:N_elements % Loop over each wing element
            %% Lift
            % Element-wise Calculation
            element(i).force_Lift(j) = 0.5 * rho * del_r * c(i) * C_L(j) * element(i).linear_vel_norm(j).^2;
            
            % Force Direction
            % Force is normal to the velocity direction via 2008 Dickson
            Lift_Direction = rotx(sign(element(i).linear_vel(3))*pi/2)*element(i).linear_vel_direction(:,j)*sign(Kinematics.phi_d(j)); 
            
            % Element force
            Force_element = element(i).force_Lift(j).*Lift_Direction;
            
            % Force for entire Element
            Force_Lift_temp = Force_Lift_temp + Force_element;
            
            % Torque for entire Element
            Torque_Lift_temp = Torque_Lift_temp + cross(element(i).locationInMovingFrame(1:3,j),Force_element);

            %% Drag
            % Element-wise Calculation
            element(i).force_Drag(j) = 0.5 * rho * del_r * c(i) * C_D(j) * element(i).linear_vel_norm(j).^2;
            
            % Force Direction 
            % Force is opposite the velocity direction via 2008 Dickson
            Drag_Direction = element(i).linear_vel_direction(:,j)*sign(Kinematics.phi_d(j)); 

            % Element force
            Force_element = element(i).force_Drag(j).*Drag_Direction;
            
            % Force for entire Element
            Force_Drag_temp = Force_Drag_temp + Force_element;
            
            % Torque for entire Element
            Torque_Drag_temp = Torque_Drag_temp + cross(element(i).locationInMovingFrame(1:3,j),Force_element);

            %% Rotational
            % Element-wise rotational force magnitude
            element(i).force_Rotation(j) = C_r * rho * c(i)^2 * del_r * element(i).linear_vel_norm(j) * Kinematics.psi_d(j);

            % Force direction 
            % Force is normal to wing surface, in velocity direction via 2008 Dickinson
            Rotation_Direction = [0; 1; 0];

            % Compute rotational force vector for the element
            Force_element = element(i).force_Rotation(j) * Rotation_Direction;

            % Force for entire Elemente
            Force_Rotation_temp = Force_Rotation_temp + Force_element;

            % Torque for entire Element
            Torque_Rotation_temp = Torque_Rotation_temp + cross(element(i).locationInMovingFrame(:, j), Force_element);

            %% Added Mass
            % Added mass force magnitude
            part1 = (rho * pi * c(i)^2 / 4) * del_r;
            part2 = (dot(element(i).linear_vel(:, j), element(i).linear_acc(:, j)) * sin(abs(Kinematics.psi(j)))) / element(i).linear_vel_norm(j);
            part3 = element(i).linear_vel_norm(j) * Kinematics.psi_d(j) * cos(abs(Kinematics.psi(j)));

            element(i).force_AddedMass(j) = part1 * (part2 + part3);

            % Force direction 
            % Force is opposite to angle of attack direction
            Added_Mass_Direction = -[0; 1; 0]; 

            % Compute added mass force vector for the element
            Force_element = element(i).force_AddedMass(j) * Added_Mass_Direction;

            % Force for entire Element
            Force_AM_temp = Force_AM_temp + Force_element;

            % Torque for entire Element
            Torque_AM_temp = Torque_AM_temp + cross(element(i).locationInMovingFrame(:, j), Force_element);
        end

        %% Store Total Forces and Torques for Time Step
        Force_Lift(:, j)        = Force_Lift_temp;
        Force_Drag(:, j)        = Force_Drag_temp;
        Force_Rotation(:, j)    = Force_Rotation_temp;
        Force_AM(:, j)          = Force_AM_temp;
        Torque_Lift(:, j)       = Torque_Lift_temp;
        Torque_Drag(:, j)       = Torque_Drag_temp;
        Torque_Rotation(:, j)   = Torque_Rotation_temp;
        Torque_AM(:, j)         = Torque_AM_temp;

    end
    %% Store Total Forces and Torques in Component Structure
    Components.Force_Lift       = Force_Lift;
    Components.Force_Drag       = Force_Drag;
    Components.Force_Rotation   = Force_Rotation;
    Components.Force_AM         = Force_AM;
    Components.Force_Total      = Force_Lift + Force_Drag + Force_Rotation + Force_AM;
    Components.Torque_Lift      = Torque_Lift;
    Components.Torque_Drag      = Torque_Drag;
    Components.Torque_Rotation  = Torque_Rotation;
    Components.Torque_AM        = Torque_AM;

    %% Ending Message
    disp('Component Calculation - End');
end
