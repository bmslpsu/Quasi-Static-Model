function [element, Dynamics, Dynamics_Body] = Dynamic_Components(Kinematics, element, Morphology, Morphology_full, rho)
    %% Preamble
    % Calculates lift, drag, rotational, and added mass forces and their corresponding 
    % torques for a flapping wing. Forces and torques are computed for each wing element
    % across multiple time steps.
    %
    % Inputs:
    %   Kinematics -    Structure containing wing kinematic properties
    %   element    -    Structure containing wing element properties
    %   Morphology -    Structure containing morphological properties
    %   rho        -    Air density
    %
    % Outputs:
    %   element    - Updated structure with force values for each element
    %   Dynamics   - Structure containing total forces and torques


    %% 1. Initialize Parameters
    % Required Kinematic Parameters
    N       = Kinematics.N;
    R_wb    = Kinematics.R_wb;
    psi     = Kinematics.psi;
    omega   = Kinematics.omega;
    alpha   = Kinematics.alpha;

    % Required Morphology Parameters
    N_elements  = Morphology.n;
    del_r       = Morphology.del_r;
    c           = Morphology.c;
    inertia     = Morphology.inertia;
    Wing_Plane_angle = Morphology.Wing_Plane_angle;

    % Required Morphology_full Parameters
    CG          = Morphology_full.total.CG;
    Joint       = Morphology_full.Body.Joint;
    Body_angle  = Morphology_full.Body.Body_angle;
    
    % Preallocate force and torque arrays for efficiency
    % Wing Frame
    Force_Lift_wing      = zeros(3, N);
    Force_Drag_wing      = zeros(3, N);
    Force_Rotation_wing  = zeros(3, N);
    Force_AM_wing        = zeros(3, N);
    Torque_Lift_wing     = zeros(3, N);
    Torque_Drag_wing     = zeros(3, N);
    Torque_Rotation_wing = zeros(3, N);
    Torque_AM_wing       = zeros(3, N);
    Torque_Inertia_wing  = zeros(3, N);

    % Body Frame
    Force_Lift_body      = zeros(3, N);
    Force_Drag_body      = zeros(3, N);
    Force_Rotation_body  = zeros(3, N);
    Force_AM_body        = zeros(3, N);
    Torque_Lift_body     = zeros(3, N);
    Torque_Drag_body     = zeros(3, N);
    Torque_Rotation_body = zeros(3, N);
    Torque_AM_body       = zeros(3, N);
    Torque_Inertia_body  = zeros(3, N);
    

    AoA = psi;

    % Rotation matrix for body angle correction
    Rot_body_angle  = rotx(deg2rad(Body_angle));

    % Rotation matrix for wing stroke plane correction
    Rot_wing_plane = rotx(deg2rad(Wing_Plane_angle));

    % Apply transformations
    CG_body     = Rot_body_angle * CG';
    CG_joint    = Rot_body_angle * Joint';

    %% 2. Compute Coefficients
    % Lift and drag coefficients are calculated based on the angle of attack
    % Source: Dickinson 1999
    % JM: Coefficients may change slighty if the wing is damaged. We should
    % consider doing a sensitivity analysis
    AoA_deg = rad2deg(AoA);
    C_L = 0.225 + 1.58 * sin(deg2rad(2.13 * abs(AoA_deg) - 7.2));   % Lift coefficient
    C_D = 1.92 - 1.55 * cos(deg2rad(2.04 * AoA_deg - 9.82));        % Drag coefficient
    C_r = 1.55;                                                     % Rotational coefficient

    %% 3. Compute Forces and Torques for Each Time Step
    for j = 1:N
        % Temporary accumulators for total force and torque at time step j
        % Wing Frame
        Force_Lift_wing_temp         = zeros(3, 1);
        Force_Drag_wing_temp         = zeros(3, 1);
        Force_Rotation_wing_temp     = zeros(3, 1);
        Force_AM_wing_temp           = zeros(3, 1);
        Torque_Lift_wing_temp        = zeros(3, 1);
        Torque_Drag_wing_temp        = zeros(3, 1);
        Torque_Rotation_wing_temp    = zeros(3, 1);
        Torque_AM_wing_temp          = zeros(3, 1);

        % Body Frame
        Force_Lift_body_temp         = zeros(3, 1);
        Force_Drag_body_temp         = zeros(3, 1);
        Force_Rotation_body_temp     = zeros(3, 1);
        Force_AM_body_temp           = zeros(3, 1);
        Torque_Lift_body_temp        = zeros(3, 1);
        Torque_Drag_body_temp        = zeros(3, 1);
        Torque_Rotation_body_temp    = zeros(3, 1);
        Torque_AM_body_temp          = zeros(3, 1);

        %% 3.1 Loop Over Each Wing Element
        for i = 1:N_elements
            % Position vector of the center of pressure for the element
            r = element(i).COP_Wing_Frame(:, j);

            % Compute CG location for the wing
            CG_wing = CG_joint + Rot_wing_plane * R_wb(:, :, j) * r;

            % Compute torque arm (CG difference)
            CG_Delta = CG_wing - CG_body;

            %% 3.1.1 Lift Force
            % Compute lift magnitude
            % Source: 2008 Dickson
            element(i).force_Lift(j) = 0.5 * rho * del_r * c(i) * C_L(j) * element(i).linear_vel_norm(j)^2;

            % Compute lift direction (normal to velocity direction)
            % Source: 2008 Dickson
            % Compute linear velocity normal
            v_linear_Normal = rotx(sign(element(i).linear_vel(2,j)) * pi/2) *element(i).linear_vel(:,j) ./ element(i).linear_vel_norm(j);


            % Compute magnitude of normal velocity
            v_linear_Norm_Normal = vecnorm(v_linear_Normal, 2, 1);

            % Avoid division by zero
            if v_linear_Norm_Normal > 1e-6
                Lift_Direction = v_linear_Normal ./ v_linear_Norm_Normal;
            else
                Lift_Direction = [0;0;0];
            end

            % Compute lift force vector
            Force_element_wing = element(i).force_Lift(j) .* Lift_Direction;

            % Accumulate force and torque contributions
            Force_Lift_wing_temp    = Force_Lift_wing_temp  + Force_element_wing;
            Torque_Lift_wing_temp   = Torque_Lift_wing_temp + cross(r, Force_element_wing);

            Force_element_Body = Rot_wing_plane * R_wb(:, :, j) * Force_element_wing;
            
            Force_Lift_body_temp    = Force_Lift_body_temp  + Force_element_Body;
            Torque_Lift_body_temp   = Torque_Lift_body_temp + cross(CG_Delta, Force_element_Body);

            %% 3.1.2 Drag Force
            % Compute drag magnitude
            % Source: 2008 Dickson
            element(i).force_Drag(j) = 0.5 * rho * del_r * c(i) * C_D(j) * element(i).linear_vel_norm(j)^2;

            % Compute drag direction (opposite velocity direction)
            % Source: 2008 Dickson
                        
            % Avoid division by zero
            if element(i).linear_vel_norm(j) > 1e-6
                Drag_Direction = element(i).linear_vel(:,j) ./ element(i).linear_vel_norm(j); 
            else
                Drag_Direction = [0;0;0];
            end

            % Compute drag force vector
            Force_element_wing = element(i).force_Drag(j) .* Drag_Direction;

            % Accumulate force and torque contributions
            Force_Drag_wing_temp    = Force_Drag_wing_temp  + Force_element_wing;
            Torque_Drag_wing_temp   = Torque_Drag_wing_temp + cross(r, Force_element_wing);

            Force_element_Body = Rot_wing_plane * R_wb(:, :, j) * Force_element_wing;
            
            Force_Drag_body_temp    = Force_Drag_body_temp  + Force_element_Body;
            Torque_Drag_body_temp   = Torque_Drag_body_temp + cross(CG_Delta, Force_element_Body);

            %% 3.1.3 Rotational Force
            % Compute rotational force magnitude
            % Source: 2008 Dickson
            element(i).force_Rotation(j) = C_r * rho * c(i)^2 * del_r * element(i).linear_vel_norm(j) * omega(1, j);

            % Compute rotational force direction (normal to wing surface)
            % Source: 2008 Dickson & 2016 Wang
            Rotation_Direction = [0; -1; 0];

            % Compute rotational force vector
            Force_element_wing = element(i).force_Rotation(j) .* Rotation_Direction;

            % Accumulate force and torque contributions
            Force_Rotation_wing_temp     = Force_Rotation_wing_temp   + Force_element_wing;
            Torque_Rotation_wing_temp    = Torque_Rotation_wing_temp  + cross(r, Force_element_wing);

            Force_element_Body = Rot_wing_plane * R_wb(:, :, j) * Force_element_wing;
            
            Force_Rotation_body_temp = Force_Rotation_body_temp   + Force_element_Body;
            Torque_Rotation_body_temp = Torque_Rotation_body_temp + cross(CG_Delta, Force_element_Body);

            %% 3.1.4 Added Mass Force
            % Compute added mass force magnitude
            % Source: 2008 Dickson
            part1 = (rho * pi * c(i)^2 / 4) * del_r;
            part2 = (dot(element(i).linear_vel(:, j), element(i).linear_acc(:, j)) * sin(abs(AoA(j)))) / element(i).linear_vel_norm(j);
            part3 = element(i).linear_vel_norm(j) * omega(1,j) * cos(abs(AoA(j)));

            element(i).force_AddedMass(j) = part1 * (part2 + part3);

            % Compute added mass force direction (normal to wing surface)
            % Source: 2008 Dickson & 2016 Wang
            Added_Mass_Direction = [0; -1; 0];

            % Compute added mass force vector
            Force_element_wing = element(i).force_AddedMass(j) .* Added_Mass_Direction;

            % Accumulate force and torque contributions
            Force_AM_wing_temp  = Force_AM_wing_temp  + Force_element_wing;
            Torque_AM_wing_temp = Torque_AM_wing_temp + cross(r, Force_element_wing);
            
            Force_element_Body  = Rot_wing_plane * R_wb(:, :, j) * Force_element_wing;
            
            Force_AM_body_temp  = Force_AM_body_temp  + Force_element_Body;
            Torque_AM_body_temp = Torque_AM_body_temp + cross(CG_Delta, Force_element_Body);    
            
        end

        %% 3.2 Compute Inertial Torques
        % Transform the inertia matrix to the global frame using rotation matrix
        Inertia_Wing = R_wb(:, :, j) * inertia * R_wb(:, :, j)';
        Inertia_Body = R_wb(:, :, j) * inertia * R_wb(:, :, j)';

        % Compute torque due to angular acceleration: T = I * α
        Torque_angular_accel = Inertia_Wing * alpha(:, j);

        % Compute gyroscopic (Coriolis) torque: T = ω × (I * ω)
        Torque_gyroscopic = cross(omega(:, j), Inertia_Wing * omega(:, j));

        % Compute total inertial torque
        Torque_Inertia_wing_temp = Torque_angular_accel + Torque_gyroscopic;


        % Compute torque due to angular acceleration: T = I * α
        Torque_angular_accel = Inertia_Body * alpha(:, j);

        % Compute gyroscopic (Coriolis) torque: T = ω × (I * ω)
        Torque_gyroscopic = cross(omega(:, j), Inertia_Body * omega(:, j));

        % Compute total inertial torque
        Torque_Inertia_body_temp = Torque_angular_accel + Torque_gyroscopic;

        %% 3.3. Store Total Forces and Torques for Current Time Step
        % Wing Frame
        Force_Lift_wing(:, j)        = Force_Lift_wing_temp;
        Force_Drag_wing(:, j)        = Force_Drag_wing_temp;
        Force_Rotation_wing(:, j)    = Force_Rotation_wing_temp;
        Force_AM_wing(:, j)          = Force_AM_wing_temp;
        Torque_Lift_wing(:, j)       = Torque_Lift_wing_temp;
        Torque_Drag_wing(:, j)       = Torque_Drag_wing_temp;
        Torque_Rotation_wing(:, j)   = Torque_Rotation_wing_temp;
        Torque_AM_wing(:, j)         = Torque_AM_wing_temp;
        Torque_Inertia_wing(:, j)    = Torque_Inertia_wing_temp;
        
        % Body Frame
        Force_Lift_body(:, j)        = Force_Lift_body_temp;
        Force_Drag_body(:, j)        = Force_Drag_body_temp;
        Force_Rotation_body(:, j)    = Force_Rotation_body_temp;
        Force_AM_body(:, j)          = Force_AM_body_temp;
        Torque_Lift_body(:, j)       = Torque_Lift_body_temp;
        Torque_Drag_body(:, j)       = Torque_Drag_body_temp;
        Torque_Rotation_body(:, j)   = Torque_Rotation_body_temp;
        Torque_AM_body(:, j)         = Torque_AM_body_temp;
        Torque_Inertia_body(:, j)    = Torque_Inertia_body_temp;
    end

    %% 4. Apply Low-Pass Filtering
%Butterworth filter to smooth  data

[b, a] = butter(2, 0.25, 'low'); % 2nd-order Butterworth filter
for j = 1:3
    Force_Lift_wing(j, :)            = filtfilt(b, a, Force_Lift_wing(j, :));
    Force_Drag_wing(j, :)            = filtfilt(b, a, Force_Drag_wing(j, :));
    Force_Rotation_wing(j, :)        = filtfilt(b, a, Force_Rotation_wing(j, :));
    Force_AM_wing(j, :)              = filtfilt(b, a, Force_AM_wing(j, :));
    Torque_Lift_wing(j, :)           = filtfilt(b, a, Torque_Lift_wing(j, :));
    Torque_Drag_wing(j, :)           = filtfilt(b, a, Torque_Drag_wing(j, :));
    Torque_Rotation_wing(j, :)       = filtfilt(b, a, Torque_Rotation_wing(j, :));
    Torque_AM_wing(j, :)             = filtfilt(b, a, Torque_AM_wing(j, :));
    Torque_Inertia_wing(j, :)        = filtfilt(b, a, Torque_Inertia_wing(j, :));

    Force_Lift_body(j, :)            = filtfilt(b, a, Force_Lift_body(j, :));
    Force_Drag_body(j, :)            = filtfilt(b, a, Force_Drag_body(j, :));
    Force_Rotation_body(j, :)        = filtfilt(b, a, Force_Rotation_body(j, :));
    Force_AM_body(j, :)              = filtfilt(b, a, Force_AM_body(j, :));
    Torque_Lift_body(j, :)           = filtfilt(b, a, Torque_Lift_body(j, :));
    Torque_Drag_body(j, :)           = filtfilt(b, a, Torque_Drag_body(j, :));
    Torque_Rotation_body(j, :)       = filtfilt(b, a, Torque_Rotation_body(j, :));
    Torque_AM_body(j, :)             = filtfilt(b, a, Torque_AM_body(j, :));
    Torque_Inertia_body(j, :)        = filtfilt(b, a, Torque_Inertia_body(j, :));

end

    %% 5. Store Results in Component Structure
    % Wing Frame
    Dynamics.Force_Lift       = Force_Lift_wing;
    Dynamics.Force_Drag       = Force_Drag_wing;
    Dynamics.Force_Rotation   = Force_Rotation_wing;
    Dynamics.Force_AM         = Force_AM_wing;
    Dynamics.Force_Total      = Force_Lift_wing + Force_Drag_wing + Force_Rotation_wing + Force_AM_wing;

    Dynamics.Torque_Lift      = Torque_Lift_wing;
    Dynamics.Torque_Drag      = Torque_Drag_wing;
    Dynamics.Torque_Rotation  = Torque_Rotation_wing;
    Dynamics.Torque_AM        = Torque_AM_wing;
    Dynamics.Torque_Inertia   = Torque_Inertia_wing;
    Dynamics.Torque_Total     = Torque_Lift_wing + Torque_Drag_wing + Torque_Rotation_wing + Torque_AM_wing + Torque_Inertia_wing;

    % Body Frame
    Dynamics_Body.Force_Lift        = Force_Lift_body;
    Dynamics_Body.Force_Drag        = Force_Drag_body;
    Dynamics_Body.Force_Rotation    = Force_Rotation_body;
    Dynamics_Body.Force_AM          = Force_AM_body;
    Dynamics_Body.Force_Total       = Force_Lift_body + Force_Drag_body + Force_Rotation_body + Force_AM_body;

    Dynamics_Body.Torque_Lift       = Torque_Lift_body;
    Dynamics_Body.Torque_Drag       = Torque_Drag_body;
    Dynamics_Body.Torque_Rotation   = Torque_Rotation_body;
    Dynamics_Body.Torque_AM         = Torque_AM_body;
    Dynamics_Body.Torque_Inertia    = Torque_Inertia_body;
    Dynamics_Body.Torque_Total      = Torque_Lift_body + Torque_Drag_body + Torque_Rotation_body + Torque_AM_body + Torque_Inertia_body;


end
