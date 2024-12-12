function Kinematics = Kin(Rotation, Stroke, Deviation, rad_or_deg, dt)
    %% Preamble
    % KIN - Calculate the angular position, velocity, acceleration, and dynamics
    %       of a wing in a flapping wing model.
    %
    % Inputs:
    %   Rotation    -   Euler angle for wing rotation (psi)
    %   Stroke      -   Euler angle for wing stroke (phi)
    %   Deviation   -   Euler angle for wing deviation (beta)
    %   rad_or_deg  -   Input angle units (0 for degrees, 1 for radians)
    %   dt          -   Time step between data points
    %
    % Outputs:
    %   Kinematics - Struct containing position, velocity, acceleration,
    %                and angular dynamics in the body and wing frames.

    %% 1. Angular Position (Convert angles to radians if needed)
    if rad_or_deg == 0 % 0 = degrees, 1 = radians
        Kinematics.psi = deg2rad(Rotation);
        Kinematics.phi = deg2rad(Stroke);
        Kinematics.beta = deg2rad(Deviation);
    else
        Kinematics.psi = Rotation;
        Kinematics.phi = Stroke;
        Kinematics.beta = Deviation;
    end

    %% 2. Angular Velocity and Acceleration
    % Calculate angular velocity (first derivative) and acceleration (second derivative)
    Kinematics.phi_d = deriv(Kinematics.phi, dt);
    Kinematics.psi_d = deriv(Kinematics.psi, dt);
    Kinematics.beta_d = deriv(Kinematics.beta, dt);

    Kinematics.phi_dd = deriv(Kinematics.phi_d, dt);
    Kinematics.psi_dd = deriv(Kinematics.psi_d, dt);
    Kinematics.beta_dd = deriv(Kinematics.beta_d, dt);

    %% 3. Rotation Matrices in Body Frame
    % Compute rotation matrices for each timestep
    [ex1, ey1, ez1, Kinematics] = Rotation_Matrix(Kinematics);

    %% 4. Angular Velocity and Acceleration in Body and Wing Frames
    % Compute angular velocity and acceleration
    [Kinematics] = Omega_and_Alpha(ex1, ey1, ez1, Kinematics);
end

%% Derivative Function
function dx = deriv(x, dt)
    % DERIV - Compute numerical derivative using central difference method.
    dx = zeros(size(x));
    dx_temp = diff(x) / dt;
    dx(1) = dx_temp(1); % Forward difference at the start
    dx(end) = dx_temp(end); % Backward difference at the end
    dx(2:end-1) = 0.5 * (dx_temp(1:end-1) + dx_temp(2:end)); % Central difference
end

%% Rotation Matrix Function
function [ex1, ey1, ez1, Kinematics] = Rotation_Matrix(Kinematics)
    % ROTATION_MATRIX - Compute rotation matrices for each timestep.

    % Extract Euler angles
    phi = Kinematics.phi;
    beta = Kinematics.beta;
    psi = Kinematics.psi;

    % Number of timesteps
    N = length(phi);

    % Preallocate rotation matrix components
    ex1 = zeros(3, 3, N);
    ey1 = zeros(3, 3, N);
    ez1 = zeros(3, 3, N);

    % Compute rotation matrices for each timestep
    for i = 1:N
        % Rotation matrices for current timestep
        Rz = [cos(phi(i)), -sin(phi(i)), 0; sin(phi(i)), cos(phi(i)), 0; 0, 0, 1];
        Ry = [cos(beta(i)), 0, sin(beta(i)); 0, 1, 0; -sin(beta(i)), 0, cos(beta(i))];
        Rx = [1, 0, 0; 0, cos(psi(i)), -sin(psi(i)); 0, sin(psi(i)), cos(psi(i))];

        % Store inverses (transposes) for angular velocity computations
        ex1(:, :, i) = Rx';
        ey1(:, :, i) = (Ry * Rx)';
        ez1(:, :, i) = (Rz * Ry * Rx)';
        Kinematics.R_inv2(:, :, i) = (Rx * Ry * Rz)'; % Full rotation matrix
    end
end

%% Angular Dynamics Function
function [Kinematics] = Omega_and_Alpha(ex1, ey1, ez1, Kinematics)
    % OMEGA_AND_ALPHA - Compute angular velocity and acceleration in the body frame.

    % Unit vectors for the body frame
    ex = [1; 0; 0]; % Unit vector along x-axis
    ey = [0; 1; 0]; % Unit vector along y-axis
    ez = [0; 0; 1]; % Unit vector along z-axis

    % Number of timesteps
    N = length(Kinematics.phi_d);

    % Preallocate angular velocity and acceleration
    Kinematics.omega = zeros(3, N);
    Kinematics.alpha = zeros(3, N);

    % Compute angular velocity and acceleration in the body frame
    for i = 1:N
        % Angular velocity components in the body frame
        Kinematics.omega(:, i) = ...
            Kinematics.phi_d(i) * ez + ... % Component due to phi_dot
            Kinematics.beta_d(i) * ey + ... % Component due to beta_dot
            Kinematics.psi_d(i) * ex; % Component due to psi_dot
        
        % Angular acceleration components in the body frame
        Kinematics.alpha(:, i) = ...
            Kinematics.phi_dd(i) * ez + ... % Component due to phi_ddot
            Kinematics.beta_dd(i) * ey + ... % Component due to beta_ddot
            Kinematics.psi_dd(i) * ex; % Component due to psi_ddot
    end

    % Apply low-pass filter for smoother results
    [b, a] = butter(2, 0.2, 'low'); % 2nd-order Butterworth filter
    for j = 1:3
    Kinematics.omega(j,:) = filtfilt(b, a, Kinematics.omega(j,:));
    Kinematics.alpha(j,:) = filtfilt(b, a, Kinematics.alpha(j,:));
    end
    Kinematics.omega_mag = filtfilt(b, a, vecnorm(Kinematics.omega));
    Kinematics.alpha_mag = filtfilt(b, a, vecnorm(Kinematics.alpha));
end

