function Kinematics = Kin(Rotation, Stroke, Deviation, rad_or_deg, dt)
    %% Preamble
    % Calculate the angular position, velocity, acceleration, and dynamics
    % of a wing in a flapping wing model.
    %
    % Inputs:
    %   Rotation    -   Euler angle for wing rotation (psi)
    %   Stroke      -   Euler angle for wing stroke (phi)
    %   Deviation   -   Euler angle for wing deviation (beta)
    %   rad_or_deg  -   Input angle units (0 for degrees, 1 for radians)
    %   dt          -   Time step between data points
    %
    % Outputs:
    %   Kinematics  -   Structure containing wing kinematic properties
    %
    % Frame (Wing):
    %   x-axis is along the length of the wing (Root to Tip)
    %   y-axis is perpendicular to the surface of the wing
    %   z-axis is along the chord of the wing starting and is parallel to the abdomen of the fly

    %% 1. Precomputations
    % Number of timesteps
    N = length(Rotation);

    %% 2. Angular Position (Convert angles to radians if needed)
    if rad_or_deg == 0 % 0 = degrees, 1 = radians
        psi  = deg2rad(Rotation);
        phi  = deg2rad(Stroke);
        beta = deg2rad(Deviation);
    else
        psi  = Rotation;
        phi  = Stroke;
        beta = Deviation;
    end

    %% 3. Angular Velocity and Acceleration
    % Calculate angular velocity (first derivative) and acceleration (second derivative)
    phi_d    = deriv(phi, dt);
    psi_d    = deriv(psi, dt);
    beta_d   = deriv(beta, dt);

    phi_dd   = deriv(phi_d, dt);
    psi_dd   = deriv(psi_d, dt);
    beta_dd  = deriv(beta_d, dt);

    %% 4. Rotation Matrices in Body Frame
    % Compute rotation matrices for each timestep

    % Preallocate matrix
    R_wb = zeros(3, 3, N);
    R_bw = zeros(3, 3, N);

    % Compute rotation matrices for each timestep
    for i = 1:N
        R_x = rotz(phi(i));
        
        R_y = roty(beta(i));
        
        R_z = rotx(psi(i));
        
        % Wing to Body Frame
        R_wb(:, :, i) = R_z' * R_y' * R_x';

        % Body to wing Frame
        R_bw(:, :, i) = R_z  * R_y  * R_x;
    end

    %% 5. Angular Velocity and Acceleration in Wing Frames
    % Compute angular velocity and acceleration in the wing frame.

    % Preallocate angular velocity and acceleration
    omega = zeros(3, N);
    alpha = zeros(3, N);

    % Compute angular velocity and acceleration in the body frame
    for i = 1:N
        % Angular velocity components in the wing frame
        % Source: 2016 Wang
        omega(:, i) = [
            psi_d(i) - phi_d(i) * sin(beta(i));
            beta_d(i) * cos(psi(i)) + phi_d(i) * cos(beta(i)) * sin(psi(i));
            phi_d(i) * cos(beta(i)) * cos(psi(i)) - beta_d(i) * sin(psi(i))
            ];

        % Angular acceleration components in the wing frame
        % Source: 2016 Wang
        alpha(:, i) = [
            psi_dd(i) - phi_dd(i) * sin(beta(i)) - phi_d(i) * beta_d(i) * cos(beta(i));
            phi_dd(i) * cos(beta(i)) * sin(psi(i)) + beta_dd(i) * cos(psi(i)) - psi_d(i) * beta_d(i) * sin(psi(i)) + phi_d(i) * (psi_d(i) * cos(psi(i)) * cos(beta(i)) - beta_d(i) * sin(psi(i)) * sin(beta(i)));
            phi_dd(i) * cos(psi(i)) * cos(beta(i)) - beta_dd(i) * sin(psi(i)) - psi_d(i) * beta_d(i) * cos(psi(i)) - phi_d(i) * (psi_d(i) * cos(beta(i)) * sin(psi(i)) + beta_d(i) * cos(psi(i)) * sin(beta(i)))
            ];
    end

    % Compute Magnitudes
    omega_mag = vecnorm(omega);
    alpha_mag = vecnorm(alpha);

    % Apply low-pass filter for smoother results

    [b, a] = butter(2, 0.2, 'low'); % 2nd-order Butterworth filter

    for j = 1:3
    AoA(j,:) = filtfilt(b, a, AoA(j,:));
    omega(j,:) = filtfilt(b, a, omega(j,:));
    alpha(j,:) = filtfilt(b, a, alpha(j,:));
    end

    omega_mag = filtfilt(b, a, omega_mag);
    alpha_mag = filtfilt(b, a, alpha_mag);

    %% 6. Store results in the Kinematic structure

    Kinematics.psi          = psi;
    Kinematics.phi          = phi;
    Kinematics.beta         = beta;
    Kinematics.psi_d        = psi_d;
    Kinematics.phi_d        = phi_d;
    Kinematics.beta_d       = beta_d;
    Kinematics.psi_dd       = psi_dd;
    Kinematics.phi_dd       = phi_dd;
    Kinematics.beta_dd      = beta_dd;
    Kinematics.R_wb         = R_wb;
    Kinematics.R_bw         = R_bw;
    Kinematics.R_x          = R_x;
    Kinematics.R_y          = R_y;
    Kinematics.R_z          = R_z;
    Kinematics.AoA          = AoA;
    Kinematics.omega        = omega;
    Kinematics.alpha        = alpha;
    Kinematics.omega_mag    = omega_mag;
    Kinematics.alpha_mag    = alpha_mag;
    

end

%% Derivative Function
function dx = deriv(x, dt)
    % DERIV - Compute numerical derivative using central difference method.
    dx          = zeros(size(x));
    dx_temp     = diff(x) / dt;
    dx(1)       = dx_temp(1); % Forward difference at the start
    dx(end)     = dx_temp(end); % Backward difference at the end
    dx(2:end-1) = 0.5 * (dx_temp(1:end-1) + dx_temp(2:end)); % Central difference
end

