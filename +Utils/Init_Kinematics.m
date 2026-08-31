function Kinematics = Init_Kinematics(stroke,rotation,deviation,period,dt)
% INIT_KINEMATICS Constructs an object storing information about angular 
% position/velocity/acceleration of a wing in a flapping wing model.
%
% Inputs:
%   stroke      -   Euler angle for LH/RH wing stroke (phi, deg)
%   rotation    -   Euler angle for LH/RH wing rotation (psi, deg)
%   deviation   -   Euler angle for LH/RH wing deviation (beta, deg)
%   period      -   Total time (s). Not to be mistaken for wingbeat period.
%   dt          -   Time step between data points (s)
%
% Outputs:
%   Kinematics  -   Structure containing wing kinematic properties
%
% Frame (Wing):
%   x-axis is along the length of the wing (Root to Tip)
%   y-axis is perpendicular to the surface of the wing
%   z-axis is along the chord of the wing starting and is parallel to the abdomen of the fly

import Utils.central_diff

% Allocate Kinematics
wing_kinematic = struct( ...
    "phi",[],"psi",[],"beta",[],"phi_d",[],"psi_d",[],"beta_d",[],...
    "phi_dd",[],"psi_dd",[],"beta_dd",[],...
    "R_wb",[],"R_bw",[],"R_x",[],"R_y",[],"R_z",[],...
    "AoA",[],"omega",[],"alpha",[],"omega_mag",[],"alpha_mag",[],"N",[]);
Kinematics = struct("LH",wing_kinematic,"RH",wing_kinematic);

% Number of timesteps (assuming all angles are equal length)
N = length(stroke);

% Loop through each wing
for side = ["LH","RH"]

    if side == "LH"
        phi = deg2rad(stroke(:,1));
        psi = deg2rad(rotation(:,1));
        beta= deg2rad(deviation(:,1));
    else % side == "RH"
        phi = deg2rad(stroke(:,2));
        psi = deg2rad(rotation(:,2));
        beta= deg2rad(deviation(:,2));
    end

    %% Angular Velocity and Acceleration
    % Calculate angular velocity (first derivative) and acceleration 
    % (second derivative)
    phi_d    = central_diff(phi) / dt;
    psi_d    = central_diff(psi) / dt;
    beta_d   = central_diff(beta)/ dt;
    
    phi_dd   = central_diff(phi_d) / dt;
    psi_dd   = central_diff(psi_d) / dt;
    beta_dd  = central_diff(beta_d)/ dt;
    
    %% Rotation Matrices in Body Frame
    % Compute rotation matrices for each timestep
    
    % Preallocate matrix
    R_wb = zeros(3, 3, N);
    R_bw = zeros(3, 3, N);
    
    % Compute rotation matrices for each timestep
    for i = 1:N
        R_x = rotz( rad2deg(phi(i)) );
        
        R_y = roty( rad2deg(beta(i)) );
        
        R_z = rotx( rad2deg(psi(i)) );
        
        % Wing to Body Frame
        R_wb(:, :, i) = R_z' * R_y' * R_x';
    
        % Body to wing Frame
        R_bw(:, :, i) = R_z  * R_y  * R_x;
    end
    
    %% Angular Velocity and Acceleration in Wing Frames
    % Compute angular velocity and acceleration in the wing frame.
    
    % Preallocate angular velocity and acceleration
    omega = zeros(3, N);
    alpha = zeros(3, N);
    AoA   = zeros(3, N);
    
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
    
    %% Store results in the Kinematic structure
    
    Kinematics.(side).N            = period;
    Kinematics.(side).phi          = phi;
    Kinematics.(side).psi          = psi;
    Kinematics.(side).beta         = beta;
    Kinematics.(side).phi_d        = phi_d;
    Kinematics.(side).psi_d        = psi_d;
    Kinematics.(side).beta_d       = beta_d;
    Kinematics.(side).phi_dd       = phi_dd;
    Kinematics.(side).psi_dd       = psi_dd;
    Kinematics.(side).beta_dd      = beta_dd;
    Kinematics.(side).R_wb         = R_wb;
    Kinematics.(side).R_bw         = R_bw;
    Kinematics.(side).R_x          = R_x;
    Kinematics.(side).R_y          = R_y;
    Kinematics.(side).R_z          = R_z;
    Kinematics.(side).AoA          = AoA;
    Kinematics.(side).omega        = omega;
    Kinematics.(side).alpha        = alpha;
    Kinematics.(side).omega_mag    = omega_mag;
    Kinematics.(side).alpha_mag    = alpha_mag;
end

end

%% References
% Q. Wang, J. F. L. Goosen, and F. Van Keulen, “A Predictive Quasi-Steady Model of Aerodynamic Loads on Flapping Wings,” J Fluid Mech, vol. 800, pp. 688–719, Aug. 2016, doi: 10.1017/jfm.2016.413.