function Frame_Body = Torque_Body_Frame(Frame_Wing, R_inv2, Frame_Body)
    %% Preamble
    % Rotates torque vectors into the proper frame and applies filtering to smooth the data.

    %% Initialize Parameters
    N = size(Frame_Wing.Torque_Lift, 2); % Number of time steps

    % Array Initialization
    Torque_Lift             = zeros(3, N);
    Torque_Drag             = zeros(3, N);
    Torque_Rotation         = zeros(3, N);
    Torque_AM               = zeros(3, N);
    Torque_Inertia          = zeros(3, N);
    Torque_Due_to_Forces    = zeros(3, N);

    %% Rotate Torques into the Proper Frame
    for i = 1:N
        Rot_mat = R_inv2(:, :, i); % Rotation matrix for timestep i

        % Apply rotation for each torque type
        Torque_Lift(:, i)        = Rot_mat * Frame_Wing.Torque_Lift(:, i);
        Torque_Drag(:, i)        = Rot_mat * Frame_Wing.Torque_Drag(:, i);
        Torque_Rotation(:, i)    = Rot_mat * Frame_Wing.Torque_Rotation(:, i);
        Torque_AM(:, i)          = Rot_mat * Frame_Wing.Torque_AM(:, i);
        Torque_Inertia(:, i)     = Rot_mat * Frame_Wing.Torque_Inertia(:, i);
    end

    %% Filter Torque Data

    % Filter data
    % Apply a low-pass Butterworth filter for smoothing
    [b, a] = butter(2, 0.25, 'low');
    for j=1:3
        Torque_Lift(j, :)           = filtfilt(b, a, Torque_Lift(j, :));
        Torque_Drag(j, :)           = filtfilt(b, a, Torque_Drag(j, :));
        Torque_Rotation(j, :)       = filtfilt(b, a, Torque_Rotation(j, :));
        Torque_AM(j, :)             = filtfilt(b, a, Torque_AM(j, :));
        Torque_Inertia(j, :)        = filtfilt(b, a, Torque_Inertia(j, :));
        Torque_Due_to_Forces(j, :)  = filtfilt(b, a, Frame_Body.Torque_Due_to_Forces(j, :));
    end

    %% Store Rotated and Filtered Torques
    Frame_Body.Torque_Lift           = Torque_Lift;
    Frame_Body.Torque_Drag           = Torque_Drag;
    Frame_Body.Torque_AM             = Torque_AM;
    Frame_Body.Torque_Rotation       = Torque_Rotation;
    Frame_Body.Torque_Inertia        = Torque_Inertia;
    Frame_Body.Torque_Due_to_Forces  = Torque_Due_to_Forces;

    % Calculate total torque vector
    Frame_Body.Torque_Total  = Torque_Lift + Torque_Drag + Torque_AM + Torque_Rotation + Torque_Inertia + Torque_Due_to_Forces;
end
