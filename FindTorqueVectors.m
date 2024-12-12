function Fly_2 = FindTorqueVectors(Fly, R_inv2, Fly_2)

    %% Preamble
    % Rotates torque vectors into the proper frame and applies filtering to smooth the data.

    %% Initialize Parameters
    N = size(Fly.Lift_torque, 2); % Number of time steps

    % Preallocate torque vectors for efficiency
    torque_types = {'Lift_torque', 'Drag_torque', 'AM_torque', 'Rot_torque', 'Inertia_torque'};
    torque_vectors = struct();

    for torque_type = torque_types
        torque_vectors.(torque_type{1}) = zeros(3, N);
    end

    %% Rotate Torques into the Proper Frame
    for i = 1:N
        Rot_mat = R_inv2(:, :, i); % Rotation matrix for timestep i

        % Apply rotation for each torque type
        torque_vectors.Lift_torque(:, i) = Rot_mat * Fly.Lift_torque(:, i);
        torque_vectors.Drag_torque(:, i) = Rot_mat * Fly.Drag_torque(:, i);
        torque_vectors.AM_torque(:, i) = Rot_mat * Fly.AM_torque(:, i);
        torque_vectors.Rot_torque(:, i) = Rot_mat * Fly.Rot_torque(:, i);
        torque_vectors.Inertia_torque(:, i) = Rot_mat * Fly.Inertia_torque(:, i);
    end

    %% Filter Torque Data
    % Apply a low-pass Butterworth filter for smoothing
    [b, a] = butter(2, 0.25, 'low');

    % Filter each torque vector
    for torque_type = torque_types
        for j = 1:3
            torque_vectors.(torque_type{1})(j, :) = filtfilt(b, a, torque_vectors.(torque_type{1})(j, :));
        end
    end

    % Filter Fly_2.torque_forces_vec
    for j = 1:3
        Fly_2.torque_forces_vec(j, :) = filtfilt(b, a, Fly_2.torque_forces_vec(j, :));
    end

    %% Store Rotated and Filtered Torques
    Fly_2.torque_Lift_vec = torque_vectors.Lift_torque;
    Fly_2.torque_Drag_vec = torque_vectors.Drag_torque;
    Fly_2.torque_AM_vec = torque_vectors.AM_torque;
    Fly_2.torque_Rot_vec = torque_vectors.Rot_torque;
    Fly_2.torque_Inertia_vec = torque_vectors.Inertia_torque;

    % Calculate total torque vector
    Fly_2.torque_total_vec = Fly_2.torque_Lift_vec + Fly_2.torque_Drag_vec + Fly_2.torque_AM_vec + Fly_2.torque_Rot_vec + Fly_2.torque_Inertia_vec + Fly_2.torque_forces_vec;
end
