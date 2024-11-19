function Fly_2 = FindTorqueVectors(Fly, R_inv2, Fly_2)

    %% Find the minimum length of force arrays to avoid indexing errors
    N=length(Fly.Lift_torque(1,:));

    %% Array Initialization
    Lift_torque_vec    = zeros(3, N);
    Drag_torque_vec    = zeros(3, N);
    AM_torque_vec    = zeros(3, N);
    Rot_torque_vec    = zeros(3, N);
    Inertia_torque_vec    = zeros(3, N);

    %% Calculate the force vectors by rotating them into the proper frame
    for i = 1:N
        Rot_mat = R_inv2(:, :, i);
        Lift_torque_vec(:, i)  = Rot_mat * (Fly.Lift_torque(:,i));
        Drag_torque_vec(:, i)  = Rot_mat * (Fly.Drag_torque(:,i));
        AM_torque_vec(:, i)  = Rot_mat * (Fly.AM_torque(:,i));
        Rot_torque_vec(:, i)  = Rot_mat * (Fly.Rot_torque(:,i));
        Inertia_torque_vec(:, i)  = Rot_mat * (Fly.Inertia_torque(:,i));
    end


    %% Filter data
    % Butter filter to compare to actual collected data
    [b, a] = butter(2, 0.25, 'low');
    for j=1:3
        Lift_torque_vec(j, :) = filtfilt(b, a, Lift_torque_vec(j, :));
        Drag_torque_vec(j, :) = filtfilt(b, a, Drag_torque_vec(j, :));
        AM_torque_vec(j, :) = filtfilt(b, a, AM_torque_vec(j, :));
        Rot_torque_vec(j, :) = filtfilt(b, a, Rot_torque_vec(j, :));
        Inertia_torque_vec(j, :) = filtfilt(b, a, Inertia_torque_vec(j, :));
        Fly_2.torque_forces_vec(j, :) = filtfilt(b, a, Fly_2.torque_forces_vec(j, :));
    end

    %% Store the force vectors in the element structure

    Fly_2.torque_Lift_vec  = Lift_torque_vec;
    Fly_2.torque_Drag_vec  = Drag_torque_vec;
    Fly_2.torque_AM_vec  = AM_torque_vec;
    Fly_2.torque_Rot_vec  = Rot_torque_vec;
    Fly_2.torque_Inertia_vec  = Inertia_torque_vec;
    Fly_2.torque_total_vec = Lift_torque_vec + Drag_torque_vec + AM_torque_vec + Rot_torque_vec + Inertia_torque_vec + Fly_2.torque_forces_vec;
end
