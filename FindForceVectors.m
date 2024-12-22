function Fly = FindForceVectors(Fly, R_inv2, ang_wing_plane)

    % Find the minimum length of force arrays to avoid indexing errors
    N=length(Fly.Lift_force(1,:));

    % Array Initialization
    Lift_vec    = zeros(3, N);
    Drag_vec    = zeros(3, N);
    AM_vec      = zeros(3, N);
    Rot_vec     = zeros(3, N);

    % Calculate the force vectors by rotating them into the proper frame
    for i = 1:N
        Rot_mat = R_inv2(:, :, i);
        Rot_wing_plane = rotx(deg2rad(ang_wing_plane));
        %Rot_mat = 1;
        Lift_vec(:, i)  = Rot_wing_plane * Rot_mat * (Fly.Lift_force(:,i));
        Drag_vec(:, i)  = Rot_wing_plane * Rot_mat * (Fly.Drag_force(:,i));
        AM_vec(:, i)    = Rot_wing_plane * Rot_mat * (Fly.AM_force(:,i));
        Rot_vec(:, i)   = Rot_wing_plane * Rot_mat * (Fly.Rot_force(:,i));
    end


    % Filter data
    % Butter filter to compare to actual collected data
    [b, a] = butter(2, 0.25, 'low');
    for j=1:3
        Lift_vec(j, :) = filtfilt(b, a, Lift_vec(j, :));
        Drag_vec(j, :) = filtfilt(b, a, Drag_vec(j, :));
        AM_vec(j, :) = filtfilt(b, a, AM_vec(j, :));
        Rot_vec(j, :) = filtfilt(b, a, Rot_vec(j, :));
    end

    % Store the force vectors in the element structure
    Fly.force_lift_vec  = Lift_vec;
    Fly.force_drag_vec  = Drag_vec;
    Fly.force_AM_vec    = AM_vec;
    Fly.force_Rot_vec   = Rot_vec;
    Fly.force_total_vec = Lift_vec + Drag_vec + AM_vec + Rot_vec;

    % Delete Unnessary data
    Fly = rmfield(Fly, {'Lift_force', 'Drag_force', 'AM_force', 'Rot_force'});

end
