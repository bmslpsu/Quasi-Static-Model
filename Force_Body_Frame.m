function Fly = Force_Body_Frame(Fly, R_inv2, ang_wing_plane)
    %% Preamble
    % Rotates force vectors into the proper frame and applies filtering to smooth the data.

    %% Initialize Parameters
    N=length(Fly.Force_Lift(1,:));

    % Array Initialization
    Lift_vec    = zeros(3, N);
    Drag_vec    = zeros(3, N);
    AM_vec      = zeros(3, N);
    Rot_vec     = zeros(3, N);

    % Calculate the force vectors by rotating them into the proper frame
    for i = 1:N
        Rot_mat = R_inv2(:, :, i);
        Rot_wing_plane = rotx(deg2rad(ang_wing_plane));
        Lift_vec(:, i)  = Rot_wing_plane * Rot_mat * (Fly.Force_Lift(:,i));
        Drag_vec(:, i)  = Rot_wing_plane * Rot_mat * (Fly.Force_Drag(:,i));
        Rot_vec(:, i)   = Rot_wing_plane * Rot_mat * (Fly.Force_Rotation(:,i));
        AM_vec(:, i)    = Rot_wing_plane * Rot_mat * (Fly.Force_AM(:,i));
    end

    % Filter data
    % Butter filter to compare to actual collected data
    [b, a] = butter(2, 0.25, 'low');
    for j=1:3
        Lift_vec(j, :)  = filtfilt(b, a, Lift_vec(j, :));
        Drag_vec(j, :)  = filtfilt(b, a, Drag_vec(j, :));
        AM_vec(j, :)    = filtfilt(b, a, AM_vec(j, :));
        Rot_vec(j, :)   = filtfilt(b, a, Rot_vec(j, :));
    end

    % Store the force vectors
    Fly.Force_Lift      = Lift_vec;
    Fly.Force_Drag      = Drag_vec;
    Fly.Force_Rotation  = Rot_vec;
    Fly.Force_AM        = AM_vec;

    % Calculate total force vector
    Fly.Force_Total     = Lift_vec + Drag_vec + Rot_vec + AM_vec;

end
