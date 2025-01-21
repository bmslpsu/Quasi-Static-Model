    Fly_num = 19;
    Fly = Fly_Master(Fly_num).Fly;
    Kinematics_LH = Fly.Kinematics_LH;
    Kinematics_RH = Fly.Kinematics_RH;

    time_normalized = Fly.time;

    Fly.force_total.Force_Body_LH.force_total_vec = Fly.force_total.Force_Body_LH.force_total_vec(:,:);
    Fly.force_total.Force_Body_RH.force_total_vec = Fly.force_total.Force_Body_RH.force_total_vec(:,:);

    figure
    subplot(4,1,1)
    hold on
    plot(time_normalized, rad2deg(Kinematics_LH.phi), "m")
    plot(time_normalized, rad2deg(Kinematics_LH.psi), "k")
    plot(time_normalized, rad2deg(Kinematics_LH.beta), "g")
    plot(time_normalized, rad2deg(Kinematics_RH.phi), "m--")
    plot(time_normalized, rad2deg(Kinematics_RH.psi), "k--")
    plot(time_normalized, rad2deg(Kinematics_RH.beta), "g--")
    plot(time_normalized, zeros(length(Kinematics_LH.phi)),"r:")
    title('Stroke angle')
    ylabel('Angle (deg)')
    legend(["LH - Phi" "LH - Psi" "LH - Beta" "RH - Phi" "RH - Psi" "RH - Beta"])
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,2)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(2,:) + Fly.force_total.Force_Body_RH.force_total_vec(2,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_total_vec(2,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_total_vec(2,:) / Fly.total.weight, "b")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(2,:) + Fly.force_total.Force_Body_RH.force_total_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_total_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_total_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    ylabel('Vertical Force (F_z/mg)')
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,3)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(1,:) + Fly.force_total.Force_Body_RH.force_total_vec(1,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_total_vec(1,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_total_vec(1,:) / Fly.total.weight, "b")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(1,:) + Fly.force_total.Force_Body_RH.force_total_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_total_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_total_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    ylabel('Forward Force (F_y/mg)')
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,4)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(3,:) - Fly.force_total.Force_Body_RH.force_total_vec(3,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_total_vec(3,:) / Fly.total.weight, "r")
    plot(time_normalized, -Fly.force_total.Force_Body_RH.force_total_vec(3,:) / Fly.total.weight, "b")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(3,:) - Fly.force_total.Force_Body_RH.force_total_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, -mean(Fly.force_total.Force_Body_LH.force_total_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_total_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    ylabel('Side Force (F_x/mg)')
    xlabel('Wingbeat Cycles')
    legend(["Total" "Left" "Right"])
    hold off

    %% LH Individual Forces

    figure
    subplot(4,1,1)
    hold on
    plot(time_normalized, rad2deg(Kinematics_LH.phi), "m")
    plot(time_normalized, rad2deg(Kinematics_LH.psi), "k")
    plot(time_normalized, rad2deg(Kinematics_LH.beta), "g")
    plot(time_normalized, rad2deg(Kinematics_RH.phi), "m--")
    plot(time_normalized, rad2deg(Kinematics_RH.psi), "k--")
    plot(time_normalized, rad2deg(Kinematics_RH.beta), "g--")
    plot(time_normalized, zeros(length(Kinematics_LH.phi)),"r:")
    title('Stroke angle')
    ylabel('Angle (deg)')
    legend(["LH - Phi" "LH - Psi" "LH - Beta" "RH - Phi" "RH - Psi" "RH - Beta"])
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,2)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(2,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_lift_vec(2,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_drag_vec(2,:) / Fly.total.weight, "b")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_Rot_vec(2,:) / Fly.total.weight, "k")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_AM_vec(2,:) / Fly.total.weight, "y")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_lift_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_drag_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_Rot_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'k--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_AM_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'y--')
    ylabel('Vertical Force (F_z/mg)')
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,3)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(1,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_lift_vec(1,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_drag_vec(1,:) / Fly.total.weight, "b")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_Rot_vec(1,:) / Fly.total.weight, "k")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_AM_vec(1,:) / Fly.total.weight, "y")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_lift_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_drag_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_Rot_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'k--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_AM_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'y--')
    ylabel('Forward Force (F_y/mg)')
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,4)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(3,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_lift_vec(3,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_drag_vec(3,:) / Fly.total.weight, "b")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_Rot_vec(3,:) / Fly.total.weight, "k")
    plot(time_normalized, Fly.force_total.Force_Body_LH.force_AM_vec(3,:) / Fly.total.weight, "y")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_lift_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_drag_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_Rot_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'k--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_AM_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'y--')
    ylabel('Side Force (F_x/mg)')
    xlabel('Wingbeat Cycles')
    legend(["Total" "Lift" "Drag" "Rotation" "Added Mass"])
    hold off

    %% RH Individual Forces
    figure
    subplot(4,1,1)
    hold on
    plot(time_normalized, rad2deg(Kinematics_LH.phi), "m")
    plot(time_normalized, rad2deg(Kinematics_LH.psi), "k")
    plot(time_normalized, rad2deg(Kinematics_LH.beta), "g")
    plot(time_normalized, rad2deg(Kinematics_RH.phi), "m--")
    plot(time_normalized, rad2deg(Kinematics_RH.psi), "k--")
    plot(time_normalized, rad2deg(Kinematics_RH.beta), "g--")
    plot(time_normalized, zeros(length(Kinematics_LH.phi)),"r:")
    title('Stroke angle')
    ylabel('Angle (deg)')
    legend(["LH - Phi" "LH - Psi" "LH - Beta" "RH - Phi" "RH - Psi" "RH - Beta"])
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,2)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_total_vec(2,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_lift_vec(2,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_drag_vec(2,:) / Fly.total.weight, "b")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_Rot_vec(2,:) / Fly.total.weight, "k")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_AM_vec(2,:) / Fly.total.weight, "y")
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_total_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_lift_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_drag_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_Rot_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'k--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_AM_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'y--')
    ylabel('Vertical Force (F_z/mg)')
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,3)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_total_vec(1,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_lift_vec(1,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_drag_vec(1,:) / Fly.total.weight, "b")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_Rot_vec(1,:) / Fly.total.weight, "k")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_AM_vec(1,:) / Fly.total.weight, "y")
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_total_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_lift_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_drag_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_Rot_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'k--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_AM_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'y--')
    ylabel('Forward Force (F_y/mg)')
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,4)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_total_vec(3,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_lift_vec(3,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_drag_vec(3,:) / Fly.total.weight, "b")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_Rot_vec(3,:) / Fly.total.weight, "k")
    plot(time_normalized, Fly.force_total.Force_Body_RH.force_AM_vec(3,:) / Fly.total.weight, "y")
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_total_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_lift_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_drag_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_Rot_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'k--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.force_AM_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'y--')
    ylabel('Side Force (F_x/mg)')
    xlabel('Wingbeat Cycles')
    legend(["Total" "Lift" "Drag" "Rotation" "Added Mass"])
    hold off

    %% Diffrences in Individual Forces (Upward)
    figure
    subplot(5,1,1)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(2,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_total_vec(2,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_total_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(2,:) - Fly.force_total.Force_Body_RH.force_total_vec(2,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(2,:) - Fly.force_total.Force_Body_RH.force_total_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    title('Upward')
    ylabel('Total Force (F_z/mg)')
    legend(["LH" "RH" "Diffrence"])
    set(gca, 'XColor', 'none')
    hold off

    subplot(5,1,2)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_lift_vec(2,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_lift_vec(2,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_lift_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_lift_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_lift_vec(2,:) - Fly.force_total.Force_Body_RH.force_lift_vec(2,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_lift_vec(2,:) - Fly.force_total.Force_Body_RH.force_lift_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    ylabel('Lift Force')
    set(gca, 'XColor', 'none')
    hold off

    subplot(5,1,3)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_drag_vec(2,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_drag_vec(2,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_drag_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_drag_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_drag_vec(2,:) - Fly.force_total.Force_Body_RH.force_drag_vec(2,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_drag_vec(2,:) - Fly.force_total.Force_Body_RH.force_drag_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    ylabel('Drag Force')
    set(gca, 'XColor', 'none')
    hold off
    
    subplot(5,1,4)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_Rot_vec(2,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_Rot_vec(2,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_Rot_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_Rot_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_Rot_vec(2,:) - Fly.force_total.Force_Body_RH.force_Rot_vec(2,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_Rot_vec(2,:) - Fly.force_total.Force_Body_RH.force_Rot_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    ylabel('Rotational Force')
    set(gca, 'XColor', 'none')
    hold off

    subplot(5,1,5)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_AM_vec(2,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_AM_vec(2,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_AM_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_AM_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_AM_vec(2,:) - Fly.force_total.Force_Body_RH.force_AM_vec(2,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_AM_vec(2,:) - Fly.force_total.Force_Body_RH.force_AM_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    ylabel('Added Mass Force')
    xlabel('Wingbeat Cycles')
    set(gca, 'XColor', 'none')
    hold off

     %% Diffrences in Individual Forces (Forward)
    figure
    subplot(5,1,1)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(1,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_total_vec(1,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_total_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(1,:) - Fly.force_total.Force_Body_RH.force_total_vec(1,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(1,:) - Fly.force_total.Force_Body_RH.force_total_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    title('Forward')
    ylabel('Total Force (F_z/mg)')
    legend(["LH" "RH" "Diffrence"])
    set(gca, 'XColor', 'none')
    hold off

    subplot(5,1,2)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_lift_vec(1,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_lift_vec(1,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_lift_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_lift_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_lift_vec(1,:) - Fly.force_total.Force_Body_RH.force_lift_vec(1,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_lift_vec(1,:) - Fly.force_total.Force_Body_RH.force_lift_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    ylabel('Lift Force')
    set(gca, 'XColor', 'none')
    hold off

    subplot(5,1,3)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_drag_vec(1,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_drag_vec(1,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_drag_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_drag_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_drag_vec(1,:) - Fly.force_total.Force_Body_RH.force_drag_vec(1,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_drag_vec(1,:) - Fly.force_total.Force_Body_RH.force_drag_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    ylabel('Drag Force')
    set(gca, 'XColor', 'none')
    hold off
    
    subplot(5,1,4)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_Rot_vec(1,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_Rot_vec(1,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_Rot_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_Rot_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_Rot_vec(1,:) - Fly.force_total.Force_Body_RH.force_Rot_vec(1,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_Rot_vec(1,:) - Fly.force_total.Force_Body_RH.force_Rot_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    ylabel('Rotational Force')
    set(gca, 'XColor', 'none')
    hold off

    subplot(5,1,5)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_AM_vec(1,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_AM_vec(1,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_AM_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_AM_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_AM_vec(1,:) - Fly.force_total.Force_Body_RH.force_AM_vec(1,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_AM_vec(1,:) - Fly.force_total.Force_Body_RH.force_AM_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    ylabel('Added Mass Force')
    xlabel('Wingbeat Cycles')
    set(gca, 'XColor', 'none')
    hold off

     %% Diffrences in Individual Forces (Sideward)
    figure
    subplot(5,1,1)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(3,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_total_vec(3,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_total_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(3,:) - Fly.force_total.Force_Body_RH.force_total_vec(3,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(3,:) - Fly.force_total.Force_Body_RH.force_total_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    title('Sideward')
    ylabel('Total Force (F_z/mg)')
    legend(["LH" "RH" "Diffrence"])
    set(gca, 'XColor', 'none')
    hold off

    subplot(5,1,2)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_lift_vec(3,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_lift_vec(3,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_lift_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_lift_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_lift_vec(3,:) - Fly.force_total.Force_Body_RH.force_lift_vec(3,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_lift_vec(3,:) - Fly.force_total.Force_Body_RH.force_lift_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    ylabel('Lift Force')
    set(gca, 'XColor', 'none')
    hold off

    subplot(5,1,3)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_drag_vec(3,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_drag_vec(3,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_drag_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_drag_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_drag_vec(3,:) - Fly.force_total.Force_Body_RH.force_drag_vec(3,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_drag_vec(3,:) - Fly.force_total.Force_Body_RH.force_drag_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    ylabel('Drag Force')
    set(gca, 'XColor', 'none')
    hold off
    
    subplot(5,1,4)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_Rot_vec(3,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_Rot_vec(3,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_Rot_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_Rot_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_Rot_vec(3,:) - Fly.force_total.Force_Body_RH.force_Rot_vec(3,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_Rot_vec(3,:) - Fly.force_total.Force_Body_RH.force_Rot_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    ylabel('Rotational Force')
    set(gca, 'XColor', 'none')
    hold off

    subplot(5,1,5)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_AM_vec(3,:)) / Fly.total.weight, "b")
    plot(time_normalized, (Fly.force_total.Force_Body_RH.force_AM_vec(3,:)) / Fly.total.weight, "r")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_AM_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean((Fly.force_total.Force_Body_RH.force_AM_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, (Fly.force_total.Force_Body_LH.force_AM_vec(3,:) - Fly.force_total.Force_Body_RH.force_AM_vec(3,:)) / Fly.total.weight, "m")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_AM_vec(3,:) - Fly.force_total.Force_Body_RH.force_AM_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    ylabel('Added Mass Force')
    xlabel('Wingbeat Cycles')
    set(gca, 'XColor', 'none')
    hold off


    %% Diffrences in Individual Forces (Norm)
    figure
    subplot(5,1,1)
    hold on
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_LH.force_total_vec(:,:)) / Fly.total.weight), "b")
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_RH.force_total_vec(:,:)) / Fly.total.weight), "r")
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_LH.force_total_vec(:,:) - Fly.force_total.Force_Body_RH.force_total_vec(:,:)) / Fly.total.weight), "m")
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_LH.force_total_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_RH.force_total_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_LH.force_total_vec(:,:) - Fly.force_total.Force_Body_RH.force_total_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'm--')
    title('Norm Vector')
    ylabel('Total Force (F/mg)')
    legend(["LH" "RH" "Diffrence"])
    set(gca, 'XColor', 'none')
    hold off

    subplot(5,1,2)
    hold on
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_LH.force_lift_vec(:,:)) / Fly.total.weight), "b")
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_RH.force_lift_vec(:,:)) / Fly.total.weight), "r")
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_LH.force_lift_vec(:,:) - Fly.force_total.Force_Body_RH.force_lift_vec(:,:)) / Fly.total.weight), "m")
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_LH.force_lift_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_RH.force_lift_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_LH.force_lift_vec(:,:) - Fly.force_total.Force_Body_RH.force_lift_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'm--')
    ylabel('Lift Force')
    set(gca, 'XColor', 'none')
    hold off

    subplot(5,1,3)
    hold on
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_LH.force_drag_vec(:,:)) / Fly.total.weight), "b")
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_RH.force_drag_vec(:,:)) / Fly.total.weight), "r")
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_LH.force_drag_vec(:,:) - Fly.force_total.Force_Body_RH.force_drag_vec(:,:)) / Fly.total.weight), "m")
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_LH.force_drag_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_RH.force_drag_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_LH.force_drag_vec(:,:) - Fly.force_total.Force_Body_RH.force_drag_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'm--')
    ylabel('Drag Force')
    set(gca, 'XColor', 'none')
    hold off
    
    subplot(5,1,4)
    hold on
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_LH.force_Rot_vec(:,:)) / Fly.total.weight), "b")
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_RH.force_Rot_vec(:,:)) / Fly.total.weight), "r")
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_LH.force_Rot_vec(:,:) - Fly.force_total.Force_Body_RH.force_Rot_vec(:,:)) / Fly.total.weight), "m")
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_LH.force_Rot_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_RH.force_Rot_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_LH.force_Rot_vec(:,:) - Fly.force_total.Force_Body_RH.force_Rot_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'm--')
    ylabel('Rotational Force')
    set(gca, 'XColor', 'none')
    hold off

    subplot(5,1,5)
    hold on
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_LH.force_AM_vec(:,:)) / Fly.total.weight), "b")
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_RH.force_AM_vec(:,:)) / Fly.total.weight), "r")
    plot(time_normalized, vecnorm((Fly.force_total.Force_Body_LH.force_AM_vec(:,:) - Fly.force_total.Force_Body_RH.force_AM_vec(:,:)) / Fly.total.weight), "m")
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_LH.force_AM_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_RH.force_AM_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(vecnorm((Fly.force_total.Force_Body_LH.force_AM_vec(:,:) - Fly.force_total.Force_Body_RH.force_AM_vec(:,:)) / Fly.total.weight)) * ones(size(time_normalized)), 'm--')
    ylabel('Added Mass Force')
    xlabel('Wingbeat Cycles')
    set(gca, 'XColor', 'none')
    hold off
