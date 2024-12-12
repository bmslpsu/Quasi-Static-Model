function [Fly] = Analysis_Robot(LH_Chord_Cut, LH_Span_Cut, RH_Chord_Cut, RH_Span_Cut, LH_Stroke_Amplitude, RH_Stroke_Amplitude, fly_num) 
%% Note on the Axese

% For the wing
% x-axis is along the length of the wing (Root to Tip)
% y-axis is perpendicular to the surface of the wing
% z-axis is along the chord of the wing starting and is parallel to the abdomen of the fly

% For the body
% z-axis is up
% y-axis is to forward
% x-axis is to the side (right positive)

%% Uncomment to Clear Everything
% clear all
% clc
% %close all
% warning off

%% Mandatory features if not running as a function
% LH_Span_Cut = 100;
% LH_Chord_Cut = 100;
% RH_Span_Cut = 100;
% RH_Chord_Cut = 100;
% 
% LH_Stroke_Amplitude = 100;
% RH_Stroke_Amplitude = 100;
% 
% fly_num = 1;


%% Runtime
current_time = datetime;
tic

%% Sig Figs
digits(4); % sets decimal point accuracy

%% Standard Constants
[metrics, ~, ~] = get_metrics();

%% Variable Decleration
%Creates structures to manage data throughout the program
Wing_Element_LH = struct();
Wing_Element_RH = struct();
Wing_Shape_LH = struct();
Wing_Shape_RH = struct();
Body_Shape = struct();
Fly = struct();


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Loads Wing Kinematic Data

% Loads previously generated data wing kinematics come from
% Flies compensate for unilateral wing damage through modular
% adjustments of wing and body kinematics, Figure 1d
% Supplemental Material: Dataset S2
% (Michael H. Dickinson et. al.) 2017
load(['Data_Sets' filesep 'Dataset_S2.mat'], 'Deviation_IntactWing', 'Rotation_IntactWing', 'Stroke_IntactWing', 'Deviation_CutWing', 'Rotation_CutWing', 'Stroke_CutWing', 'Time_norm')


%% Time Set Up
% This section sets the time scale up and the kinematics to one period

[peak, peak_index] = findpeaks(-Stroke_IntactWing(:,1));

Period = peak_index(1):peak_index(2);
time = Time_norm(Period)/250;
dt=diff(time(1:2));

%Store time in Fly structure
Fly.time = time;


%% Kinematic data
Kinematics_LH = Kin(Rotation_CutWing(Period,1), Stroke_CutWing(Period,1).*LH_Stroke_Amplitude/100, Deviation_CutWing(Period,1), 0, dt);
Kinematics_RH = Kin(Rotation_IntactWing(Period,1), Stroke_IntactWing(Period,1).*RH_Stroke_Amplitude/100, Deviation_IntactWing(Period,1), 0, dt);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Wing and Boddy Selection

% LH and RH wing uploader
[Fly.wing_LH.wing_shape, Fly.wing_RH.wing_shape, Fly.body.body_shape, Fly.body.Joint] = wingPlotGUI(Wing_Shape_LH, Wing_Shape_RH, Body_Shape,true,LH_Span_Cut,LH_Chord_Cut,RH_Span_Cut,RH_Chord_Cut);

% Stores the data in the "Fly" structure for future analysis 
[Fly] = mass_and_inertia(Fly.wing_LH.wing_shape,Fly.wing_RH.wing_shape, Fly.body.body_shape, Fly);

[Fly.wing_LH.c, Fly.wing_LH.n, Fly.wing_LH.wing_length, Fly.wing_LH.del_r] = Wing_Characteristics(Fly.wing_LH.wing_shape);
[Fly.wing_RH.c, Fly.wing_RH.n, Fly.wing_RH.wing_length, Fly.wing_RH.del_r] = Wing_Characteristics(Fly.wing_RH.wing_shape);

%% Find the Second and Third moment of area
% Calculates the 3rd moment of area
Fly.wing_LH.S_3 = thirdMomentOfArea(Fly.wing_LH.wing_length, Fly.wing_LH.c, Fly.wing_LH.n);
Fly.wing_RH.S_3 = thirdMomentOfArea(Fly.wing_RH.wing_length, Fly.wing_RH.c, Fly.wing_RH.n);
Fly.total.S_3_Ratio = Fly.wing_LH.S_3/Fly.wing_RH.S_3;

% Calculates the 2nd moment of area
Fly.wing_LH.S_2 = secondMomentOfArea(Fly.wing_LH.wing_length, Fly.wing_LH.c, Fly.wing_LH.n);
Fly.wing_RH.S_2 = secondMomentOfArea(Fly.wing_RH.wing_length, Fly.wing_RH.c, Fly.wing_RH.n);
Fly.total.S_2_Ratio = Fly.wing_LH.S_2/Fly.wing_RH.S_2;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the Location of the Center of Pressure for each Wing Element
% Calculates the center of pressure of each wing element 
Wing_Element_LH = FindCOP(Kinematics_LH.psi, Fly.wing_LH.n, Fly.wing_LH.c, Kinematics_LH.R_inv2, Fly.wing_LH.wing_length);
Wing_Element_RH = FindCOP(Kinematics_RH.psi, Fly.wing_RH.n, Fly.wing_RH.c, Kinematics_RH.R_inv2, Fly.wing_RH.wing_length);

%% Find Linear Velocity of each Element for each Time Step
% Calculates the linear velocity of each element based on the magnitude of
% the angluar velocity
Wing_Element_LH = FindLinearVelocity(Wing_Element_LH, Kinematics_LH.omega, Kinematics_LH.alpha);
Wing_Element_RH = FindLinearVelocity(Wing_Element_RH, Kinematics_RH.omega, Kinematics_RH.alpha);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the Lift and Drag Forces Acting on Each Wing
[Wing_Element_LH, Fly.force_components.lh.Lift_force, Fly.force_components.lh.Drag_force, Fly.force_components.lh.Lift_torque, Fly.force_components.lh.Drag_torque] = Force_LiftAndDrag(Wing_Element_LH, Kinematics_LH, Fly.wing_LH.del_r, metrics.airDensity, Fly.wing_LH.c,  Fly.total.weight);
[Wing_Element_RH, Fly.force_components.rh.Lift_force, Fly.force_components.rh.Drag_force, Fly.force_components.rh.Lift_torque, Fly.force_components.rh.Drag_torque] = Force_LiftAndDrag(Wing_Element_RH, Kinematics_RH, Fly.wing_RH.del_r, metrics.airDensity, Fly.wing_RH.c,  Fly.total.weight);

%% Find the Rotational Force acting on Each Wing
[Wing_Element_LH, Fly.force_components.lh.Rot_force, Fly.force_components.lh.Rot_torque] = Force_Rotational(Wing_Element_LH, Kinematics_LH, Fly.wing_LH.del_r, Fly.wing_LH.c, metrics.airDensity, Fly.total.weight);
[Wing_Element_RH, Fly.force_components.rh.Rot_force, Fly.force_components.rh.Rot_torque] = Force_Rotational(Wing_Element_RH, Kinematics_RH, Fly.wing_RH.del_r, Fly.wing_RH.c, metrics.airDensity, Fly.total.weight);

%% Find Added Mass Force acting on Each Wing
[Wing_Element_LH, Fly.force_components.lh.AM_force, Fly.force_components.lh.AM_torque] = Force_AddedMass(Wing_Element_LH,  Kinematics_LH, Fly.wing_LH.del_r, Fly.wing_LH.c, metrics.airDensity, Fly.total.weight);
[Wing_Element_RH, Fly.force_components.rh.AM_force, Fly.force_components.rh.AM_torque] = Force_AddedMass(Wing_Element_RH,  Kinematics_RH, Fly.wing_RH.del_r, Fly.wing_RH.c, metrics.airDensity, Fly.total.weight);

%% Torque due to Inertia
[Wing_Element_LH, Fly.force_components.lh.Inertia_torque] = Torque_Inertia(Wing_Element_LH,  Kinematics_LH, Fly.wing_LH.inertia, Kinematics_LH.R_inv2);
[Wing_Element_RH, Fly.force_components.rh.Inertia_torque] = Torque_Inertia(Wing_Element_RH,  Kinematics_RH, Fly.wing_RH.inertia, Kinematics_RH.R_inv2);

%% Find Force Directions
Fly.force_total.Force_Body_LH = FindForceVectors(Fly.force_components.lh, Kinematics_LH.R_inv2);
Fly.force_total.Force_Body_RH = FindForceVectors(Fly.force_components.rh, Kinematics_RH.R_inv2);

%% Torque due to Force offset
Fly.force_total.Force_Body_LH.torque_forces_vec = Torque_Forces(Fly, Wing_Element_LH, Fly.wing_LH, Kinematics_LH.R_inv2, Fly.force_total.Force_Body_LH,1, fly_num);
Fly.force_total.Force_Body_RH.torque_forces_vec = Torque_Forces(Fly, Wing_Element_RH, Fly.wing_RH, Kinematics_RH.R_inv2, Fly.force_total.Force_Body_RH,2, fly_num);

%% Find Torque Directions
Fly.force_total.Force_Body_LH = FindTorqueVectors(Fly.force_components.lh, Kinematics_LH.R_inv2, Fly.force_total.Force_Body_LH);
Fly.force_total.Force_Body_RH = FindTorqueVectors(Fly.force_components.rh, Kinematics_RH.R_inv2, Fly.force_total.Force_Body_RH);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot Choices
% Select the plots you would like to see
Plot_Angles_and_Focres_Normalized = false;
%% Plot of Stroke Angle and Torques (Normalized)

if Plot_Angles_and_Focres_Normalized == true

    % Normalize the time vector to the range [0, 100]
    time_normalized = (time(1:end) - min(time(1:end))) / (max(time(1:end)) - min(time(1:end))) * 100;

    Fly.force_total.Force_Body_LH.torque_total_vec = Fly.force_total.Force_Body_LH.torque_total_vec(:,:);
    Fly.force_total.Force_Body_RH.torque_total_vec = Fly.force_total.Force_Body_RH.torque_total_vec(:,:);

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
    plot(time_normalized, (Fly.force_total.Force_Body_LH.torque_total_vec(2,:) - Fly.force_total.Force_Body_RH.torque_total_vec(2,:)) / (Fly.total.weight * (Fly.wing_LH.wing_length+Fly.wing_RH.wing_length)/2), "m")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_total_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "r")
    plot(time_normalized, -Fly.force_total.Force_Body_RH.torque_total_vec(2,:) / (Fly.total.weight * Fly.wing_RH.wing_length), "b")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.torque_total_vec(2,:) - Fly.force_total.Force_Body_RH.torque_total_vec(2,:)) / (Fly.total.weight * (Fly.wing_LH.wing_length+Fly.wing_RH.wing_length)/2)) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_total_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, -mean(Fly.force_total.Force_Body_RH.torque_total_vec(2,:) / (Fly.total.weight * Fly.wing_RH.wing_length)) * ones(size(time_normalized)), 'b--')
    ylabel('Yaw (T_z/mg)')
    set(gca, 'XColor', 'none')
    % ylim([-1 1]);
    hold off

    subplot(4,1,3)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.torque_total_vec(1,:) - Fly.force_total.Force_Body_RH.torque_total_vec(1,:)) / (Fly.total.weight * (Fly.wing_LH.wing_length+Fly.wing_RH.wing_length)/2), "m")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_total_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "r")
    plot(time_normalized, -Fly.force_total.Force_Body_RH.torque_total_vec(1,:) / (Fly.total.weight * Fly.wing_RH.wing_length), "b")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.torque_total_vec(1,:) - Fly.force_total.Force_Body_RH.torque_total_vec(1,:)) / (Fly.total.weight * (Fly.wing_LH.wing_length+Fly.wing_RH.wing_length)/2)) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_total_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, -mean(Fly.force_total.Force_Body_RH.torque_total_vec(1,:) / (Fly.total.weight * Fly.wing_RH.wing_length)) * ones(size(time_normalized)), 'b--')
    ylabel('Roll (T_x/mg)')
    set(gca, 'XColor', 'none')
    % ylim([-1 1]);
    hold off
    
    subplot(4,1,4)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.torque_total_vec(3,:) + Fly.force_total.Force_Body_RH.torque_total_vec(3,:)) / (Fly.total.weight * (Fly.wing_LH.wing_length+Fly.wing_RH.wing_length)/2), "m")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_total_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "r")
    plot(time_normalized, Fly.force_total.Force_Body_RH.torque_total_vec(3,:) / (Fly.total.weight * Fly.wing_RH.wing_length), "b")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.torque_total_vec(3,:) + Fly.force_total.Force_Body_RH.torque_total_vec(3,:)) / (Fly.total.weight * (Fly.wing_LH.wing_length+Fly.wing_LH.wing_length)/2)) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_total_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_RH.torque_total_vec(3,:) / (Fly.total.weight * Fly.wing_RH.wing_length)) * ones(size(time_normalized)), 'b--')
    ylabel('Pitch (T_y/mg)')
    %xlabel('Wingbeat Cycle (%)')
    xlabel('Wingbeat Cycles')
    legend(["Total" "Left" "Right"])
    ylim([-3 3]);
    hold off

end

%% Plot of Stroke Angle, Vertical Force, and Forward Force (Normalized)

if Plot_Angles_and_Focres_Normalized == true

    % Normalize the time vector to the range [0, 100]
    time_normalized = (time(1:end) - min(time(1:end))) / (max(time(1:end)) - min(time(1:end))) * 100;

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
    %xlabel('Wingbeat Cycle (%)')
    xlabel('Wingbeat Cycles')
    legend(["Total" "Left" "Right"])
    hold off

end

%% Plot of Stroke Angle vs Each Force (Both Normalized)

% if Plot_Angles_and_Focres_Normalized == true
% 
%     % Normalize the time vector to the range [0, 100]
%     time_normalized = (time(1:end) - min(time(1:end))) / (max(time(1:end)) - min(time(1:end))) * 100;
% 
% 
%     Fly.force_total.Force_Body_LH.force_total_vec = Fly.force_total.Force_Body_LH.force_total_vec(:,:);
%     Fly.force_total.Force_Body_LH.force_lift_vec = Fly.force_total.Force_Body_LH.force_lift_vec(:,:);
%     Fly.force_total.Force_Body_LH.force_drag_vec = Fly.force_total.Force_Body_LH.force_drag_vec(:,:);
%     Fly.force_total.Force_Body_LH.force_Rot_vec = Fly.force_total.Force_Body_LH.force_Rot_vec(:,:);
%     Fly.force_total.Force_Body_LH.force_AM_vec = Fly.force_total.Force_Body_LH.force_AM_vec(:,:);
% 
%     figure
%     subplot(4,1,1)
%     hold on
%     plot(time_normalized, rad2deg(Kinematics_LH.phi), "m")
%     plot(time_normalized, rad2deg(Kinematics_LH.psi), "k")
%     plot(time_normalized, rad2deg(Kinematics_LH.beta), "g")
%     plot(time_normalized, rad2deg(Kinematics_RH.phi), "m--")
%     plot(time_normalized, rad2deg(Kinematics_RH.psi), "k--")
%     plot(time_normalized, rad2deg(Kinematics_RH.beta), "g--")
%     plot(time_normalized, zeros(length(Kinematics_LH.phi)),"r:")
%     title('Stroke angle')
%     ylabel('Angle (deg)')
%     legend(["LH - Phi" "LH - Psi" "LH - Beta" "RH - Phi" "RH - Psi" "RH - Beta"])
%     set(gca, 'XColor', 'none')
%     hold off
% 
%     subplot(4,1,2)
%     hold on
%     plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(2,:)) / Fly.total.weight, "m")
%     plot(time_normalized, Fly.force_total.Force_Body_LH.force_lift_vec(2,:) / Fly.total.weight, "r")
%     plot(time_normalized, Fly.force_total.Force_Body_LH.force_drag_vec(2,:) / Fly.total.weight, "b")
%     plot(time_normalized, Fly.force_total.Force_Body_LH.force_Rot_vec(2,:) / Fly.total.weight, "k")
%     plot(time_normalized, Fly.force_total.Force_Body_LH.force_AM_vec(2,:) / Fly.total.weight, "y")
%     plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
%     plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_lift_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
%     plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_drag_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
%     plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_Rot_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'k--')
%     plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_AM_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'y--')
%     ylabel('Vertical Force (F_z/mg)')
%     set(gca, 'XColor', 'none')
%     hold off
% 
%     subplot(4,1,3)
%     hold on
%     plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(1,:)) / Fly.total.weight, "m")
%     plot(time_normalized, Fly.force_total.Force_Body_LH.force_lift_vec(1,:) / Fly.total.weight, "r")
%     plot(time_normalized, Fly.force_total.Force_Body_LH.force_drag_vec(1,:) / Fly.total.weight, "b")
%     plot(time_normalized, Fly.force_total.Force_Body_LH.force_Rot_vec(1,:) / Fly.total.weight, "k")
%     plot(time_normalized, Fly.force_total.Force_Body_LH.force_AM_vec(1,:) / Fly.total.weight, "y")
%     plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
%     plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_lift_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
%     plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_drag_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
%     plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_Rot_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'k--')
%     plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_AM_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'y--')
%     ylabel('Forward Force (F_y/mg)')
%     set(gca, 'XColor', 'none')
%     hold off
% 
%     subplot(4,1,4)
%     hold on
%     plot(time_normalized, (Fly.force_total.Force_Body_LH.force_total_vec(3,:)) / Fly.total.weight, "m")
%     plot(time_normalized, Fly.force_total.Force_Body_LH.force_lift_vec(3,:) / Fly.total.weight, "r")
%     plot(time_normalized, Fly.force_total.Force_Body_LH.force_drag_vec(3,:) / Fly.total.weight, "b")
%     plot(time_normalized, Fly.force_total.Force_Body_LH.force_Rot_vec(3,:) / Fly.total.weight, "k")
%     plot(time_normalized, Fly.force_total.Force_Body_LH.force_AM_vec(3,:) / Fly.total.weight, "y")
%     plot(time_normalized, mean((Fly.force_total.Force_Body_LH.force_total_vec(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
%     plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_lift_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
%     plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_drag_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
%     plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_Rot_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'k--')
%     plot(time_normalized, mean(Fly.force_total.Force_Body_LH.force_AM_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'y--')
%     ylabel('Side Force (F_x/mg)')
%     %xlabel('Wingbeat Cycle (%)')
%     xlabel('Wingbeat Cycles')
%     legend(["Total" "Lift" "Drag" "Rotation" "Added Mass"])
%     hold off
% 
% end

%% Plot of Stroke Angle vs Each Torque (Both Normalized)

if Plot_Angles_and_Focres_Normalized == true

    % Normalize the time vector to the range [0, 100]
    time_normalized = (time(1:end) - min(time(1:end))) / (max(time(1:end)) - min(time(1:end))) * 100;

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
    plot(time_normalized, (Fly.force_total.Force_Body_LH.torque_total_vec(2,:)) / (Fly.total.weight * Fly.wing_LH.wing_length), "m")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_Lift_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "r")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_Drag_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "b")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_Rot_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "k")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_AM_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "y")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_Inertia_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "g")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_forces_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "c")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.torque_total_vec(2,:)) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_Lift_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_Drag_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_Rot_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'k--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_AM_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'y--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_Inertia_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length))* ones(size(time_normalized)), "g--")
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_forces_vec(2,:) / (Fly.total.weight * Fly.wing_LH.wing_length))* ones(size(time_normalized)), "c--")
    ylabel('Yaw (T_z/mg)')
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,3)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.torque_total_vec(1,:)) / (Fly.total.weight * Fly.wing_LH.wing_length), "m")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_Lift_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "r")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_Drag_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "b")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_Rot_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "k")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_AM_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "y")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_Inertia_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "g")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_forces_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "c")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.torque_total_vec(1,:)) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_Lift_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_Drag_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_Rot_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'k--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_AM_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'y--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_Inertia_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length))* ones(size(time_normalized)), "g--")
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_forces_vec(1,:) / (Fly.total.weight * Fly.wing_LH.wing_length))* ones(size(time_normalized)), "c--")
    ylabel('Roll (T_x/mg)')
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,4)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_LH.torque_total_vec(3,:)) / (Fly.total.weight * Fly.wing_LH.wing_length), "m")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_Lift_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "r")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_Drag_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "b")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_Rot_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "k")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_AM_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "y")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_Inertia_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "g")
    plot(time_normalized, Fly.force_total.Force_Body_LH.torque_forces_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length), "c")
    plot(time_normalized, mean((Fly.force_total.Force_Body_LH.torque_total_vec(3,:)) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_Lift_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_Drag_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_Rot_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'k--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_AM_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length)) * ones(size(time_normalized)), 'y--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_Inertia_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length))* ones(size(time_normalized)), "g--")
    plot(time_normalized, mean(Fly.force_total.Force_Body_LH.torque_forces_vec(3,:) / (Fly.total.weight * Fly.wing_LH.wing_length))* ones(size(time_normalized)), "c--")
    ylabel('Pitch (T_y/mg)')
    %xlabel('Wingbeat Cycle (%)')
    xlabel('Wingbeat Cycles')
    legend(["Total" "Lift" "Drag" "Rotation" "Added Mass" "Inertia" "Force Offset"])
    hold off

end
%% End of Code Timer
toc
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Functions---------------------------------------------------------------

function S_2 = secondMomentOfArea(wing_length, c, n_elements)
    % Calculate the second moment of area (S_2)
    % Inputs:
    %   wing_length - Total length of the wing
    %   c - Array of chord lengths at each discrete point
    %   n_elements - Number of discrete points
    
    % Distance between points (assuming uniform spacing)
    Delta_r = wing_length / (n_elements - 1);
    
    % Calculate the second moment of area (S_2)
    S_2 = 0;
    for i = 1:n_elements
        r = (i-1) * Delta_r;
        S_2 = S_2 + c(i) * r^2 * Delta_r;
    end
end

function S_3 = thirdMomentOfArea(wing_length, c, n_elements)
    % Calculate the third moment of area (S_3)
    % Inputs:
    %   wing_length - Total length of the wing
    %   c - Array of chord lengths at each discrete point
    %   n_elements - Number of discrete points
    
    % Distance between points (assuming uniform spacing)
    Delta_r = wing_length / (n_elements - 1);
    
    % Calculate the third moment of area (S_3)
    S_3 = 0;
    for i = 1:n_elements
        r = (i-1) * Delta_r;
        S_3 = S_3 + c(i) * r^3 * Delta_r;
    end
end



