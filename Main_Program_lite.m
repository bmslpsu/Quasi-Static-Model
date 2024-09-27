
%% Improvement Ideas
% 1. Calculate accelerations in main code ad pass the values rather than in
% the forces section
% 2. Save the data from the "filtered" kinematics for kinematics 1 to remove the complexity
% 3. Collect functions into one code

%% Note on the Axese

% For the wing
% z-axis is along the length of the wing
% y-axis is perpendicular to the surface of the wing
% x-axis is along the chord of the wing starting at the wing base and
% is parallel to the abdomen of the fly

% For the body
% z-axis is up
% x-axis is to forward
% y-axis is to the side (right positive)

%% Uncomment to Clear Everything
clear all
clc
close all
warning off


%% Runtime
current_time = datetime;
tic

%% Clear Everything
close all
warning off

%% Sig Figs
digits(4); % sets decimal point accuracy

%% Standard Constants
[metrics, ~, ~] = get_metrics();
rho = metrics.airDensity;
g = metrics.gravity;

%% Plot Choices
% Select the plots you would like to see
Plot_kinematics =  false;
Plot_kinematic_Velocity = false;
Plot_Forces_On_Wing =  false;
Plot_Angles_and_Focres = false;
Plot_Force_Over_Stroke = false;
Plot_Angles_and_Focres_Normalized = true;

%% Variable Decleration
%Creates structures to manage data throughout the program
Wing_Element_lh = struct();
Wing_Element_rh = struct();
Force_Body_lh = struct();
Force_Body_rh = struct();
Wing_Shape_lh = struct();
Wing_Shape_rh = struct();
Body_Shape = struct();
Fly = struct();


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Loads Wing Kinematic Data

% Loads previously generated data wing kinematics come from
% Flies compensate for unilateral wing damage through modular
% adjustments of wing and body kinematics, Figure 1d
% Supplemental Material: Dataset S2
% (Michael H. Dickinson et. al.) 2017
load(['Data_Sets' filesep 'Dataset_S2.mat'], 'Deviation_IntactWing', 'Rotation_IntactWing', 'Stroke_IntactWing', 'Time_norm')

%% Flip Kinematics
% Deviation_IntactWing = -Deviation_IntactWing;
% Rotation_IntactWing = -Rotation_IntactWing;
% Stroke_IntactWing = -Stroke_IntactWing;

%% Time Set Up
% This section sets the time scale up and the kinematics to one period
time = Time_norm(1:115)/250;

%Kinematic data
psi = -Rotation_IntactWing(1:115,1);
phi = -Stroke_IntactWing(1:115,1);
beta = Deviation_IntactWing(1:115,1);
%62:177 is an alternate period

%% Find the Angular Velocity
% Calculates the angular velocity using a data diffrential function.
phi_dot = diff(phi)/(time(2)-time(1));
psi_dot = diff(psi)/(time(2)-time(1));
beta_dot = diff(beta)/(time(2)-time(1));

%% Find the Angular Acceleration
% Calculates the angular acceleration using a data diffrential function.
phi_dot_dot = diff(phi_dot)/(time(2)-time(1));
psi_dot_dot = diff(psi_dot)/(time(2)-time(1));
beta_dot_dot = diff(beta_dot)/(time(2)-time(1));

%% Find Moving Vectors in Stationary Frame
% Calculate the vectors from the origin of the body frame and the
% assoicated rotation matrix. 
% Right now this does not diffrentiate from left and right wing
[ex11, ey11, ez11, R_inv2, R_2] = Find_vectors(phi, psi, beta);

%% Find Angular Velocity of each Wing with respect to Stationary Frame
% This finds the magnitude of the angluar velocity with respect to the body
% frame
% omega_mag is in deg
% omega is in deg
[omega, omega_mag, alpha, alpha_mag] = GetWingAngVel(ex11, ey11, ez11, phi_dot, psi_dot, beta_dot, phi_dot_dot, psi_dot_dot, beta_dot_dot);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Wing Selection

LH_Span_Cut = 100;
LH_Chord_Cut = 100;
RH_Span_Cut = 100;
RH_Chord_Cut = 100;

% LH and RH wing uploader
[Wing_Shape_lh, Wing_Shape_rh, Body_Shape] = wingPlotGUI(Wing_Shape_lh, Wing_Shape_rh, Body_Shape,true,LH_Span_Cut,LH_Chord_Cut,RH_Span_Cut,RH_Chord_Cut);

% Stores the data in the "Fly" structure for future analysis 
[Fly.wing_lh, Fly.wing_rh, Fly.body] = mass_and_inertia(Wing_Shape_lh,Wing_Shape_rh, Body_Shape);
Fly.total.mass = Fly.wing_lh.mass + Fly.wing_rh.mass + Fly.body.mass;
Fly.total.weight = Fly.total.mass * g; 

Fly.wing_lh.wing_shape = Wing_Shape_lh;
Fly.wing_rh.wing_shape = Wing_Shape_rh;
Fly.body.body_shape = Body_Shape;
Fly.time = time;

[Fly.wing_lh.c, Fly.wing_lh.n, Fly.wing_lh.wing_length, Fly.wing_lh.del_r] = Wing_Characteristics(Fly.wing_lh.wing_shape);
[Fly.wing_rh.c, Fly.wing_rh.n, Fly.wing_rh.wing_length, Fly.wing_rh.del_r] = Wing_Characteristics(Fly.wing_rh.wing_shape);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the Second and Third moment of area
% Calculates the 3rd moment of area
Fly.wing_lh.S_3 = thirdMomentOfArea(Fly.wing_lh.wing_length, Fly.wing_lh.c, Fly.wing_lh.n);
Fly.wing_rh.S_3 = thirdMomentOfArea(Fly.wing_rh.wing_length, Fly.wing_rh.c, Fly.wing_rh.n);
Fly.total.S_3_Ratio = Fly.wing_lh.S_3/Fly.wing_rh.S_3;

% Calculates the 2nd moment of area
Fly.wing_lh.S_2 = secondMomentOfArea(Fly.wing_lh.wing_length, Fly.wing_lh.c, Fly.wing_lh.n);
Fly.wing_rh.S_2 = secondMomentOfArea(Fly.wing_rh.wing_length, Fly.wing_rh.c, Fly.wing_rh.n);
Fly.total.S_2_Ratio = Fly.wing_lh.S_2/Fly.wing_rh.S_2;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the Location of the Center of Pressure for each Wing Element
% Calculates the center of pressure of each wing element 
Wing_Element_lh = FindDistanceOfCOP(psi, Fly.wing_lh.n, Fly.wing_lh.c, R_inv2, Fly.wing_lh.wing_length);
Wing_Element_rh = FindDistanceOfCOP(psi, Fly.wing_rh.n, Fly.wing_rh.c, R_inv2, Fly.wing_rh.wing_length);

%% Find Linear Velocity of each Element for each Time Step
% Calculates the linear velocity of each element based on the magnitude of
% the angluar velocity
Wing_Element_lh = FindLinearVelocity(Wing_Element_lh, omega, alpha); %omega instead of phi_f
Wing_Element_rh = FindLinearVelocity(Wing_Element_rh, omega, alpha); %omega instead of phi_f

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the Lift and Drag Forces Acting on Each Wing
[Fly.wing_lh.C_L, Fly.wing_lh.C_D, Wing_Element_lh, Fly.force_components.Lift_force_lh, Fly.force_components.Drag_force_lh, Fly.force_components.Force_Translation_lh] = LiftAndDragForces(Wing_Element_lh, psi, Fly.wing_lh.del_r, rho, Fly.wing_lh.c, phi_dot);
[Fly.wing_rh.C_L, Fly.wing_rh.C_D, Wing_Element_rh, Fly.force_components.Lift_force_rh, Fly.force_components.Drag_force_rh, Fly.force_components.Force_Translation_rh] = LiftAndDragForces(Wing_Element_rh, psi, Fly.wing_rh.del_r, rho, Fly.wing_rh.c, phi_dot);

%% Find the Rotational Force acting on Each Wing
[Wing_Element_lh, Fly.force_components.Rot_force_lh] = RotationalForce(Wing_Element_lh, psi_dot, Fly.wing_lh.del_r, Fly.wing_lh.c, rho, psi, time, psi_dot_dot);
[Wing_Element_rh, Fly.force_components.Rot_force_rh] = RotationalForce(Wing_Element_rh, psi_dot, Fly.wing_rh.del_r, Fly.wing_rh.c, rho, psi, time, psi_dot_dot);

%% Find Added Mass Force acting on Each Wing
[Wing_Element_lh, Fly.force_components.Am_force_lh] = AddedMassForce(Wing_Element_lh, psi_dot, psi, Fly.wing_lh.del_r, Fly.wing_lh.c, time, rho, phi_dot);
[Wing_Element_rh, Fly.force_components.Am_force_rh] = AddedMassForce(Wing_Element_rh, psi_dot, psi, Fly.wing_rh.del_r, Fly.wing_rh.c, time, rho, phi_dot);

%% Find Force Directions
Wing_Element_lh = FindForceVectors(Wing_Element_lh, R_inv2, beta, phi, phi_dot, psi, psi_dot);
Wing_Element_rh = FindForceVectors(Wing_Element_rh, R_inv2, beta, phi, phi_dot, psi, psi_dot);

%% Find the Total Forces in x, y, and z Directions
Fly.force_total.Force_Body_lh = Find_forces_XYZ(Wing_Element_lh, Force_Body_lh);
Fly.force_total.Force_Body_rh = Find_forces_XYZ(Wing_Element_rh, Force_Body_rh);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot Forces on Wing

if Plot_Forces_On_Wing == true

    %Added Mass
    figure
    hold on
    plot(Am_force_lh)
    plot(Am_force_rh)
    title('Added Mass force for the whole wing throughout a stroke')
    legend(["LH" "RH"])
    hold off

    %Lift and Drag
    figure 
    hold on
    plot(Lift_force_lh)
    plot(Drag_force_lh)
    plot(Lift_force_rh)
    plot(Drag_force_rh)
    title('Lift and Drag force for the whole wing throughout a stroke')
    legend(["Lift - LH" "Drag - LH" "Lift - RH" "Drag - RH"])
    hold off
    
    figure
    hold on
    plot(Force_Translation_lh)
    plot(Force_Translation_rh)
    title('Translation force for the whole wing throughout a stroke')
    legend(["LH" "RH"])
    hold off

    figure
    hold on
    plot(Rot_force_lh)
    plot(Rot_force_rh)
    title('Rotational for the whole wing throughout a stroke')
    legend(["LH" "RH"])
    hold off

end

%% Plot of Stroke Angle, Vertical Force, and Forward Force

if Plot_Angles_and_Focres ==  true

    figure
    subplot(3,1,1)
    hold on
    plot(time,phi)
    plot(time,phi,"--")
    title('Stroke angle')
    ylabel('Angle (deg)')
    hold off
    subplot(3,1,2)
    hold on
    plot(time(1:end-2),Fly.force_total.Force_Body_lh.force_Total(2,:))
    plot(time(1:end-2),Fly.force_total.Force_Body_rh.force_Total(2,:))
    title('Force in y-direction (Total Lift)')
    ylabel('Force (N)')
    hold off
    subplot(3,1,3)
    hold on
    plot(time(1:end-2),Fly.force_total.Force_Body_lh.force_Total(1,:))
    plot(time(1:end-2),Fly.force_total.Force_Body_rh.force_Total(1,:))
    title('Force in x-direction (Total Drag)')
    ylabel('Force (N)')
    legend(["LH" "RH"])
    hold off

end


%% Plot Figure 3 from Dickinson (2002)

if Plot_Force_Over_Stroke == true

    figure
    hold on
    plot(Fly.force_total.Force_Body_lh.force_Rot_vec(2,:), 'Color', [0.85, 0.1, 0.85])
    plot(Fly.force_total.Force_Body_lh.force_AM_vec(2,1:112), 'Color', [0, 0.5, 1])
    plot(Fly.force_components.Force_Translation_lh(:), 'g')
    plot(Fly.force_total.Force_Body_lh.force_Rot_vec(2,:) + Fly.force_total.Force_Body_lh.force_AM_vec(2,1:112) + Fly.force_components.Force_Translation_lh(2,:), 'r')
    title("Figure 3 (Lift) from Dickinson (2002)")
    subtitle("for One Period (LH)")
    legend(["Rotation" "Added Mass" "Translation" "Total"]);
    hold off
    
    figure
    hold on
    plot(Fly.force_total.Force_Body_rh.force_Rot_vec(2,:), 'Color', [0.85, 0.1, 0.85])
    plot(Fly.force_total.Force_Body_rh.force_AM_vec(2,1:112), 'Color', [0, 0.5, 1])
    plot(Fly.force_components.Force_Translation_rh(:), 'g')
    plot(Fly.force_total.Force_Body_rh.force_Rot_vec(2,:) + Fly.force_total.Force_Body_rh.force_AM_vec(2,1:112) + Fly.force_components.Force_Translation_rh(:), 'r')
    title("Figure 3 (Lift) from Dickinson (2002)")
    subtitle("for One Period (RH)")
    legend(["Rotation" "Added Mass" "Translation" "Total"]);
    hold off

end

%% Plot of Stroke Angle, Vertical Force, and Forward Force (Both Normalized)

if Plot_Angles_and_Focres_Normalized == true

    % Normalize the time vector to the range [0, 100]
    time_normalized = (time(1:end-2) - min(time(1:end-2))) / (max(time(1:end-2)) - min(time(1:end-2))) * 100;

    % Check the size of time_normalized and force vectors
    len_time = length(time_normalized);
    len_force_lh = length(Fly.force_total.Force_Body_lh.force_Total(2,:));
    len_force_rh = length(Fly.force_total.Force_Body_rh.force_Total(2,:));

    % Ensure the lengths match
    N = min([len_time, len_force_lh, len_force_rh]);

    % Truncate the vectors to match the shortest length
    time_normalized = time_normalized(1:N);
    phi = phi(1:N);
    psi = psi(1:N);
    beta = beta(1:N);

    Fly.force_total.Force_Body_lh.force_Total = Fly.force_total.Force_Body_lh.force_Total(:,1:N);
    Fly.force_total.Force_Body_rh.force_Total = Fly.force_total.Force_Body_rh.force_Total(:,1:N);

    figure
    subplot(4,1,1)
    hold on
    plot(time_normalized, phi, "m")
    plot(time_normalized, psi, "k")
    plot(time_normalized, beta, "g")
    title('Stroke angle')
    ylabel('Angle (deg)')
    legend(["Phi" "Alpha" "Gamma"])
    set(gca, 'XColor', 'none')
    hold off
    
    subplot(4,1,2)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_lh.force_Total(2,:) + Fly.force_total.Force_Body_rh.force_Total(2,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_Total(2,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_rh.force_Total(2,:) / Fly.total.weight, "b")
    plot(time_normalized, mean((Fly.force_total.Force_Body_lh.force_Total(2,:) + Fly.force_total.Force_Body_rh.force_Total(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_Total(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_rh.force_Total(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    ylabel('Vertical Force (F_z/mg)')
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,3)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_lh.force_Total(1,:) + Fly.force_total.Force_Body_rh.force_Total(1,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_Total(1,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_rh.force_Total(1,:) / Fly.total.weight, "b")
    plot(time_normalized, mean((Fly.force_total.Force_Body_lh.force_Total(1,:) + Fly.force_total.Force_Body_rh.force_Total(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_Total(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_rh.force_Total(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    ylabel('Forward Force (F_x/mg)')
    set(gca, 'XColor', 'none')
    hold off
    
    subplot(4,1,4)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_lh.force_Total(3,:) + Fly.force_total.Force_Body_rh.force_Total(3,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_Total(3,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_rh.force_Total(3,:) / Fly.total.weight, "b")
    plot(time_normalized, mean((Fly.force_total.Force_Body_lh.force_Total(3,:) + Fly.force_total.Force_Body_rh.force_Total(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_Total(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_rh.force_Total(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    ylabel('Side Force (F_y/mg)')
    xlabel('Wingbeat Cycle (%)')
    legend(["Total" "Left" "Right"])
    hold off

end




%% Plot of Stroke Angle vs Each Force (Both Normalized)

if Plot_Angles_and_Focres_Normalized == true

    % Normalize the time vector to the range [0, 100]
    time_normalized = (time(1:end-2) - min(time(1:end-2))) / (max(time(1:end-2)) - min(time(1:end-2))) * 100;

    % Find the minimum length of all relevant arrays to avoid size mismatches
    len_time = length(time_normalized);
    len_force_total = length(Fly.force_total.Force_Body_lh.force_Total(2,:));
    len_lift_vec = length(Fly.force_total.Force_Body_lh.force_lift_vec(2,:));
    len_drag_vec = length(Fly.force_total.Force_Body_lh.force_drag_vec(2,:));
    len_Rot_vec = length(Fly.force_total.Force_Body_lh.force_Rot_vec(2,:));
    len_AM_vec = length(Fly.force_total.Force_Body_lh.force_AM_vec(2,:));

    % Set the loop size to the smallest of these lengths
    N = min([len_time, len_force_total, len_lift_vec, len_drag_vec, len_Rot_vec, len_AM_vec]);

    % Truncate all arrays to the same length
    time_normalized = time_normalized(1:N);
    phi = phi(1:N);
    alpha = alpha(1:N);
    beta = beta(1:N);

    Fly.force_total.Force_Body_lh.force_Total = Fly.force_total.Force_Body_lh.force_Total(:,1:N);
    Fly.force_total.Force_Body_lh.force_lift_vec = Fly.force_total.Force_Body_lh.force_lift_vec(:,1:N);
    Fly.force_total.Force_Body_lh.force_drag_vec = Fly.force_total.Force_Body_lh.force_drag_vec(:,1:N);
    Fly.force_total.Force_Body_lh.force_Rot_vec = Fly.force_total.Force_Body_lh.force_Rot_vec(:,1:N);
    Fly.force_total.Force_Body_lh.force_AM_vec = Fly.force_total.Force_Body_lh.force_AM_vec(:,1:N);

    figure
    subplot(4,1,1)
    hold on
    plot(time_normalized, phi, "m")
    plot(time_normalized, psi, "k")
    plot(time_normalized, beta, "g")
    plot(time_normalized, zeros(length(phi)),"r:")
    title('Stroke angle')
    ylabel('Angle (deg)')
    legend(["Phi" "Alpha" "Gamma"])
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,2)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_lh.force_Total(2,:)) / Fly.total.weight, "m")
    % plot(time_normalized, (Fly.force_total.Force_Body_lh.force_lift_vec(2,:)+Fly.force_total.Force_Body_lh.force_drag_vec(2,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_lift_vec(2,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_drag_vec(2,:) / Fly.total.weight, "b")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_Rot_vec(2,:) / Fly.total.weight, "k")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_AM_vec(2,:) / Fly.total.weight, "y")
    plot(time_normalized, mean((Fly.force_total.Force_Body_lh.force_Total(2,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_lift_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_drag_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_Rot_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'k--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_AM_vec(2,:) / Fly.total.weight) * ones(size(time_normalized)), 'y--')
    ylabel('Vertical Force (F_z/mg)')
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,3)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_lh.force_Total(1,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_lift_vec(1,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_drag_vec(1,:) / Fly.total.weight, "b")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_Rot_vec(1,:) / Fly.total.weight, "k")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_AM_vec(1,:) / Fly.total.weight, "y")
    plot(time_normalized, mean((Fly.force_total.Force_Body_lh.force_Total(1,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_lift_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_drag_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_Rot_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'k--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_AM_vec(1,:) / Fly.total.weight) * ones(size(time_normalized)), 'y--')
    ylabel('Forward Force (F_x/mg)')
    set(gca, 'XColor', 'none')
    hold off

    subplot(4,1,4)
    hold on
    plot(time_normalized, (Fly.force_total.Force_Body_lh.force_Total(3,:)) / Fly.total.weight, "m")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_lift_vec(3,:) / Fly.total.weight, "r")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_drag_vec(3,:) / Fly.total.weight, "b")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_Rot_vec(3,:) / Fly.total.weight, "k")
    plot(time_normalized, Fly.force_total.Force_Body_lh.force_AM_vec(3,:) / Fly.total.weight, "y")
    plot(time_normalized, mean((Fly.force_total.Force_Body_lh.force_Total(3,:)) / Fly.total.weight) * ones(size(time_normalized)), 'm--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_lift_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'r--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_drag_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'b--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_Rot_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'k--')
    plot(time_normalized, mean(Fly.force_total.Force_Body_lh.force_AM_vec(3,:) / Fly.total.weight) * ones(size(time_normalized)), 'y--')
    ylabel('Side Force (F_y/mg)')
    xlabel('Wingbeat Cycle (%)')
    legend(["Total" "Lift" "Drag" "Rotation" "Added Mass"])
    hold off

end


%% End of Code Timer
toc

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

function force_Body = Find_forces_XYZ(element, force_Body)

    force_lift_vec = 0;
    force_drag_vec = 0;
    force_Rot_vec = 0;
    force_AM_vec = 0;
    force_Total = 0; % the total force in x y and z directions for each wing
    
    for j=1:length(element)
        force_lift_vec = force_lift_vec + element(j).force_lift_vec;
        force_drag_vec = force_drag_vec + element(j).force_drag_vec;
        force_Rot_vec = force_Rot_vec + element(j).force_Rot_vec;
        force_AM_vec = force_AM_vec + element(j).force_AM_vec;
        force_Total = force_Total + element(j).force_Rot_vec + element(j).force_AM_vec + element(j).force_lift_vec + element(j).force_drag_vec;
    end

    force_Body.force_lift_vec = force_lift_vec;
    force_Body.force_drag_vec = force_drag_vec;
    force_Body.force_Rot_vec = force_Rot_vec;
    force_Body.force_AM_vec = force_AM_vec;
    force_Body.force_Total = force_Total;
    
end