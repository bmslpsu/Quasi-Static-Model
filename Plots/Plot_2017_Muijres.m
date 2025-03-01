%% Preamble
% Validation plots from 2017 Muijres paper

%% Plot of Stroke Angle and Torques (Normalized)

% Normalize the time vector to the range [0, 100]
time_normalized = (time(1:end) - min(time(1:end))) / (max(time(1:end)) - min(time(1:end))) * 100;

Fly.Dynamics.Frame_Body.LH.Torque_Total = Fly.Dynamics.Frame_Body.LH.Torque_Total(:,:);
Fly.Dynamics.Frame_Body.RH.Torque_Total = Fly.Dynamics.Frame_Body.RH.Torque_Total(:,:);

figure
subplot(4,1,1)
hold on
plot(time_normalized, rad2deg(Kinematics.LH.phi), "m")
plot(time_normalized, rad2deg(Kinematics.LH.psi), "k")
plot(time_normalized, rad2deg(Kinematics.LH.beta), "g")
plot(time_normalized, rad2deg(Kinematics.RH.phi), "m--")
plot(time_normalized, rad2deg(Kinematics.RH.psi), "k--")
plot(time_normalized, rad2deg(Kinematics.RH.beta), "g--")
plot(time_normalized, zeros(length(Kinematics.LH.phi)),"r:")
title('Stroke angle')
ylabel('Angle (deg)')
legend(["LH - Phi" "LH - Psi" "LH - Beta" "RH - Phi" "RH - Psi" "RH - Beta"])
set(gca, 'XColor', 'none')
hold off

subplot(4,1,2)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Torque_Total(2,:) + Fly.Dynamics.Frame_Body.RH.Torque_Total(2,:)) / (Fly.Morphology.total.weight * (Morphology.Wing_LH.wing_length+Morphology.Wing_RH.wing_length)/2), "m")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Total(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "r")
plot(time_normalized, Fly.Dynamics.Frame_Body.RH.Torque_Total(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_RH.wing_length), "b")
plot(time_normalized, mean((Fly.Dynamics.Frame_Body.LH.Torque_Total(2,:) + Fly.Dynamics.Frame_Body.RH.Torque_Total(2,:)) / (Fly.Morphology.total.weight * (Morphology.Wing_LH.wing_length+Morphology.Wing_RH.wing_length)/2)) * ones(size(time_normalized)), 'm--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Total(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'r--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.RH.Torque_Total(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_RH.wing_length)) * ones(size(time_normalized)), 'b--')
ylabel('Yaw (T_z/mg)')
set(gca, 'XColor', 'none')
% ylim([-1 1]);
hold off

subplot(4,1,3)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Torque_Total(1,:) + Fly.Dynamics.Frame_Body.RH.Torque_Total(1,:)) / (Fly.Morphology.total.weight * (Morphology.Wing_LH.wing_length+Morphology.Wing_RH.wing_length)/2), "m")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Total(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "r")
plot(time_normalized, Fly.Dynamics.Frame_Body.RH.Torque_Total(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_RH.wing_length), "b")
plot(time_normalized, mean((Fly.Dynamics.Frame_Body.LH.Torque_Total(1,:) + Fly.Dynamics.Frame_Body.RH.Torque_Total(1,:)) / (Fly.Morphology.total.weight * (Morphology.Wing_LH.wing_length+Morphology.Wing_RH.wing_length)/2)) * ones(size(time_normalized)), 'm--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Total(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'r--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.RH.Torque_Total(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_RH.wing_length)) * ones(size(time_normalized)), 'b--')
ylabel('Roll (T_x/mg)')
set(gca, 'XColor', 'none')
% ylim([-1 1]);
hold off

subplot(4,1,4)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Torque_Total(3,:) + Fly.Dynamics.Frame_Body.RH.Torque_Total(3,:)) / (Fly.Morphology.total.weight * (Morphology.Wing_LH.wing_length+Morphology.Wing_RH.wing_length)/2), "m")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Total(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "r")
plot(time_normalized, Fly.Dynamics.Frame_Body.RH.Torque_Total(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_RH.wing_length), "b")
plot(time_normalized, mean((Fly.Dynamics.Frame_Body.LH.Torque_Total(3,:) + Fly.Dynamics.Frame_Body.RH.Torque_Total(3,:)) / (Fly.Morphology.total.weight * (Morphology.Wing_LH.wing_length+Morphology.Wing_LH.wing_length)/2)) * ones(size(time_normalized)), 'm--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Total(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'r--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.RH.Torque_Total(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_RH.wing_length)) * ones(size(time_normalized)), 'b--')
ylabel('Pitch (T_y/mg)')
%xlabel('Wingbeat Cycle (%)')
xlabel('Wingbeat Cycles')
legend(["Total" "Left" "Right"])
hold off

%% Plot of Stroke Angle and Forces (Normalized)


% Normalize the time vector to the range [0, 100]
time_normalized = (time(1:end) - min(time(1:end))) / (max(time(1:end)) - min(time(1:end))) * 100;

Fly.Dynamics.Frame_Body.LH.Force_Total = Fly.Dynamics.Frame_Body.LH.Force_Total(:,:);
Fly.Dynamics.Frame_Body.RH.Force_Total = Fly.Dynamics.Frame_Body.RH.Force_Total(:,:);

figure
subplot(4,1,1)
hold on
plot(time_normalized, rad2deg(Kinematics.LH.phi), "m")
plot(time_normalized, rad2deg(Kinematics.LH.psi), "k")
plot(time_normalized, rad2deg(Kinematics.LH.beta), "g")
plot(time_normalized, rad2deg(Kinematics.RH.phi), "m--")
plot(time_normalized, rad2deg(Kinematics.RH.psi), "k--")
plot(time_normalized, rad2deg(Kinematics.RH.beta), "g--")
plot(time_normalized, zeros(length(Kinematics.LH.phi)),"r:")
title('Stroke angle')
ylabel('Angle (deg)')
legend(["LH - Phi" "LH - Psi" "LH - Beta" "RH - Phi" "RH - Psi" "RH - Beta"])
set(gca, 'XColor', 'none')
hold off

subplot(4,1,2)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Force_Total(3,:) + Fly.Dynamics.Frame_Body.RH.Force_Total(3,:)) / Fly.Morphology.total.weight, "m")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Total(3,:) / Fly.Morphology.total.weight, "r")
plot(time_normalized, Fly.Dynamics.Frame_Body.RH.Force_Total(3,:) / Fly.Morphology.total.weight, "b")
plot(time_normalized, mean((Fly.Dynamics.Frame_Body.LH.Force_Total(3,:) + Fly.Dynamics.Frame_Body.RH.Force_Total(3,:)) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'm--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_Total(3,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'r--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.RH.Force_Total(3,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'b--')
ylabel('Vertical Force (F_z/mg)')
set(gca, 'XColor', 'none')
hold off

subplot(4,1,3)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Force_Total(2,:) + Fly.Dynamics.Frame_Body.RH.Force_Total(2,:)) / Fly.Morphology.total.weight, "m")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Total(2,:) / Fly.Morphology.total.weight, "r")
plot(time_normalized, Fly.Dynamics.Frame_Body.RH.Force_Total(2,:) / Fly.Morphology.total.weight, "b")
plot(time_normalized, mean((Fly.Dynamics.Frame_Body.LH.Force_Total(2,:) + Fly.Dynamics.Frame_Body.RH.Force_Total(2,:)) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'm--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_Total(2,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'r--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.RH.Force_Total(2,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'b--')
ylabel('Forward Force (F_y/mg)')
set(gca, 'XColor', 'none')
hold off

subplot(4,1,4)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Force_Total(1,:) + Fly.Dynamics.Frame_Body.RH.Force_Total(1,:)) / Fly.Morphology.total.weight, "m")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Total(1,:) / Fly.Morphology.total.weight, "r")
plot(time_normalized, Fly.Dynamics.Frame_Body.RH.Force_Total(1,:) / Fly.Morphology.total.weight, "b")
plot(time_normalized, mean((Fly.Dynamics.Frame_Body.LH.Force_Total(1,:) + Fly.Dynamics.Frame_Body.RH.Force_Total(1,:)) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'm--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_Total(1,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'r--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.RH.Force_Total(1,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'b--')
ylabel('Side Force (F_x/mg)')
%xlabel('Wingbeat Cycle (%)')
xlabel('Wingbeat Cycles')
legend(["Total" "Left" "Right"])
hold off


%% Plot of Stroke Angle vs Each Force (Both Normalized)


% Normalize the time vector to the range [0, 100]
time_normalized = (time(1:end) - min(time(1:end))) / (max(time(1:end)) - min(time(1:end))) * 100;


Fly.Dynamics.Frame_Body.LH.Force_Total = Fly.Dynamics.Frame_Body.LH.Force_Total(:,:);
Fly.Dynamics.Frame_Body.LH.Force_Lift = Fly.Dynamics.Frame_Body.LH.Force_Lift(:,:);
Fly.Dynamics.Frame_Body.LH.Force_Drag = Fly.Dynamics.Frame_Body.LH.Force_Drag(:,:);
Fly.Dynamics.Frame_Body.LH.Force_Rotation = Fly.Dynamics.Frame_Body.LH.Force_Rotation(:,:);
Fly.Dynamics.Frame_Body.LH.Force_AM = Fly.Dynamics.Frame_Body.LH.Force_AM(:,:);

figure
subplot(4,1,1)
hold on
plot(time_normalized, rad2deg(Kinematics.LH.phi), "m")
plot(time_normalized, rad2deg(Kinematics.LH.psi), "k")
plot(time_normalized, rad2deg(Kinematics.LH.beta), "g")
plot(time_normalized, rad2deg(Kinematics.RH.phi), "m--")
plot(time_normalized, rad2deg(Kinematics.RH.psi), "k--")
plot(time_normalized, rad2deg(Kinematics.RH.beta), "g--")
plot(time_normalized, zeros(length(Kinematics.LH.phi)),"r:")
title('Stroke angle')
ylabel('Angle (deg)')
legend(["LH - Phi" "LH - Psi" "LH - Beta" "RH - Phi" "RH - Psi" "RH - Beta"])
set(gca, 'XColor', 'none')
hold off

subplot(4,1,2)
hold on
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_AM(3,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'y--')
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Force_Total(3,:)) / Fly.Morphology.total.weight, "m")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Lift(3,:) / Fly.Morphology.total.weight, "r")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Drag(3,:) / Fly.Morphology.total.weight, "b")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Rotation(3,:) / Fly.Morphology.total.weight, "k")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_AM(3,:) / Fly.Morphology.total.weight, "y")
plot(time_normalized, mean((Fly.Dynamics.Frame_Body.LH.Force_Total(3,:)) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'm--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_Lift(3,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'r--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_Drag(3,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'b--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_Rotation(3,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'k--')
ylabel('Vertical Force (F_z/mg)')
set(gca, 'XColor', 'none')
hold off

subplot(4,1,3)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Force_Total(2,:)) / Fly.Morphology.total.weight, "m")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Lift(2,:) / Fly.Morphology.total.weight, "r")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Drag(2,:) / Fly.Morphology.total.weight, "b")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Rotation(2,:) / Fly.Morphology.total.weight, "k")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_AM(2,:) / Fly.Morphology.total.weight, "y")
plot(time_normalized, mean((Fly.Dynamics.Frame_Body.LH.Force_Total(2,:)) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'm--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_Lift(2,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'r--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_Drag(2,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'b--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_Rotation(2,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'k--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_AM(2,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'y--')
ylabel('Forward Force (F_y/mg)')
set(gca, 'XColor', 'none')
hold off

subplot(4,1,4)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Force_Total(1,:)) / Fly.Morphology.total.weight, "m")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Lift(1,:) / Fly.Morphology.total.weight, "r")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Drag(1,:) / Fly.Morphology.total.weight, "b")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Rotation(1,:) / Fly.Morphology.total.weight, "k")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_AM(1,:) / Fly.Morphology.total.weight, "y")
plot(time_normalized, mean((Fly.Dynamics.Frame_Body.LH.Force_Total(1,:)) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'm--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_Lift(1,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'r--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_Drag(1,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'b--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_Rotation(1,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'k--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Force_AM(1,:) / Fly.Morphology.total.weight) * ones(size(time_normalized)), 'y--')
ylabel('Side Force (F_x/mg)')
%xlabel('Wingbeat Cycle (%)')
xlabel('Wingbeat Cycles')
legend(["Total" "Lift" "Drag" "Rotation" "Added Mass"])
hold off

%% Plot of Stroke Angle vs Each Torque (Both Normalized)

% Normalize the time vector to the range [0, 100]
time_normalized = (time(1:end) - min(time(1:end))) / (max(time(1:end)) - min(time(1:end))) * 100;

figure
subplot(4,1,1)
hold on
plot(time_normalized, rad2deg(Kinematics.LH.phi), "m")
plot(time_normalized, rad2deg(Kinematics.LH.psi), "k")
plot(time_normalized, rad2deg(Kinematics.LH.beta), "g")
plot(time_normalized, rad2deg(Kinematics.RH.phi), "m--")
plot(time_normalized, rad2deg(Kinematics.RH.psi), "k--")
plot(time_normalized, rad2deg(Kinematics.RH.beta), "g--")
plot(time_normalized, zeros(length(Kinematics.LH.phi)),"r:")
title('Stroke angle')
ylabel('Angle (deg)')
legend(["LH - Phi" "LH - Psi" "LH - Beta" "RH - Phi" "RH - Psi" "RH - Beta"])
set(gca, 'XColor', 'none')
hold off

subplot(4,1,2)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Torque_Total(2,:)) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "m")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Lift(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "r")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Drag(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "b")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Rotation(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "k")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_AM(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "y")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Inertia(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "g")
plot(time_normalized, mean((Fly.Dynamics.Frame_Body.LH.Torque_Total(2,:)) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'm--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Lift(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'r--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Drag(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'b--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Rotation(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'k--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_AM(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'y--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Inertia(2,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length))* ones(size(time_normalized)), "g--")
ylabel('Yaw (T_z/mg)')
set(gca, 'XColor', 'none')
hold off

subplot(4,1,3)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Torque_Total(1,:)) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "m")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Lift(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "r")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Drag(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "b")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Rotation(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "k")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_AM(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "y")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Inertia(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "g")
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Lift(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'r--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Drag(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'b--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Rotation(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'k--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_AM(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'y--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Inertia(1,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length))* ones(size(time_normalized)), "g--")
ylabel('Roll (T_x/mg)')
set(gca, 'XColor', 'none')
hold off

subplot(4,1,4)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Torque_Total(3,:)) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "m")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Lift(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "r")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Drag(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "b")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Rotation(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "k")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_AM(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "y")
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Inertia(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), "g")
plot(time_normalized, mean((Fly.Dynamics.Frame_Body.LH.Torque_Total(3,:)) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'm--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Lift(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'r--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Drag(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'b--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Rotation(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'k--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_AM(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length)) * ones(size(time_normalized)), 'y--')
plot(time_normalized, mean(Fly.Dynamics.Frame_Body.LH.Torque_Inertia(3,:) / (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length))* ones(size(time_normalized)), "g--")
ylabel('Pitch (T_y/mg)')
%xlabel('Wingbeat Cycle (%)')
xlabel('Wingbeat Cycles')
legend(["Total" "Lift" "Drag" "Rotation" "Added Mass" "Inertia"])
hold off
