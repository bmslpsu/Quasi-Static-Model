%% Preamble
% Jacob Taylor
% Fly-by-fly selection of torques and forces with kinematics

%% Step 1: Clear and Setup
clc            % Clear command window
warning off    % Suppress all warnings
% close all    % Uncomment to close any open figures

% Snapshot of variables that existed before the script
vars_before = who;

%% Step 2: Fly Selection
% Build display strings like: "Fly #001 - DamageLeft"
flyOptions = arrayfun(@(f) sprintf('Fly #%d - %s', ...
                        f.Fly_Num, f.Attributes), Fly_Master, 'UniformOutput', false);

[selectedIdx, ok] = listdlg( ...
    'PromptString', 'Select one fly for plotting:', ...
    'ListString', flyOptions, ...
    'SelectionMode', 'single', ...
    'ListSize', [300 300], ...
    'Name', 'Select Fly');

if ~ok || isempty(selectedIdx)
    disp('No selection made. Script aborted.');
    return;
end

%% Step 3: Extract Fly Data
FlyStruct    = Fly_Master(selectedIdx);
Fly          = FlyStruct.Fly;
Morphology   = Fly.Morphology;
Kinematics   = Fly.Kinematics;   % If Kinematics not inside Fly
time         = Fly.time;         % If Time not inside Fly

% Normalize time to percent
time_normalized = (time - min(time)) / (max(time) - min(time)) * 100;

% Reshape matrices if needed
Fly.Dynamics.Frame_Body.LH.Torque_Total = Fly.Dynamics.Frame_Body.LH.Torque_Total(:,:);
Fly.Dynamics.Frame_Body.RH.Torque_Total = Fly.Dynamics.Frame_Body.RH.Torque_Total(:,:);
Fly.Dynamics.Frame_Body.LH.Force_Total  = Fly.Dynamics.Frame_Body.LH.Force_Total(:,:);
Fly.Dynamics.Frame_Body.RH.Force_Total  = Fly.Dynamics.Frame_Body.RH.Force_Total(:,:);

meanWingLen = (Morphology.Wing_LH.wing_length + Morphology.Wing_RH.wing_length) / 2;

%% Step 4: Plot Stroke Angles and Torques
figure('Name', 'Kinematics + Torque');

subplot(4,1,1)
hold on
plot(time_normalized, rad2deg(Kinematics.LH.phi), 'm')
plot(time_normalized, rad2deg(Kinematics.LH.psi), 'k')
plot(time_normalized, rad2deg(Kinematics.LH.beta), 'g')
plot(time_normalized, rad2deg(Kinematics.RH.phi), 'm--')
plot(time_normalized, rad2deg(Kinematics.RH.psi), 'k--')
plot(time_normalized, rad2deg(Kinematics.RH.beta), 'g--')
plot(time_normalized, zeros(size(time)), 'r:')
title('Stroke Angles')
ylabel('Angle (deg)')
legend({'LH - Phi', 'LH - Psi', 'LH - Beta', ...
        'RH - Phi', 'RH - Psi', 'RH - Beta'})
set(gca, 'XColor', 'none')
hold off

subplot(4,1,2)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Torque_Total(2,:) + ...
                       Fly.Dynamics.Frame_Body.RH.Torque_Total(2,:)) / ...
                       (Fly.Morphology.total.weight * meanWingLen), 'm')
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Total(2,:) / ...
                       (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), 'r')
plot(time_normalized, Fly.Dynamics.Frame_Body.RH.Torque_Total(2,:) / ...
                       (Fly.Morphology.total.weight * Morphology.Wing_RH.wing_length), 'b')
ylabel('Yaw Torque (T_z / mgl)')
set(gca, 'XColor', 'none')
hold off

subplot(4,1,3)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Torque_Total(1,:) + ...
                       Fly.Dynamics.Frame_Body.RH.Torque_Total(1,:)) / ...
                       (Fly.Morphology.total.weight * meanWingLen), 'm')
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Total(1,:) / ...
                       (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), 'r')
plot(time_normalized, Fly.Dynamics.Frame_Body.RH.Torque_Total(1,:) / ...
                       (Fly.Morphology.total.weight * Morphology.Wing_RH.wing_length), 'b')
ylabel('Roll Torque (T_x / mgl)')
set(gca, 'XColor', 'none')
hold off

subplot(4,1,4)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Torque_Total(3,:) + ...
                       Fly.Dynamics.Frame_Body.RH.Torque_Total(3,:)) / ...
                       (Fly.Morphology.total.weight * meanWingLen), 'm')
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Torque_Total(3,:) / ...
                       (Fly.Morphology.total.weight * Morphology.Wing_LH.wing_length), 'r')
plot(time_normalized, Fly.Dynamics.Frame_Body.RH.Torque_Total(3,:) / ...
                       (Fly.Morphology.total.weight * Morphology.Wing_RH.wing_length), 'b')
ylabel('Pitch Torque (T_y / mgl)')
xlabel('Wingbeat Cycle(s) (%)')
legend({'Total', 'Left', 'Right'})
hold off

%% Step 5: Plot Stroke Angles and Forces (New Figure)
figure('Name', 'Kinematics + Force');

subplot(4,1,1)
hold on
plot(time_normalized, rad2deg(Kinematics.LH.phi), 'm')
plot(time_normalized, rad2deg(Kinematics.LH.psi), 'k')
plot(time_normalized, rad2deg(Kinematics.LH.beta), 'g')
plot(time_normalized, rad2deg(Kinematics.RH.phi), 'm--')
plot(time_normalized, rad2deg(Kinematics.RH.psi), 'k--')
plot(time_normalized, rad2deg(Kinematics.RH.beta), 'g--')
plot(time_normalized, zeros(size(time)), 'r:')
title('Stroke Angles')
ylabel('Angle (deg)')
legend({'LH - Phi', 'LH - Psi', 'LH - Beta', ...
        'RH - Phi', 'RH - Psi', 'RH - Beta'})
set(gca, 'XColor', 'none')
hold off

% Vertical Force
subplot(4,1,2)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Force_Total(3,:) + ...
                       Fly.Dynamics.Frame_Body.RH.Force_Total(3,:)) / ...
                       Fly.Morphology.total.weight, 'm')
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Total(3,:) / ...
                       Fly.Morphology.total.weight, 'r')
plot(time_normalized, Fly.Dynamics.Frame_Body.RH.Force_Total(3,:) / ...
                       Fly.Morphology.total.weight, 'b')
ylabel('Vertical Force (F_z / mg)')
set(gca, 'XColor', 'none')
hold off

% Forward Force
subplot(4,1,3)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Force_Total(2,:) + ...
                       Fly.Dynamics.Frame_Body.RH.Force_Total(2,:)) / ...
                       Fly.Morphology.total.weight, 'm')
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Total(2,:) / ...
                       Fly.Morphology.total.weight, 'r')
plot(time_normalized, Fly.Dynamics.Frame_Body.RH.Force_Total(2,:) / ...
                       Fly.Morphology.total.weight, 'b')
ylabel('Forward Force (F_y / mg)')
set(gca, 'XColor', 'none')
hold off

% Side Force
subplot(4,1,4)
hold on
plot(time_normalized, (Fly.Dynamics.Frame_Body.LH.Force_Total(1,:) + ...
                       Fly.Dynamics.Frame_Body.RH.Force_Total(1,:)) / ...
                       Fly.Morphology.total.weight, 'm')
plot(time_normalized, Fly.Dynamics.Frame_Body.LH.Force_Total(1,:) / ...
                       Fly.Morphology.total.weight, 'r')
plot(time_normalized, Fly.Dynamics.Frame_Body.RH.Force_Total(1,:) / ...
                       Fly.Morphology.total.weight, 'b')
ylabel('Side Force (F_x / mg)')
xlabel('Wingbeat Cycle(s) (%)')
legend({'Total', 'Left', 'Right'})
hold off

%% Step 6: Clear Created Variables
% Get all current variables
vars_after = who;

% Determine which variables were added by the script
vars_created = setdiff(vars_after, vars_before);

% Clear only the variables created during script execution
clear(vars_created{:});

% Clear the temporary tracking variables too
clear vars_after vars_created vars_before;
