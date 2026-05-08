%% Uncomment to Clear Everything
clear all
clc
%close all
warning off


%% Runtime
current_time = datetime;

%% Run Selector
Data_Set_Selector = 'Robot_1';  % Wing Damage
%Data_Set_Selector = 'Robot_2';  % Wing Stroke Amplitude
%Data_Set_Selector = 'Robot_3';  % Single Robot Simulator

% Flies compensate for unilateral wing damage through modular
% adjustments of wing and Morphology.Body kinematics, Figure 1d
% Supplemental Material: Dataset S2
% (Michael H. Dickinson et. al.) 2017


% Load Data
load(['Data_Sets' filesep Data_Set_Selector filesep 'Inputs' filesep 'Kinematics.mat']);
load(['Data_Sets' filesep Data_Set_Selector filesep 'Inputs' filesep 'Fly_Data.mat']);





%% Program Runner
for i=1:length(Fly_Data.Stroke_Amplitude_LH)

    % Flip RH to LH data
    if Fly_Data.Chord_Cut_RH(i) < 100 || Fly_Data.Span_Cut_RH(i) < 100
        [Fly_Data.Chord_Cut_LH(i),        Fly_Data.Chord_Cut_RH(i)]        = deal(Fly_Data.Chord_Cut_RH(i),        Fly_Data.Chord_Cut_LH(i));
        [Fly_Data.Span_Cut_LH(i),         Fly_Data.Span_Cut_RH(i)]         = deal(Fly_Data.Span_Cut_RH(i),         Fly_Data.Span_Cut_LH(i));
        [Fly_Data.Stroke_Amplitude_LH(i), Fly_Data.Stroke_Amplitude_RH(i)] = deal(Fly_Data.Stroke_Amplitude_RH(i), Fly_Data.Stroke_Amplitude_LH(i));
        [Fly_Data.Wing_Plane_Angle_LH(i), Fly_Data.Wing_Plane_Angle_RH(i)] = deal(Fly_Data.Wing_Plane_Angle_RH(i), Fly_Data.Wing_Plane_Angle_LH(i));
        [FilteredAngleL,                  FilteredAngleR]                  = deal(FilteredAngleR,                  FilteredAngleL);
    end

    % Cut kinematics for each time period
    [~, peak_index] = findpeaks(-FilteredAngleL(:,1));

    Time = peak_index(1):peak_index(2);

    Kinematics_LH = FilteredAngleL(Time,:);
    Kinematics_RH = FilteredAngleR(Time,:);

    % Time stamps are indexs and need converted to time domain based on the
    % frame rate
    Frame_Rate = 28000;
    dt = 1/Frame_Rate;

    % Run Analysis
    Fly_Master(i).Fly = Analysis(Fly_Data.Chord_Cut_LH(i), Fly_Data.Span_Cut_LH(i), Fly_Data.Chord_Cut_RH(i), Fly_Data.Span_Cut_RH(i), ...
        Fly_Data.Stroke_Amplitude_LH(i), Fly_Data.Stroke_Amplitude_RH(i), Fly_Data.Wing_Plane_Angle_LH(i), Fly_Data.Wing_Plane_Angle_RH(i), ...
        Fly_Data.Body_Angle(i), Kinematics_LH, Kinematics_RH, Time, dt, true);

    % Save Features
    Fly_Master(i).Chord_Cut_LH          = Fly_Data.Chord_Cut_LH(i);
    Fly_Master(i).Span_Cut_LH           = Fly_Data.Span_Cut_LH(i);
    Fly_Master(i).Chord_Cut_RH          = Fly_Data.Chord_Cut_RH(i);
    Fly_Master(i).Span_Cut_RH           = Fly_Data.Span_Cut_RH(i);
    Fly_Master(i).Wing_Plane_Angle_LH   = Fly_Data.Wing_Plane_Angle_LH(i);
    Fly_Master(i).Wing_Plane_Angle_RH   = Fly_Data.Wing_Plane_Angle_RH(i);
    Fly_Master(i).Stroke_Amplitude_LH   = Fly_Data.Stroke_Amplitude_LH(i);
    Fly_Master(i).Stroke_Amplitude_RH   = Fly_Data.Stroke_Amplitude_RH(i);

end

%% Save data
% Save the Fly_Data table to the specified location
% Define the path and filename
saveFolder = fullfile('Data_Sets', Data_Set_Selector, 'Outputs');
saveFile = 'Fly_Master.mat';

% Optionally create the folder if it doesn't exist
if ~exist(saveFolder, 'dir')
    mkdir(saveFolder);
end

% Save the Fly_Master variable
save(fullfile(saveFolder, saveFile), 'Fly_Master');

%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 2017 Data Figure S_2
Stroke_Position = [100, 94, 88, 82, 76];
Force_Stroke_x = [0, -0.01, 0.02, 0.03, 0.04];
Force_Stroke_y = [0.15, 0.1, .09, .06, .08];
Force_Stroke_z = [1, .95, .9, .85, .8];
Torque_Stroke_Yaw = [0, 0.01, -0.005, -0.015, -0.02];
Torque_Stroke_Roll = [0, 0.055, .13, .2, .25];
Torque_Stroke_Pitch = [0, -0.005, -0.01, -0.04, -0.055];

Damage_chord_Position = [.98, .92, .8, .7, .52];
Force_Damage_chord_x = [0.03, -.01, .02, .05, .07];
Force_Damage_chord_y = [0.15, 0.145, 0.14, 0.12, 0.1];
Force_Damage_chord_z = [0.98, 0.92, 0.88, 0.82, 0.74];
Torque_Damage_chord_Yaw = [0.015, 0.01, 0.009, -0.01, -0.015];
Torque_Damage_chord_Roll = [.001, 0.04, .09, .15, .22];
Torque_Damage_chord_Pitch = [.002, .001, .005, .009, .01];

Damage_span_Position = [.8, .53, .34, .18, .1];
Force_Damage_span_x = [0.01, 0.03, .06, .07, .1];
Force_Damage_span_y = [0.12, 0.07, 0.05, 0.04, 0.03];
Force_Damage_span_z = [0.9, 0.77, 0.57, 0.49, 0.44];
Torque_Damage_span_Yaw = [-0.01, -0.02, -.026, -0.04, -0.035];
Torque_Damage_span_Roll = [.11, 0.25, .35, .41, .43];
Torque_Damage_span_Pitch = [.02, .03, .05, .06, .055];


%% Force means

for i=1:length(Fly_Master)
    S_2_Ratio(i) = Fly_Master(i).Fly.Morphology.total.S_2_Ratio;
    Force_X_mean(i) = (mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(1,:)) + mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(1,:)))/Fly_Master(i).Fly.Morphology.total.weight;
    Force_Y_mean(i) = -((mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(2,:)) + mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(2,:)))/Fly_Master(i).Fly.Morphology.total.weight);
    Force_Z_mean(i) = ((mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(3,:)) + mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(3,:)))/Fly_Master(i).Fly.Morphology.total.weight);
end

%% Torques means

Pitch_offset = (mean((Fly_Master(end).Fly.Dynamics.Frame_Body.LH.Torque_Total(1,:) + Fly_Master(end).Fly.Dynamics.Frame_Body.RH.Torque_Total(1,:)) / (Fly_Master(end).Fly.Morphology.total.weight * (Fly_Master(end).Fly.Morphology.Wing_LH.wing_length+Fly_Master(end).Fly.Morphology.Wing_RH.wing_length)/2)));

for i=1:length(Fly_Master)
    S_3_Ratio(i) = Fly_Master(i).Fly.Morphology.total.S_3_Ratio;
    Moment_Pitch_mean(i) = -(Pitch_offset - mean((Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total(1,:) + Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total(1,:)) / (Fly_Master(i).Fly.Morphology.total.weight * (Fly_Master(i).Fly.Morphology.Wing_LH.wing_length+Fly_Master(i).Fly.Morphology.Wing_RH.wing_length)/2)));
    Moment_Roll_mean(i) = (mean((Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total(2,:) + Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total(2,:)) / (Fly_Master(i).Fly.Morphology.total.weight * (Fly_Master(i).Fly.Morphology.Wing_LH.wing_length+Fly_Master(i).Fly.Morphology.Wing_RH.wing_length)/2)));
    Moment_Yaw_mean(i) = -(mean((Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total(3,:) + Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total(3,:)) / (Fly_Master(i).Fly.Morphology.total.weight * (Fly_Master(i).Fly.Morphology.Wing_LH.wing_length+Fly_Master(i).Fly.Morphology.Wing_RH.wing_length)/2)));
end

%% S_2 versus force
figure
hold on
plot(S_2_Ratio,Force_X_mean,'Color',[1, 0.5, 0])
plot(S_2_Ratio,Force_Y_mean,'Color',"g")
plot(S_2_Ratio,Force_Z_mean,'Color',"b")

scatter(Damage_chord_Position, Force_Damage_chord_x, 'd', 'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
scatter(Damage_chord_Position, Force_Damage_chord_y, 'd', 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
scatter(Damage_chord_Position, Force_Damage_chord_z, 'd', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');

scatter(Damage_span_Position, Force_Damage_span_x, 'o', 'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
scatter(Damage_span_Position, Force_Damage_span_y, 'o', 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
scatter(Damage_span_Position, Force_Damage_span_z, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');

legend(["X" "Y" "Z"])
ylabel("Normalized Forces (F/mg)")
xlabel("Second moment of area Ration S_2")
% axis([.5 1 0 1])
hold off
%% S_3 versus torque
figure
hold on
plot(S_3_Ratio,Moment_Roll_mean,'Color',[1, 0.5, 0])
plot(S_3_Ratio,Moment_Pitch_mean,'Color',"g")
plot(S_3_Ratio,Moment_Yaw_mean,'Color',"b")

plot([0, 1], [0, 0], '--', 'LineWidth', 1.5, 'Color', [0.6, 0.6, 0.6]); % Dashed line

scatter(Damage_chord_Position, Torque_Damage_chord_Roll, 'd', 'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
scatter(Damage_chord_Position, Torque_Damage_chord_Pitch, 'd', 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
scatter(Damage_chord_Position, Torque_Damage_chord_Yaw, 'd', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');

scatter(Damage_span_Position, Torque_Damage_span_Roll, 'o', 'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
scatter(Damage_span_Position, Torque_Damage_span_Pitch, 'o', 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
scatter(Damage_span_Position, Torque_Damage_span_Yaw, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');

legend(["Roll" "Pitch" "Yaw"])
ylabel("Normalized Torques (T/mgl)")
xlabel("Third moment of area Ration S_3")
% axis([.5 1 -.05 .25])
hold off
%% Stroke Amplitude versus force
figure
hold on
plot(Fly_Master.Stroke_Amplitude_LH,Force_X_mean,'Color',[1, 0.5, 0])
plot(Fly_Master.Stroke_Amplitude_LH,Force_Y_mean,'Color',"g")
plot(Fly_Master.Stroke_Amplitude_LH,Force_Z_mean,'Color',"b")

plot([75, 100], [1, 1], '--', 'LineWidth', 1.5, 'Color', [0.6, 0.6, 0.6]); % Dashed line

scatter(Stroke_Position, Force_Stroke_x, 's', 'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
scatter(Stroke_Position, Force_Stroke_y, 's', 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
scatter(Stroke_Position, Force_Stroke_z, 's', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');

legend(["X" "Y" "Z"])
ylabel("Normalized Forces (F/mg)")
xlabel("Stroke Amplitude")
% axis([.5 1 0 1])
hold off

%% Stroke Amplitude versus torque

figure
hold on
plot(Fly_Master.Stroke_Amplitude_LH,Moment_Roll_mean,'Color',[1, 0.5, 0])
plot(Fly_Master.Stroke_Amplitude_LH,Moment_Pitch_mean,'Color',"g")
plot(Fly_Master.Stroke_Amplitude_LH,Moment_Yaw_mean,'Color',"b")

plot([75, 100], [0, 0], '--', 'LineWidth', 1.5, 'Color', [0.6, 0.6, 0.6]); % Dashed line

scatter(Stroke_Position, Torque_Stroke_Roll, 's', 'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
scatter(Stroke_Position, Torque_Stroke_Pitch, 's', 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
scatter(Stroke_Position, Torque_Stroke_Yaw, 's', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');

legend(["Roll" "Pitch" "Yaw"])
ylabel("Normalized Torques (T/mgl)")
xlabel("Stroke Amplitude")
%axis([.5 1 0 1])
hold off

%% Run Time End
Duration = datetime-current_time