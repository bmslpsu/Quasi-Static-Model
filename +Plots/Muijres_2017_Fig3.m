%% Preamble
% Jacob Taylor
% Must run "Main_Program" first to populate Fly_Master

%% Step 1: Clear and Setup
clc            % Clear command window
warning off    % Suppress all warnings
% close all    % Uncomment to close any open figures

% Snapshot of variables that existed before the script
vars_before = who;

%% Step 2: Load Experimental Comparison Data (2017 Muijers)
% Data was collected based on estimates picked fromt he plot
load("+Plots/Data_Muijres_2017.mat");

%% Step 3: Compute Normalized Mean Forces from Each Fly
for i = 1:length(Fly_Master)
    S_2_Ratio(i) = Fly_Master(i).Fly.Morphology.total.S_2_Ratio;
    Force_X_mean(i) = (mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(1,:)) + ...
                       mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(1,:))) / ...
                       Fly_Master(i).Fly.Morphology.total.weight;
    Force_Y_mean(i) = -(mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(2,:)) + ...
                        mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(2,:))) / ...
                        Fly_Master(i).Fly.Morphology.total.weight;
    Force_Z_mean(i) = (mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(3,:)) + ...
                       mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(3,:))) / ...
                       Fly_Master(i).Fly.Morphology.total.weight;
end

%% Step 4: Compute Normalized Mean Torques
for i = 1:length(Fly_Master)
    avgWingLength = (Fly_Master(i).Fly.Morphology.Wing_LH.wing_length + ...
                     Fly_Master(i).Fly.Morphology.Wing_RH.wing_length) / 2;
    normalization = Fly_Master(i).Fly.Morphology.total.weight * avgWingLength;

    S_3_Ratio(i) = Fly_Master(i).Fly.Morphology.total.S_3_Ratio;
    Moment_Pitch_mean(i) = -mean((Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total(1,:) + ...
                                  Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total(1,:)) / normalization);
    Moment_Roll_mean(i)  =  mean((Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total(2,:) + ...
                                  Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total(2,:)) / normalization);
    Moment_Yaw_mean(i)   = -mean((Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total(3,:) + ...
                                  Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total(3,:)) / normalization);
end

%% Step 5: Plot S_2 vs Force Components
figure('Name', 'S_2 + Force');
hold on
S_2_Ratio_holder = 1:length(Fly_Master);
unique_S2 = unique(S_2_Ratio(S_2_Ratio_holder));
mean_force_x_means = zeros(size(unique_S2));
mean_force_y_means = zeros(size(unique_S2));
mean_force_z_means = zeros(size(unique_S2));

for i = 1:length(unique_S2)
    indices = find(S_2_Ratio(S_2_Ratio_holder) == unique_S2(i));
    for j = 1:length(indices)
        k = S_2_Ratio_holder(indices(j));
        mean_force_x_means(i) = mean(Force_X_mean(k));
        mean_force_y_means(i) = mean(Force_Y_mean(k));
        mean_force_z_means(i) = mean(Force_Z_mean(k));
    end
end

% Plot mean forces for each S_2 ratio
scatter(unique_S2, mean_force_x_means, 'MarkerEdgeColor', "g")
scatter(unique_S2, mean_force_y_means, 'MarkerEdgeColor', [1, 0.5, 0])
scatter(unique_S2, mean_force_z_means, 'MarkerEdgeColor', "b")


% Linear regression fits
px = polyfit(unique_S2, mean_force_x_means, 1);
py = polyfit(unique_S2, mean_force_y_means, 1);
pz = polyfit(unique_S2, mean_force_z_means, 1);

X_fit = polyval(px, unique_S2);
Y_fit = polyval(py, unique_S2);
Z_fit = polyval(pz, unique_S2);

plot(unique_S2, X_fit, 'Color', "g")
plot(unique_S2, Y_fit, 'Color', [1, 0.5, 0])
plot(unique_S2, Z_fit, 'Color', "b")

% Plot 2017 Muijers data
scatter(Data_Muijers_2017.Damage_chord_Position, Data_Muijers_2017.Force_Damage_chord_x, 'd', ...
        'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
scatter(Data_Muijers_2017.Damage_chord_Position, Data_Muijers_2017.Force_Damage_chord_y, 'd', ...
        'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
scatter(Data_Muijers_2017.Damage_chord_Position, Data_Muijers_2017.Force_Damage_chord_z, 'd', ...
        'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');

scatter(Data_Muijers_2017.Damage_span_Position, Data_Muijers_2017.Force_Damage_span_x, 'o', ...
        'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
scatter(Data_Muijers_2017.Damage_span_Position, Data_Muijers_2017.Force_Damage_span_y, 'o', ...
        'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
scatter(Data_Muijers_2017.Damage_span_Position, Data_Muijers_2017.Force_Damage_span_z, 'o', ...
        'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');

title('Normalized Aerodynamic Forces vs. Second Moment of Wing Area (S_2)', ...
      'FontWeight', 'bold', 'FontSize', 12);
ylabel("Normalized Forces (F/mg)")
xlabel("Second Moment of Area Ratio (S_2)")
legend({'Simulation Roll', 'Simulation Pitch', 'Simulation Yaw', ...
    'Simulation Fit Roll', 'Simulation Fit Pitch', 'Simulation Fit Yaw' ...
    '2017 Chord Roll', '2017 Chord Pitch', '2017 Chord Yaw', ...
    '2017 Span Roll', '2017 Span Pitch', '2017 Span Yaw'}, ...
    'Location', 'north', ...
    'NumColumns', 2);

hold off

%% Step 6: Plot S_3 vs Torque Components
figure('Name', 'S_3 + Torque');
hold on
S_3_Ratio_holder = 1:length(Fly_Master);
unique_S3 = unique(S_3_Ratio(S_3_Ratio_holder));
mean_Moment_Roll_mean  = zeros(size(unique_S3));
mean_Moment_Pitch_mean = zeros(size(unique_S3));
mean_Moment_Yaw_mean   = zeros(size(unique_S3));

for i = 1:length(unique_S3)
    indices = find(S_3_Ratio(S_3_Ratio_holder) == unique_S3(i));
    for j = 1:length(indices)
        k = S_3_Ratio_holder(indices(j));
        mean_Moment_Roll_mean(i)  = mean(Moment_Roll_mean(k));
        mean_Moment_Pitch_mean(i) = mean(Moment_Pitch_mean(k));
        mean_Moment_Yaw_mean(i)   = mean(Moment_Yaw_mean(k));
    end
end

scatter(unique_S3, mean_Moment_Roll_mean, 'MarkerEdgeColor', [1, 0.5, 0])
scatter(unique_S3, mean_Moment_Pitch_mean, 'MarkerEdgeColor', "g")
scatter(unique_S3, mean_Moment_Yaw_mean, 'MarkerEdgeColor', "b")

% Fit lines
px = polyfit(unique_S3, mean_Moment_Roll_mean, 1);
py = polyfit(unique_S3, mean_Moment_Pitch_mean, 1);
pz = polyfit(unique_S3, mean_Moment_Yaw_mean, 1);

X_fit = polyval(px, unique_S3);
Y_fit = polyval(py, unique_S3);
Z_fit = polyval(pz, unique_S3);

plot(unique_S3, X_fit, 'Color', [1, 0.5, 0])
plot(unique_S3, Y_fit, 'Color', "g")
plot(unique_S3, Z_fit, 'Color', "b")

% 2017 Muijers data
scatter(Data_Muijers_2017.Damage_chord_Position, Data_Muijers_2017.Torque_Damage_chord_Roll, 'd', ...
        'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
scatter(Data_Muijers_2017.Damage_chord_Position, Data_Muijers_2017.Torque_Damage_chord_Pitch, 'd', ...
        'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
scatter(Data_Muijers_2017.Damage_chord_Position, Data_Muijers_2017.Torque_Damage_chord_Yaw, 'd', ...
        'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');

scatter(Data_Muijers_2017.Damage_span_Position, Data_Muijers_2017.Torque_Damage_span_Roll, 'o', ...
        'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
scatter(Data_Muijers_2017.Damage_span_Position, Data_Muijers_2017.Torque_Damage_span_Pitch, 'o', ...
        'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
scatter(Data_Muijers_2017.Damage_span_Position, Data_Muijers_2017.Torque_Damage_span_Yaw, 'o', ...
        'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');

title('Normalized Aerodynamic Torques vs. Third Moment of Wing Area (S_3)', ...
      'FontWeight', 'bold', 'FontSize', 12);
ylabel("Normalized Torques (T/mgl)")
xlabel("Third Moment of Area Ratio (S_3)")
legend({'Simulation Roll', 'Simulation Pitch', 'Simulation Yaw', ...
    'Simulation Fit Roll', 'Simulation Fit Pitch', 'Simulation Fit Yaw' ...
    '2017 Chord Roll', '2017 Chord Pitch', '2017 Chord Yaw', ...
    '2017 Span Roll', '2017 Span Pitch', '2017 Span Yaw'}, ...
    'Location', 'north', ...
    'NumColumns', 2);

hold off

%% Step 7: Clear Created Variables
% Get all current variables
vars_after = who;

% Determine which variables were added by the script
vars_created = setdiff(vars_after, vars_before);

% Clear only the variables created during script execution
clear(vars_created{:});

% Clear the temporary tracking variables too
clear vars_after vars_created vars_before;
