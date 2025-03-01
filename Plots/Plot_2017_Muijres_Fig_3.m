%% Preamble
%Must run "Multi-Fly" or "Multi-Robot" first

% 2017 Data Figure S_2
Stroke_Position = [100, 94, 88, 82, 76];
Force_Stroke_x = [0.15, 0.1, .09, .06, .08];
Force_Stroke_y = [0, -0.01, 0.02, 0.03, 0.04];
Force_Stroke_z = [1, .95, .9, .85, .8];
Torque_Stroke_Yaw = [0, 0.01, -0.005, -0.015, -0.02];
Torque_Stroke_Roll = [0, 0.055, .13, .2, .25];
Torque_Stroke_Pitch = [0, -0.005, -0.01, -0.04, -0.055];

Damage_chord_Position = [.98, .92, .8, .7, .52];
Force_Damage_chord_x = [0.15, 0.145, 0.14, 0.12, 0.1];
Force_Damage_chord_y = [0.03, -.01, .02, .05, .07];
Force_Damage_chord_z = [0.98, 0.92, 0.88, 0.82, 0.74];
Torque_Damage_chord_Yaw = [0.015, 0.01, 0.009, -0.01, -0.015];
Torque_Damage_chord_Roll = [.001, 0.04, .09, .15, .22];
Torque_Damage_chord_Pitch = [.002, .001, .005, .009, .01];

Damage_span_Position = [.8, .53, .34, .18, .1];
Force_Damage_span_x = [0.12, 0.07, 0.05, 0.04, 0.03];
Force_Damage_span_y = [0.01, 0.03, .06, .07, .1];
Force_Damage_span_z = [0.9, 0.77, 0.57, 0.49, 0.44];
Torque_Damage_span_Yaw = [-0.01, -0.02, -.026, -0.04, -0.035];
Torque_Damage_span_Roll = [.11, 0.25, .35, .41, .43];
Torque_Damage_span_Pitch = [.02, .03, .05, .06, .055];

%% S_2 versus force

figure
hold on

% Extract unique S_2_Ratio values
S_2_Ratio_holder = [1:6,8:11,13:14,16:19,21:22,24:25];
unique_S2 = unique(S_2_Ratio(S_2_Ratio_holder));

% Initialize arrays to store the means for each unique S_2_Ratio
mean_force_x_means = zeros(size(unique_S2));
mean_force_y_means = zeros(size(unique_S2));
mean_force_z_means = zeros(size(unique_S2));

% Calculate the mean for each unique S_2_Ratio
for i = 1:length(unique_S2)
    % Find indices corresponding to the current unique S_2_Ratio
    indices = find(S_2_Ratio(S_2_Ratio_holder) == unique_S2(i));
    
    for j = 1:length(indices)
        k = S_2_Ratio_holder(indices(j));
    % Compute means for X, Y, and Z forces
    mean_force_x_means(i) = mean(Force_X_mean(k));
    mean_force_y_means(i) = mean(Force_Y_mean(k));
    mean_force_z_means(i) = mean(Force_Z_mean(k));
    end
end

% Scatter all individual points
scatter(unique_S2, mean_force_x_means, 'MarkerEdgeColor', "g")
scatter(unique_S2, mean_force_y_means, 'MarkerEdgeColor', [1, 0.5, 0])
scatter(unique_S2, mean_force_z_means, 'MarkerEdgeColor', "b")

% % 2017 Data
% scatter(Damage_chord_Position, Force_Damage_chord_x, 'd', 'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
% scatter(Damage_chord_Position, Force_Damage_chord_y, 'd', 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
% scatter(Damage_chord_Position, Force_Damage_chord_z, 'd', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
% 
% scatter(Damage_span_Position, Force_Damage_span_x, 'o', 'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
% scatter(Damage_span_Position, Force_Damage_span_y, 'o', 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
% scatter(Damage_span_Position, Force_Damage_span_z, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');


% Fit lines (1st-degree polynomial) to the mean data
px = polyfit(unique_S2, mean_force_x_means, 1); % X forces
py = polyfit(unique_S2, mean_force_y_means, 1); % Y forces
pz = polyfit(unique_S2, mean_force_z_means, 1); % Z forces

% Evaluate the fitted lines at the unique S_2_Ratio values
X_fit = polyval(px, unique_S2);
Y_fit = polyval(py, unique_S2);
Z_fit = polyval(pz, unique_S2);

% Plot the fitted lines
plot(unique_S2, X_fit, 'Color', "g")
plot(unique_S2, Y_fit, 'Color', [1, 0.5, 0])
plot(unique_S2, Z_fit, 'Color', "b")

% Labels and legend
ylabel("Normalized Forces (F/mg)")
xlabel("Second Moment of Area Ratio S_2")
legend(["Sideward" "Forward" "Upward", "Fit", "Fit", "Fit"],Location="north",NumColumns=2)
hold off



%% S_3 versus torque
figure
hold on

% Extract unique S_3_Ratio values
S_3_Ratio_holder = [1:6,8:11,13:14,16:19,21:22,24:25];
unique_S3 = unique(S_3_Ratio(S_3_Ratio_holder));

% Initialize arrays to store the means for each unique S_2_Ratio
mean_Moment_Roll_mean = zeros(size(unique_S3));
mean_Moment_Pitch_mean = zeros(size(unique_S3));
mean_Moment_Yaw_mean = zeros(size(unique_S3));

% Calculate the mean for each unique S_2_Ratio
for i = 1:length(unique_S3)
    % Find indices corresponding to the current unique S_2_Ratio
    indices = find(S_3_Ratio(S_3_Ratio_holder) == unique_S3(i));
    
    for j = 1:length(indices)
        k = S_3_Ratio_holder(indices(j));
    % Compute means for X, Y, and Z forces
    mean_Moment_Roll_mean(i) = mean(Moment_Roll_mean(k));
    mean_Moment_Pitch_mean(i) = mean(Moment_Pitch_mean(k));
    mean_Moment_Yaw_mean(i) = mean(Moment_Yaw_mean(k));
    end
end



scatter(unique_S3,mean_Moment_Roll_mean,'MarkerEdgeColor',[1, 0.5, 0])
scatter(unique_S3,mean_Moment_Pitch_mean,'MarkerEdgeColor',"g")
scatter(unique_S3,mean_Moment_Yaw_mean,'MarkerEdgeColor',"b")

% 2017 Data
% scatter(Damage_chord_Position, Torque_Damage_chord_Roll, 'd', 'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
% scatter(Damage_chord_Position, Torque_Damage_chord_Pitch, 'd', 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
% scatter(Damage_chord_Position, Torque_Damage_chord_Yaw, 'd', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
% 
% scatter(Damage_span_Position, Torque_Damage_span_Roll, 'o', 'MarkerEdgeColor', [1, 0.5, 0], 'MarkerFaceColor', [1, 0.5, 0]);
% scatter(Damage_span_Position, Torque_Damage_span_Pitch, 'o', 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');
% scatter(Damage_span_Position, Torque_Damage_span_Yaw, 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');

% Fit a line (1st-degree polynomial) to the data
px = polyfit(unique_S3,mean_Moment_Roll_mean, 1);
py = polyfit(unique_S3,mean_Moment_Pitch_mean, 1);
pz = polyfit(unique_S3,mean_Moment_Yaw_mean, 1);


% Evaluate the line at the x data points
X_fit = polyval(px, unique_S3);
Y_fit = polyval(py, unique_S3);
Z_fit = polyval(pz, unique_S3);


plot(unique_S3,X_fit,'Color',[1, 0.5, 0])
plot(unique_S3,Y_fit,'Color',"g")
plot(unique_S3,Z_fit,'Color',"b")


legend(["Roll" "Pitch" "Yaw", "Fit", "Fit", "Fit"],Location="north",NumColumns=2)
ylabel("Normalized Torques (T/mgl)")
xlabel("Third moment of area Ration S_3")
%axis([.5 1 0 1])
hold off

