%% Uncomment to Clear Everything
clear all
clc
%close all
warning off


%% Runtime
current_time = datetime;

%% Wing Values
Wing_damage_LH = 100;
Wing_damage_RH = 100;

filePath = 'C:\Users\jacob\OneDrive - The Pennsylvania State University\Research\Experimental Data\wing_area_ratio.mat';
load(filePath); % Load the MAT file

filePath = 'C:\Users\jacob\OneDrive - The Pennsylvania State University\Research\Experimental Data\damage_side.mat';
load(filePath); % Load the MAT file


%% Fly Numbers
% Specify the folder path
folderPath = 'C:\Users\jacob\OneDrive - The Pennsylvania State University\Research\Experimental Data\Fly_data\Fly_data';

% Get a list of all files and folders in the specified folder
allItems = dir(folderPath);

% Extract only the folder names
folderNames = {allItems([allItems.isdir]).name};

% Remove '.' and '..' entries
folderNames = folderNames(~ismember(folderNames, {'.', '..'}));

% Extract numbers from folder names
numbers = cellfun(@(x) regexp(x, '\d+', 'match'), folderNames, 'UniformOutput', false);

% Convert cell array to numeric array, if the extracted numbers are numeric
Fly_Numbers = cellfun(@(x) str2double(x), [numbers{:}]);


total_states = 0;
fly_count = 1;
for i=1:length(Fly_Numbers)
    disp(Fly_Numbers(i))
    %% Load Data
    load(['Data_Sets' filesep 'fly_' num2str(Fly_Numbers(i)) filesep 'angles' filesep 'wing_data.mat']);



    % %% Find each period via peak
    [peaks, peakIndices] = findpeaks(FilteredAngleL(:,1));

    Pre_cut = 1:200;
    Post_cut = 201:600;
    Steady_State = 601:length(FilteredAngleL(:,1));

    if length(FilteredAngleL(:,1)) > 605
        states = 3;
    else
        states = 2;
    end

    for k=1:states
        if k ==1
            Period = Pre_cut;
            period_text = 'Pre Cut';
            Wing_damage_LH = 100;
            Wing_damage_RH = 100;
        elseif k==2
            Period = Post_cut;
            period_text = 'Post Cut';
            if sum(ismember(left_damage_list, Fly_Numbers(i)))==1
                Wing_damage_LH = str2double(strrep(wing_area_ratio.(['fly' num2str(Fly_Numbers(i))]), '%', ''));
                Wing_damage_RH = 100;
            else
                Wing_damage_LH = 100;
                Wing_damage_RH = str2double(strrep(wing_area_ratio.(['fly' num2str(Fly_Numbers(i))]), '%', ''));
            end
            total_states = total_states +1;
        else
            Period = Steady_State;
            period_text = 'Steady State';
            if sum(ismember(left_damage_list, Fly_Numbers(i)))==1
                Wing_damage_LH = str2double(strrep(wing_area_ratio.(['fly' num2str(Fly_Numbers(i))]), '%', ''));
                Wing_damage_RH = 100;
            else
                Wing_damage_LH = 100;
                Wing_damage_RH = str2double(strrep(wing_area_ratio.(['fly' num2str(Fly_Numbers(i))]), '%', ''));
            end
            total_states = total_states +1;
        end

        %% Time Set Up
        % This section sets the time scale up and the kinematics to one period
        % Period = peakIndices(1):peakIndices(end);
        time = Period;

        Fly_Master(fly_count).Fly  = Analysis_Fly(Wing_damage_LH, Wing_damage_RH, 100, 100,100,100, i, FilteredAngleL, FilteredAngleR, time);
        Fly_Master(fly_count).chord_cut_LH = Wing_damage_LH;
        Fly_Master(fly_count).chord_cut_RH = Wing_damage_RH;
        Fly_Master(fly_count).span_cut_LH = 100;
        Fly_Master(fly_count).span_cut_RH = 100;
        Fly_Master(fly_count).Stroke_Amplitude_LH = 100;
        Fly_Master(fly_count).Stroke_Amplitude_RH = 100;
        Fly_Master(fly_count).Period = Period;
        Fly_Master(fly_count).Fly_Num = Fly_Numbers(i);
        Fly_Master(fly_count).State = period_text;

        fly_count = fly_count +1;

    end

end

%% Run Time End
Duration = datetime-current_time
% return

%% Save data

% % Define the fly folder path
% flyFolderPath = fullfile('C:\Users\jacob\OneDrive - The Pennsylvania State University\Research\Progress Reports\2024.11.28 Meeting');
% 
% % Save your fly data to the Forces folder
% save(fullfile(flyFolderPath, 'Fly.mat'), 'Fly_Master');
% return

%% Plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Force means

for i=1:length(Fly_Master)
    S_2_Ratio(i) = Fly_Master(i).Fly.total.S_2_Ratio;
    Force_X_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_LH.force_total_vec(1,:)) + mean(Fly_Master(i).Fly.force_total.Force_Body_RH.force_total_vec(1,:)))/Fly_Master(i).Fly.total.weight;
    Force_Y_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_LH.force_total_vec(3,:)) - mean(Fly_Master(i).Fly.force_total.Force_Body_RH.force_total_vec(3,:)))/Fly_Master(i).Fly.total.weight;
    Force_Z_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_LH.force_total_vec(2,:)) + mean(Fly_Master(i).Fly.force_total.Force_Body_RH.force_total_vec(2,:)))/Fly_Master(i).Fly.total.weight;
end

%% force Box and Whisker
% figure;
% hold on;
% 
% % Concatenate your force data into a single vector
% allForces = [Force_X_mean; Force_Y_mean; Force_Z_mean];
% 
% % Create a grouping variable
% group = [repmat("X", length(Force_X_mean), 1);
%     repmat("Y", length(Force_Y_mean), 1);
%     repmat("Z", length(Force_Z_mean), 1)];
% 
% % Plot the boxplot
% boxplot(allForces, group);
% %boxplot(allForces, group, 'Colors', [1, 0.5, 0; 0, 1, 0; 0, 0, 1]);
% 
% % Customize the plot
% ylabel('Normalized Forces (F/mg)');
% title(['Box and Whisker Plot of Normalized Forces for Fly ' num2str(Fly_Numbers(j))]);
% 
% hold off;
% 
% % Save the figure to the Forces folder
% figureFilePath = fullfile(forcesFolderPath, ['Normalized_Forces_Fly_' num2str(Fly_Numbers(j)) '.png']);
% saveas(gcf, figureFilePath);
% 
% % Save the figure to the Forces folder
% figureFilePath = fullfile(forcesFolderPath, ['Normalized_Forces_Fly_' num2str(Fly_Numbers(j)) '.fig']);
% saveas(gcf, figureFilePath);
% 
% % Close the figure after saving
% close(gcf);


%% S_2 versus force
figure
hold on
scatter(S_2_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]),Force_X_mean([1:6,8:11,13:14,16:19,21:22,24:25]),'MarkerEdgeColor',[1, 0.5, 0])
scatter(S_2_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]),Force_Y_mean([1:6,8:11,13:14,16:19,21:22,24:25]),'MarkerEdgeColor',"g")
scatter(S_2_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]),Force_Z_mean([1:6,8:11,13:14,16:19,21:22,24:25]),'MarkerEdgeColor',"b")

% Initialize variables to store unique S_2_Ratio values and their corresponding means
unique_S2 = unique(S_2_Ratio([1:6,8:11,13:14,16:19,21:22,24:25])); % Get the unique S_2_Ratio values

% Loop through each unique S_2_Ratio
for i = 1:length(unique_S2)
    % Find the indices of the current S_2_Ratio
    indices = find(S_2_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]) == unique_S2(i));
    
    % Calculate the mean of Force_X_mean for these indices
    mean_force_x_means(i) = mean(Force_X_mean(indices));
    mean_force_y_means(i) = mean(Force_Y_mean(indices));
    mean_force_z_means(i) = mean(Force_Z_mean(indices));
end


% Fit a line (1st-degree polynomial) to the data
px = polyfit(unique_S2, mean_force_x_means, 1);
% Evaluate the line at the x data points
X_fit = polyval(px, unique_S2);

% Fit a line (1st-degree polynomial) to the data
py = polyfit(unique_S2, mean_force_y_means, 1);
% Evaluate the line at the x data points
Y_fit = polyval(py, unique_S2);

% Fit a line (1st-degree polynomial) to the data
pz = polyfit(unique_S2, mean_force_z_means, 1);
% Evaluate the line at the x data points
Z_fit = polyval(pz, unique_S2);


plot(unique_S2,X_fit,'Color',[1, 0.5, 0])
plot(unique_S2,Y_fit,'Color',"g")
plot(unique_S2,Z_fit,'Color',"b")


ylabel("Normalized Forces (F/mg)")
xlabel("Second moment of area Ration S_2")
legend(["X" "Y" "Z"])
% axis([0.1 1 -1 1])
hold off

%% Torques means

for i=1:length(Fly_Master)
    S_3_Ratio(i) = Fly_Master(i).Fly.total.S_3_Ratio;
    Moment_Roll_mean(i) = mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(1,:) - Fly_Master(i).Fly.force_total.Force_Body_RH.torque_total_vec(1,:)) / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length+Fly_Master(i).Fly.wing_RH.wing_length)/2));
    Moment_Pitch_mean(i) = mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(3,:) + Fly_Master(i).Fly.force_total.Force_Body_RH.torque_total_vec(3,:)) / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length+Fly_Master(i).Fly.wing_RH.wing_length)/2));
    Moment_Yaw_mean(i) = mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(2,:) - Fly_Master(i).Fly.force_total.Force_Body_RH.torque_total_vec(2,:)) / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length+Fly_Master(i).Fly.wing_RH.wing_length)/2));
    %Moment_Yaw_mean(i) = mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(2,:)  / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length))));
    % Moment_Yaw_mean(i) = mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(1,:) - Fly_Master(i).Fly.force_total.Force_Body_RH.torque_total_vec(1,:)) / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length+Fly_Master(i).Fly.wing_RH.wing_length)/2));
    % Moment_Roll_mean(i) = -mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(3,:) - Fly_Master(i).Fly.force_total.Force_Body_RH.torque_total_vec(3,:)) / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length+Fly_Master(i).Fly.wing_RH.wing_length)/2));
    % Moment_Pitch_mean(i) = -mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(2,:) - Fly_Master(i).Fly.force_total.Force_Body_RH.torque_total_vec(2,:)) / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length+Fly_Master(i).Fly.wing_RH.wing_length)/2));
end

%% S_3 versus torque
figure
hold on
scatter(S_3_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]),Moment_Roll_mean([1:6,8:11,13:14,16:19,21:22,24:25]),'MarkerEdgeColor',[1, 0.5, 0])
scatter(S_3_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]),Moment_Pitch_mean([1:6,8:11,13:14,16:19,21:22,24:25]),'MarkerEdgeColor',"g")
scatter(S_3_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]),Moment_Yaw_mean([1:6,8:11,13:14,16:19,21:22,24:25]),'MarkerEdgeColor',"b")

% Fit a line (1st-degree polynomial) to the data
px = polyfit(S_3_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]), Moment_Roll_mean([1:6,8:11,13:14,16:19,21:22,24:25]), 1);
% Evaluate the line at the x data points
X_fit = polyval(px, S_3_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]));

% Fit a line (1st-degree polynomial) to the data
py = polyfit(S_3_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]), Moment_Pitch_mean([1:6,8:11,13:14,16:19,21:22,24:25]), 1);
% Evaluate the line at the x data points
Y_fit = polyval(py, S_3_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]));

% Fit a line (1st-degree polynomial) to the data
pz = polyfit(S_3_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]), Moment_Yaw_mean([1:6,8:11,13:14,16:19,21:22,24:25]), 1);
% Evaluate the line at the x data points
Z_fit = polyval(pz, S_3_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]));


plot(S_3_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]),X_fit,'Color',[1, 0.5, 0])
plot(S_3_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]),Y_fit,'Color',"g")
plot(S_3_Ratio([1:6,8:11,13:14,16:19,21:22,24:25]),Z_fit,'Color',"b")


legend(["Roll" "Pitch" "Yaw"])
ylabel("Normalized Torques (T/mgl)")
xlabel("Third moment of area Ration S_3")
%axis([.5 1 0 1])
hold off

%% Plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Mean vector plot
% Define the origin for each vector (set to [0, 0, 0])
originX = zeros(size(Force_X_mean)); % X-coordinates of the origin
originY = zeros(size(Force_Y_mean)); % Y-coordinates of the origin
originZ = zeros(size(Force_Z_mean)); % Z-coordinates of the origin

% Custom distinct colors (manually chosen for high contrast)
colors = [...
    0.85, 0.33, 0.10; % Red-orange
    0.47, 0.67, 0.19; % Green
    0.30, 0.75, 0.93; % Light blue
    0.93, 0.69, 0.13; % Yellow-orange
    0.64, 0.08, 0.18; % Maroon
    0.49, 0.18, 0.56; % Purple
    0.00, 0.45, 0.74; % Blue
    0.25, 0.25, 0.25; % Gray
    0.94, 0.39, 0.39; % Salmon
    0.10, 0.60, 0.40]; % Teal

num_colors = size(colors, 1); % Number of available colors

% Initialize handles and legend labels for "Pre Cut" only
h_pre = []; % Handles for "Pre Cut"
legend_labels_pre = {}; % Labels for "Pre Cut"

% 3D Vector Plot
figure;
hold on;
i = 1; % Color index



for k = 1:length(Fly_Master)
    if Fly_Master(k).Fly_Num == 23 || Fly_Master(k).Fly_Num == 24 || Fly_Master(k).Fly_Num == 5
        % Ensure the color index wraps around if there are more than num_colors flies
        color_idx = mod(i-1, num_colors) + 1; % Cycles through 1 to num_colors

        % Calculate the endpoint of the vector
        endX = originX(k) + Force_X_mean(k);
        endY = originY(k) + Force_Y_mean(k);
        endZ = originZ(k) + Force_Z_mean(k);

        if Fly_Master(k).State == "Pre Cut"
            % Plot solid line for "Pre Cut"
            h_pre(end+1) = plot3([originX(k), endX], [originY(k), endY], [originZ(k), endZ], ...
                'Color', colors(color_idx, :), 'LineStyle', '-', ...
                'LineWidth', 1.5);
            % Add legend label for "Pre Cut"
            legend_labels_pre{end+1} = ['Fly ' num2str(Fly_Master(k).Fly_Num)];

        elseif Fly_Master(k).State == "Post Cut"
            % Plot dashed line for "Post Cut" without adding to the legend
            plot3([originX(k), endX], [originY(k), endY], [originZ(k), endZ], ...
                'Color', colors(color_idx, :), 'LineStyle', '--', ...
                'LineWidth', 1.5);
            i = i + 1; % Increment color index

            % elseif Fly_Master(k).State == "Steady State"
            %     % Plot dotted line for "Steady State" without adding to the legend
            %     plot3([originX(k), endX], [originY(k), endY], [originZ(k), endZ], ...
            %         'Color', colors(color_idx, :), 'LineStyle', ':', ...
            %         'LineWidth', 1.5);
            %     i = i + 1; % Increment color index
        end
    end
end


% Add title and axis labels
xlabel('Sideward (F/mg)');
ylabel('Forward (F/mg)');
zlabel('Upward (F/mg)');
axis equal;
% grid on;
view([0 0]);


% % Set axis limits
% xmax = max(Force_X_mean); % Maximum x value
% zmax = max(Force_Z_mean); % Maximum z value
xlim([0, 0.3]);
zlim([0, 0.45]);
%ylim([0, 0.5])


% Add legend for "Pre Cut" vectors
if ~isempty(h_pre) % Only add legend if there are vectors
    legend(h_pre, legend_labels_pre, 'Location', 'best');
else
    warning('No vectors were plotted, so no legend is displayed.');
end

%% Plot of change in force over periods

% Define fly numbers
fly_numbers = [1,2];

% Initialize storage for results
results = struct();

% Initialize a figure for combined plotting
figure;

% Loop through each fly number
for fly_idx = 1:length(fly_numbers)

    clear Period_Index

    % Access Fly data from the structured array
    fly_nums = Fly_Master(1, fly_numbers(fly_idx));
    phi = fly_nums.Fly.Kinematics_LH.phi; % Stroke angle (phi)

    % Find each period via peaks
    [peaks, peakIndices] = findpeaks(phi);

    % Initialize period indices
    j = 1;
    Period_Index(1) = 1;

    i = 2;
    while j < length(phi)
        % Get the last held value
        last_value = phi(1);

        % Search for the next value after the current peak within ±0.1 of last_value
        for k = peakIndices(i-1):length(phi)
            if abs(phi(k) - last_value) <= 0.2
                Period_Index(i) = k; % Store the index
                j = k; % Update the loop index
                break;
            end
        end

        i = i + 1; % Move to the next period index
        if j > peakIndices(end) || i > length(peakIndices)
            break;
        end
    end

    % Initialize arrays for storing mean forces
    Force_X_mean = zeros(1, length(Period_Index) - 1);
    Force_Y_mean = zeros(1, length(Period_Index) - 1);
    Force_Z_mean = zeros(1, length(Period_Index) - 1);

    % Loop through each period to calculate mean forces
    for p = 1:(length(Period_Index) - 1)
        % Extract the start and end indices for the current period
        start_idx = Period_Index(p);
        end_idx = Period_Index(p + 1) - 1; % Exclude the endpoint of the next period

        % Calculate mean forces for the current period
        Force_X_mean(p) = (mean(fly_nums.Fly.force_total.Force_Body_LH.force_total_vec(1, start_idx:end_idx)) + ...
                           mean(fly_nums.Fly.force_total.Force_Body_RH.force_total_vec(1, start_idx:end_idx))) / fly_nums.Fly.total.weight;

        Force_Y_mean(p) = (mean(fly_nums.Fly.force_total.Force_Body_LH.force_total_vec(3, start_idx:end_idx)) - ...
                           mean(fly_nums.Fly.force_total.Force_Body_RH.force_total_vec(3, start_idx:end_idx))) / fly_nums.Fly.total.weight;

        Force_Z_mean(p) = (mean(fly_nums.Fly.force_total.Force_Body_LH.force_total_vec(2, start_idx:end_idx)) + ...
                           mean(fly_nums.Fly.force_total.Force_Body_RH.force_total_vec(2, start_idx:end_idx))) / fly_nums.Fly.total.weight;
    end

    % Define time intervals for each period
    period_times = fly_nums.Fly.time(Period_Index);

    % Interpolate mean forces over the entire time vector
    time_vector = fly_nums.Fly.time;
    Force_X_interp = interp1(period_times(1:end-1), Force_X_mean, time_vector, 'previous', 'extrap');
    Force_Y_interp = interp1(period_times(1:end-1), Force_Y_mean, time_vector, 'previous', 'extrap');
    Force_Z_interp = interp1(period_times(1:end-1), Force_Z_mean, time_vector, 'previous', 'extrap');

    % Mask positive and negative values
    Force_X_pos = Force_X_interp;
    Force_X_neg = Force_X_interp;
    Force_X_pos(Force_X_interp <= 0) = NaN;
    Force_X_neg(Force_X_interp >= 0) = NaN;

    Force_Y_pos = Force_Y_interp;
    Force_Y_neg = Force_Y_interp;
    Force_Y_pos(Force_Y_interp <= 0) = NaN;
    Force_Y_neg(Force_Y_interp >= 0) = NaN;

    Force_Z_pos = Force_Z_interp;
    Force_Z_neg = Force_Z_interp;
    Force_Z_pos(Force_Z_interp <= 0) = NaN;
    Force_Z_neg(Force_Z_interp >= 0) = NaN;

if strcmp(fly_nums.State, 'Pre Cut') % Check if it's pre-cut
    color_pos = 'b'; % Blue for positive
    color_neg = 'b'; % Blue for negative
else
    color_pos = 'r'; % Orange (use 'r' for red; MATLAB doesn't have orange) for positive
    color_neg = 'r'; % Orange for negative
end
    
    
    % Plot forces over time
subplot(3, 1, 1);
hold on;

plot(time_vector / 8000, abs(Force_X_pos), '-', 'LineWidth', 1.5, 'Color', color_pos, 'DisplayName', [fly_nums.State, ' Positive']);
plot(time_vector / 8000, abs(Force_X_neg), '--', 'LineWidth', 1.5, 'Color', color_neg, 'DisplayName', [fly_nums.State, ' Negative']);
ylabel('Side Force (F_x/mg)');
title('Forces Over Time');
legend('show');
grid on;

subplot(3, 1, 2);
hold on;
plot(time_vector / 8000, abs(Force_Y_pos), '-', 'LineWidth', 1.5, 'Color', color_pos);
plot(time_vector / 8000, abs(Force_Y_neg), '--', 'LineWidth', 1.5, 'Color', color_neg);
ylabel('Forward Force (F_y/mg)');
grid on;

subplot(3, 1, 3);
hold on;
plot(time_vector / 8000, abs(Force_Z_pos), '-', 'LineWidth', 1.5, 'Color', color_pos);
plot(time_vector / 8000, abs(Force_Z_neg), '--', 'LineWidth', 1.5, 'Color', color_neg);
xlabel('Time (s)');
ylabel('Vertical Force (F_z/mg)');
grid on;

end

% Overall plot adjustments
sgtitle(['Forces Over Time: Fly: ', num2str(fly_nums.Fly_Num)]);



%% New 
% Initialize arrays for storing forces for all flies
All_Fly_Forces_X_Pre = [];
All_Fly_Forces_Y_Pre = [];
All_Fly_Forces_Z_Pre = [];

All_Fly_Forces_X_Post = [];
All_Fly_Forces_Y_Post = [];
All_Fly_Forces_Z_Post = [];

% Define maximum number of periods for consistent padding
max_periods = 0;

% First loop to determine maximum number of periods
for k = 1:length(Fly_Master)
    % Access current fly data
    fly = Fly_Master(k).Fly;
    phi = fly.Kinematics_LH.phi; % Replace with correct angle variable

    % Find each period via peak
    [peaks, peakIndices] = findpeaks(phi);

    % Initialize period indices
    j = 1;
    Period_Index = zeros(1, length(phi)); % Pre-allocate for performance
    Period_Index(1) = 1;
    i = 2;

    while j < length(phi)
        if i > length(peakIndices)
            break;
        end
        for kk = peakIndices(i-1):length(phi)
            if abs(phi(kk) - phi(1)) <= 0.2
                Period_Index(i) = kk;
                j = kk;
                break;
            end
        end
        i = i + 1;
    end
    Period_Index = Period_Index(1:i-1); % Trim to actual indices
    max_periods = max(max_periods, length(Period_Index) - 1);
end

% Second loop to calculate forces and pad arrays
for k = 1:length(Fly_Master)
    % Access current fly data
    fly = Fly_Master(k).Fly;
    phi = fly.Kinematics_LH.phi;
    weight = fly.total.weight; % Fly weight

    % Find each period via peak
    [peaks, peakIndices] = findpeaks(phi);

    % Initialize period indices
    j = 1;
    Period_Index = zeros(1, length(phi)); % Pre-allocate for performance
    Period_Index(1) = 1;
    i = 2;

    while j < length(phi)
        if i > length(peakIndices)
            break;
        end
        for kk = peakIndices(i-1):length(phi)
            if abs(phi(kk) - phi(1)) <= 0.2
                Period_Index(i) = kk;
                j = kk;
                break;
            end
        end
        i = i + 1;
    end
    Period_Index = Period_Index(1:i-1); % Trim to actual indices

    % Calculate mean forces for each period
    num_periods = length(Period_Index) - 1;
    Force_X_mean = zeros(1, max_periods);
    Force_Y_mean = zeros(1, max_periods);
    Force_Z_mean = zeros(1, max_periods);

    for p = 1:num_periods
        start_idx = Period_Index(p);
        end_idx = Period_Index(p + 1) - 1;
        Force_X_mean(p) = (mean(fly.force_total.Force_Body_LH.force_total_vec(1, start_idx:end_idx)) + ...
                           mean(fly.force_total.Force_Body_RH.force_total_vec(1, start_idx:end_idx))) / weight;
        Force_Y_mean(p) = (mean(fly.force_total.Force_Body_LH.force_total_vec(3, start_idx:end_idx)) - ...
                           mean(fly.force_total.Force_Body_RH.force_total_vec(3, start_idx:end_idx))) / weight;
        Force_Z_mean(p) = (mean(fly.force_total.Force_Body_LH.force_total_vec(2, start_idx:end_idx)) + ...
                           mean(fly.force_total.Force_Body_RH.force_total_vec(2, start_idx:end_idx))) / weight;
    end

    % Store forces based on the fly's state
    if Fly_Master(k).State == "Pre Cut"
        All_Fly_Forces_X_Pre = [All_Fly_Forces_X_Pre; Force_X_mean];
        All_Fly_Forces_Y_Pre = [All_Fly_Forces_Y_Pre; Force_Y_mean];
        All_Fly_Forces_Z_Pre = [All_Fly_Forces_Z_Pre; Force_Z_mean];
    elseif Fly_Master(k).State == "Post Cut"
        All_Fly_Forces_X_Post = [All_Fly_Forces_X_Post; Force_X_mean];
        All_Fly_Forces_Y_Post = [All_Fly_Forces_Y_Post; Force_Y_mean];
        All_Fly_Forces_Z_Post = [All_Fly_Forces_Z_Post; Force_Z_mean];
    end
end

% Plot all forces grouped by component with Pre and Post Cut states
figure;

% Plot Force X
subplot(3, 1, 1);
hold on;
h_pre = []; % Handles for Pre Cut
h_post = []; % Handles for Post Cut

for i = 1:size(All_Fly_Forces_X_Pre, 1)
    data = All_Fly_Forces_X_Pre(i, :);
    data(data == 0) = NaN; % Replace zeros with NaN to avoid plotting them
    h_pre = [h_pre, plot(data, 'r', 'LineWidth', 1.5)]; % Pre Cut
end
for i = 1:size(All_Fly_Forces_X_Post, 1)
    data = All_Fly_Forces_X_Post(i, :);
    data(data == 0) = NaN; % Replace zeros with NaN to avoid plotting them
    h_post = [h_post, plot(data, 'r--', 'LineWidth', 1.5)]; % Post Cut
end
xlabel('Period Index');
ylabel('Side Force (F_x/mg)')
legend([h_pre(1), h_post(1)], 'Pre Cut', 'Post Cut'); % Use the first handles for legend
grid on;
hold off;

% Plot Force Y
subplot(3, 1, 2);
hold on;
h_pre = []; % Reset handles for Pre Cut
h_post = []; % Reset handles for Post Cut

for i = 1:size(All_Fly_Forces_Y_Pre, 1)
    data = All_Fly_Forces_Y_Pre(i, :);
    data(data == 0) = NaN; % Replace zeros with NaN to avoid plotting them
    h_pre = [h_pre, plot(data, 'g', 'LineWidth', 1.5)]; % Pre Cut
end
for i = 1:size(All_Fly_Forces_Y_Post, 1)
    data = All_Fly_Forces_Y_Post(i, :);
    data(data == 0) = NaN; % Replace zeros with NaN to avoid plotting them
    h_post = [h_post, plot(data, 'g--', 'LineWidth', 1.5)]; % Post Cut
end
ylabel('Forward Force (F_y/mg)')
grid on;
hold off;

% Plot Force Z
subplot(3, 1, 3);
hold on;
h_pre = []; % Reset handles for Pre Cut
h_post = []; % Reset handles for Post Cut

for i = 1:size(All_Fly_Forces_Z_Pre, 1)
    data = All_Fly_Forces_Z_Pre(i, :);
    data(data == 0) = NaN; % Replace zeros with NaN to avoid plotting them
    h_pre = [h_pre, plot(data, 'b', 'LineWidth', 1.5)]; % Pre Cut
end
for i = 1:size(All_Fly_Forces_Z_Post, 1)
    data = All_Fly_Forces_Z_Post(i, :);
    data(data == 0) = NaN; % Replace zeros with NaN to avoid plotting them
    h_post = [h_post, plot(data, 'b--', 'LineWidth', 1.5)]; % Post Cut
end
ylabel('Vertical Force (F_z/mg)')
grid on;
hold off;

% Overall adjustments
sgtitle('Forces Grouped by Component (Pre Cut vs. Post Cut)');



