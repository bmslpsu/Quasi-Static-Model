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
        if ismember(left_damage_list, Fly_Numbers(i))
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
        if ismember(left_damage_list, Fly_Numbers(i))
            Wing_damage_LH = str2double(strrep(wing_area_ratio.(['fly' num2str(Fly_Numbers(i))]), '%', ''));
            Wing_damage_RH = 100;
        else
            Wing_damage_LH = 100;
            Wing_damage_RH = str2double(strrep(wing_area_ratio.(['fly' num2str(Fly_Numbers(i))]), '%', ''));
        end
        total_states = total_states +1;
    end



% figure
% plot(FilteredAngleL(period,:))
% title([num2str(Fly_Numbers(i)) ' ' period_text])


%% Program Runner
% 
% for i=1:length(peakIndices)-1
%     disp(peakIndices(i))

    %% Time Set Up
    % This section sets the time scale up and the kinematics to one period
    % Period = peakIndices(1):peakIndices(end);
    time = Period;


    % Chordwise cut
    Fly_Master(fly_count).Fly  = Analysis_Fly(Wing_damage_LH, Wing_damage_RH, 100, 100,100,100, i, FilteredAngleL, FilteredAngleR, time);
    Fly_Master(fly_count).chord_cut_LH = Wing_damage_LH;
    Fly_Master(fly_count).chord_cut_RH = Wing_damage_RH;
    Fly_Master(fly_count).span_cut = 100;
    Fly_Master(fly_count).Stroke_Amplitude = 100;
    Fly_Master(fly_count).Stroke_Amplitude = 100;
    Fly_Master(fly_count).Period = Period;
    Fly_Master(fly_count).Fly_Num = Fly_Numbers(i);
    Fly_Master(fly_count).State = period_text;

   fly_count = fly_count +1; 
end
end
%% Run Time End
Duration = datetime-current_time
%end



%% Save data

% % Define the fly folder path
%     flyFolderPath = fullfile(folderPath, ['fly_' num2str(Fly_Numbers(j))]);
% 
%     % Define the Forces folder path
%     forcesFolderPath = fullfile(flyFolderPath, 'Forces');
% 
%     % Create the Forces folder if it doesn't exist
%     if ~exist(forcesFolderPath, 'dir')
%         mkdir(forcesFolderPath);
%     end
% 
%     % Save your fly data to the Forces folder
%     % Assuming 'Fly_Data' is the variable you want to save
%     save(fullfile(forcesFolderPath, 'Forces.mat'), 'Fly_Master');

% Define the fly folder path
    flyFolderPath = fullfile('C:\Users\jacob\OneDrive - The Pennsylvania State University\Research\Progress Reports\2024.11.21 Meeting');

    % Save your fly data to the Forces folder
    % Assuming 'Fly_Data' is the variable you want to save
    save(fullfile(flyFolderPath, 'Fly.mat'), 'Fly_Master');
return

%% Plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Force means

for i=1:length(Fly_Master)
    S_2_Ratio(i) = Fly_Master(i).Fly.total.S_2_Ratio;
    Force_X_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_LH.force_total_vec(1,:)) + mean(Fly_Master(i).Fly.force_total.Force_Body_RH.force_total_vec(1,:)))/Fly_Master(i).Fly.total.weight;
    Force_Y_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_LH.force_total_vec(3,:)) - mean(Fly_Master(i).Fly.force_total.Force_Body_RH.force_total_vec(3,:)))/Fly_Master(i).Fly.total.weight;
    Force_Z_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_LH.force_total_vec(2,:)) + mean(Fly_Master(i).Fly.force_total.Force_Body_RH.force_total_vec(2,:)))/Fly_Master(i).Fly.total.weight;
end

for i=26:-1:10
    S_2_Ratio(i) = [];
    Force_X_mean(i) = [];
    Force_Y_mean(i) = [];
    Force_Z_mean(i) = [];
end

%% force Box and Whisker
figure;
hold on;

% Concatenate your force data into a single vector
allForces = [Force_X_mean; Force_Y_mean; Force_Z_mean];

% Create a grouping variable
group = [repmat("X", length(Force_X_mean), 1); 
         repmat("Y", length(Force_Y_mean), 1); 
         repmat("Z", length(Force_Z_mean), 1)];

% Plot the boxplot
boxplot(allForces, group);
%boxplot(allForces, group, 'Colors', [1, 0.5, 0; 0, 1, 0; 0, 0, 1]);

% Customize the plot
ylabel('Normalized Forces (F/mg)');
title(['Box and Whisker Plot of Normalized Forces for Fly ' num2str(Fly_Numbers(j))]);

hold off;

    % Save the figure to the Forces folder
    figureFilePath = fullfile(forcesFolderPath, ['Normalized_Forces_Fly_' num2str(Fly_Numbers(j)) '.png']);
    saveas(gcf, figureFilePath);

        % Save the figure to the Forces folder
    figureFilePath = fullfile(forcesFolderPath, ['Normalized_Forces_Fly_' num2str(Fly_Numbers(j)) '.fig']);
    saveas(gcf, figureFilePath);
    
    % Close the figure after saving
    close(gcf);


%% S_2 versus force
figure
hold on
scatter(S_2_Ratio,Force_X_mean,'MarkerEdgeColor',[1, 0.5, 0])
scatter(S_2_Ratio,Force_Y_mean,'MarkerEdgeColor',"g")
scatter(S_2_Ratio,Force_Z_mean,'MarkerEdgeColor',"b")


% Fit a line (1st-degree polynomial) to the data
px = polyfit(S_2_Ratio, Force_X_mean, 1);
% Evaluate the line at the x data points
X_fit = polyval(px, S_2_Ratio);

% Fit a line (1st-degree polynomial) to the data
py = polyfit(S_2_Ratio, Force_Y_mean, 1);
% Evaluate the line at the x data points
Y_fit = polyval(py, S_2_Ratio);

% Fit a line (1st-degree polynomial) to the data
pz = polyfit(S_2_Ratio, Force_Z_mean, 1);
% Evaluate the line at the x data points
Z_fit = polyval(pz, S_2_Ratio);


plot(S_2_Ratio,X_fit,'Color',[1, 0.5, 0])
plot(S_2_Ratio,Y_fit,'Color',"g")
plot(S_2_Ratio,Z_fit,'Color',"b")


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
plot(S_3_Ratio,Moment_Roll_mean,'Color',[1, 0.5, 0])
plot(S_3_Ratio,Moment_Pitch_mean,'Color',"g")
plot(S_3_Ratio,Moment_Yaw_mean,'Color',"b")
legend(["Roll" "Pitch" "Yaw"])
ylabel("Normalized Torques (T/mgl)")
xlabel("Third moment of area Ration S_3")
%axis([.5 1 0 1])
hold off

%% Plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Mean vector plot
% Define the origin
originX = zeros(size(Force_X_mean));
originY = zeros(size(Force_X_mean));
originZ = zeros(size(Force_X_mean)); % Only needed for 3D

% Define colors for each vector
colors = lines(length(Force_X_mean)); % Generate a set of distinct colors

% 3D Vector Plot
figure;
hold on;
for i = 1:length(Force_X_mean)
    % Plot each vector with a different color
    h(i) = quiver3(originX(i), originY(i), originZ(i), ...
                   Force_X_mean(i), Force_Y_mean(i), Force_Z_mean(i), ...
                   'Color', colors(i, :), 'LineWidth', 1.5, 'AutoScale', 'on');
end


% Add title and axis labels
title('3D Vector Plot from Origin');
xlabel('Sideward (X)');
ylabel('Forward (Y)');
zlabel('Upward (Z)');
axis equal;
grid on;
view(3);

% Set axis limits to start at zero
xmax = max(Force_X_mean); % Maximum x value
zmax = max(Force_Z_mean); % Maximum z value
xlim([0, xmax]);
zlim([0, zmax]);

% Convert numeric titles to strings
legend_labels = arrayfun(@num2str, Fly_Numbers, 'UniformOutput', false);

% Add legend
legend(h, legend_labels, 'Location', 'best');

quiver3(originX(1), originY(1), originZ(1), ...
                   0.06, 0.18, 0.61, ...
                   'Color', 'k', 'LineWidth', 3, 'AutoScale', 'on');
