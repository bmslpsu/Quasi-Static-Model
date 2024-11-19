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


for j=1:length(Fly_Numbers)-1
    disp(Fly_Numbers(j))
%% Load Data
load(['Data_Sets' filesep 'fly_' num2str(Fly_Numbers(j)) filesep 'angles' filesep 'wing_data.mat']);


%% FInd each period via peak
[peaks, peakIndices] = findpeaks(FilteredAngleL(:,1));

%% Program Runner

for i=1:length(peakIndices)-1
    disp(peakIndices(i))

    %% Time Set Up
    % This section sets the time scale up and the kinematics to one period
    Period = peakIndices(i):peakIndices(i+1);
    time = Period;

    % Chordwise cut
    Fly_Master(i).Fly  = Analyis_Fly(Wing_damage_LH, Wing_damage_RH, 100, 100,100,100, i, FilteredAngleL, FilteredAngleR, Period);
    Fly_Master(i).chord_cut_LH = Wing_damage_LH;
    Fly_Master(i).chord_cut_RH = Wing_damage_RH;
    Fly_Master(i).span_cut = 100;
    Fly_Master(i).Stroke_Amplitude = 100;
    Fly_Master(i).Stroke_Amplitude = 100;
    Fly_Master(i).Period = i;
    Fly_Master(i).Fly_Num = Fly_Numbers(j);


end

%% Save data

% Define the fly folder path
    flyFolderPath = fullfile(folderPath, ['fly_' num2str(Fly_Numbers(j))]);
    
    % Define the Forces folder path
    forcesFolderPath = fullfile(flyFolderPath, 'Forces');
    
    % Create the Forces folder if it doesn't exist
    if ~exist(forcesFolderPath, 'dir')
        mkdir(forcesFolderPath);
    end
    
    % Save your fly data to the Forces folder
    % Assuming 'Fly_Data' is the variable you want to save
    save(fullfile(forcesFolderPath, 'Forces.mat'), 'Fly_Master');

%% Plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Force means

for i=1:length(Fly_Master)
    S_2_Ratio(i) = Fly_Master(i).Fly.total.S_2_Ratio;
    Force_X_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_LH.force_total_vec(1,:)) + mean(Fly_Master(i).Fly.force_total.Force_Body_RH.force_total_vec(1,:)))/Fly_Master(i).Fly.total.weight;
    Force_Y_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_LH.force_total_vec(3,:)) - mean(Fly_Master(i).Fly.force_total.Force_Body_RH.force_total_vec(3,:)))/Fly_Master(i).Fly.total.weight;
    Force_Z_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_LH.force_total_vec(2,:)) + mean(Fly_Master(i).Fly.force_total.Force_Body_RH.force_total_vec(2,:)))/Fly_Master(i).Fly.total.weight;
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
% figure
% hold on
% boxplot(S_2_Ratio,Force_X_mean,'Color',[1, 0.5, 0])
% boxplot(S_2_Ratio,Force_Y_mean,'Color',"g")
% boxplot(S_2_Ratio,Force_Z_mean,'Color',"b")
% 
% legend(["X" "Y" "Z"])
% ylabel("Normalized Forces (F/mg)")
% xlabel("Second moment of area Ration S_2")
% % axis([.5 1 0 1])
% hold off

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

%% Run Time End
Duration = datetime-current_time
end