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

filePath = 'C:\Users\jacob\OneDrive - The Pennsylvania State University\Research\Experimental Data\Wing_Stroke_Angle\Wing_Stroke_Plane.mat';
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



    % Find each period via peak
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
            index = find(Wing_Stroke_Plane.Fly == Fly_Numbers(i));
            ang_wing_plane_LH = Wing_Stroke_Plane.LH_Pre(index);
            ang_wing_plane_RH = Wing_Stroke_Plane.RH_Pre(index);
            Kinematics_LH = FilteredAngleL;
            Kinematics_RH = FilteredAngleR;
        elseif k==2
            Period = Post_cut;
            period_text = 'Post Cut';
            index = find(Wing_Stroke_Plane.Fly == Fly_Numbers(i));
            if sum(ismember(left_damage_list, Fly_Numbers(i)))==1
                Kinematics_LH = FilteredAngleL;
                Kinematics_RH = FilteredAngleR;
                ang_wing_plane_LH = Wing_Stroke_Plane.LH_Post(index);
                ang_wing_plane_RH = Wing_Stroke_Plane.RH_Post(index);
                Wing_damage_LH = str2double(strrep(wing_area_ratio.(['fly' num2str(Fly_Numbers(i))]), '%', ''));
                Wing_damage_RH = 100;
            else
                Kinematics_LH = FilteredAngleR;
                Kinematics_RH = FilteredAngleL;
                ang_wing_plane_LH = Wing_Stroke_Plane.RH_Post(index);
                ang_wing_plane_RH = Wing_Stroke_Plane.LH_Post(index);
                Wing_damage_LH = str2double(strrep(wing_area_ratio.(['fly' num2str(Fly_Numbers(i))]), '%', ''));
                Wing_damage_RH = 100;
            end
            total_states = total_states + 1;
        else
            Period = Steady_State;
            period_text = 'Steady State';
            index = find(Wing_Stroke_Plane.Fly == Fly_Numbers(i));
            if sum(ismember(left_damage_list, Fly_Numbers(i)))==1
                Kinematics_LH = FilteredAngleL;
                Kinematics_RH = FilteredAngleR;
                ang_wing_plane_LH = Wing_Stroke_Plane.LH_Post(index);
                ang_wing_plane_RH = Wing_Stroke_Plane.RH_Post(index);
                Wing_damage_LH = str2double(strrep(wing_area_ratio.(['fly' num2str(Fly_Numbers(i))]), '%', ''));
                Wing_damage_RH = 100;
            else
                Kinematics_LH = FilteredAngleR;
                Kinematics_RH = FilteredAngleL;
                ang_wing_plane_LH = Wing_Stroke_Plane.RH_Post(index);
                ang_wing_plane_RH = Wing_Stroke_Plane.LH_Post(index);
                Wing_damage_LH = str2double(strrep(wing_area_ratio.(['fly' num2str(Fly_Numbers(i))]), '%', ''));
                Wing_damage_RH = 100;
            end
            total_states = total_states + 1;
        end

        %% Run Simulation
        % This section sets the time scale up and the kinematics to one period
        % Period = peakIndices(1):peakIndices(end);
        time = Period;

        Fly_Master(fly_count).Fly  = Analysis_Fly(Wing_damage_LH, Wing_damage_RH, 100, 100,100,100, i, Kinematics_LH, Kinematics_RH, time, ang_wing_plane_LH, ang_wing_plane_RH);
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

%% Save data

% % Define the fly folder path
% flyFolderPath = fullfile('C:\Users\jacob\OneDrive - The Pennsylvania State University\Research\Progress Reports\2024.11.28 Meeting');
% 
% % Save your fly data to the Forces folder
% save(fullfile(flyFolderPath, 'Fly.mat'), 'Fly_Master');
% return

%% Force means

for i=1:length(Fly_Master)
    S_2_Ratio(i) = Fly_Master(i).Fly.Morphology.total.S_2_Ratio;
    Force_X_mean(i) = (mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(1,:)) + mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(1,:)))/Fly_Master(i).Fly.Morphology.total.weight;
    Force_Y_mean(i) = -(mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(2,:)) + mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(2,:)))/Fly_Master(i).Fly.Morphology.total.weight;
    Force_Z_mean(i) = (mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(3,:)) + mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(3,:)))/Fly_Master(i).Fly.Morphology.total.weight;
end

%% Torques means

for i=1:length(Fly_Master)
    S_3_Ratio(i) = Fly_Master(i).Fly.Morphology.total.S_3_Ratio;
    Moment_Pitch_mean(i) = -mean((Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total(1,:) + Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total(1,:)) / (Fly_Master(i).Fly.Morphology.total.weight * (Fly_Master(i).Fly.Morphology.Wing_LH.wing_length+Fly_Master(i).Fly.Morphology.Wing_RH.wing_length)/2));
    Moment_Roll_mean(i) = (mean((Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total(2,:) + Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total(2,:)) / (Fly_Master(i).Fly.Morphology.total.weight * (Fly_Master(i).Fly.Morphology.Wing_LH.wing_length+Fly_Master(i).Fly.Morphology.Wing_RH.wing_length)/2)));
    Moment_Yaw_mean(i) = -(mean((Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total(3,:) + Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total(3,:)) / (Fly_Master(i).Fly.Morphology.total.weight * (Fly_Master(i).Fly.Morphology.Wing_LH.wing_length+Fly_Master(i).Fly.Morphology.Wing_RH.wing_length)/2)));
end
