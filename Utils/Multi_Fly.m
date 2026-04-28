%% Uncomment to Clear Everything
clear all
clc
%close all
warning off


%% Runtime
current_time = datetime;

%% Fly Numbers
% Get a list of all items in 'Data_Sets'
allItems = dir(fullfile('Data_Sets'));

% Keep only folders and remove '.' and '..'
isFolder = [allItems.isdir];
folderNames = {allItems(isFolder).name};
folderNames = folderNames(~ismember(folderNames, {'.', '..'}));

% Keep only folders matching pattern 'Fly_' followed by digits
flyFolderMask = ~cellfun(@isempty, regexp(folderNames, '^fly_\d+$', 'once'));
flyFolders = folderNames(flyFolderMask);

% Extract numeric part from folder names (e.g., 'Fly_00001' -> 1)
Fly_Numbers = cellfun(@(x) str2double(regexp(x, '\d+', 'match', 'once')), flyFolders);


%%
fly_count = 1;
for i=1:length(Fly_Numbers)
    disp(Fly_Numbers(i))
    % Load Data
    Data_Set_Selector = ['fly_' num2str(Fly_Numbers(i))];

    load(['Data_Sets' filesep Data_Set_Selector filesep 'Inputs' filesep 'Kinematics.mat']);
    load(['Data_Sets' filesep Data_Set_Selector filesep 'Inputs' filesep 'Fly_Data.mat']);


    % Kinematics Length
    Kin_Length = length(FilteredAngleL(:,1));

    % Find each period via peak
    [peaks, peakIndices] = findpeaks(FilteredAngleL(:,1));

    for k=1:3
        try
            %Flip RH to LH data
            if Fly_Data.Chord_Cut_RH(k) < 100 || Fly_Data.Span_Cut_RH(k) < 100
                [Fly_Data.Chord_Cut_LH(k),        Fly_Data.Chord_Cut_RH(k)]        = deal(Fly_Data.Chord_Cut_RH(k),        Fly_Data.Chord_Cut_LH(k));
                [Fly_Data.Span_Cut_LH(k),         Fly_Data.Span_Cut_RH(k)]         = deal(Fly_Data.Span_Cut_RH(k),         Fly_Data.Span_Cut_LH(k));
                [Fly_Data.Stroke_Amplitude_LH(k), Fly_Data.Stroke_Amplitude_RH(k)] = deal(Fly_Data.Stroke_Amplitude_RH(k), Fly_Data.Stroke_Amplitude_LH(k));
                [Fly_Data.Wing_Plane_Angle_LH(k), Fly_Data.Wing_Plane_Angle_RH(k)] = deal(Fly_Data.Wing_Plane_Angle_RH(k), Fly_Data.Wing_Plane_Angle_LH(k));
                [FilteredAngleL,                  FilteredAngleR]                  = deal(FilteredAngleR,                  FilteredAngleL);
            end

            % Cut kinematics for each time period
            if k==1
                Time = 1:200; %Time of simulation before wing cut
                period_text = 'Pre Cut';
            elseif k==2
                Time = 201:600; %Time of simulation after wing cut prior to steady state
                period_text = 'Post Cut';
            else
                Time = 601:Kin_Length; %Time of simulation after steady state occurs
                period_text = 'Steady State';
            end

            Kinematics_LH = FilteredAngleL(Time,:);
            Kinematics_RH = FilteredAngleR(Time,:);

            % Time stamps are indexs and need converted to time domain based on the
            % frame rate
            Frame_Rate = 8000;
            dt = 1/Frame_Rate;

            %% Run Simulation
            Fly_Master(fly_count).Fly  = Analysis(Fly_Data.Chord_Cut_LH(k), Fly_Data.Span_Cut_LH(k), Fly_Data.Chord_Cut_RH(k), Fly_Data.Span_Cut_RH(k), ...
                        Fly_Data.Stroke_Amplitude_LH(k), Fly_Data.Stroke_Amplitude_RH(k), Fly_Data.Wing_Plane_Angle_LH(k), Fly_Data.Wing_Plane_Angle_RH(k), ...
                        Fly_Data.Body_Angle(k), Kinematics_LH, Kinematics_RH, Time, dt, false);


            Fly_Master(fly_count).Chord_Cut_LH          = Fly_Data.Chord_Cut_LH(k);
            Fly_Master(fly_count).Span_Cut_LH           = Fly_Data.Span_Cut_LH(k);
            Fly_Master(fly_count).Chord_Cut_RH          = Fly_Data.Chord_Cut_RH(k);
            Fly_Master(fly_count).Span_Cut_RH           = Fly_Data.Span_Cut_RH(k);
            Fly_Master(fly_count).Wing_Plane_Angle_LH   = Fly_Data.Wing_Plane_Angle_LH(k);
            Fly_Master(fly_count).Wing_Plane_Angle_RH   = Fly_Data.Wing_Plane_Angle_RH(k);
            Fly_Master(fly_count).Stroke_Amplitude_LH   = Fly_Data.Stroke_Amplitude_LH(k);
            Fly_Master(fly_count).Stroke_Amplitude_RH   = Fly_Data.Stroke_Amplitude_RH(k);

            Fly_Master(fly_count).Fly_Num               = Fly_Numbers(i);
            Fly_Master(fly_count).State                 = period_text;

            fly_count = fly_count +1;

        end
    end

    %% Save data
    % Get entries just created in this fly loop
    FlyIndices = find([Fly_Master.Fly_Num] == Fly_Numbers(i));

    % Save them as a struct array
    Fly = Fly_Master(FlyIndices);

    % Construct output folder path for this fly
    outputFolder = fullfile('Data_Sets', ['fly_' num2str(Fly_Numbers(i))], 'Outputs');

    % Create the folder if it doesn't exist
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end

    % Save the Fly struct (this fly's 3 entries)
    save(fullfile(outputFolder, 'Fly_Master.mat'), 'Fly');

end

%% Run Time End
Duration = datetime-current_time

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
