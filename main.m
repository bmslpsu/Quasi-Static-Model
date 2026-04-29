%% Preamble
% Jacob Taylor
% Main code runner for Drosophila Quasi-Steady Model

%% Step 1: Clear Environment
clear all        % Clear all variables, functions, etc
clc              % Clear command window
warning off      % Suppress all warnings
close all hidden % Close all figures, including minimized ones

addpath(genpath("Utils/")) % temporary fix to find Utils functions

%% Step 2: Runtime Timestamp
current_time = datetime;

%% Step 3: Select Data Set Folders
mainFolder = fullfile(pwd, 'Data_Sets');
dirInfo = dir(mainFolder);
isSubFolder = [dirInfo.isdir] & ~ismember({dirInfo.name}, {'.', '..'});
folderNames = {dirInfo(isSubFolder).name};

if isempty(folderNames)
    error('No subfolders found in "%s".', mainFolder);
end

[selectedIdx, ok] = listdlg( ...
    'ListString', folderNames, ...
    'SelectionMode', 'multiple', ...
    'PromptString', 'Select folders for processing:', ...
    'ListSize', [300, 300]);

if ~ok || isempty(selectedIdx)
    disp('No selection made or action canceled.');
    return;
end

selectedFolderNames = string(folderNames(selectedIdx))';
disp('Folders selected.');
clear dirInfo folderNames isSubFolder mainFolder ok selectedIdx

%% Step 4: Process Each Selected Folder
for i = 1:length(selectedFolderNames)
    disp(selectedFolderNames(i))
    Data_Set_Selector = char(selectedFolderNames(i));

    % Load Kinematic and Fly Data
    load(['Data_Sets' filesep Data_Set_Selector filesep 'Inputs' filesep 'Kinematics.mat']);
    load(['Data_Sets' filesep Data_Set_Selector filesep 'Inputs' filesep 'Fly_Data.mat']);

    % Flip RH/LH if RH is damaged
    if Fly_Data.Chord_Cut_RH < 100 || Fly_Data.Span_Cut_RH < 100
        [Fly_Data.Chord_Cut_LH, Fly_Data.Chord_Cut_RH]                  = deal(Fly_Data.Chord_Cut_RH, Fly_Data.Chord_Cut_LH);
        [Fly_Data.Span_Cut_LH, Fly_Data.Span_Cut_RH]                    = deal(Fly_Data.Span_Cut_RH, Fly_Data.Span_Cut_LH);
        [Fly_Data.Stroke_Amplitude_LH, Fly_Data.Stroke_Amplitude_RH]    = deal(Fly_Data.Stroke_Amplitude_RH, Fly_Data.Stroke_Amplitude_LH);
        [Fly_Data.Wing_Plane_Angle_LH, Fly_Data.Wing_Plane_Angle_RH]    = deal(Fly_Data.Wing_Plane_Angle_RH, Fly_Data.Wing_Plane_Angle_LH);
        [FilteredAngleL, FilteredAngleR]                                = deal(FilteredAngleR, FilteredAngleL);
    end

    % Assign Kinematics and Time
    Period = length(FilteredAngleL);
    Kinematics_LH = FilteredAngleL;
    Kinematics_RH = FilteredAngleR;

    % Run Analysis
    Fly_Master(i).Fly = Analysis(Fly_Data.Chord_Cut_LH, Fly_Data.Span_Cut_LH, Fly_Data.Chord_Cut_RH, Fly_Data.Span_Cut_RH, ...
        Fly_Data.Stroke_Amplitude_LH, Fly_Data.Stroke_Amplitude_RH, Fly_Data.Wing_Plane_Angle_LH, Fly_Data.Wing_Plane_Angle_RH, ...
        Fly_Data.Body_Angle, Kinematics_LH, Kinematics_RH, Period, 1/Frame_Rate, true);

    % Save Parameters
    Fly_Master(i).Chord_Cut_LH          = Fly_Data.Chord_Cut_LH;
    Fly_Master(i).Span_Cut_LH           = Fly_Data.Span_Cut_LH;
    Fly_Master(i).Chord_Cut_RH          = Fly_Data.Chord_Cut_RH;
    Fly_Master(i).Span_Cut_RH           = Fly_Data.Span_Cut_RH;
    Fly_Master(i).Wing_Plane_Angle_LH   = Fly_Data.Wing_Plane_Angle_LH;
    Fly_Master(i).Wing_Plane_Angle_RH   = Fly_Data.Wing_Plane_Angle_RH;
    Fly_Master(i).Stroke_Amplitude_LH   = Fly_Data.Stroke_Amplitude_LH;
    Fly_Master(i).Stroke_Amplitude_RH   = Fly_Data.Stroke_Amplitude_RH;
    Fly_Master(i).Fly_Num               = Fly_Data.Fly_Num;
    Fly_Master(i).Attributes            = Fly_Data.Attributes;

    %% Step 5: Save Processed Data
    saveFolder = fullfile('Data_Sets', Data_Set_Selector, 'Outputs');
    Fly_Data = Fly_Master(i);
    saveFile = 'Fly_Data.mat';

    if ~exist(saveFolder, 'dir')
        mkdir(saveFolder);
    end

    save(fullfile(saveFolder, saveFile), 'Fly_Data');
end

%% Step 6: Report Total Script Runtime
disp(datetime - current_time)

% Clear all the temporary variables, leave only Fly_Master
clear -regexp ^((?!Fly_Master).)*$