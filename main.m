%% Preamble
% Jacob Taylor
% Main code runner for Drosophila Quasi-Steady Model

%% Set Up Environment
clear
clc
close all hidden

% When using delaunayTriangulation (Utils.mass_and_inertia), a warning
% might be thrown (DupPtsWarnId). This warning simply informs you that
% duplicate points have been removed, which is not concerning for our use
% cases. We will assume the given morphology is correct, thus, we'll 
% suppress this warning for the duration of the script
warning('off','MATLAB:delaunayTriangulation:DupPtsWarnId');

import Utils.Analysis
import Utils.select_datasets

% Runtime Timestamp
current_time = datetime;

%% Select Data Set Folders
selectedFolderNames = select_datasets();
disp('Folders selected.');

%% Process Each Selected Folder
for i = 1:length(selectedFolderNames)
    disp(selectedFolderNames(i))
    Data_Set_Selector = char(selectedFolderNames(i));

    % Load Kinematic and Fly Data
    load(...
        ['Data_Sets' filesep Data_Set_Selector filesep 'Inputs' filesep ...
            'Kinematics.mat'],...
        "FilteredAngleL","FilteredAngleR","Frame_Rate");
    load(...
        ['Data_Sets' filesep Data_Set_Selector filesep 'Inputs' filesep ...
            'Fly_Data.mat'],...
        "Fly_Data");

    % Run Analysis
    Fly_Master(i).Fly = Analysis(...
        Fly_Data,...
        FilteredAngleL,...
        FilteredAngleR,...
        length(FilteredAngleL),...
        1/Frame_Rate);

    % Save the Fly_Data Parameters into Fly_Master
    % TODO: move this into Analysis.m
    Fly_Master(i).Fly.Morphology.Wing_LH.Chord_Cut ...
        = Fly_Data.Chord_Cut_LH;
    Fly_Master(i).Fly.Morphology.Wing_LH.Span_Cut ...
        = Fly_Data.Span_Cut_LH;
    Fly_Master(i).Fly.Morphology.Wing_RH.Chord_Cut ...
        = Fly_Data.Chord_Cut_RH;
    Fly_Master(i).Fly.Morphology.Wing_RH.Span_Cut ...
        = Fly_Data.Span_Cut_RH;
    Fly_Master(i).Fly.Kinematics.LH.Stroke_Amplitude ...
        = Fly_Data.Stroke_Amplitude_LH;
    Fly_Master(i).Fly.Kinematics.RH.Stroke_Amplitude ...
        = Fly_Data.Stroke_Amplitude_RH;
    Fly_Master(i).Fly_Num = Fly_Data.Fly_Num;
    Fly_Master(i).Attributes = Fly_Data.Attributes;

    %% Save Processed Data
    saveFolder = fullfile('Data_Sets', Data_Set_Selector, 'Outputs');
    Fly_Data = Fly_Master(i);
    saveFile = 'Fly_Data.mat';

    if ~exist(saveFolder, 'dir')
        mkdir(saveFolder);
    end

    save(fullfile(saveFolder, saveFile), 'Fly_Data');
end

%% Close out

% Turn the suppressed warning back on
warning('on','MATLAB:delaunayTriangulation:DupPtsWarnId');

% Report Total Script Runtime
disp(datetime - current_time)

% Clear all the temporary variables, leave only Fly_Master
clear -regexp ^((?!Fly_Master).)*$