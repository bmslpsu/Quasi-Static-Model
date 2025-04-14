Fly_Data_Master = Fly_Data;

    
    Frame_Rate = 28000;

    Fly_Data.Attributes = ['Normal'];

    % Output folder (as string but convert to char for file ops)
    outputFolder = fullfile('Data_Sets', ['Robot_3_Normal'], 'Inputs');

    % Create folder if it doesn't exist
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end

    % Save files
    save(fullfile(outputFolder, 'Kinematics.mat'), 'FilteredAngleL', 'FilteredAngleR', 'Frame_Rate');
    save(fullfile(outputFolder, 'Fly_Data.mat'), 'Fly_Data');

    fprintf('✅ Processed and saved Fly %d to %s\n', i, outputFolder);
