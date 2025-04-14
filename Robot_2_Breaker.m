Fly_Data_Master = Fly_Data;

for i = 1:height(Fly_Data_Master)

    Fly_Data = table2struct(Fly_Data_Master(i, :));

    % Clean up folder name
    amp = Fly_Data.Stroke_Amplitude_LH;
    state = num2str(amp);
    
    Frame_Rate = 28000;
    Fly_Data.Fly_Num = i;
    Fly_Data.Attributes = ['Amplitude_Percentage' state];

    FilteredAngleL(1,:) = amp*FilteredAngleL(1,:);

    % Output folder (as string but convert to char for file ops)
    outputFolder = fullfile('Data_Sets', ['Robot_2_Amplitude_' state], 'Inputs');

    % Create folder if it doesn't exist
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end

    % Save files
    save(fullfile(outputFolder, 'Kinematics.mat'), 'FilteredAngleL', 'FilteredAngleR', 'Frame_Rate');
    save(fullfile(outputFolder, 'Fly_Data.mat'), 'Fly_Data');

    fprintf('✅ Processed and saved Fly %d to %s\n', i, outputFolder);

end