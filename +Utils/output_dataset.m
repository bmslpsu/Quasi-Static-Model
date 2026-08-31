function output_dataset(dataset_name,Fly_Data)
% OUTPUT_DATASET - Save data in its respective Output folder

saveFolder = fullfile('Data_Sets', dataset_name, 'Outputs');
saveFile = 'Fly_Data.mat';

if ~exist(saveFolder, 'dir')
    mkdir(saveFolder);
end

save(fullfile(saveFolder, saveFile), 'Fly_Data');

end