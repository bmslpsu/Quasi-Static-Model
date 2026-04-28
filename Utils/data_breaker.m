%% Fly Numbers
% Get a list of all items in 'Data_Sets'
allItems = dir(fullfile('Data_Sets'));

% Keep only folders and remove '.' and '..'
isFolder = [allItems.isdir];
folderNames = {allItems(isFolder).name};
folderNames = folderNames(~ismember(folderNames, {'.', '..'}));

% Keep only folders matching pattern 'fly_' followed by digits
flyFolderMask = ~cellfun(@isempty, regexp(folderNames, '^fly_\d+$', 'once'));
flyFolders = folderNames(flyFolderMask);

% Extract numeric part from folder names (e.g., 'fly_00001' -> 1)
Fly_Numbers = cellfun(@(x) str2double(regexp(x, '\d+', 'match', 'once')), flyFolders);

%% Process Each Fly
for i = 1:length(Fly_Numbers)
    disp(Fly_Numbers(i));

    % Set selector string
    Data_Set_Selector = ['fly_' num2str(Fly_Numbers(i))];

    % Process each phase
    for k = 1:3
        % Load input files
        inputFolder = fullfile('Data_Sets', Data_Set_Selector, 'Inputs');
        load(fullfile(inputFolder, 'Kinematics.mat'));
        load(fullfile(inputFolder, 'Fly_Data.mat'));

        try
            % Determine cut and assign state
            if k == 1
                state = 'pre_cut';
                FilteredAngleL = FilteredAngleL(1:200,:);
                FilteredAngleR = FilteredAngleR(1:200,:);
                Fly_Data = Fly_Data(1,:);
            elseif k == 2
                state = 'post_cut';
                FilteredAngleL = FilteredAngleL(201:600,:);
                FilteredAngleR = FilteredAngleR(201:600,:);
                Fly_Data = Fly_Data(2,:);
            elseif k == 3
                if length(FilteredAngleL)<620
                    continue 
                end
                state = 'steady_state';
                FilteredAngleL = FilteredAngleL(601:end,:);
                FilteredAngleR = FilteredAngleR(601:end,:);
                Fly_Data = Fly_Data(3,:);
            end
            Fly_Data.Fly_Num = Fly_Numbers(i);
            Frame_Rate = 8000;
        catch ME
            warning('Error processing fly %d, state %d: %s', Fly_Numbers(i), k, ME.message);
            continue
        end

        % Output folder
        outputFolder = fullfile('Data_Sets', ['fly_' num2str(Fly_Numbers(i)) '_' state], 'Inputs');

        if ~exist(outputFolder, 'dir')
            mkdir(outputFolder);
        end

        % Save files
        save(fullfile(outputFolder, 'Kinematics.mat'), 'FilteredAngleL', 'FilteredAngleR', 'Frame_Rate');
        save(fullfile(outputFolder, 'Fly_Data.mat'), 'Fly_Data');
    end
end
