function selectedFolderNames = select_datasets(allowMultiSelect)
% SELECT_DATASETS - opens a dialogue box for selecting datasets
%
% Inputs:
%   allowMultiSelect - If multiple selections are permitted in the dialogue
%       box. Defaults to `true`
%
% Outputs:
%   selectedFolderNames - returns an array of string names if successful,
%       empty otherwise

arguments
    allowMultiSelect (1,1) logical = true
end

% get file tree
mainFolder = fullfile(pwd, 'Data_Sets');
dirInfo = dir(mainFolder);
isSubFolder = [dirInfo.isdir] & ~ismember({dirInfo.name}, {'.', '..'});
folderNames = {dirInfo(isSubFolder).name};

% check if data exists
if isempty(folderNames)
    error('No subfolders found in "%s".', mainFolder);
end

% handle single or multiple selection logic
selMode = "single";
pString = "Select folder for processing:";
if allowMultiSelect
    selMode = "multiple";
    pString = "Select folders for processing:";
end

% prompt user with dialogue box
[selectedIdx, ok] = listdlg( ...
    'ListString', folderNames, ...
    'SelectionMode', selMode, ...
    'PromptString', pString, ...
    'ListSize', [300, 300]);

if ~ok || isempty(selectedIdx)
    disp('No selection made or action canceled.');
    selectedFolderNames = [];
else
    selectedFolderNames = string(folderNames(selectedIdx))';
end
end