function wingImageSelector
    % Declare the global variable
    global buttonChoice;
    
    % Initialize the variable
    buttonChoice = 0;
    
    % Create a UI figure
    fig = uifigure('Position', [100 100 300 150], 'Name', 'Wing Image Selector');
    
    % Create a button for using the standard wing
    btnStandard = uibutton(fig, 'push', ...
        'Text', 'Use Standard Wing', ...
        'Position', [50 90 200 30], ...
        'ButtonPushedFcn', @(btnStandard, event) useStandardWing(fig));
    
    % Create a button for uploading a custom wing image
    btnUpload = uibutton(fig, 'push', ...
        'Text', 'Upload Custom Wing', ...
        'Position', [50 50 200 30], ...
        'ButtonPushedFcn', @(btnUpload, event) uploadCustomWing(fig));

    % Wait until uiresume is called
    uiwait(fig);
end

function useStandardWing(fig)
    global Wing_Upload_Type;
    Wing_Upload_Type = 1;
    uiresume(fig);  % Resume execution of the script
    close(fig);  % Close the figure
end

function uploadCustomWing(fig)
    global Wing_Upload_Type;
    Wing_Upload_Type = 2;
    uiresume(fig);  % Resume execution of the script
    close(fig);  % Close the figure
end
