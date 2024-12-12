function [Wing_Shape_lh, Wing_Shape_rh, Body_Shape, Joint] = wingPlotGUI(Wing_Shape_lh, Wing_Shape_rh, Body_Shape, Wing_Shapes_Given, lhSpanwiseCut, lhChordwiseCut, rhSpanwiseCut, rhChordwiseCut)
   
    % Global variables
    global Wing_Shape_lh Wing_Shape_rh Body_Shape Joint
    

    if Wing_Shapes_Given == true
        [~, ~, ~, ~, ~, ~, lhWingLength_Max, lhChordLength_Max, rhWingLength_Max, rhChordLength_Max] = Standard_Wing(0, 0, 100, 100, 0, 0, 100, 100);
        Wing_Chossen(lhWingLength_Max, lhChordLength_Max, lhSpanwiseCut, lhChordwiseCut, rhWingLength_Max, rhChordLength_Max, rhSpanwiseCut, rhChordwiseCut, false)
        return
    end

    % Create a UI figure
    fig = uifigure('Position', [100 100 1000 400], 'Name', 'Wing Plot GUI');

    % Create a grid layout with adjusted column width
    grid = uigridlayout(fig, [1, 2]);
    grid.ColumnWidth = {'1.1x', '2x'};


    % Initial plot
    [~, ~, ~, ~, ~, ~, lhWingLength_Max, lhChordLength_Max, rhWingLength_Max, rhChordLength_Max] = Standard_Wing(0, 0, 100, 100, 0, 0, 100, 100);
  
    
    % Create a panel for the input fields
    inputPanel = uipanel(grid, 'Title', 'Inputs');
    inputGrid = uigridlayout(inputPanel, [6, 3]);
    inputGrid.RowHeight = {'1x', '1x', '1x', '1x', '1x', '1x'};
    inputGrid.ColumnWidth = {'1x', '1x', '1x'};
    
    % Add input fields and labels for LH wing
    uilabel(inputGrid, 'Text', '', 'HorizontalAlignment', 'right');
    uilabel(inputGrid, 'Text', 'LH Wing', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    uilabel(inputGrid, 'Text', 'RH Wing', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    
    uilabel(inputGrid, 'Text', 'Wing Length (m)', 'HorizontalAlignment', 'right');
    %set min/max
    lhWingLengthField = uieditfield(inputGrid, 'numeric', 'value',lhWingLength_Max);
    rhWingLengthField = uieditfield(inputGrid, 'numeric', 'value',rhWingLength_Max);
    
    uilabel(inputGrid, 'Text', 'Chord Length (m)', 'HorizontalAlignment', 'right');
    %set min/max
    lhChordLengthField = uieditfield(inputGrid, 'numeric', 'value',lhChordLength_Max);
    rhChordLengthField = uieditfield(inputGrid, 'numeric', 'value',rhChordLength_Max);
    
    uilabel(inputGrid, 'Text', 'Spanwise Cut (%)', 'HorizontalAlignment', 'right');
    %set min/max
    lhSpanwiseCutField = uieditfield(inputGrid, 'numeric', 'Value', 100);
    rhSpanwiseCutField = uieditfield(inputGrid, 'numeric', 'Value', 100);
    
    uilabel(inputGrid, 'Text', 'Chordwise Cut (%)', 'HorizontalAlignment', 'right');
    %set min/max
    lhChordwiseCutField = uieditfield(inputGrid, 'numeric', 'Value', 100);
    rhChordwiseCutField = uieditfield(inputGrid, 'numeric', 'Value', 100);

    % Create axes for the plot
    ax = uiaxes(grid);
    hold(ax, 'on');
    title(ax, 'Wing Plot');
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    axis(ax, 'equal');
    axis(ax,[-4,4,-2,4])
    title(ax, 'Wing Plot');

    %grid(ax, 'on'); 

    % Initial plot
    updatePlot(lhWingLength_Max, lhChordLength_Max, 100, 100, rhWingLength_Max, rhChordLength_Max, 100, 100, ax);
    hold(ax, 'off');
      
    % Add reset button
    resetButton = uibutton(inputGrid, 'push', ...
        'Text', 'Reset', ...
        'ButtonPushedFcn', @(btn, event) resetValues( ...
        lhWingLengthField, lhChordLengthField, lhSpanwiseCutField, lhChordwiseCutField, rhWingLengthField, rhChordLengthField, ...
        rhSpanwiseCutField, rhChordwiseCutField, lhWingLength_Max, lhChordLength_Max, rhWingLength_Max, rhChordLength_Max, ax));

    % Add update button
    updateButton = uibutton(inputGrid, 'push', ...
        'Text', 'Update Plot', ...
        'ButtonPushedFcn', @(btn, event) updatePlot( ...
            lhWingLengthField.Value, lhChordLengthField.Value, lhSpanwiseCutField.Value, lhChordwiseCutField.Value, ...
            rhWingLengthField.Value, rhChordLengthField.Value, rhSpanwiseCutField.Value, rhChordwiseCutField.Value, ax));

    % Add continue button
    continueButton = uibutton(inputGrid, 'push', ...
        'Text', 'Continue', ...
        'ButtonPushedFcn', @(btn, event) Wing_Chossen(...
            lhWingLengthField.Value, lhChordLengthField.Value, lhSpanwiseCutField.Value, lhChordwiseCutField.Value, ...
            rhWingLengthField.Value, rhChordLengthField.Value, rhSpanwiseCutField.Value, rhChordwiseCutField.Value, fig));

    % Wait until uiresume is called
    uiwait(fig);
end

% Function to reset values
function resetValues(lhWingLengthField, lhChordLengthField, lhSpanwiseCutField, lhChordwiseCutField, rhWingLengthField, rhChordLengthField, rhSpanwiseCutField, rhChordwiseCutField, lhWingLength_Max, lhChordLength_Max, rhWingLength_Max, rhChordLength_Max, ax)
    
    % Reset all input fields to initial values
    lhWingLengthField.Value = lhWingLength_Max;
    lhChordLengthField.Value = lhChordLength_Max;
    lhSpanwiseCutField.Value = 100;
    lhChordwiseCutField.Value = 100;

    rhWingLengthField.Value = rhWingLength_Max;
    rhChordLengthField.Value = rhChordLength_Max;
    rhSpanwiseCutField.Value = 100;
    rhChordwiseCutField.Value = 100;

    % Update plot with initial values
    updatePlot(lhWingLength_Max, lhChordLength_Max, 100, 100, rhWingLength_Max, rhChordLength_Max, 100, 100, ax);
end

% Function to update image
function [Wing_left_x_data, Wing_left_y_data, Wing_left_z_data, Wing_right_x_data, Wing_right_y_data, Wing_right_z_data] = updatePlot(lhWingLength, lhChordLength, lhSpanwiseCut, lhChordwiseCut, rhWingLength, rhChordLength, rhSpanwiseCut, rhChordwiseCut, ax)


    cla(ax);  % Clear existing plot

    
    [Wing_left_x_data, Wing_left_y_data, Wing_left_z_data, Wing_right_x_data, Wing_right_y_data, Wing_right_z_data] = Standard_Wing(lhWingLength, lhChordLength, lhSpanwiseCut, lhChordwiseCut, rhWingLength, rhChordLength, rhSpanwiseCut, rhChordwiseCut);
    
    hold(ax, 'on');

    % Plot the LH dataset
    plot3(ax, Wing_left_x_data, Wing_left_y_data, Wing_left_z_data, '-k', 'LineWidth', 2);
    fill3(ax, Wing_left_x_data, Wing_left_y_data, Wing_left_z_data, 'g', 'FaceAlpha', .9, 'EdgeColor', 'none','DisplayName',"Left Hand");

    % Plot the RH dataset
    plot3(ax, Wing_right_x_data, Wing_right_y_data, Wing_right_z_data, '-k', 'LineWidth', 2);
    fill3(ax, Wing_right_x_data, Wing_right_y_data, Wing_right_z_data, 'r', 'FaceAlpha', .9, 'EdgeColor', 'none','DisplayName',"Right Hand");

    Standard_Body(ax);
  
    legendEntries = ["";'LH Wing';"";'RH Wing'];
    legend(ax, legendEntries);
    hold(ax, 'off');
end

function Wing_Chossen(lhWingLength, lhChordLength, lhSpanwiseCut, lhChordwiseCut, rhWingLength, rhChordLength, rhSpanwiseCut, rhChordwiseCut, fig)
    [Wing_left_x_data, Wing_left_y_data, Wing_left_z_data, Wing_right_x_data, Wing_right_y_data, Wing_right_z_data] = Standard_Wing(lhWingLength, lhChordLength, lhSpanwiseCut, lhChordwiseCut, rhWingLength, rhChordLength, rhSpanwiseCut, rhChordwiseCut);

    global Wing_Shape_lh Wing_Shape_rh Body_Shape Joint

    [Body_Shape, Joint] = Standard_Body(0);

    %lh wing
    Wing_Shape_lh.Wing_x = Wing_left_x_data;
    Wing_Shape_lh.Wing_y = Wing_left_y_data;
    Wing_Shape_lh.Wing_z = Wing_left_z_data;
    Wing_Shape_lh.Wing_root_index = find(Wing_left_x_data == max(Wing_left_x_data));
    Wing_Shape_lh.Wing_root = [Wing_left_x_data(Wing_Shape_lh.Wing_root_index), Wing_left_y_data(Wing_Shape_lh.Wing_root_index)];
    
    Max_1 = find(Wing_left_x_data == min(Wing_left_x_data),1,"first");
    Max_2 = find(Wing_left_x_data == min(Wing_left_x_data),1,"last") ; 
    if Max_1 ~= Max_2
        Max_12 = Max_1;
    else
        Max_12 = round(Max_1 + (Max_2-Max_1)/2);
    end

    Wing_Shape_lh.Wing_tip_index = Max_12;
    Wing_Shape_lh.Wing_tip = [Wing_left_x_data(Max_12), Wing_left_y_data(Max_12)];

    %rh wing
    Wing_Shape_rh.Wing_x = Wing_right_x_data;
    Wing_Shape_rh.Wing_y = Wing_right_y_data;
    Wing_Shape_rh.Wing_z = Wing_right_z_data;
    Wing_Shape_rh.Wing_root_index = find(Wing_right_x_data == min(Wing_right_x_data));
    Wing_Shape_rh.Wing_root = [Wing_right_x_data(Wing_Shape_rh.Wing_root_index), Wing_right_y_data(Wing_Shape_rh.Wing_root_index)];

    Min_1 = find(Wing_right_x_data == max(Wing_right_x_data),1,"first");
    Min_2 = find(Wing_right_x_data == max(Wing_right_x_data),1,"last");   
    if Min_1 ~= Min_2
        Min_12 = Min_1;
    else
        Min_12 = round(Min_1 + (Min_2-Min_1)/2);
    end
    
    Wing_Shape_rh.Wing_tip_index = Min_12;
    Wing_Shape_rh.Wing_tip = [Wing_right_x_data(Min_12), Wing_right_y_data(Min_12)];

    % Skip figure portion 
    if fig == false
        return
    end

    uiresume(fig);  % Resume execution of the script
    close(fig);  % Close the figure
end
