%% Preamble
% Vector plots comparing force and torque before and after wing damage
% Jacob Taylor
% Updated to include interactive fly selection and enhanced legend explanations


%% Step 1: Clear and Setup
clc            % Clear command window
warning off    % Suppress all warnings
% close all    % Uncomment to close any open figures

% Snapshot of variables that existed before the script
vars_before = who;



%% Step 2: Fly Selection (based only on unique Fly_Num)
% Get unique Fly_Nums
allFlyNums = [Fly_Master.Fly_Num];
uniqueFlyNums = unique(allFlyNums);

% Create selection list with just Fly numbers
flyListStr = arrayfun(@(n) sprintf("Fly #%d", n), uniqueFlyNums, 'UniformOutput', false);

[selectedIdx, ok] = listdlg( ...
    'PromptString', 'Select flies to include in vector analysis:', ...
    'ListString', flyListStr, ...
    'SelectionMode', 'multiple', ...
    'ListSize', [300 300], ...
    'Name', 'Fly Selector');

if ~ok || isempty(selectedIdx)
    disp('No selection made. Aborting...');
    return;
end

selectedFlyNums = uniqueFlyNums(selectedIdx); % Vector of selected Fly_Num

%% Step 3: Compute Mean Forces and Torques for All Flies
for i = 1:length(Fly_Master)
    S_2_Ratio(i) = Fly_Master(i).Fly.Morphology.total.S_2_Ratio;
    Force_X_mean(i) = (mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(1,:)) + ...
                       mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(1,:))) / ...
                       Fly_Master(i).Fly.Morphology.total.weight;
    Force_Y_mean(i) = -(mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(2,:)) + ...
                        mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(2,:))) / ...
                        Fly_Master(i).Fly.Morphology.total.weight;
    Force_Z_mean(i) = (mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(3,:)) + ...
                       mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(3,:))) / ...
                       Fly_Master(i).Fly.Morphology.total.weight;

    avgWingLength = (Fly_Master(i).Fly.Morphology.Wing_LH.wing_length + ...
                     Fly_Master(i).Fly.Morphology.Wing_RH.wing_length) / 2;
    normalization = Fly_Master(i).Fly.Morphology.total.weight * avgWingLength;

    Moment_Pitch_mean(i) = -mean((Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total(1,:) + ...
                                  Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total(1,:)) / normalization);
    Moment_Roll_mean(i)  =  mean((Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total(2,:) + ...
                                  Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total(2,:)) / normalization);
    Moment_Yaw_mean(i)   = -mean((Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total(3,:) + ...
                                  Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total(3,:)) / normalization);
end

%% Step 4: Force Vector Plot (from origin)
figure; hold on;
originX = 0; originY = 0; originZ = 0;

colors = lines(10);
num_colors = size(colors, 1);

h_pre = [];
legend_labels_pre = {};
meanX_pre = []; meanY_pre = []; meanZ_pre = [];
meanX_post = []; meanY_post = []; meanZ_post = [];

i = 1;
for k = 1:length(Fly_Master)
    if ismember(Fly_Master(k).Fly_Num, selectedFlyNums)
        color_idx = mod(i-1, num_colors) + 1;
        endX = Force_X_mean(k); endY = Force_Y_mean(k); endZ = Force_Z_mean(k);

        if strcmpi(Fly_Master(k).Attributes, "Pre-Cut")
            h_pre(end+1) = plot3([originX, endX], [originY, endY], [originZ, endZ], ...
                'Color', colors(color_idx, :), 'LineStyle', '-', 'LineWidth', 1.5);
            legend_labels_pre{end+1} = ['Fly ' num2str(Fly_Master(k).Fly_Num)];
            meanX_pre(end+1) = endX;
            meanY_pre(end+1) = endY;
            meanZ_pre(end+1) = endZ;
        elseif strcmpi(Fly_Master(k).Attributes, "Post-Cut")
            plot3([originX, endX], [originY, endY], [originZ, endZ], ...
                'Color', colors(color_idx, :), 'LineStyle', '--', 'LineWidth', 1.5);
            meanX_post(end+1) = endX;
            meanY_post(end+1) = endY;
            meanZ_post(end+1) = endZ;
            i = i + 1;
        end
    end
end

% Mean vector
h_pre(end+1) = plot3([originX, mean(meanX_pre)], [originY, mean(meanY_pre)], [originZ, mean(meanZ_pre)], ...
    'k-', 'LineWidth', 1.5);
legend_labels_pre{end+1} = 'Mean Pre-Cut';

plot3([originX, mean(meanX_post)], [originY, mean(meanY_post)], [originZ, mean(meanZ_post)], ...
    'k--', 'LineWidth', 1.5);

% Dummy handles for legend explanation
h_legend_dummy_pre = plot3(NaN, NaN, NaN, 'k-',  'LineWidth', 1.5);
h_legend_dummy_post = plot3(NaN, NaN, NaN, 'k--', 'LineWidth', 1.5);

legend([h_pre, h_legend_dummy_pre, h_legend_dummy_post], ...
       [legend_labels_pre, 'Pre-Cut (solid)', 'Post-Cut (dashed)'], ...
       'Location', 'eastoutside');

xlabel('Sideward (F/mg)');
ylabel('Forward (F/mg)');
zlabel('Upward (F/mg)');
view(3); axis equal; grid on;
title('Force Vectors Before and After Cut');

%% Step 6: Torque Vector Plot (from origin)
figure; hold on;
originX = 0; originY = 0; originZ = 0;
h_pre = [];
legend_labels_pre = {};
meanX_pre = []; meanY_pre = []; meanZ_pre = [];
meanX_post = []; meanY_post = []; meanZ_post = [];

i = 1;
for k = 1:length(Fly_Master)
    if ismember(Fly_Master(k).Fly_Num, selectedFlyNums)
        color_idx = mod(i-1, num_colors) + 1;
        endX = Moment_Roll_mean(k); endY = Moment_Pitch_mean(k); endZ = Moment_Yaw_mean(k);

        if strcmpi(Fly_Master(k).Attributes, "Pre-Cut")
            h_pre(end+1) = plot3([originX, endX], [originY, endY], [originZ, endZ], ...
                'Color', colors(color_idx, :), 'LineStyle', '-', 'LineWidth', 1.5);
            legend_labels_pre{end+1} = ['Fly ' num2str(Fly_Master(k).Fly_Num)];
            meanX_pre(end+1) = endX;
            meanY_pre(end+1) = endY;
            meanZ_pre(end+1) = endZ;
        elseif strcmpi(Fly_Master(k).Attributes, "Post-Cut")
            plot3([originX, endX], [originY, endY], [originZ, endZ], ...
                'Color', colors(color_idx, :), 'LineStyle', '--', 'LineWidth', 1.5);
            meanX_post(end+1) = endX;
            meanY_post(end+1) = endY;
            meanZ_post(end+1) = endZ;
            i = i + 1;
        end
    end
end

% Mean vector
h_pre(end+1) = plot3([originX, mean(meanX_pre)], [originY, mean(meanY_pre)], [originZ, mean(meanZ_pre)], ...
    'k-', 'LineWidth', 1.5);
legend_labels_pre{end+1} = 'Mean';

plot3([originX, mean(meanX_post)], [originY, mean(meanY_post)], [originZ, mean(meanZ_post)], ...
    'k--', 'LineWidth', 1.5);

% Dummy handles for legend explanation
h_legend_dummy_pre = plot3(NaN, NaN, NaN, 'k-',  'LineWidth', 1.5);
h_legend_dummy_post = plot3(NaN, NaN, NaN, 'k--', 'LineWidth', 1.5);

legend([h_pre, h_legend_dummy_pre, h_legend_dummy_post], ...
       [legend_labels_pre, 'Pre-Cut (solid)', 'Post-Cut (dashed)'], ...
       'Location', 'eastoutside');

xlabel('Roll (T/mgl)');
ylabel('Pitch (T/mgl)');
zlabel('Yaw (T/mgl)');
view(3); axis equal; grid on;
title('Torque Vectors Before and After Cut');

%% Step 7: Non-Origin Force Vector Plot (Pre-Cut to Post-Cut per Fly)
figure;
hold on;

% Define distinct colors
colors = lines(10);
num_colors = size(colors, 1);

% Initialize storage
originX = [];
originY = [];
originZ = [];
endX = [];
endY = [];
endZ = [];
legend_labels = {};

meanX_pre = [];
meanY_pre = [];
meanZ_pre = [];
meanX_post = [];
meanY_post = [];
meanZ_post = [];

% Start/stop point storage for dashed line
startX = [];
startY = [];
startZ = [];
stopX = [];
stopY = [];
stopZ = [];

% Recompute Force_X_mean, Force_Y_mean, Force_Z_mean since they may have been cleared
Force_X_mean = zeros(1, length(Fly_Master));
Force_Y_mean = zeros(1, length(Fly_Master));
Force_Z_mean = zeros(1, length(Fly_Master));

for i = 1:length(Fly_Master)
    Force_X_mean(i) = (mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(1,:)) + ...
                       mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(1,:))) / ...
                       Fly_Master(i).Fly.Morphology.total.weight;
    Force_Y_mean(i) = -(mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(2,:)) + ...
                        mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(2,:))) / ...
                        Fly_Master(i).Fly.Morphology.total.weight;
    Force_Z_mean(i) = (mean(Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total(3,:)) + ...
                       mean(Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total(3,:))) / ...
                       Fly_Master(i).Fly.Morphology.total.weight;
end

% Loop through Fly_Master to find matching pre- and post-cut for selected flies
for k = 1:length(Fly_Master)
    flyNum_k = Fly_Master(k).Fly_Num;

    if ismember(flyNum_k, selectedFlyNums) && strcmpi(Fly_Master(k).Attributes, "Pre-Cut")
        % Find corresponding Post-Cut entry
        matchIdx = find([Fly_Master.Fly_Num] == flyNum_k & strcmpi(string({Fly_Master.Attributes}), "Post-Cut"), 1);
        if ~isempty(matchIdx)
            % Start point (Pre-Cut)
            originX(end+1) = Force_X_mean(k);
            originY(end+1) = Force_Y_mean(k);
            originZ(end+1) = Force_Z_mean(k);

            meanX_pre(end+1) = Force_X_mean(k);
            meanY_pre(end+1) = Force_Y_mean(k);
            meanZ_pre(end+1) = Force_Z_mean(k);

            % End point (Post-Cut)
            endX(end+1) = Force_X_mean(matchIdx);
            endY(end+1) = Force_Y_mean(matchIdx);
            endZ(end+1) = Force_Z_mean(matchIdx);

            meanX_post(end+1) = Force_X_mean(matchIdx);
            meanY_post(end+1) = Force_Y_mean(matchIdx);
            meanZ_post(end+1) = Force_Z_mean(matchIdx);

            % Start and stop point (redundant, but explicit for plot overlays)
            startX(end+1) = Force_X_mean(k);
            startY(end+1) = Force_Y_mean(k);
            startZ(end+1) = Force_Z_mean(k);
            stopX(end+1) = Force_X_mean(matchIdx);
            stopY(end+1) = Force_Y_mean(matchIdx);
            stopZ(end+1) = Force_Z_mean(matchIdx);

            % Label
            legend_labels{end+1} = sprintf('Fly %d', flyNum_k);
        end
    end
end

% Plot vectors from Pre-Cut to Post-Cut
vector_handles = [];
for i = 1:length(originX)
    color_idx = mod(i-1, num_colors) + 1;
    vector_handles(end+1) = plot3([originX(i), endX(i)], [originY(i), endY(i)], [originZ(i), endZ(i)], ...
        'Color', colors(color_idx, :), 'LineWidth', 1.5);
end

% Plot mean vector
vector_handles(end+1) = plot3([mean(meanX_pre), mean(meanX_post)], [mean(meanY_pre), mean(meanY_post)], [mean(meanZ_pre), mean(meanZ_post)], ...
    'Color', 'k', 'LineWidth', 2);
legend_labels{end+1} = 'Mean';

% Add markers
h_pre = scatter3(startX, startY, startZ, 50, 'k', 'filled', 'o', 'DisplayName', 'Pre Cut (Dot)');
h_post = scatter3(stopX, stopY, stopZ, 100, 'k', 'filled', '^', 'DisplayName', 'Post Cut (Triangle)');
mean_pre = scatter3(mean(meanX_pre), mean(meanY_pre), mean(meanZ_pre), 50, 'k', 'filled', 'o');
mean_post = scatter3(mean(meanX_post), mean(meanY_post), mean(meanZ_post), 50, 'k', 'filled', '^');

for i = 1:length(originX)
    color_idx = mod(i-1, num_colors) + 1;
    scatter3(originX(i), originY(i), originZ(i), 50, colors(color_idx, :), 'filled', 'o');
    scatter3(endX(i), endY(i), endZ(i), 100, colors(color_idx, :), 'filled', '^');
end

xlabel('Sideward (F/mg)');
ylabel('Forward (F/mg)');
zlabel('Upward (F/mg)');
grid on;
view([0, 90]);
title('Force Change Vectors (Pre-Cut to Post-Cut)');

legend([vector_handles, h_pre, h_post], [legend_labels, 'Pre Cut (Dot)', 'Post Cut (Triangle)'], 'Location', 'eastoutside');

%% Step 8: Non-Origin Torque Vector Plot (Pre-Cut to Post-Cut per Fly)
figure;
hold on;

% Initialize storage
originX = [];
originY = [];
originZ = [];
endX = [];
endY = [];
endZ = [];
legend_labels = {};

meanX_pre = [];
meanY_pre = [];
meanZ_pre = [];
meanX_post = [];
meanY_post = [];
meanZ_post = [];

startX = [];
startY = [];
startZ = [];
stopX = [];
stopY = [];
stopZ = [];

% Loop through Fly_Master to find matching pre- and post-cut for selected flies
for k = 1:length(Fly_Master)
    flyNum_k = Fly_Master(k).Fly_Num;

    if ismember(flyNum_k, selectedFlyNums) && strcmpi(Fly_Master(k).Attributes, "Pre-Cut")
        matchIdx = find([Fly_Master.Fly_Num] == flyNum_k & strcmpi(string({Fly_Master.Attributes}), "Post-Cut"), 1);
        if ~isempty(matchIdx)
            originX(end+1) = Moment_Roll_mean(k);
            originY(end+1) = Moment_Pitch_mean(k);
            originZ(end+1) = Moment_Yaw_mean(k);

            meanX_pre(end+1) = Moment_Roll_mean(k);
            meanY_pre(end+1) = Moment_Pitch_mean(k);
            meanZ_pre(end+1) = Moment_Yaw_mean(k);

            endX(end+1) = Moment_Roll_mean(matchIdx);
            endY(end+1) = Moment_Pitch_mean(matchIdx);
            endZ(end+1) = Moment_Yaw_mean(matchIdx);

            meanX_post(end+1) = Moment_Roll_mean(matchIdx);
            meanY_post(end+1) = Moment_Pitch_mean(matchIdx);
            meanZ_post(end+1) = Moment_Yaw_mean(matchIdx);

            startX(end+1) = Moment_Roll_mean(k);
            startY(end+1) = Moment_Pitch_mean(k);
            startZ(end+1) = Moment_Yaw_mean(k);
            stopX(end+1) = Moment_Roll_mean(matchIdx);
            stopY(end+1) = Moment_Pitch_mean(matchIdx);
            stopZ(end+1) = Moment_Yaw_mean(matchIdx);

            legend_labels{end+1} = sprintf('Fly %d', flyNum_k);
        end
    end
end

% Plot vectors
vector_handles = [];
for i = 1:length(originX)
    color_idx = mod(i-1, num_colors) + 1;
    vector_handles(end+1) = plot3([originX(i), endX(i)], [originY(i), endY(i)], [originZ(i), endZ(i)], ...
        'Color', colors(color_idx, :), 'LineWidth', 1.5);
end

vector_handles(end+1) = plot3([mean(meanX_pre), mean(meanX_post)], [mean(meanY_pre), mean(meanY_post)], [mean(meanZ_pre), mean(meanZ_post)], ...
    'Color', 'k', 'LineWidth', 2);
legend_labels{end+1} = 'Mean';

h_pre = scatter3(startX, startY, startZ, 50, 'k', 'filled', 'o', 'DisplayName', 'Pre Cut (Dot)');
h_post = scatter3(stopX, stopY, stopZ, 100, 'k', 'filled', '^', 'DisplayName', 'Post Cut (Triangle)');
mean_pre = scatter3(mean(meanX_pre), mean(meanY_pre), mean(meanZ_pre), 50, 'k', 'filled', 'o');
mean_post = scatter3(mean(meanX_post), mean(meanY_post), mean(meanZ_post), 50, 'k', 'filled', '^');

for i = 1:length(originX)
    color_idx = mod(i-1, num_colors) + 1;
    scatter3(originX(i), originY(i), originZ(i), 50, colors(color_idx, :), 'filled', 'o');
    scatter3(endX(i), endY(i), endZ(i), 100, colors(color_idx, :), 'filled', '^');
end

xlabel('Roll (T/mgl)');
ylabel('Pitch (T/mgl)');
zlabel('Yaw (T/mgl)');
grid on;
view([0, 90]);
title('Torque Change Vectors (Pre-Cut to Post-Cut)');

legend([vector_handles, h_pre, h_post], [legend_labels, 'Pre Cut (Dot)', 'Post Cut (Triangle)'], 'Location', 'eastoutside');

%% Step 9: Clear Created Variables
% Get all current variables
vars_after = who;

% Determine which variables were added by the script
vars_created = setdiff(vars_after, vars_before);

% Clear only the variables created during script execution
clear(vars_created{:});

% Clear the temporary tracking variables too
clear vars_after vars_created vars_before;