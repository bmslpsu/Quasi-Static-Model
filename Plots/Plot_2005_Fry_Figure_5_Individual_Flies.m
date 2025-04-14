%% Preamble
% Jacob Taylor
% 2005 Fry Figure 5 Recreation — Mean Force and Torque per Fly

%% Step 1: Clear and Setup
clc            % Clear command window
warning off    % Suppress all warnings
% close all    % Uncomment to close any open figures

% Snapshot of variables that existed before the script
vars_before = who;

%% Step 2: Fly Selection via Popup
% Create list of fly display names for selection
flyOptions = arrayfun(@(f) sprintf('Fly #%d - %s', f.Fly_Num, f.Attributes), ...
                      Fly_Master, 'UniformOutput', false);

% User selection
[selectedIdx, ok] = listdlg( ...
    'PromptString', 'Select flies to include in Fry 2005 Figure 5:', ...
    'ListString', flyOptions, ...
    'SelectionMode', 'multiple', ...
    'ListSize', [300, 300], ...
    'Name', 'Select Flies');

% Abort if canceled
if ~ok || isempty(selectedIdx)
    disp('No flies selected. Script aborted.');
    return;
end

%% Step 3: Setup and Allocation
units_force = 1e-6;
units_torque = 1e-10;

numTimePoints = 100;
numFlies = length(selectedIdx);
normalizedTime = linspace(0, 1, numTimePoints);

% Allocate force/torque mean matrices
meanForcesSumAllFlies  = zeros(3, numTimePoints, numFlies);
meanTorquesSumAllFlies = zeros(3, numTimePoints, numFlies);

% Unique color for each fly
flyColors = lines(numFlies);

%% Step 4: Analyze Each Fly
for flyIdx = 1:numFlies
    fly = Fly_Master(selectedIdx(flyIdx)).Fly;

    phi       = fly.Kinematics.LH.phi;
    force_lh  = fly.Dynamics.Frame_Body.LH.Force_Total * units_force .* [1; -1; 1];
    force_rh  = fly.Dynamics.Frame_Body.RH.Force_Total * units_force .* [1; -1; 1];
    torque_lh = fly.Dynamics.Frame_Body.LH.Torque_Total * units_torque .* [-1; 1; 1];
    torque_rh = fly.Dynamics.Frame_Body.RH.Torque_Total * units_torque .* [-1; 1; 1];

    % Find stroke peaks
    [~, peakIndices] = findpeaks(phi);

    % Skip fly if fewer than 2 peaks
    if length(peakIndices) < 2
        continue;
    end

    % Use first complete stroke only
    startIdx = peakIndices(1);
    endIdx   = peakIndices(2);

    % Normalize force
    forceSum_norm = interp1(linspace(0, 1, endIdx - startIdx + 1), ...
        (force_lh(:, startIdx:endIdx) + force_rh(:, startIdx:endIdx))', ...
        normalizedTime, 'linear', 'extrap')';

    % Normalize torque
    torqueSum_norm = interp1(linspace(0, 1, endIdx - startIdx + 1), ...
        (torque_lh(:, startIdx:endIdx) + torque_rh(:, startIdx:endIdx))', ...
        normalizedTime, 'linear', 'extrap')';

    % Store in fly-wise matrices
    meanForcesSumAllFlies(:, :, flyIdx)  = forceSum_norm;
    meanTorquesSumAllFlies(:, :, flyIdx) = torqueSum_norm;
end

%% Step 5: Compute Mean & Std Across All Flies
meanForcesSumAcrossFlies  = mean(meanForcesSumAllFlies, 3);
stdForcesSumAcrossFlies   = std(meanForcesSumAllFlies, 0, 3);
meanTorquesSumAcrossFlies = mean(meanTorquesSumAllFlies, 3);
stdTorquesSumAcrossFlies  = std(meanTorquesSumAllFlies, 0, 3);

%% Step 6: Plot - Fry 2005 Figure 5 Style
figure;
components   = {'Upward Force', 'Yaw Torque', 'Forward Force', ...
                'Roll Torque', 'Sideward Force', 'Pitch Torque'};
ylabels      = {'Force (N)', 'Torque (Nm)', 'Force (N)', ...
                'Torque (Nm)', 'Force (N)', 'Torque (Nm)'};
ylim_values  = [-5e-5, 5e-5; -5e-8, 5e-8; -5e-5, 5e-5; -5e-8, 5e-8; -5e-5, 5e-5; -5e-8, 5e-8];
indices_force  = [3, 3, 2, 2, 1, 1];
indices_torque = [3, 3, 2, 2, 1, 1];

for i = 1:6
    subplot(3, 2, i);
    hold on;

    % Shaded std dev area
    if mod(i, 2) == 1
        fill([normalizedTime, fliplr(normalizedTime)], ...
             [meanForcesSumAcrossFlies(indices_force(i), :) + stdForcesSumAcrossFlies(indices_force(i), :), ...
              fliplr(meanForcesSumAcrossFlies(indices_force(i), :) - stdForcesSumAcrossFlies(indices_force(i), :))], ...
             [0.8, 0.8, 0.8], 'EdgeColor', 'none');
        plot(normalizedTime, meanForcesSumAcrossFlies(indices_force(i), :), 'k', 'LineWidth', 2);
    else
        fill([normalizedTime, fliplr(normalizedTime)], ...
             [meanTorquesSumAcrossFlies(indices_torque(i), :) + stdTorquesSumAcrossFlies(indices_torque(i), :), ...
              fliplr(meanTorquesSumAcrossFlies(indices_torque(i), :) - stdTorquesSumAcrossFlies(indices_torque(i), :))], ...
             [0.8, 0.8, 0.8], 'EdgeColor', 'none');
        plot(normalizedTime, meanTorquesSumAcrossFlies(indices_torque(i), :), 'k', 'LineWidth', 2);
    end

    % Plot individual fly lines
    for flyIdx = 1:numFlies
        if mod(i, 2) == 1
            plot(normalizedTime, meanForcesSumAllFlies(indices_force(i), :, flyIdx), ...
                'Color', flyColors(flyIdx, :), 'LineWidth', 1);
        else
            plot(normalizedTime, meanTorquesSumAllFlies(indices_torque(i), :, flyIdx), ...
                'Color', flyColors(flyIdx, :), 'LineWidth', 1);
        end
    end

    % X-labels and legend
    if i == 5 || i == 6
        xlabel('Normalized Stroke Cycle');
    else
        set(gca, 'XColor', 'none');
    end

    if i == 6
        legend({'Std Dev', 'Overall Mean', flyOptions{selectedIdx}}, ...
               'Location', 'bestoutside');
    end

    title(components{i});
    ylabel(ylabels{i});
    ylim(ylim_values(i, :));
    grid on;
end

sgtitle('2005 Fry Figure 5: Force and Torque Means Over Stroke Cycle - Post Cut');

%% Step 7: Clear Created Variables
% Get all current variables
vars_after = who;

% Determine which variables were added by the script
vars_created = setdiff(vars_after, vars_before);

% Clear only the variables created during script execution
clear(vars_created{:});

% Clear the temporary tracking variables too
clear vars_after vars_created vars_before;

