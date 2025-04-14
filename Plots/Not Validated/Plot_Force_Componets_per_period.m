%% Preamble
% Jacob Taylor
% Analyze and Plot Forces for One Selected Fly (Pre-cut, Post-cut, Steady-state)

clc;
warning off;
% close all;  % Uncomment to close existing figures

% Take workspace snapshot
vars_before = who;

%% Step 1: Fly Selection via Popup (Allow Single Fly Selection)
flyOptions = arrayfun(@(f) sprintf('Fly #%d', f.Fly_Num), Fly_Master, 'UniformOutput', false);

[selectedIdx, ok] = listdlg( ...
    'PromptString', 'Select one fly for analysis:', ...
    'ListString', flyOptions, ...
    'SelectionMode', 'single', ...
    'ListSize', [300, 300], ...
    'Name', 'Select Fly');

if ~ok || isempty(selectedIdx)
    disp('No fly selected. Aborting.');
    return;
end

selectedFlyNum = Fly_Master(selectedIdx).Fly_Num; % Get the selected fly number

%% Step 2: Setup
units_force  = 1e-6;
units_torque = 1e-10;
numTimePoints = 100;
normalizedTime = linspace(0, 1, numTimePoints);

% Allocate arrays for storing results
meanForcesSumAllFlies   = zeros(3, numTimePoints, 3); % [x, y, z] x time x 3 states (pre-cut, post-cut, steady-state)
meanTorquesSumAllFlies  = zeros(3, numTimePoints, 3);
meanForcesLHAllFlies    = zeros(3, numTimePoints, 3);
meanForcesRHAllFlies    = zeros(3, numTimePoints, 3);
meanTorquesLHAllFlies   = zeros(3, numTimePoints, 3);
meanTorquesRHAllFlies   = zeros(3, numTimePoints, 3);

%% Colors
% Custom distinct colors (manually chosen for high contrast)
colors = [...
    0.85, 0.33, 0.10; % Red-orange
    0.47, 0.67, 0.19; % Green
    0.30, 0.75, 0.93; % Light blue
    0.93, 0.69, 0.13; % Yellow-orange
    0.64, 0.08, 0.18; % Maroon
    0.49, 0.18, 0.56; % Purple
    0.00, 0.45, 0.74; % Blue
    0.25, 0.25, 0.25; % Gray
    0.94, 0.39, 0.39; % Salmon
    0.10, 0.60, 0.40]; % Teal

%% Step 3: Loop Over Pre-Cut, Post-Cut, and Steady-State Data
states = {'Pre Cut', 'Post Cut', 'Steady State'};
for stateIdx = 1:3
    % Get the fly data for the selected fly number and current state
    flyData = Fly_Master([Fly_Master.Fly_Num] == selectedFlyNum & ...
                         strcmp({Fly_Master.Attributes}, states{stateIdx}));
                     
    if isempty(flyData)
        continue;
    end
    fly = flyData(1).Fly;  % Access the selected fly data

    % Extract relevant data
    phi       = fly.Kinematics.LH.phi;
    force_lh  = fly.Dynamics.Frame_Body.LH.Force_Total * units_force;
    force_rh  = fly.Dynamics.Frame_Body.RH.Force_Total * units_force;
    torque_lh = fly.Dynamics.Frame_Body.LH.Torque_Total * units_torque;
    torque_rh = fly.Dynamics.Frame_Body.RH.Torque_Total * units_torque;

    % Find Peaks (Periods)
    [~, peakIndices] = findpeaks(phi);

    if length(peakIndices) < 2
        continue;  % Skip if not enough peaks for analysis
    end

    % Pre-allocate for storing forces and torques for each stroke cycle
    nf = @(x) zeros(3, numTimePoints, length(peakIndices) - 1);
    fSum = nf(0); fLH = nf(0); fRH = nf(0);
    tSum = nf(0); tLH = nf(0); tRH = nf(0);

    % Loop through each stroke cycle (between peaks)
    for cycleIdx = 1:length(peakIndices) - 1
        startIdx = peakIndices(cycleIdx);
        endIdx   = peakIndices(cycleIdx + 1);

        % Interpolate to a fixed time base (normalized)
        fSum(:,:,cycleIdx) = interp1(linspace(0, 1, endIdx - startIdx + 1), ...
            (force_lh(:, startIdx:endIdx) + force_rh(:, startIdx:endIdx))', ...
            normalizedTime, 'linear', 'extrap')';
        fLH(:,:,cycleIdx)  = interp1(linspace(0, 1, endIdx - startIdx + 1), force_lh(:, startIdx:endIdx)', ...
            normalizedTime, 'linear', 'extrap')';
        fRH(:,:,cycleIdx)  = interp1(linspace(0, 1, endIdx - startIdx + 1), force_rh(:, startIdx:endIdx)', ...
            normalizedTime, 'linear', 'extrap')';
        tSum(:,:,cycleIdx) = interp1(linspace(0, 1, endIdx - startIdx + 1), ...
            (torque_lh(:, startIdx:endIdx) + torque_rh(:, startIdx:endIdx))', ...
            normalizedTime, 'linear', 'extrap')';
        tLH(:,:,cycleIdx)  = interp1(linspace(0, 1, endIdx - startIdx + 1), torque_lh(:, startIdx:endIdx)', ...
            normalizedTime, 'linear', 'extrap')';
        tRH(:,:,cycleIdx)  = interp1(linspace(0, 1, endIdx - startIdx + 1), torque_rh(:, startIdx:endIdx)', ...
            normalizedTime, 'linear', 'extrap')';
    end

    % Store the calculated mean forces and torques for the current fly and state
    meanForcesSumAllFlies(:, :, stateIdx)   = mean(fSum, 3);
    meanForcesLHAllFlies(:, :, stateIdx)    = mean(fLH, 3);
    meanForcesRHAllFlies(:, :, stateIdx)    = mean(fRH, 3);
    meanTorquesSumAllFlies(:, :, stateIdx)  = mean(tSum, 3);
    meanTorquesLHAllFlies(:, :, stateIdx)   = mean(tLH, 3);
    meanTorquesRHAllFlies(:, :, stateIdx)   = mean(tRH, 3);
end

%% Step 4: Plot Forces and Torques
figure;
% Define components for the plot
components = {'Upward Force', 'Yaw Torque', 'Forward Force', 'Roll Torque', 'Sideward Force', 'Pitch Torque'};
ylabels = {'Force (N)', 'Torque (Nm)', 'Force (N)', 'Torque (Nm)', 'Force (N)', 'Torque (Nm)'};
ylims = [-5e-5, 5e-5; -5e-8, 5e-8; -5e-5, 5e-5; -5e-8, 5e-8; -5e-5, 5e-5; -5e-8, 5e-8];
indices_force = [3, 3, 2, 2, 1, 1]; % Indices for forces
indices_torque = [3, 3, 2, 2, 1, 1]; % Indices for torques

% Loop to generate subplots
for i = 1:6
    subplot(3, 2, i);
    hold on;
    
    % Shaded region for std deviation
    if mod(i, 2) == 1
        fill([normalizedTime, fliplr(normalizedTime)], ...
             [meanForcesSumAllFlies(indices_force(i), :, 1) + std(meanForcesSumAllFlies(indices_force(i), :, :), 0, 3), ...
              fliplr(meanForcesSumAllFlies(indices_force(i), :, 1) - std(meanForcesSumAllFlies(indices_force(i), :, :), 0, 3))], ...
             [0.8, 0.8, 0.8], 'EdgeColor', 'none');
        plot(normalizedTime, meanForcesSumAllFlies(indices_force(i), :, 1), 'k', 'LineWidth', 2);
    else
        fill([normalizedTime, fliplr(normalizedTime)], ...
             [meanTorquesSumAllFlies(indices_torque(i), :, 1) + std(meanTorquesSumAllFlies(indices_torque(i), :, :), 0, 3), ...
              fliplr(meanTorquesSumAllFlies(indices_torque(i), :, 1) - std(meanTorquesSumAllFlies(indices_torque(i), :, :), 0, 3))], ...
             [0.8, 0.8, 0.8], 'EdgeColor', 'none');
        plot(normalizedTime, meanTorquesSumAllFlies(indices_torque(i), :, 1), 'k', 'LineWidth', 2);
    end

    % Add individual fly lines
    for flyIdx = 1:1
        if mod(i, 2) == 1
            plot(normalizedTime, meanForcesSumAllFlies(indices_force(i), :, flyIdx), 'Color', colors(flyIdx, :), 'LineWidth', 1);
        else
            plot(normalizedTime, meanTorquesSumAllFlies(indices_torque(i), :, flyIdx), 'Color', colors(flyIdx, :), 'LineWidth', 1);
        end
    end

    % Labeling and formatting
    title(components{i});
    ylabel(ylabels{i});
    ylim(ylims(i, :));
    grid on;

    if i == 5 || i == 6
        xlabel('Normalized Stroke Cycle');
    else
        set(gca, 'XColor', 'none');
    end
end

% Add floating legend outside of the plots
lgd = legend('show', 'Location', 'northeastoutside');
sgtitle(['Forces and Torques for Fly #', num2str(selectedFlyNum)]);

%% Step 5: Clear Created Variables
vars_after = who;
vars_created = setdiff(vars_after, vars_before);
clear(vars_created{:});
clear vars_after vars_created vars_before;
