%% Preamble
% Jacob Taylor
% Fly-by-fly selection of torques and forces with kinematics

%% Step 1: Clear and Setup
clc            % Clear command window
warning off    % Suppress all warnings
% close all    % Uncomment to close any open figures

% Snapshot of variables that existed before the script
vars_before = who;

%% Step 2: Select Flies for Analysis via Popup
% Generate display names for all flies
flyOptions = arrayfun(@(f) sprintf('Fly #%d - %s', f.Fly_Num, f.Attributes), ...
                      Fly_Master, 'UniformOutput', false);

% Prompt user to select multiple flies
[selectedIdx, ok] = listdlg( ...
    'PromptString', 'Select flies to include in the analysis:', ...
    'ListString', flyOptions, ...
    'SelectionMode', 'multiple', ...
    'ListSize', [300, 300], ...
    'Name', 'Select Flies');

% Exit if cancelled
if ~ok || isempty(selectedIdx)
    disp('No flies selected. Script aborted.');
    return;
end

%% Step 3: Setup
units_force = 10^-6;
units_torque = 10^-10;

numTimePoints = 100; % For normalized stroke cycle
numFlies = length(selectedIdx);
normalizedTime = linspace(0, 1, numTimePoints);

% Initialize storage
meanForcesSumAllFlies   = zeros(3, numTimePoints, numFlies);
meanForcesLHAllFlies    = zeros(3, numTimePoints, numFlies);
meanForcesRHAllFlies    = zeros(3, numTimePoints, numFlies);
meanTorquesSumAllFlies  = zeros(3, numTimePoints, numFlies);
meanTorquesLHAllFlies   = zeros(3, numTimePoints, numFlies);
meanTorquesRHAllFlies   = zeros(3, numTimePoints, numFlies);

%% Step 4: Analyze Each Selected Fly
for flyIdx = 1:numFlies
    fly = Fly_Master(selectedIdx(flyIdx)).Fly;
    phi        = fly.Kinematics.LH.phi;
    force_lh   = fly.Dynamics.Frame_Body.LH.Force_Total * units_force .* [1; -1; 1];
    force_rh   = fly.Dynamics.Frame_Body.RH.Force_Total * units_force .* [1; -1; 1];
    torque_lh  = fly.Dynamics.Frame_Body.LH.Torque_Total * units_torque .* [-1; 1; 1];
    torque_rh  = fly.Dynamics.Frame_Body.RH.Torque_Total * units_torque .* [-1; 1; 1];

    [~, peakIndices] = findpeaks(phi);

    % Pre-allocate for cycles
    nCycles = length(peakIndices) - 1;
    nf = @(x) zeros(3, numTimePoints, nCycles);
    normalizedForcesSum  = nf(0); normalizedForcesLH  = nf(0); normalizedForcesRH  = nf(0);
    normalizedTorquesSum = nf(0); normalizedTorquesLH = nf(0); normalizedTorquesRH = nf(0);

    for cycleIdx = 1:nCycles
        sIdx = peakIndices(cycleIdx);
        eIdx = peakIndices(cycleIdx + 1);

        % Extract and normalize
        normT = linspace(0, 1, eIdx - sIdx + 1);
        forceSum  = force_lh(:, sIdx:eIdx) + force_rh(:, sIdx:eIdx);
        torqueSum = torque_lh(:, sIdx:eIdx) + torque_rh(:, sIdx:eIdx);

        normalizedForcesSum(:, :, cycleIdx)  = interp1(normT, forceSum' , normalizedTime, 'linear', 'extrap')';
        normalizedForcesLH(:, :, cycleIdx)   = interp1(normT, force_lh(:, sIdx:eIdx)' , normalizedTime, 'linear', 'extrap')';
        normalizedForcesRH(:, :, cycleIdx)   = interp1(normT, force_rh(:, sIdx:eIdx)' , normalizedTime, 'linear', 'extrap')';
        normalizedTorquesSum(:, :, cycleIdx) = interp1(normT, torqueSum' , normalizedTime, 'linear', 'extrap')';
        normalizedTorquesLH(:, :, cycleIdx)  = interp1(normT, torque_lh(:, sIdx:eIdx)' , normalizedTime, 'linear', 'extrap')';
        normalizedTorquesRH(:, :, cycleIdx)  = interp1(normT, torque_rh(:, sIdx:eIdx)' , normalizedTime, 'linear', 'extrap')';
    end

    % Mean across all strokes for each fly
    meanForcesSumAllFlies(:, :, flyIdx)   = mean(normalizedForcesSum, 3);
    meanForcesLHAllFlies(:, :, flyIdx)    = mean(normalizedForcesLH, 3);
    meanForcesRHAllFlies(:, :, flyIdx)    = mean(normalizedForcesRH, 3);
    meanTorquesSumAllFlies(:, :, flyIdx)  = mean(normalizedTorquesSum, 3);
    meanTorquesLHAllFlies(:, :, flyIdx)   = mean(normalizedTorquesLH, 3);
    meanTorquesRHAllFlies(:, :, flyIdx)   = mean(normalizedTorquesRH, 3);
end

%% Step 5: Compute Statistics Across Flies
meanForcesSumAcrossFlies   = mean(meanForcesSumAllFlies, 3);
stdForcesSumAcrossFlies    = std(meanForcesSumAllFlies, 0, 3);
meanTorquesSumAcrossFlies  = mean(meanTorquesSumAllFlies, 3);
stdTorquesSumAcrossFlies   = std(meanTorquesSumAllFlies, 0, 3);

%% Step 6: Plot Forces and Torques
figure('Name', 'Forces and Torques Summary');

componentLabels = {'Sideward', 'Forward', 'Upward'};
torqueLabels    = {'Pitch', 'Roll', 'Yaw'};

for i = 1:3
    % Force subplot
    subplot(3, 2, 2*i - 1)
    hold on
    fill([normalizedTime, fliplr(normalizedTime)], ...
         [meanForcesSumAcrossFlies(i, :) + stdForcesSumAcrossFlies(i, :), ...
          fliplr(meanForcesSumAcrossFlies(i, :) - stdForcesSumAcrossFlies(i, :))], ...
         [0.85 0.85 0.85], 'EdgeColor', 'none');
    plot(normalizedTime, meanForcesSumAcrossFlies(i, :), 'k', 'LineWidth', 2);
    plot(normalizedTime, mean(meanForcesLHAllFlies(i, :, :), 3), 'r', 'LineWidth', 1.5);
    plot(normalizedTime, mean(meanForcesRHAllFlies(i, :, :), 3), 'b', 'LineWidth', 1.5);
    title([componentLabels{i} ' Force'])
    ylabel('Force (N)')
    if i == 3, xlabel('Stroke Cycle (%)'); end
    ylim([-5e-5, 5e-5])

    % Torque subplot
    subplot(3, 2, 2*i)
    hold on
    fill([normalizedTime, fliplr(normalizedTime)], ...
         [meanTorquesSumAcrossFlies(i, :) + stdTorquesSumAcrossFlies(i, :), ...
          fliplr(meanTorquesSumAcrossFlies(i, :) - stdTorquesSumAcrossFlies(i, :))], ...
         [0.85 0.85 0.85], 'EdgeColor', 'none');
    plot(normalizedTime, meanTorquesSumAcrossFlies(i, :), 'k', 'LineWidth', 2);
    plot(normalizedTime, mean(meanTorquesLHAllFlies(i, :, :), 3), 'r', 'LineWidth', 1.5);
    plot(normalizedTime, mean(meanTorquesRHAllFlies(i, :, :), 3), 'b', 'LineWidth', 1.5);
    title([torqueLabels{i} ' Torque'])
    ylabel('Torque (Nm)')
    if i == 3, xlabel('Stroke Cycle (%)'); end
    ylim([-5e-8, 5e-8])
end

sgtitle('Forces and Torques with Variability Across Selected Flies')

%% Step 7: Clear Created Variables
% Get all current variables
vars_after = who;

% Determine which variables were added by the script
vars_created = setdiff(vars_after, vars_before);

% Clear only the variables created during script execution
clear(vars_created{:});

% Clear the temporary tracking variables too
clear vars_after vars_created vars_before;
