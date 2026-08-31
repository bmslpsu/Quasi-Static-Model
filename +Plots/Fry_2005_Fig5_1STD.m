%% Preamble
% Jacob Taylor
% Filtered Forces and Torques - Mean ± STD Across Valid Flies

%% Step 1: Clear and Setup
clc            % Clear command window
warning off    % Suppress all warnings
% close all    % Uncomment to close any open figures

% Snapshot of variables that existed before the script
vars_before = who;

%% Step 2: Fly Selection
flyOptions = arrayfun(@(f) sprintf('Fly #%d - %s', f.Fly_Num, f.Attributes), ...
                      Fly_Master, 'UniformOutput', false);

[selectedIdx, ok] = listdlg( ...
    'PromptString', 'Select flies for force/torque filtering:', ...
    'ListString', flyOptions, ...
    'SelectionMode', 'multiple', ...
    'ListSize', [300, 300], ...
    'Name', 'Fly Selection');

if ~ok || isempty(selectedIdx)
    disp('No flies selected. Aborting.');
    return;
end

%% Step 3: Setup
units_force  = 1e-6;
units_torque = 1e-10;
numTimePoints = 100;
normalizedTime = linspace(0, 1, numTimePoints);
numFlies = length(selectedIdx);

% Allocate arrays
meanForcesSumAllFlies   = zeros(3, numTimePoints, numFlies);
meanTorquesSumAllFlies  = zeros(3, numTimePoints, numFlies);
meanForcesLHAllFlies    = zeros(3, numTimePoints, numFlies);
meanForcesRHAllFlies    = zeros(3, numTimePoints, numFlies);
meanTorquesLHAllFlies   = zeros(3, numTimePoints, numFlies);
meanTorquesRHAllFlies   = zeros(3, numTimePoints, numFlies);

%% Step 4: Loop Over Flies
for flyIdx = 1:numFlies
    fly = Fly_Master(selectedIdx(flyIdx)).Fly;

    phi = fly.Kinematics.LH.phi;
    force_lh  = fly.Dynamics.Frame_Body.LH.Force_Total * units_force;
    force_rh  = fly.Dynamics.Frame_Body.RH.Force_Total * units_force;
    torque_lh = fly.Dynamics.Frame_Body.LH.Torque_Total * units_torque;
    torque_rh = fly.Dynamics.Frame_Body.RH.Torque_Total * units_torque;

    [~, peakIndices] = findpeaks(phi);
    nCycles = length(peakIndices) - 1;

    if nCycles < 1
        continue;
    end

    % Pre-allocate
    nf = @(x) zeros(3, numTimePoints, nCycles);
    fSum = nf(0); fLH = nf(0); fRH = nf(0);
    tSum = nf(0); tLH = nf(0); tRH = nf(0);

    for j = 1:nCycles
        sIdx = peakIndices(j);
        eIdx = peakIndices(j+1);

        % Interpolate to fixed time base
        fSum(:,:,j) = interp1(linspace(0,1,eIdx-sIdx+1), (force_lh(:,sIdx:eIdx)+force_rh(:,sIdx:eIdx))', normalizedTime, 'linear', 'extrap')';
        fLH(:,:,j)  = interp1(linspace(0,1,eIdx-sIdx+1), force_lh(:,sIdx:eIdx)', normalizedTime, 'linear', 'extrap')';
        fRH(:,:,j)  = interp1(linspace(0,1,eIdx-sIdx+1), force_rh(:,sIdx:eIdx)', normalizedTime, 'linear', 'extrap')';
        tSum(:,:,j) = interp1(linspace(0,1,eIdx-sIdx+1), (torque_lh(:,sIdx:eIdx)+torque_rh(:,sIdx:eIdx))', normalizedTime, 'linear', 'extrap')';
        tLH(:,:,j)  = interp1(linspace(0,1,eIdx-sIdx+1), torque_lh(:,sIdx:eIdx)', normalizedTime, 'linear', 'extrap')';
        tRH(:,:,j)  = interp1(linspace(0,1,eIdx-sIdx+1), torque_rh(:,sIdx:eIdx)', normalizedTime, 'linear', 'extrap')';
    end

    % Store means
    meanForcesSumAllFlies(:,:,flyIdx)   = mean(fSum, 3);
    meanForcesLHAllFlies(:,:,flyIdx)    = mean(fLH, 3);
    meanForcesRHAllFlies(:,:,flyIdx)    = mean(fRH, 3);
    meanTorquesSumAllFlies(:,:,flyIdx)  = mean(tSum, 3);
    meanTorquesLHAllFlies(:,:,flyIdx)   = mean(tLH, 3);
    meanTorquesRHAllFlies(:,:,flyIdx)   = mean(tRH, 3);
end

%% Step 5: Filter Flies Based on Std Dev
meanF = mean(meanForcesSumAllFlies, 3);
stdF  = std(meanForcesSumAllFlies, 0, 3);
validFlies = [];

for flyIdx = 1:numFlies
    flyMean = mean(meanForcesSumAllFlies(:,:,flyIdx), 2);
    if all(abs(flyMean - mean(flyMean, 2)) <= stdF(:,1))
        validFlies = [validFlies, flyIdx];
    end
end

% Recalculate based on validFlies
filteredMeanForcesSum   = mean(meanForcesSumAllFlies(:,:,validFlies), 3);
filteredStdForcesSum    = std(meanForcesSumAllFlies(:,:,validFlies), 0, 3);
filteredMeanTorquesSum  = mean(meanTorquesSumAllFlies(:,:,validFlies), 3);
filteredStdTorquesSum   = std(meanTorquesSumAllFlies(:,:,validFlies), 0, 3);

%% Step 6: Plot with Floating Legend
figure('Name', 'Filtered Forces and Torques');
layout = tiledlayout(3,2, 'TileSpacing', 'compact', 'Padding', 'compact');

labels = {'Upward Force', 'Yaw Torque', 'Forward Force', ...
          'Roll Torque', 'Sideward Force', 'Pitch Torque'};
ylabels = {'Force (N)', 'Torque (Nm)', 'Force (N)', ...
           'Torque (Nm)', 'Force (N)', 'Torque (Nm)'};
ylims = [-5e-5, 5e-5; -5e-8, 5e-8; -5e-5, 5e-5; -5e-8, 5e-8; -5e-5, 5e-5; -5e-8, 5e-8];
forceIdx = [2,2,1,1,3,3];
torqueIdx = [2,2,1,1,3,3];

for i = 1:6
    nexttile;
    hold on;

    % Fill STD region
    if mod(i,2)==1
        idx = forceIdx(i);
        fill([normalizedTime, fliplr(normalizedTime)], ...
             [filteredMeanForcesSum(idx,:) + filteredStdForcesSum(idx,:), ...
              fliplr(filteredMeanForcesSum(idx,:) - filteredStdForcesSum(idx,:))], ...
              [0.85, 0.85, 0.85], 'EdgeColor', 'none');
        plot(normalizedTime, filteredMeanForcesSum(idx,:), 'k', 'LineWidth', 2);
        plot(normalizedTime, mean(meanForcesLHAllFlies(idx,:,validFlies),3), 'r', 'LineWidth', 1.5);
        plot(normalizedTime, mean(meanForcesRHAllFlies(idx,:,validFlies),3), 'b', 'LineWidth', 1.5);
    else
        idx = torqueIdx(i);
        fill([normalizedTime, fliplr(normalizedTime)], ...
             [filteredMeanTorquesSum(idx,:) + filteredStdTorquesSum(idx,:), ...
              fliplr(filteredMeanTorquesSum(idx,:) - filteredStdTorquesSum(idx,:))], ...
              [0.85, 0.85, 0.85], 'EdgeColor', 'none');
        plot(normalizedTime, filteredMeanTorquesSum(idx,:), 'k', 'LineWidth', 2);
        plot(normalizedTime, mean(meanTorquesLHAllFlies(idx,:,validFlies),3), 'r', 'LineWidth', 1.5);
        plot(normalizedTime, mean(meanTorquesRHAllFlies(idx,:,validFlies),3), 'b', 'LineWidth', 1.5);
    end

    title(labels{i});
    ylabel(ylabels{i});
    ylim(ylims(i,:));
    grid on;

    if i >= 5
        xlabel('Normalized Stroke Cycle');
    else
        set(gca, 'XColor', 'none');
    end
end

% Add floating legend
lgd = legend(layout.Children(1), {'STD', 'Total', 'LH', 'RH'}, ...
             'Location', 'northeastoutside', 'Box', 'off');
lgd.Title.String = 'Legend';

sgtitle('Filtered Forces and Torques with Standard Deviation');

%% Step 7: Clear Created Variables
% Get all current variables
vars_after = who;

% Determine which variables were added by the script
vars_created = setdiff(vars_after, vars_before);

% Clear only the variables created during script execution
clear(vars_created{:});

% Clear the temporary tracking variables too
clear vars_after vars_created vars_before;
