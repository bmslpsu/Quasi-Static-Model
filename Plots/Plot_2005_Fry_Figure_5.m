fly_num = [1, 3, 5, 8, 10, 13, 16, 18, 21, 24];

units_force = 10^-6;
units_torque = 10^-10;

% Define storage for all flies' mean forces and torques
numTimePoints = 100; % Fixed number of normalized time points
meanForcesSumAllFlies = zeros(3, numTimePoints, length(fly_num)); % [x, y, z] x time x flies
meanForcesLHAllFlies = zeros(3, numTimePoints, length(fly_num)); % For LH forces
meanForcesRHAllFlies = zeros(3, numTimePoints, length(fly_num)); % For RH forces
meanTorquesSumAllFlies = zeros(3, numTimePoints, length(fly_num)); % For summed torques
meanTorquesLHAllFlies = zeros(3, numTimePoints, length(fly_num)); % For LH torques
meanTorquesRHAllFlies = zeros(3, numTimePoints, length(fly_num)); % For RH torques

for flyIdx = 1:length(fly_num)
    % Extract the phi, force, and torque data for the current fly
    phi = Fly_Master(fly_num(flyIdx)).Fly.Kinematics.LH.phi;
    force_lh = Fly_Master(fly_num(flyIdx)).Fly.Dynamics.Frame_Body.LH.Force_Total * units_force;
    force_rh = Fly_Master(fly_num(flyIdx)).Fly.Dynamics.Frame_Body.RH.Force_Total * units_force;
    torque_lh = Fly_Master(fly_num(flyIdx)).Fly.Dynamics.Frame_Body.LH.Torque_Total * units_torque;
    torque_rh = Fly_Master(fly_num(flyIdx)).Fly.Dynamics.Frame_Body.RH.Torque_Total * units_torque;

    % Find Peaks and Validate
    [peaks, peakIndices] = findpeaks(phi);

    % Define storage for normalized forces and torques for this fly
    normalizedForcesSum = zeros(3, numTimePoints, length(peakIndices) - 1);
    normalizedForcesLH = zeros(3, numTimePoints, length(peakIndices) - 1);
    normalizedForcesRH = zeros(3, numTimePoints, length(peakIndices) - 1);
    normalizedTorquesSum = zeros(3, numTimePoints, length(peakIndices) - 1);
    normalizedTorquesLH = zeros(3, numTimePoints, length(peakIndices) - 1);
    normalizedTorquesRH = zeros(3, numTimePoints, length(peakIndices) - 1);

    % Loop through each stroke cycle (between peaks)
    for cycleIdx = 1:length(peakIndices) - 1
        % Get the indices for the current stroke cycle
        startIdx = peakIndices(cycleIdx);
        endIdx = peakIndices(cycleIdx + 1);

        % Extract force and torque data for this stroke cycle
        forceCycle_lh = force_lh(:, startIdx:endIdx);
        forceCycle_rh = force_rh(:, startIdx:endIdx);
        torqueCycle_lh = torque_lh(:, startIdx:endIdx);
        torqueCycle_rh = torque_rh(:, startIdx:endIdx);

        % Normalize the time to a fixed number of points (0 to 1)
        normalizedTime = linspace(0, 1, numTimePoints);
        forceCycleSum_norm = interp1(linspace(0, 1, size(forceCycle_lh, 2)), ...
                                     (forceCycle_lh + forceCycle_rh)', normalizedTime, 'linear', 'extrap')';
        forceCycleLH_norm = interp1(linspace(0, 1, size(forceCycle_lh, 2)), forceCycle_lh', normalizedTime, 'linear', 'extrap')';
        forceCycleRH_norm = interp1(linspace(0, 1, size(forceCycle_rh, 2)), forceCycle_rh', normalizedTime, 'linear', 'extrap')';
        torqueCycleSum_norm = interp1(linspace(0, 1, size(torqueCycle_lh, 2)), ...
                                      (torqueCycle_lh + torqueCycle_rh)', normalizedTime, 'linear', 'extrap')';
        torqueCycleLH_norm = interp1(linspace(0, 1, size(torqueCycle_lh, 2)), torqueCycle_lh', normalizedTime, 'linear', 'extrap')';
        torqueCycleRH_norm = interp1(linspace(0, 1, size(torqueCycle_rh, 2)), torqueCycle_rh', normalizedTime, 'linear', 'extrap')';

        % Store normalized forces and torques for this cycle
        normalizedForcesSum(:, :, cycleIdx) = forceCycleSum_norm;
        normalizedForcesLH(:, :, cycleIdx) = forceCycleLH_norm;
        normalizedForcesRH(:, :, cycleIdx) = forceCycleRH_norm;
        normalizedTorquesSum(:, :, cycleIdx) = torqueCycleSum_norm;
        normalizedTorquesLH(:, :, cycleIdx) = torqueCycleLH_norm;
        normalizedTorquesRH(:, :, cycleIdx) = torqueCycleRH_norm;
    end

    % Calculate the mean forces and torques for this fly across all cycles
    meanForcesSumAllFlies(:, :, flyIdx) = mean(normalizedForcesSum, 3);
    meanForcesLHAllFlies(:, :, flyIdx) = mean(normalizedForcesLH, 3);
    meanForcesRHAllFlies(:, :, flyIdx) = mean(normalizedForcesRH, 3);
    meanTorquesSumAllFlies(:, :, flyIdx) = mean(normalizedTorquesSum, 3);
    meanTorquesLHAllFlies(:, :, flyIdx) = mean(normalizedTorquesLH, 3);
    meanTorquesRHAllFlies(:, :, flyIdx) = mean(normalizedTorquesRH, 3);
end

% Calculate the mean and standard deviation across all flies
meanForcesSumAcrossFlies = mean(meanForcesSumAllFlies, 3);
stdForcesSumAcrossFlies = std(meanForcesSumAllFlies, 0, 3);
meanTorquesSumAcrossFlies = mean(meanTorquesSumAllFlies, 3);
stdTorquesSumAcrossFlies = std(meanTorquesSumAllFlies, 0, 3);

% Plot Forces and Torques with Shaded Standard Deviation
figure;

% Forces (Z-Component)
subplot(3, 2, 1);
hold on;
fill([normalizedTime, fliplr(normalizedTime)], ...
     [meanForcesSumAcrossFlies(2, :) + stdForcesSumAcrossFlies(2, :), ...
      fliplr(meanForcesSumAcrossFlies(2, :) - stdForcesSumAcrossFlies(2, :))], ...
     [0.8, 0.8, 0.8], 'EdgeColor', 'none'); % Shaded region
plot(normalizedTime, meanForcesSumAcrossFlies(2, :), 'k', 'LineWidth', 2); % Mean line
plot(normalizedTime, mean(meanForcesLHAllFlies(2, :, :), 3), 'r', 'LineWidth', 1.5); % LH line
plot(normalizedTime, mean(meanForcesRHAllFlies(2, :, :), 3), 'b', 'LineWidth', 1.5); % RH line
title('Upward Force');
ylabel('Force (N)');


% Torques (Z-Component)
subplot(3, 2, 2);
hold on;
fill([normalizedTime, fliplr(normalizedTime)], ...
     [meanTorquesSumAcrossFlies(2, :) + stdTorquesSumAcrossFlies(2, :), ...
      fliplr(meanTorquesSumAcrossFlies(2, :) - stdTorquesSumAcrossFlies(2, :))], ...
     [0.8, 0.8, 0.8], 'EdgeColor', 'none'); % Shaded region
plot(normalizedTime, meanTorquesSumAcrossFlies(2, :), 'k', 'LineWidth', 2); % Mean line
plot(normalizedTime, mean(meanTorquesLHAllFlies(2, :, :), 3), 'r', 'LineWidth', 1.5); % LH line
plot(normalizedTime, mean(meanTorquesRHAllFlies(2, :, :), 3), 'b', 'LineWidth', 1.5); % RH line
title('Yaw Torque');
ylabel('Torque (Nm)');

% Forces (Y-Component)
subplot(3, 2, 3);
hold on;
fill([normalizedTime, fliplr(normalizedTime)], ...
     [meanForcesSumAcrossFlies(1, :) + stdForcesSumAcrossFlies(1, :), ...
      fliplr(meanForcesSumAcrossFlies(1, :) - stdForcesSumAcrossFlies(1, :))], ...
     [0.8, 0.8, 0.8], 'EdgeColor', 'none'); % Shaded region
plot(normalizedTime, meanForcesSumAcrossFlies(1, :), 'k', 'LineWidth', 2); % Mean line
plot(normalizedTime, mean(meanForcesLHAllFlies(1, :, :), 3), 'r', 'LineWidth', 1.5); % LH line
plot(normalizedTime, mean(meanForcesRHAllFlies(1, :, :), 3), 'b', 'LineWidth', 1.5); % RH line
title('Forward Force');
ylabel('Force (N)');

% Torques (Y-Component)
subplot(3, 2, 4);
hold on;
fill([normalizedTime, fliplr(normalizedTime)], ...
     [meanTorquesSumAcrossFlies(1, :) + stdTorquesSumAcrossFlies(1, :), ...
      fliplr(meanTorquesSumAcrossFlies(1, :) - stdTorquesSumAcrossFlies(1, :))], ...
     [0.8, 0.8, 0.8], 'EdgeColor', 'none'); % Shaded region
plot(normalizedTime, meanTorquesSumAcrossFlies(1, :), 'k', 'LineWidth', 2); % Mean line
plot(normalizedTime, mean(meanTorquesLHAllFlies(1, :, :), 3), 'r', 'LineWidth', 1.5); % LH line
plot(normalizedTime, mean(meanTorquesRHAllFlies(1, :, :), 3), 'b', 'LineWidth', 1.5); % RH line
title('Roll Torque');
ylabel('Torque (Nm)');

% Forces (X-Component)
subplot(3, 2, 5);
hold on;
fill([normalizedTime, fliplr(normalizedTime)], ...
     [meanForcesSumAcrossFlies(3, :) + stdForcesSumAcrossFlies(3, :), ...
      fliplr(meanForcesSumAcrossFlies(3, :) - stdForcesSumAcrossFlies(3, :))], ...
     [0.8, 0.8, 0.8], 'EdgeColor', 'none'); % Shaded region
plot(normalizedTime, meanForcesSumAcrossFlies(3, :), 'k', 'LineWidth', 2); % Mean line
plot(normalizedTime, mean(meanForcesLHAllFlies(3, :, :), 3), 'r', 'LineWidth', 1.5); % LH line
plot(normalizedTime, mean(meanForcesRHAllFlies(3, :, :), 3), 'b', 'LineWidth', 1.5); % RH line
title('Sideward Force');
ylabel('Force (N)');
xlabel('Stroke Cycle');


% Torques (X-Component)
subplot(3, 2, 6);
hold on;
fill([normalizedTime, fliplr(normalizedTime)], ...
     [meanTorquesSumAcrossFlies(3, :) + stdTorquesSumAcrossFlies(3, :), ...
      fliplr(meanTorquesSumAcrossFlies(3, :) - stdTorquesSumAcrossFlies(3, :))], ...
     [0.8, 0.8, 0.8], 'EdgeColor', 'none'); % Shaded region
plot(normalizedTime, meanTorquesSumAcrossFlies(3, :), 'k', 'LineWidth', 2); % Mean line
plot(normalizedTime, mean(meanTorquesLHAllFlies(3, :, :), 3), 'r', 'LineWidth', 1.5); % LH line
plot(normalizedTime, mean(meanTorquesRHAllFlies(3, :, :), 3), 'b', 'LineWidth', 1.5); % RH line
title('Pitch Torque');
ylabel('Torque (Nm)');
xlabel('Stroke Cycle');

sgtitle('Forces and Torques with Variability Across Multiple Flies');
