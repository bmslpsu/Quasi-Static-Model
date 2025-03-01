fly_num = [1, 3, 5, 8, 10, 13, 16, 18, 21, 24];
%fly_num=fly_num+1;

units_force = 10^-6;
units_torque = 10^-10;

% Define storage for all flies' mean forces and torques
numTimePoints = 100; % Fixed number of normalized time points
meanForcesSumAllFlies = zeros(3, numTimePoints, length(fly_num)); % [x, y, z] x time x flies
meanForcesLHAllFlies = zeros(3, numTimePoints, length(fly_num)); % LH forces
meanForcesRHAllFlies = zeros(3, numTimePoints, length(fly_num)); % RH forces
meanTorquesSumAllFlies = zeros(3, numTimePoints, length(fly_num)); % [x, y, z] x time x flies
meanTorquesLHAllFlies = zeros(3, numTimePoints, length(fly_num)); % LH torques
meanTorquesRHAllFlies = zeros(3, numTimePoints, length(fly_num)); % RH torques

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
        forceCycleLH = force_lh(:, startIdx:endIdx);
        forceCycleRH = force_rh(:, startIdx:endIdx);
        forceCycleSum = forceCycleLH + forceCycleRH;

        torqueCycleLH = torque_lh(:, startIdx:endIdx);
        torqueCycleRH = torque_rh(:, startIdx:endIdx);
        torqueCycleSum = torqueCycleLH + torqueCycleRH;

        % Normalize the time to a fixed number of points (0 to 1)
        normalizedTime = linspace(0, 1, numTimePoints);
        forceCycleSum_norm = interp1(linspace(0, 1, size(forceCycleSum, 2)), forceCycleSum', normalizedTime, 'linear', 'extrap')';
        forceCycleLH_norm = interp1(linspace(0, 1, size(forceCycleLH, 2)), forceCycleLH', normalizedTime, 'linear', 'extrap')';
        forceCycleRH_norm = interp1(linspace(0, 1, size(forceCycleRH, 2)), forceCycleRH', normalizedTime, 'linear', 'extrap')';
        torqueCycleSum_norm = interp1(linspace(0, 1, size(torqueCycleSum, 2)), torqueCycleSum', normalizedTime, 'linear', 'extrap')';
        torqueCycleLH_norm = interp1(linspace(0, 1, size(torqueCycleLH, 2)), torqueCycleLH', normalizedTime, 'linear', 'extrap')';
        torqueCycleRH_norm = interp1(linspace(0, 1, size(torqueCycleRH, 2)), torqueCycleRH', normalizedTime, 'linear', 'extrap')';

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

% Calculate the overall mean and standard deviation across all flies
meanForcesSumAcrossFlies = mean(meanForcesSumAllFlies, 3);
stdForcesSumAcrossFlies = std(meanForcesSumAllFlies, 0, 3);
meanTorquesSumAcrossFlies = mean(meanTorquesSumAllFlies, 3);
stdTorquesSumAcrossFlies = std(meanTorquesSumAllFlies, 0, 3);

% Apply the filter based on standard deviation
validFlies = [];
for flyIdx = 1:length(fly_num)
    flyMeanForce = mean(meanForcesSumAllFlies(:, :, flyIdx), 2); % Mean force for this fly across all components
    overallMeanForce = mean(meanForcesSumAcrossFlies, 2); % Overall mean across all flies
    overallStdForce = std(meanForcesSumAcrossFlies, 0, 2); % Standard deviation across flies

    % Check if this fly's mean force is within one standard deviation
    if all(abs(flyMeanForce - overallMeanForce) <= overallStdForce)
        validFlies = [validFlies, flyIdx];
    end
end

% Recalculate the mean and standard deviation for valid flies
filteredMeanForcesSum = mean(meanForcesSumAllFlies(:, :, validFlies), 3);
filteredStdForcesSum = std(meanForcesSumAllFlies(:, :, validFlies), 0, 3);
filteredMeanTorquesSum = mean(meanTorquesSumAllFlies(:, :, validFlies), 3);
filteredStdTorquesSum = std(meanTorquesSumAllFlies(:, :, validFlies), 0, 3);


% Forces and torques with shaded standard deviation
figure;

% Forces (Z-Component)
subplot(3, 2, 1);
hold on;
fill([normalizedTime, fliplr(normalizedTime)], ...
     [filteredMeanForcesSum(2, :) + filteredStdForcesSum(2, :), ...
      fliplr(filteredMeanForcesSum(2, :) - filteredStdForcesSum(2, :))], ...
     [0.8, 0.8, 0.8], 'EdgeColor', 'none'); % Shaded area
plot(normalizedTime, filteredMeanForcesSum(2, :), 'k', 'LineWidth', 2); % Mean line
plot(normalizedTime, mean(meanForcesLHAllFlies(2, :, validFlies), 3), 'r', 'LineWidth', 1.5); % LH line
plot(normalizedTime, mean(meanForcesRHAllFlies(2, :, validFlies), 3), 'b', 'LineWidth', 1.5); % RH line
title('Upward Force');
ylabel('Force (N)');
ylim([-5*10^-5 5*10^-5])

% Torques (Z-Component)
subplot(3, 2, 2);
hold on;
fill([normalizedTime, fliplr(normalizedTime)], ...
     [filteredMeanTorquesSum(2, :) + filteredStdTorquesSum(2, :), ...
      fliplr(filteredMeanTorquesSum(2, :) - filteredStdTorquesSum(2, :))], ...
     [0.8, 0.8, 0.8], 'EdgeColor', 'none'); % Shaded area
plot(normalizedTime, filteredMeanTorquesSum(2, :), 'k', 'LineWidth', 2); % Mean line
plot(normalizedTime, mean(meanTorquesLHAllFlies(2, :, validFlies), 3), 'r', 'LineWidth', 1.5); % LH line
plot(normalizedTime, mean(meanTorquesRHAllFlies(2, :, validFlies), 3), 'b', 'LineWidth', 1.5); % RH line
title('Yaw Torque');
ylabel('Torque (Nm)');
ylim([-5*10^-8 5*10^-8])

% Forces (Y-Component)
subplot(3, 2, 3);
hold on;
fill([normalizedTime, fliplr(normalizedTime)], ...
     [filteredMeanForcesSum(1, :) + filteredStdForcesSum(1, :), ...
      fliplr(filteredMeanForcesSum(1, :) - filteredStdForcesSum(1, :))], ...
     [0.8, 0.8, 0.8], 'EdgeColor', 'none'); % Shaded area
plot(normalizedTime, filteredMeanForcesSum(1, :), 'k', 'LineWidth', 2); % Mean line
plot(normalizedTime, mean(meanForcesLHAllFlies(1, :, validFlies), 3), 'r', 'LineWidth', 1.5); % LH line
plot(normalizedTime, mean(meanForcesRHAllFlies(1, :, validFlies), 3), 'b', 'LineWidth', 1.5); % RH line
title('Forward Force');
ylabel('Force (N)');
ylim([-5*10^-5 5*10^-5])

% Torques (Y-Component)
subplot(3, 2, 4);
hold on;
fill([normalizedTime, fliplr(normalizedTime)], ...
     [filteredMeanTorquesSum(1, :) + filteredStdTorquesSum(1, :), ...
      fliplr(filteredMeanTorquesSum(1, :) - filteredStdTorquesSum(1, :))], ...
     [0.8, 0.8, 0.8], 'EdgeColor', 'none'); % Shaded area
plot(normalizedTime, filteredMeanTorquesSum(1, :), 'k', 'LineWidth', 2); % Mean line
plot(normalizedTime, mean(meanTorquesLHAllFlies(1, :, validFlies), 3), 'r', 'LineWidth', 1.5); % LH line
plot(normalizedTime, mean(meanTorquesRHAllFlies(1, :, validFlies), 3), 'b', 'LineWidth', 1.5); % RH line
title('Roll Torque');
ylabel('Torque (Nm)');
ylim([-5*10^-8 5*10^-8])

% Forces (X-Component)
subplot(3, 2, 5);
hold on;
fill([normalizedTime, fliplr(normalizedTime)], ...
     [filteredMeanForcesSum(3, :) + filteredStdForcesSum(3, :), ...
      fliplr(filteredMeanForcesSum(3, :) - filteredStdForcesSum(3, :))], ...
     [0.8, 0.8, 0.8], 'EdgeColor', 'none'); % Shaded area
plot(normalizedTime, filteredMeanForcesSum(3, :), 'k', 'LineWidth', 2); % Mean line
plot(normalizedTime, mean(meanForcesLHAllFlies(3, :, validFlies), 3), 'r', 'LineWidth', 1.5); % LH line
plot(normalizedTime, mean(meanForcesRHAllFlies(3, :, validFlies), 3), 'b', 'LineWidth', 1.5); % RH line
title('Sideward Force');
ylabel('Force (N)');
xlabel('Stroke Cycle');
ylim([-5*10^-5 5*10^-5])

% Torques (X-Component)
subplot(3, 2, 6);
hold on;
fill([normalizedTime, fliplr(normalizedTime)], ...
     [filteredMeanTorquesSum(3, :) + filteredStdTorquesSum(3, :), ...
      fliplr(filteredMeanTorquesSum(3, :) - filteredStdTorquesSum(3, :))], ...
     [0.8, 0.8, 0.8], 'EdgeColor', 'none'); % Shaded area
plot(normalizedTime, filteredMeanTorquesSum(3, :), 'k', 'LineWidth', 2); % Mean line
plot(normalizedTime, mean(meanTorquesLHAllFlies(3, :, validFlies), 3), 'r', 'LineWidth', 1.5); % LH line
plot(normalizedTime, mean(meanTorquesRHAllFlies(3, :, validFlies), 3), 'b', 'LineWidth', 1.5); % RH line
title('Pitch Torque');
ylabel('Torque (Nm)');
xlabel('Stroke Cycle');
ylim([-5*10^-8 5*10^-8])

sgtitle('Filtered Forces and Torques with Standard Deviation');