fly_num = [1, 3, 5, 8, 10, 13, 16, 18, 21, 24];
fly_num=fly_num+1;

units_force = 10^-6;
units_torque = 10^-10;

numTimePoints = 100; % Fixed number of normalized time points
numFlies = length(fly_num);

% Define storage for all flies' mean forces and torques
meanForcesSumAllFlies = zeros(3, numTimePoints, numFlies);
meanTorquesSumAllFlies = zeros(3, numTimePoints, numFlies);

% Define color map for each fly
flyColors = lines(numFlies); % MATLAB 'lines' colormap for distinct colors
normalizedTime = linspace(0, 1, numTimePoints);

for flyIdx = 1:numFlies
    phi = Fly_Master(fly_num(flyIdx)).Fly.Kinematics.LH.phi;
    force_lh = Fly_Master(fly_num(flyIdx)).Fly.Dynamics.Frame_Body.LH.Force_Total * units_force.* [1;-1;1];
    force_rh = Fly_Master(fly_num(flyIdx)).Fly.Dynamics.Frame_Body.RH.Force_Total * units_force.* [1;-1;1];
    torque_lh = Fly_Master(fly_num(flyIdx)).Fly.Dynamics.Frame_Body.LH.Torque_Total * units_torque.* [-1;1;1];
    torque_rh = Fly_Master(fly_num(flyIdx)).Fly.Dynamics.Frame_Body.RH.Torque_Total * units_torque.* [-1;1;1];

    [~, peakIndices] = findpeaks(phi);
    
    if length(peakIndices) < 2
        continue; % Skip if not enough peaks
    end

    startIdx = peakIndices(1);
    endIdx = peakIndices(2);

    % Normalize forces
    forceSum_norm = interp1(linspace(0, 1, endIdx - startIdx + 1), ...
                            (force_lh(:, startIdx:endIdx) + force_rh(:, startIdx:endIdx))', ...
                            normalizedTime, 'linear', 'extrap')';

    % Normalize torques
    torqueSum_norm = interp1(linspace(0, 1, endIdx - startIdx + 1), ...
                             (torque_lh(:, startIdx:endIdx) + torque_rh(:, startIdx:endIdx))', ...
                             normalizedTime, 'linear', 'extrap')';

    % Store mean values for each fly
    meanForcesSumAllFlies(:, :, flyIdx) = forceSum_norm;
    meanTorquesSumAllFlies(:, :, flyIdx) = torqueSum_norm;
end

% Compute mean and std across all flies
meanForcesSumAcrossFlies = mean(meanForcesSumAllFlies, 3);
stdForcesSumAcrossFlies = std(meanForcesSumAllFlies, 0, 3);
meanTorquesSumAcrossFlies = mean(meanTorquesSumAllFlies, 3);
stdTorquesSumAcrossFlies = std(meanTorquesSumAllFlies, 0, 3);

% Plot Forces and Torques with Overlaid Individual Means
figure;
components = {'Upward Force', 'Yaw Torque', 'Forward Force', ...
             'Roll Torque', 'Sideward Force', 'Pitch Torque'};
ylabels = {'Force (N)', 'Torque (Nm)', 'Force (N)', 'Torque (Nm)', 'Force (N)', 'Torque (Nm)'};
ylim_values = [-5e-5, 5e-5; -5e-8, 5e-8; -5e-5, 5e-5; -5e-8, 5e-8; -5e-5, 5e-5; -5e-8, 5e-8];
indices_force = [3, 3, 2, 2, 1, 1]; % Indices for forces and torques
indices_torque = [3, 3, 2, 2, 1, 1];

for i = 1:6
    subplot(3, 2, i);
    hold on;
    
    
    % Standard deviation shading
    if mod(i, 2) == 1  % Force Plots
        fill([normalizedTime, fliplr(normalizedTime)], ...
             [meanForcesSumAcrossFlies(indices_force(i), :) + stdForcesSumAcrossFlies(indices_force(i), :), ...
              fliplr(meanForcesSumAcrossFlies(indices_force(i), :) - stdForcesSumAcrossFlies(indices_force(i), :))], ...
             [0.8, 0.8, 0.8], 'EdgeColor', 'none');
    else  % Torque Plots
        fill([normalizedTime, fliplr(normalizedTime)], ...
             [meanTorquesSumAcrossFlies(indices_torque(i), :) + stdTorquesSumAcrossFlies(indices_torque(i), :), ...
              fliplr(meanTorquesSumAcrossFlies(indices_torque(i), :) - stdTorquesSumAcrossFlies(indices_torque(i), :))], ...
             [0.8, 0.8, 0.8], 'EdgeColor', 'none');
    end

        % Plot overall mean as bold black line
    if mod(i, 2) == 1  % Force Plots
        plot(normalizedTime, meanForcesSumAcrossFlies(indices_force(i), :), 'k', 'LineWidth', 2);
    else  % Torque Plots
        plot(normalizedTime, meanTorquesSumAcrossFlies(indices_torque(i), :), 'k', 'LineWidth', 2);
    end
    
    % Individual fly means in unique colors
    for flyIdx = 1:numFlies
        if mod(i, 2) == 1  % Force Plots
            plot(normalizedTime, meanForcesSumAllFlies(indices_force(i), :, flyIdx), ...
                 'Color', flyColors(flyIdx, :), 'LineWidth', 1);
        else  % Torque Plots
            plot(normalizedTime, meanTorquesSumAllFlies(indices_torque(i), :, flyIdx), ...
                 'Color', flyColors(flyIdx, :), 'LineWidth', 1);
        end
    end
    

    if i == 5
        xlabel('Normalized Stroke Cycle');
    elseif i == 6
        legend({'Std Dev', 'Overall Mean', 'Fly 1', 'Fly 3', 'Fly 5', 'Fly 8', 'Fly 10', 'Fly 13', 'Fly 16', 'Fly 18', 'Fly 21', 'Fly 24'}, ...
       'Location', 'bestoutside')
        xlabel('Normalized Stroke Cycle');
    else
    set(gca, 'XColor', 'none') % Hides the x-axis
    end

    % Formatting
    title(components{i});
    ylabel(ylabels{i});
    ylim(ylim_values(i, :));
    grid on;
    
end

sgtitle('2005 Fry Figure 5: Force and Torque Means Over Stroke Cycle - Post Cut');
