%% Preamble


[cut_time_force, cut_time_torque] = cut_time(Fly_Master);

%% Force
% Determine the maximum length of all_last data for x, y, z
maxDataLengthLast_x = max(arrayfun(@(x) length(x.all_last.x_first), cut_time_force));
maxDataLengthLast_y = max(arrayfun(@(x) length(x.all_last.y_first), cut_time_force));
maxDataLengthLast_z = max(arrayfun(@(x) length(x.all_last.z_first), cut_time_force));

% Preallocate the arrays for the all_last part
numCuts = length(cut_time_force);
x_period_force_change = zeros(numCuts, maxDataLengthLast_x);
y_period_force_change = zeros(numCuts, maxDataLengthLast_y);
z_period_force_change = zeros(numCuts, maxDataLengthLast_z);

% Initialize cell arrays to store the combined data (all_last + all_first)
x_period_combined = cell(numCuts, 1);
y_period_combined = cell(numCuts, 1);
z_period_combined = cell(numCuts, 1);

% Loop through each cut_time_force entry and extract data for x, y, z
for i = 1:numCuts
    % Extract x, y, z data from all_last and all_first
    xDataLast = cut_time_force(i).all_last.x_first(:)';
    xDataFirst = cut_time_force(i).all_first.x_first(:)';
    yDataLast = cut_time_force(i).all_last.y_first(:)';
    yDataFirst = cut_time_force(i).all_first.y_first(:)';
    zDataLast = cut_time_force(i).all_last.z_first(:)';
    zDataFirst = cut_time_force(i).all_first.z_first(:)';

    % Determine the start index for all_last to align at the end
    startIdxLast_x = maxDataLengthLast_x - length(xDataLast) + 1;
    startIdxLast_y = maxDataLengthLast_y - length(yDataLast) + 1;
    startIdxLast_z = maxDataLengthLast_z - length(zDataLast) + 1;

    % Assign all_last data to the preallocated arrays (right-aligned)
    x_period_force_change(i, startIdxLast_x:end) = xDataLast;
    y_period_force_change(i, startIdxLast_y:end) = yDataLast;
    z_period_force_change(i, startIdxLast_z:end) = zDataLast;

    % Combine all_last (aligned) and all_first (appended) into cells
    x_period_combined{i} = [x_period_force_change(i, :), xDataFirst];
    y_period_combined{i} = [y_period_force_change(i, :), yDataFirst];
    z_period_combined{i} = [z_period_force_change(i, :), zDataFirst];
end

% Convert the cell arrays to matrices (zero-padded where necessary)
maxTotalLength_x = max(cellfun(@length, x_period_combined));
maxTotalLength_y = max(cellfun(@length, y_period_combined));
maxTotalLength_z = max(cellfun(@length, z_period_combined));

x_period_combined_matrix = zeros(numCuts, maxTotalLength_x);
y_period_combined_matrix = zeros(numCuts, maxTotalLength_y);
z_period_combined_matrix = zeros(numCuts, maxTotalLength_z);

for i = 1:numCuts
    x_period_combined_matrix(i, 1:length(x_period_combined{i})) = x_period_combined{i};
    y_period_combined_matrix(i, 1:length(y_period_combined{i})) = y_period_combined{i};
    z_period_combined_matrix(i, 1:length(z_period_combined{i})) = z_period_combined{i};
end

% Replace zeros with NaN
x_period_combined_matrix(x_period_combined_matrix == 0) = NaN;
y_period_combined_matrix(y_period_combined_matrix == 0) = NaN;
z_period_combined_matrix(z_period_combined_matrix == 0) = NaN;

% Calculate norms
norm_period_combined_matrix = sqrt(x_period_combined_matrix.^2 + y_period_combined_matrix.^2 + z_period_combined_matrix.^2);

%% Box-and-Whisker Plots
figure;
subplot(2, 2, 1);
boxplot(x_period_combined_matrix, 'Whisker', 1.5);
xlabel('Time Periods');
ylabel('Sideward Values');
title('Box-and-Whisker Plot for Sideward');
line([5.5, 5.5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

subplot(2, 2, 2);
boxplot(y_period_combined_matrix, 'Whisker', 1.5);
xlabel('Time Periods');
ylabel('Forward Values');
title('Box-and-Whisker Plot for Forward');
line([5.5, 5.5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

subplot(2, 2, 3);
boxplot(z_period_combined_matrix, 'Whisker', 1.5);
xlabel('Time Periods');
ylabel('Upward Values');
title('Box-and-Whisker Plot for Upward');
line([5.5, 5.5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

subplot(2, 2, 4);
boxplot(norm_period_combined_matrix, 'Whisker', 1.5);
xlabel('Time Periods');
ylabel('Norm Values');
title('Box-and-Whisker Plot for Norms');
line([5.5, 5.5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

%% Differences Between Periods
x_period_differences = diff(x_period_combined_matrix, 1, 2);
y_period_differences = diff(y_period_combined_matrix, 1, 2);
z_period_differences = diff(z_period_combined_matrix, 1, 2);
norm_period_differences = diff(norm_period_combined_matrix, 1, 2);

figure;
subplot(2, 2, 1);
boxplot(x_period_differences, 'Whisker', 1.5);
xlabel('Time Period Differences');
ylabel('Sideward Differences');
title('Differences in Sideward Between Periods');
line([5, 5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

subplot(2, 2, 2);
boxplot(y_period_differences, 'Whisker', 1.5);
xlabel('Time Period Differences');
ylabel('Forward Differences');
title('Differences in Forward Between Periods');
line([5, 5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

subplot(2, 2, 3);
boxplot(z_period_differences, 'Whisker', 1.5);
xlabel('Time Period Differences');
ylabel('Upward Differences');
title('Differences in Upward Between Periods');
line([5, 5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

subplot(2, 2, 4);
boxplot(norm_period_differences, 'Whisker', 1.5);
xlabel('Time Period Differences');
ylabel('Norm Differences');
title('Differences in Norm Between Periods');
line([5, 5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;


%% Torque
% Determine the maximum length of all_last data for x, y, z
maxDataLengthLast_x = max(arrayfun(@(x) length(x.all_last.x_first), cut_time_torque));
maxDataLengthLast_y = max(arrayfun(@(x) length(x.all_last.y_first), cut_time_torque));
maxDataLengthLast_z = max(arrayfun(@(x) length(x.all_last.z_first), cut_time_torque));

% Preallocate the arrays for the all_last part
numCuts = length(cut_time_torque);
x_period_force_change = zeros(numCuts, maxDataLengthLast_x);
y_period_force_change = zeros(numCuts, maxDataLengthLast_y);
z_period_force_change = zeros(numCuts, maxDataLengthLast_z);

% Initialize cell arrays to store the combined data (all_last + all_first)
x_period_combined = cell(numCuts, 1);
y_period_combined = cell(numCuts, 1);
z_period_combined = cell(numCuts, 1);

% Loop through each cut_time_torque entry and extract data for x, y, z
for i = 1:numCuts
    % Extract x, y, z data from all_last and all_first
    xDataLast = cut_time_torque(i).all_last.x_first(:)';
    xDataFirst = cut_time_torque(i).all_first.x_first(:)';
    yDataLast = cut_time_torque(i).all_last.y_first(:)';
    yDataFirst = cut_time_torque(i).all_first.y_first(:)';
    zDataLast = cut_time_torque(i).all_last.z_first(:)';
    zDataFirst = cut_time_torque(i).all_first.z_first(:)';

    % Determine the start index for all_last to align at the end
    startIdxLast_x = maxDataLengthLast_x - length(xDataLast) + 1;
    startIdxLast_y = maxDataLengthLast_y - length(yDataLast) + 1;
    startIdxLast_z = maxDataLengthLast_z - length(zDataLast) + 1;

    % Assign all_last data to the preallocated arrays (right-aligned)
    x_period_force_change(i, startIdxLast_x:end) = xDataLast;
    y_period_force_change(i, startIdxLast_y:end) = yDataLast;
    z_period_force_change(i, startIdxLast_z:end) = zDataLast;

    % Combine all_last (aligned) and all_first (appended) into cells
    x_period_combined{i} = [x_period_force_change(i, :), xDataFirst];
    y_period_combined{i} = [y_period_force_change(i, :), yDataFirst];
    z_period_combined{i} = [z_period_force_change(i, :), zDataFirst];
end

% Convert the cell arrays to matrices (zero-padded where necessary)
maxTotalLength_x = max(cellfun(@length, x_period_combined));
maxTotalLength_y = max(cellfun(@length, y_period_combined));
maxTotalLength_z = max(cellfun(@length, z_period_combined));

x_period_combined_matrix = zeros(numCuts, maxTotalLength_x);
y_period_combined_matrix = zeros(numCuts, maxTotalLength_y);
z_period_combined_matrix = zeros(numCuts, maxTotalLength_z);

for i = 1:numCuts
    x_period_combined_matrix(i, 1:length(x_period_combined{i})) = x_period_combined{i};
    y_period_combined_matrix(i, 1:length(y_period_combined{i})) = y_period_combined{i};
    z_period_combined_matrix(i, 1:length(z_period_combined{i})) = z_period_combined{i};
end

% Replace zeros with NaN
x_period_combined_matrix(x_period_combined_matrix == 0) = NaN;
y_period_combined_matrix(y_period_combined_matrix == 0) = NaN;
z_period_combined_matrix(z_period_combined_matrix == 0) = NaN;

% Calculate norms
norm_period_combined_matrix = sqrt(x_period_combined_matrix.^2 + y_period_combined_matrix.^2 + z_period_combined_matrix.^2);

%% Box-and-Whisker Plots
figure;
subplot(2, 2, 1);
boxplot(x_period_combined_matrix, 'Whisker', 1.5);
xlabel('Time Periods');
ylabel('Pitch Values');
title('Box-and-Whisker Plot for Pitch');
line([5.5, 5.5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

subplot(2, 2, 2);
boxplot(y_period_combined_matrix, 'Whisker', 1.5);
xlabel('Time Periods');
ylabel('Roll Values');
title('Box-and-Whisker Plot for Roll');
line([5.5, 5.5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

subplot(2, 2, 3);
boxplot(z_period_combined_matrix, 'Whisker', 1.5);
xlabel('Time Periods');
ylabel('Yaw Values');
title('Box-and-Whisker Plot for Yaw');
line([5.5, 5.5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

subplot(2, 2, 4);
boxplot(norm_period_combined_matrix, 'Whisker', 1.5);
xlabel('Time Periods');
ylabel('Norm Values');
title('Box-and-Whisker Plot for Norms');
line([5.5, 5.5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

%% Differences Between Periods
x_period_differences = diff(x_period_combined_matrix, 1, 2);
y_period_differences = diff(y_period_combined_matrix, 1, 2);
z_period_differences = diff(z_period_combined_matrix, 1, 2);
norm_period_differences = diff(norm_period_combined_matrix, 1, 2);

figure;
subplot(2, 2, 1);
boxplot(x_period_differences, 'Whisker', 1.5);
xlabel('Time Period Differences');
ylabel('Pitch Differences');
title('Differences in Pitch Between Periods');
line([5, 5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

subplot(2, 2, 2);
boxplot(y_period_differences, 'Whisker', 1.5);
xlabel('Time Period Differences');
ylabel('Roll Differences');
title('Differences in Roll Between Periods');
line([5, 5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

subplot(2, 2, 3);
boxplot(z_period_differences, 'Whisker', 1.5);
xlabel('Time Period Differences');
ylabel('Yaw Differences');
title('Differences in Yaw Between Periods');
line([5, 5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;

subplot(2, 2, 4);
boxplot(norm_period_differences, 'Whisker', 1.5);
xlabel('Time Period Differences');
ylabel('Norm Differences');
title('Differences in Norm Between Periods');
line([5, 5], ylim, 'Color', 'red', 'LineWidth', 2, 'LineStyle', '--');
grid on;


%% Functions

function [cut_time_force, cut_time_torque] = cut_time(Fly_Master)
j=1;
for i=1:length(Fly_Master)
    if Fly_Master(i).State == "Pre Cut"
        cut_time_force(j).Fly_Num = Fly_Master(i).Fly_Num;
        cut_time_torque(j).Fly_Num = Fly_Master(i).Fly_Num;


        weight = Fly_Master(i).Fly.Morphology.total.weight;

        if Fly_Master(i+1).chord_cut_LH == 100
            cut_time_force(j).cut_percentage = Fly_Master(i+1).chord_cut_RH;
            cut_time_torque(j).cut_percentage = Fly_Master(i+1).chord_cut_RH;

            Phi  = Fly_Master(i).Fly.Kinematics.RH.phi;
            [~, ~, ~, cut_time_force(j).x_last_pre, cut_time_force(j).y_last_pre, cut_time_force(j).z_last_pre, ~, cut_time_force(j).all_last] = force_period_Mean(i, Phi, Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Force_Total, weight);
            [~, ~, ~, cut_time_torque(j).x_last_pre, cut_time_torque(j).y_last_pre, cut_time_torque(j).z_last_pre, ~, cut_time_torque(j).all_last] = torque_period_Mean(i, Phi, Fly_Master(i).Fly.Dynamics.Frame_Body.LH.Torque_Total, weight);

            Phi  = Fly_Master(i+1).Fly.Kinematics.RH.phi;
            [cut_time_force(j).x_last_post, cut_time_force(j).y_last_post, cut_time_force(j).z_last_post,~, ~, ~, cut_time_force(j).all_first, ~] = force_period_Mean(i+1, Phi, Fly_Master(i+1).Fly.Dynamics.Frame_Body.LH.Force_Total, weight);
            [cut_time_torque(j).x_last_post, cut_time_torque(j).y_last_post, cut_time_torque(j).z_last_post,~, ~, ~, cut_time_torque(j).all_first, ~] = torque_period_Mean(i+1, Phi, Fly_Master(i+1).Fly.Dynamics.Frame_Body.LH.Torque_Total, weight);

        else
            cut_time_force(j).cut_percentage = Fly_Master(i+1).chord_cut_LH;
            cut_time_torque(j).cut_percentage = Fly_Master(i+1).chord_cut_LH;

            Phi  = Fly_Master(i).Fly.Kinematics.LH.phi;
            [~, ~, ~, cut_time_force(j).x_last_pre, cut_time_force(j).y_last_pre, cut_time_force(j).z_last_pre, ~, cut_time_force(j).all_last] = force_period_Mean(i, Phi, Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Force_Total, weight);
            [~, ~, ~, cut_time_torque(j).x_last_pre, cut_time_torque(j).y_last_pre, cut_time_torque(j).z_last_pre, ~, cut_time_torque(j).all_last] = torque_period_Mean(i, Phi, Fly_Master(i).Fly.Dynamics.Frame_Body.RH.Torque_Total, weight);

            Phi  = Fly_Master(i+1).Fly.Kinematics.LH.phi;
            [cut_time_force(j).x_last_post, cut_time_force(j).y_last_post, cut_time_force(j).z_last_post,~, ~, ~, cut_time_force(j).all_first, ~] = force_period_Mean(i+1, Phi, Fly_Master(i+1).Fly.Dynamics.Frame_Body.RH.Force_Total, weight);
            [cut_time_torque(j).x_last_post, cut_time_torque(j).y_last_post, cut_time_torque(j).z_last_post,~, ~, ~, cut_time_torque(j).all_first, ~] = torque_period_Mean(i+1, Phi, Fly_Master(i+1).Fly.Dynamics.Frame_Body.RH.Torque_Total, weight);

        end
        [~,Peaks] = findpeaks(Phi);
        Period = Peaks(end)- Peaks(end-1);
        cut_time_force(j).Percentage_after_peak = (length(Phi) - Peaks(end))/Period;
        cut_time_torque(j).Percentage_after_peak = (length(Phi) - Peaks(end))/Period;

        j = j+1;
    end
end
end

function [x_first, y_first, z_first, x_last, y_last, z_last, all_first, all_last] = force_period_Mean(k, phi, Force_Body, weight)

% Find each period via peaks
[~, peakIndices] = findpeaks(phi);

% Calculate average period length
average_period_length = mean(diff(peakIndices));

% Initialize period indices for the first period
Period_Index_First(1) = 1;

% Initialize counters for the first period
j = 1;
i = 2;
while j < length(phi)
    % Get the reference value for the first period
    ref_value_first = phi(1);

    % Search for the next value after the current peak within ±0.2 of ref_value_first
    for k = peakIndices(i-1):length(phi)
        if abs(phi(k) - ref_value_first) <= 0.2
            % Check if the difference is at least 75% of average period length
            if (k - Period_Index_First(i-1)) >= 0.9 * average_period_length
                Period_Index_First(i) = k; % Store the index
                j = k; % Update the loop index
                break;
            end
        end
    end

    i = i + 1; % Move to the next period index
    if j > peakIndices(end) || i > length(peakIndices)
        break;
    end
end

% Find each period via peaks (negative phi for last period detection)
[~, peakIndices] = findpeaks(-phi);

% Initialize period indices for the last period
Period_Index_Last(1) = length(phi);

% Initialize counters for the last period
j = length(phi);
i = 2;
while j > 1
    % Get the reference value for the last period
    ref_value_last = phi(end);

    % Search backward for the previous value within ±0.2 of ref_value_last
    for k = peakIndices(end-i+2):-1:1
        if abs(phi(k) - ref_value_last) <= 0.2
            % Check if the difference is at least 75% of average period length
            if (Period_Index_Last(i-1) - k) >= 0.9 * average_period_length
                Period_Index_Last(i) = k; % Store the index
                j = k; % Update the loop index
                break;
            end
        end
    end

    i = i + 1; % Move to the next period index
    if j < peakIndices(1) || i > length(peakIndices)
        break;
    end
end

% Ensure the indices are sorted for consistency
Period_Index_First = sort(Period_Index_First);
Period_Index_Last = sort(Period_Index_Last, 'descend');

% Initialize array for mean forces (first and last periods)
Mean_Forces = zeros(2, 3); % Rows: [First, Last]; Columns: [X, Y, Z]

% Initialize a matrix to store mean forces for first and last periods
Mean_Forces = zeros(2, 3);

% Loop for first and last periods
for p = 1:2
    % Define start and end indices based on the period
    if p == 1
        % First period
        start_idx = Period_Index_First(1);
        end_idx = Period_Index_First(end);
    else
        % Last period
        start_idx = Period_Index_Last(end);
        end_idx = Period_Index_Last(1);
    end

    % Calculate mean forces for the period
    Mean_Forces(p, 1) = mean(Force_Body(1, start_idx:end_idx)) / weight; % x-direction
    Mean_Forces(p, 3) = mean(Force_Body(3, start_idx:end_idx)) / weight; % z-direction
    Mean_Forces(p, 2) = mean(Force_Body(2, start_idx:end_idx)) / weight; % y-direction
end

% Store the results in separate variables for clarity
x_first = Mean_Forces(1, 1); % x-direction mean for first period
y_first = Mean_Forces(1, 3); % y-direction mean for first period
z_first = Mean_Forces(1, 2); % z-direction mean for first period

x_last = Mean_Forces(2, 1); % x-direction mean for last period
y_last = Mean_Forces(2, 3); % y-direction mean for last period
z_last = Mean_Forces(2, 2); % z-direction mean for last period

% Loop for all first periods
for p =1:length(Period_Index_First)-1
    % Define start and end indices based on the period
    start_idx = Period_Index_First(p);
    end_idx = Period_Index_First(p+1);

    % Calculate mean forces for the period
    all_first.x_first(p) = mean(Force_Body(1, start_idx:end_idx)) / weight; % x-direction
    all_first.y_first(p) = mean(Force_Body(3, start_idx:end_idx)) / weight; % z-direction
    all_first.z_first(p) = mean(Force_Body(2, start_idx:end_idx)) / weight; % y-direction
end

% Loop for all last periods
for p =1:length(Period_Index_Last)-1
    % Define start and end indices based on the period
    end_idx = Period_Index_Last(p);
    start_idx = Period_Index_Last(p+1);

    % Calculate mean forces for the period
    all_last.x_first(p) = mean(Force_Body(1, start_idx:end_idx)) / weight; % x-direction
    all_last.y_first(p) = mean(Force_Body(3, start_idx:end_idx)) / weight; % z-direction
    all_last.z_first(p) = mean(Force_Body(2, start_idx:end_idx)) / weight; % y-direction
end

end

function [x_first, y_first, z_first, x_last, y_last, z_last, all_first, all_last] = torque_period_Mean(k, phi, Torque_Body, weight)

% Find each period via peaks
[~, peakIndices] = findpeaks(phi);

% Calculate average period length
average_period_length = mean(diff(peakIndices));

% Initialize period indices for the first period
Period_Index_First(1) = 1;

% Initialize counters for the first period
j = 1;
i = 2;
while j < length(phi)
    % Get the reference value for the first period
    ref_value_first = phi(1);

    % Search for the next value after the current peak within ±0.2 of ref_value_first
    for k = peakIndices(i-1):length(phi)
        if abs(phi(k) - ref_value_first) <= 0.2
            % Check if the difference is at least 75% of average period length
            if (k - Period_Index_First(i-1)) >= 0.9 * average_period_length
                Period_Index_First(i) = k; % Store the index
                j = k; % Update the loop index
                break;
            end
        end
    end

    i = i + 1; % Move to the next period index
    if j > peakIndices(end) || i > length(peakIndices)
        break;
    end
end

% Find each period via peaks (negative phi for last period detection)
[~, peakIndices] = findpeaks(-phi);

% Initialize period indices for the last period
Period_Index_Last(1) = length(phi);

% Initialize counters for the last period
j = length(phi);
i = 2;
while j > 1
    % Get the reference value for the last period
    ref_value_last = phi(end);

    % Search backward for the previous value within ±0.2 of ref_value_last
    for k = peakIndices(end-i+2):-1:1
        if abs(phi(k) - ref_value_last) <= 0.2
            % Check if the difference is at least 75% of average period length
            if (Period_Index_Last(i-1) - k) >= 0.9 * average_period_length
                Period_Index_Last(i) = k; % Store the index
                j = k; % Update the loop index
                break;
            end
        end
    end

    i = i + 1; % Move to the next period index
    if j < peakIndices(1) || i > length(peakIndices)
        break;
    end
end

% Ensure the indices are sorted for consistency
Period_Index_First = sort(Period_Index_First);
Period_Index_Last = sort(Period_Index_Last, 'descend');


% Initialize array for mean forces (first and last periods)
Mean_Torques = zeros(2, 3); % Rows: [First, Last]; Columns: [X, Y, Z]

% Initialize a matrix to store mean forces for first and last periods
Mean_Torques = zeros(2, 3);

% Loop for first and last periods
for p = 1:2
    % Define start and end indices based on the period
    if p == 1
        % First period
        start_idx = Period_Index_First(1);
        end_idx = Period_Index_First(end);
    else
        % Last period
        start_idx = Period_Index_Last(end);
        end_idx = Period_Index_Last(1);
    end

    % Calculate mean forces for the period
    Mean_Torques(p, 1) = mean(Torque_Body(1, start_idx:end_idx)) / weight; % x-direction
    Mean_Torques(p, 3) = mean(Torque_Body(3, start_idx:end_idx)) / weight; % z-direction
    Mean_Torques(p, 2) = mean(Torque_Body(2, start_idx:end_idx)) / weight; % y-direction
end

% Store the results in separate variables for clarity
x_first = Mean_Torques(1, 1); % x-direction mean for first period
y_first = Mean_Torques(1, 3); % y-direction mean for first period
z_first = Mean_Torques(1, 2); % z-direction mean for first period

x_last = Mean_Torques(2, 1); % x-direction mean for last period
y_last = Mean_Torques(2, 3); % y-direction mean for last period
z_last = Mean_Torques(2, 2); % z-direction mean for last period

% Loop for all first periods
for p =1:length(Period_Index_First)-1
    % Define start and end indices based on the period
    start_idx = Period_Index_First(p);
    end_idx = Period_Index_First(p+1);

    % Calculate mean forces for the period
    all_first.x_first(p) = mean(Torque_Body(1, start_idx:end_idx)) / weight; % x-direction
    all_first.y_first(p) = mean(Torque_Body(3, start_idx:end_idx)) / weight; % z-direction
    all_first.z_first(p) = mean(Torque_Body(2, start_idx:end_idx)) / weight; % y-direction
end

% Loop for all last periods
for p =1:length(Period_Index_Last)-1
    % Define start and end indices based on the period
    end_idx = Period_Index_Last(p);
    start_idx = Period_Index_Last(p+1);

    % Calculate mean forces for the period
    all_last.x_first(p) = mean(Torque_Body(1, start_idx:end_idx)) / weight; % x-direction
    all_last.y_first(p) = mean(Torque_Body(3, start_idx:end_idx)) / weight; % z-direction
    all_last.z_first(p) = mean(Torque_Body(2, start_idx:end_idx)) / weight; % y-direction
end

end
