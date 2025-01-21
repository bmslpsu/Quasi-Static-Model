
j=1;
for i=1:length(Fly_Master)
    if Fly_Master(i).State == "Pre Cut"
        cut_time(j).Fly_Num = Fly_Master(i).Fly_Num;
        if Fly_Master(i+1).chord_cut_LH == 100
            cut_time(j).cut_percentage = Fly_Master(i+1).chord_cut_RH;

            Phi  = Fly_Master(i).Fly.Kinematics_RH.phi;
            [~, ~, ~, cut_time(j).x_last_pre, cut_time(j).y_last_pre, cut_time(j).z_last_pre, ~, cut_time(j).all_last] = force_period_Mean(i, Phi, Fly_Master(i).Fly.force_total.Force_Body_LH, Fly_Master(i).Fly.total.weight);

            Phi  = Fly_Master(i+1).Fly.Kinematics_RH.phi;
            [cut_time(j).x_last_post, cut_time(j).y_last_post, cut_time(j).z_last_post,~, ~, ~, cut_time(j).all_first, ~] = force_period_Mean(i+1, Phi, Fly_Master(i+1).Fly.force_total.Force_Body_LH, Fly_Master(i).Fly.total.weight);
        else
            cut_time(j).cut_percentage = Fly_Master(i+1).chord_cut_LH;
            
            Phi  = Fly_Master(i).Fly.Kinematics_LH.phi;
            [~, ~, ~, cut_time(j).x_last_pre, cut_time(j).y_last_pre, cut_time(j).z_last_pre, ~, cut_time(j).all_last] = force_period_Mean(i, Phi, Fly_Master(i).Fly.force_total.Force_Body_RH, Fly_Master(i).Fly.total.weight);

            Phi  = Fly_Master(i+1).Fly.Kinematics_LH.phi;
            [cut_time(j).x_last_post, cut_time(j).y_last_post, cut_time(j).z_last_post,~, ~, ~, cut_time(j).all_first, ~] = force_period_Mean(i+1, Phi, Fly_Master(i+1).Fly.force_total.Force_Body_RH, Fly_Master(i).Fly.total.weight);
        end
        [~,Peaks] = findpeaks(Phi);
        Period = Peaks(end)- Peaks(end-1);
        cut_time(j).Percentage_after_peak = (length(Phi) - Peaks(end))/Period;

        j = j+1;
    end
end


%%
% % figure;
% % hold on;
% % 
% % % Extract the data
% % percentages = [cut_time.cut_percentage]; % Extract cut_percentage values
% % x_values = [cut_time.Percentage_after_peak];
% % y_values = abs([cut_time.x_last_post] - [cut_time.x_last_pre]);
% % 
% % % Create a scatter plot with colors mapped to cut_percentage
% % scatter(x_values, y_values, 36, percentages, 'filled');
% % 
% % % Add colorbar
% % colormap(jet); % Use 'jet' colormap, or any other colormap you prefer
% % c = colorbar;
% % c.Label.String = 'Cut Percentage';
% % 
% % % Add labels and title
% % xlabel('Percentage After Peak');
% % ylabel('Change in Force');
% % title("X Force (Sidward)")
% % hold off;
% % 
% % figure;
% % hold on;
% % 
% % % Extract the data
% % percentages = [cut_time.cut_percentage]; % Extract cut_percentage values
% % x_values = [cut_time.Percentage_after_peak];
% % y_values = abs([cut_time.y_last_post] - [cut_time.y_last_pre]);
% % 
% % % Create a scatter plot with colors mapped to cut_percentage
% % scatter(x_values, y_values, 36, percentages, 'filled');
% % 
% % % Add colorbar
% % colormap(jet); % Use 'jet' colormap, or any other colormap you prefer
% % c = colorbar;
% % c.Label.String = 'Cut Percentage';
% % 
% % % Add labels and title
% % xlabel('Percentage After Peak');
% % ylabel('Change in Force');
% % title("Y Force (Forward)")
% % hold off;
% % 
% % figure;
% % hold on;
% % 
% % % Extract the data
% % percentages = [cut_time.cut_percentage]; % Extract cut_percentage values
% % x_values = [cut_time.Percentage_after_peak];
% % y_values = abs([cut_time.z_last_post] - [cut_time.z_last_pre]);
% % 
% % % Create a scatter plot with colors mapped to cut_percentage
% % scatter(x_values, y_values, 36, percentages, 'filled');
% % 
% % % Add colorbar
% % colormap(jet); % Use 'jet' colormap, or any other colormap you prefer
% % c = colorbar;
% % c.Label.String = 'Cut Percentage';
% % 
% % % Add labels and title
% % xlabel('Percentage After Peak');
% % ylabel('Change in Force');
% % title("Z Force (Up)")
% % hold off;

figure;
hold on;

% Extract the data
percentages = [cut_time.cut_percentage]; % Extract cut_percentage values
x_values = [cut_time.Percentage_after_peak];

% Calculate the vector norm for x, y, and z differences
norm_values = sqrt( ...
    ([cut_time.x_last_post] - [cut_time.x_last_pre]).^2 + ...
    ([cut_time.y_last_post] - [cut_time.y_last_pre]).^2 + ...
    ([cut_time.z_last_post] - [cut_time.z_last_pre]).^2);


% Create a scatter plot with colors mapped to cut_percentage
scatter(x_values, norm_values, 36, percentages, 'filled');
t_length = 0:1;
plot(t_length, mean(norm_values)*ones(length(t_length)), 'k--');

% Add colorbar
colormap(jet); % Use 'jet' colormap, or any other colormap you prefer
c = colorbar;
c.Label.String = 'Cut Percentage';

ylim([0, 1.4]);

% Add labels and title
xlabel('Percentage After Peak');
ylabel('Force Change');
title('Undamaged Wing');
hold off;

%%
% Define fly numbers
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


% % Plot the stroke angle (phi)
% figure;
% hold on;
% plot(phi, 'b', 'LineWidth', 1.5); % Plot phi with a blue line
% plot(Period_Index_First, phi(Period_Index_First), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5); % First period indices in red
% plot(Period_Index_Last, phi(Period_Index_Last), 'go', 'MarkerSize', 8, 'LineWidth', 1.5); % Last period indices in green
% xlabel('Time Step');
% ylabel('Stroke Angle (\phi)');
% title('Stroke Angle (\phi) with Period Indices');
% legend({'\phi (Stroke Angle)', 'First Period Indices', 'Last Period Indices'}, 'Location', 'Best');
% grid on;
% hold off;


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
    Mean_Forces(p, 1) = mean(Force_Body.force_total_vec(1, start_idx:end_idx)) / weight; % x-direction
    Mean_Forces(p, 3) = mean(Force_Body.force_total_vec(3, start_idx:end_idx)) / weight; % z-direction
    Mean_Forces(p, 2) = mean(Force_Body.force_total_vec(2, start_idx:end_idx)) / weight; % y-direction
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
    all_first.x_first(p) = mean(Force_Body.force_total_vec(1, start_idx:end_idx)) / weight; % x-direction
    all_first.y_first(p) = mean(Force_Body.force_total_vec(3, start_idx:end_idx)) / weight; % z-direction
    all_first.z_first(p) = mean(Force_Body.force_total_vec(2, start_idx:end_idx)) / weight; % y-direction
end

% Loop for all last periods
for p =1:length(Period_Index_Last)-1
    % Define start and end indices based on the period
    end_idx = Period_Index_Last(p);
    start_idx = Period_Index_Last(p+1);

    % Calculate mean forces for the period
    all_last.x_first(p) = mean(Force_Body.force_total_vec(1, start_idx:end_idx)) / weight; % x-direction
    all_last.y_first(p) = mean(Force_Body.force_total_vec(3, start_idx:end_idx)) / weight; % z-direction
    all_last.z_first(p) = mean(Force_Body.force_total_vec(2, start_idx:end_idx)) / weight; % y-direction
end

end