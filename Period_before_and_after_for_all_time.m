j = 1;
for i = 1:length(Fly_Master)
    if Fly_Master(i).State == "Pre Cut"
        cut_time(j).Fly_Num = Fly_Master(i).Fly_Num;
        if Fly_Master(i + 1).chord_cut_LH == 100
            cut_time(j).cut_percentage = Fly_Master(i + 1).chord_cut_RH;

            Phi = Fly_Master(i).Fly.Kinematics_RH.phi;
            [~, ~, ~, cut_time(j).x_last_pre, cut_time(j).y_last_pre, cut_time(j).z_last_pre] = ...
                force_period_Mean(i, Phi, Fly_Master(i).Fly.force_total.Force_Body_LH, Fly_Master(i).Fly.total.weight);

            Phi = Fly_Master(i + 1).Fly.Kinematics_RH.phi;
            [cut_time(j).x_first_post, cut_time(j).y_first_post, cut_time(j).z_first_post, ~, ~, ~] = ...
                force_period_Mean(i + 1, Phi, Fly_Master(i + 1).Fly.force_total.Force_Body_LH, Fly_Master(i).Fly.total.weight);
        else
            cut_time(j).cut_percentage = Fly_Master(i + 1).chord_cut_LH;

            Phi = Fly_Master(i).Fly.Kinematics_LH.phi;
            [~, ~, ~, cut_time(j).x_last_pre, cut_time(j).y_last_pre, cut_time(j).z_last_pre] = ...
                force_period_Mean(i, Phi, Fly_Master(i).Fly.force_total.Force_Body_RH, Fly_Master(i).Fly.total.weight);

            Phi = Fly_Master(i + 1).Fly.Kinematics_LH.phi;
            [cut_time(j).x_first_post, cut_time(j).y_first_post, cut_time(j).z_first_post, ~, ~, ~] = ...
                force_period_Mean(i + 1, Phi, Fly_Master(i + 1).Fly.force_total.Force_Body_RH, Fly_Master(i).Fly.total.weight);
        end

        [~, Peaks] = findpeaks(Phi);
        Period = Peaks(end) - Peaks(end - 1);
        cut_time(j).Percentage_after_peak = (length(Phi) - Peaks(end)) / Period;

        j = j + 1;
    end
end

%% Scatter Plots
fields = {'x', 'y', 'z'};
titles = {'X Force (Sideward)', 'Y Force (Forward)', 'Z Force (Upward)'};
for c = 1:3
    figure;
    hold on;

    % Extract data
    percentages = [cut_time.cut_percentage];
    x_values = [cut_time.Percentage_after_peak];
    y_values = abs([cut_time.([fields{c}, '_first_post'])] - [cut_time.([fields{c}, '_last_pre'])]);

    % Scatter plot
    scatter(x_values, y_values, 36, percentages, 'filled');
    colormap(jet);
    cbar = colorbar;
    cbar.Label.String = 'Cut Percentage';

    % Labels
    xlabel('Percentage After Peak');
    ylabel('Change in Force');
    title(titles{c});
    hold off;
end

% Vector Norm Plot
figure;
hold on;

% Extract data for norm calculation
percentages = [cut_time.cut_percentage];
x_values = [cut_time.Percentage_after_peak];
norm_values = sqrt( ...
    ([cut_time.x_first_post] - [cut_time.x_last_pre]).^2 + ...
    ([cut_time.y_first_post] - [cut_time.y_last_pre]).^2 + ...
    ([cut_time.z_first_post] - [cut_time.z_last_pre]).^2);

% Scatter plot
scatter(x_values, norm_values, 36, percentages, 'filled');
colormap(jet);
cbar = colorbar;
cbar.Label.String = 'Cut Percentage';

% Labels
xlabel('Percentage After Peak');
ylabel('Vector Norm of Change');
title('Change in Force (Vector Norm)');
hold off;

%% Helper Function: force_period_Mean
function [x_first, y_first, z_first, x_last, y_last, z_last] = force_period_Mean(k, phi, Force_Body, weight)

    % Find each period via peaks
    [~, peakIndices] = findpeaks(phi);

    % Calculate average period length
    average_period_length = mean(diff(peakIndices));

    % Indices for first period
    start_first = peakIndices(1);
    end_first = peakIndices(2);

    % Indices for last period
    start_last = peakIndices(end - 1);
    end_last = peakIndices(end);

    % Calculate mean forces
    x_first = mean(Force_Body.force_total_vec(1, start_first:end_first)) / weight; % X-direction (first period)
    y_first = mean(Force_Body.force_total_vec(2, start_first:end_first)) / weight; % Y-direction (first period)
    z_first = mean(Force_Body.force_total_vec(3, start_first:end_first)) / weight; % Z-direction (first period)

    x_last = mean(Force_Body.force_total_vec(1, start_last:end_last)) / weight; % X-direction (last period)
    y_last = mean(Force_Body.force_total_vec(2, start_last:end_last)) / weight; % Y-direction (last period)
    z_last = mean(Force_Body.force_total_vec(3, start_last:end_last)) / weight; % Z-direction (last period)
end
