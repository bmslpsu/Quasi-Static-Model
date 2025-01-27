%% Preamble


%%
figure;
hold on;

% Extract the data
percentages = [cut_time.cut_percentage]; % Extract cut_percentage values
x_values = [cut_time.Percentage_after_peak];
y_values = abs([cut_time.x_last_post] - [cut_time.x_last_pre]);

% Create a scatter plot with colors mapped to cut_percentage
scatter(x_values, y_values, 36, percentages, 'filled');

% Add colorbar
colormap(jet); % Use 'jet' colormap, or any other colormap you prefer
c = colorbar;
c.Label.String = 'Cut Percentage';

% Add labels and title
xlabel('Percentage After Peak');
ylabel('Change in Force');
title("X Force (Sidward)")
hold off;

figure;
hold on;

% Extract the data
percentages = [cut_time.cut_percentage]; % Extract cut_percentage values
x_values = [cut_time.Percentage_after_peak];
y_values = abs([cut_time.y_last_post] - [cut_time.y_last_pre]);

% Create a scatter plot with colors mapped to cut_percentage
scatter(x_values, y_values, 36, percentages, 'filled');

% Add colorbar
colormap(jet); % Use 'jet' colormap, or any other colormap you prefer
c = colorbar;
c.Label.String = 'Cut Percentage';

% Add labels and title
xlabel('Percentage After Peak');
ylabel('Change in Force');
title("Y Force (Forward)")
hold off;

figure;
hold on;

% Extract the data
percentages = [cut_time.cut_percentage]; % Extract cut_percentage values
x_values = [cut_time.Percentage_after_peak];
y_values = abs([cut_time.z_last_post] - [cut_time.z_last_pre]);

% Create a scatter plot with colors mapped to cut_percentage
scatter(x_values, y_values, 36, percentages, 'filled');

% Add colorbar
colormap(jet); % Use 'jet' colormap, or any other colormap you prefer
c = colorbar;
c.Label.String = 'Cut Percentage';

% Add labels and title
xlabel('Percentage After Peak');
ylabel('Change in Force');
title("Z Force (Up)")
hold off;

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


figure;
hold on;

% Extract the data
percentages = [cut_time.cut_percentage]; % Extract cut_percentage values
x_values = [cut_time.Percentage_after_peak];
y_values = abs([cut_time.x_last_post] - [cut_time.x_last_pre]);

% Create a scatter plot with colors mapped to cut_percentage
scatter(x_values, y_values, 36, percentages, 'filled');

% Add colorbar
colormap(jet); % Use 'jet' colormap, or any other colormap you prefer
c = colorbar;
c.Label.String = 'Cut Percentage';

% Add labels and title
xlabel('Percentage After Peak');
ylabel('Change in Force');
title("X Force (Sidward)")
hold off;

figure;
hold on;

% Extract the data
percentages = [cut_time.cut_percentage]; % Extract cut_percentage values
x_values = [cut_time.Percentage_after_peak];
y_values = abs([cut_time.y_last_post] - [cut_time.y_last_pre]);

% Create a scatter plot with colors mapped to cut_percentage
scatter(x_values, y_values, 36, percentages, 'filled');

% Add colorbar
colormap(jet); % Use 'jet' colormap, or any other colormap you prefer
c = colorbar;
c.Label.String = 'Cut Percentage';

% Add labels and title
xlabel('Percentage After Peak');
ylabel('Change in Force');
title("Y Force (Forward)")
hold off;

figure;
hold on;

% Extract the data
percentages = [cut_time.cut_percentage]; % Extract cut_percentage values
x_values = [cut_time.Percentage_after_peak];
y_values = abs([cut_time.z_last_post] - [cut_time.z_last_pre]);

% Create a scatter plot with colors mapped to cut_percentage
scatter(x_values, y_values, 36, percentages, 'filled');

% Add colorbar
colormap(jet); % Use 'jet' colormap, or any other colormap you prefer
c = colorbar;
c.Label.String = 'Cut Percentage';

% Add labels and title
xlabel('Percentage After Peak');
ylabel('Change in Force');
title("Z Force (Up)")
hold off;

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

% ylim([0, 1.4]);

% Add labels and title
xlabel('Percentage After Peak');
ylabel('Torque Change');
title('Undamaged Wing');
hold off;