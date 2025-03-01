%%Unknown Plots


%% force Box and Whisker
figure;
hold on;

% Concatenate your force data into a single vector
allForces = [Force_X_mean; Force_Y_mean; Force_Z_mean];

% Create a grouping variable
group = [repmat("X", length(Force_X_mean), 1);
    repmat("Y", length(Force_Y_mean), 1);
    repmat("Z", length(Force_Z_mean), 1)];

% Plot the boxplot
boxplot(allForces, group);
%boxplot(allForces, group, 'Colors', [1, 0.5, 0; 0, 1, 0; 0, 0, 1]);

% Customize the plot
ylabel('Normalized Forces (F/mg)');
title(['Box and Whisker Plot of Normalized Forces for Flys']);

hold off;

%% Plot of change in force over periods

% Define fly numbers
fly_numbers = [1,2];

% Initialize storage for results
results = struct();

% Initialize a figure for combined plotting
figure;

% Loop through each fly number
for fly_idx = 1:length(fly_numbers)

    clear Period_Index

    % Access Fly data from the structured array
    fly_num = Fly_Master(1, fly_numbers(fly_idx));
    phi = fly_num.Fly.Kinematics.LH.phi; % Stroke angle (phi)

    % Find each period via peaks
    [peaks, peakIndices] = findpeaks(phi);

    % Initialize period indices
    j = 1;
    Period_Index(1) = 1;

    i = 2;
    while j < length(phi)
        % Get the last held value
        last_value = phi(1);

        % Search for the next value after the current peak within ±0.1 of last_value
        for k = peakIndices(i-1):length(phi)
            if abs(phi(k) - last_value) <= 0.2
                Period_Index(i) = k; % Store the index
                j = k; % Update the loop index
                break;
            end
        end

        i = i + 1; % Move to the next period index
        if j > peakIndices(end) || i > length(peakIndices)
            break;
        end
    end

    % Initialize arrays for storing mean forces
    Force_X_mean = zeros(1, length(Period_Index) - 1);
    Force_Y_mean = zeros(1, length(Period_Index) - 1);
    Force_Z_mean = zeros(1, length(Period_Index) - 1);

    % Loop through each period to calculate mean forces
    for p = 1:(length(Period_Index) - 1)
        % Extract the start and end indices for the current period
        start_idx = Period_Index(p);
        end_idx = Period_Index(p + 1) - 1; % Exclude the endpoint of the next period

        % Calculate mean forces for the current period
        Force_X_mean(p) = (mean(fly_num.Fly.Dynamics.Frame_Body.LH.Dynamics.Force_Total(1, start_idx:end_idx)) + ...
                           mean(fly_num.Fly.Dynamics.Frame_Body.RH.Dynamics.Force_Total(1, start_idx:end_idx))) / fly_num.Fly.total.weight;

        Force_Y_mean(p) = (mean(fly_num.Fly.Dynamics.Frame_Body.LH.Dynamics.Force_Total(3, start_idx:end_idx)) - ...
                           mean(fly_num.Fly.Dynamics.Frame_Body.RH.Dynamics.Force_Total(3, start_idx:end_idx))) / fly_num.Fly.total.weight;

        Force_Z_mean(p) = (mean(fly_num.Fly.Dynamics.Frame_Body.LH.Dynamics.Force_Total(2, start_idx:end_idx)) + ...
                           mean(fly_num.Fly.Dynamics.Frame_Body.RH.Dynamics.Force_Total(2, start_idx:end_idx))) / fly_num.Fly.total.weight;
    end

    % Define time intervals for each period
    period_times = fly_num.Fly.time(Period_Index);

    % Interpolate mean forces over the entire time vector
    time_vector = fly_num.Fly.time;
    Force_X_interp = interp1(period_times(1:end-1), Force_X_mean, time_vector, 'previous', 'extrap');
    Force_Y_interp = interp1(period_times(1:end-1), Force_Y_mean, time_vector, 'previous', 'extrap');
    Force_Z_interp = interp1(period_times(1:end-1), Force_Z_mean, time_vector, 'previous', 'extrap');

    % Mask positive and negative values
    Force_X_pos = Force_X_interp;
    Force_X_neg = Force_X_interp;
    Force_X_pos(Force_X_interp <= 0) = NaN;
    Force_X_neg(Force_X_interp >= 0) = NaN;

    Force_Y_pos = Force_Y_interp;
    Force_Y_neg = Force_Y_interp;
    Force_Y_pos(Force_Y_interp <= 0) = NaN;
    Force_Y_neg(Force_Y_interp >= 0) = NaN;

    Force_Z_pos = Force_Z_interp;
    Force_Z_neg = Force_Z_interp;
    Force_Z_pos(Force_Z_interp <= 0) = NaN;
    Force_Z_neg(Force_Z_interp >= 0) = NaN;

if strcmp(fly_num.State, 'Pre Cut') % Check if it's pre-cut
    color_pos = 'b'; % Blue for positive
    color_neg = 'b'; % Blue for negative
else
    color_pos = 'r'; % Orange (use 'r' for red; MATLAB doesn't have orange) for positive
    color_neg = 'r'; % Orange for negative
end
    
    
    % Plot forces over time
subplot(3, 1, 1);
hold on;

plot(time_vector / 8000, abs(Force_X_pos), '-', 'LineWidth', 1.5, 'Color', color_pos, 'DisplayName', [fly_num.State, ' Positive']);
plot(time_vector / 8000, abs(Force_X_neg), '--', 'LineWidth', 1.5, 'Color', color_neg, 'DisplayName', [fly_num.State, ' Negative']);
ylabel('Side Force (F_x/mg)');
title('Forces Over Time');
legend('show');
grid on;

subplot(3, 1, 2);
hold on;
plot(time_vector / 8000, abs(Force_Y_pos), '-', 'LineWidth', 1.5, 'Color', color_pos);
plot(time_vector / 8000, abs(Force_Y_neg), '--', 'LineWidth', 1.5, 'Color', color_neg);
ylabel('Forward Force (F_y/mg)');
grid on;

subplot(3, 1, 3);
hold on;
plot(time_vector / 8000, abs(Force_Z_pos), '-', 'LineWidth', 1.5, 'Color', color_pos);
plot(time_vector / 8000, abs(Force_Z_neg), '--', 'LineWidth', 1.5, 'Color', color_neg);
xlabel('Time (s)');
ylabel('Vertical Force (F_z/mg)');
grid on;

end

% Overall plot adjustments
sgtitle(['Forces Over Time: Fly: ', num2str(fly_num.Fly_Num)]);



%% New 
% Initialize arrays for storing forces for all flies
All_Fly_Forces_X_Pre = [];
All_Fly_Forces_Y_Pre = [];
All_Fly_Forces_Z_Pre = [];

All_Fly_Forces_X_Post = [];
All_Fly_Forces_Y_Post = [];
All_Fly_Forces_Z_Post = [];

% Define maximum number of periods for consistent padding
max_periods = 0;

% First loop to determine maximum number of periods
for k = 1:length(Fly_Master)
    % Access current fly data
    fly = Fly_Master(k).Fly;
    phi = fly.Kinematics_LH.phi; % Replace with correct angle variable

    % Find each period via peak
    [peaks, peakIndices] = findpeaks(phi);

    % Initialize period indices
    j = 1;
    Period_Index = zeros(1, length(phi)); % Pre-allocate for performance
    Period_Index(1) = 1;
    i = 2;

    while j < length(phi)
        if i > length(peakIndices)
            break;
        end
        for kk = peakIndices(i-1):length(phi)
            if abs(phi(kk) - phi(1)) <= 0.2
                Period_Index(i) = kk;
                j = kk;
                break;
            end
        end
        i = i + 1;
    end
    Period_Index = Period_Index(1:i-1); % Trim to actual indices
    max_periods = max(max_periods, length(Period_Index) - 1);
end

% Second loop to calculate forces and pad arrays
for k = 1:length(Fly_Master)
    % Access current fly data
    fly = Fly_Master(k).Fly;
    phi = fly.Kinematics_LH.phi;
    weight = fly.total.weight; % Fly weight

    % Find each period via peak
    [peaks, peakIndices] = findpeaks(phi);

    % Initialize period indices
    j = 1;
    Period_Index = zeros(1, length(phi)); % Pre-allocate for performance
    Period_Index(1) = 1;
    i = 2;

    while j < length(phi)
        if i > length(peakIndices)
            break;
        end
        for kk = peakIndices(i-1):length(phi)
            if abs(phi(kk) - phi(1)) <= 0.2
                Period_Index(i) = kk;
                j = kk;
                break;
            end
        end
        i = i + 1;
    end
    Period_Index = Period_Index(1:i-1); % Trim to actual indices

    % Calculate mean forces for each period
    num_periods = length(Period_Index) - 1;
    Force_X_mean = zeros(1, max_periods);
    Force_Y_mean = zeros(1, max_periods);
    Force_Z_mean = zeros(1, max_periods);

    for p = 1:num_periods
        start_idx = Period_Index(p);
        end_idx = Period_Index(p + 1) - 1;
        Force_X_mean(p) = (mean(fly.Dynamics.Frame_Body.LH.Dynamics.Force_Total(1, start_idx:end_idx)) + ...
                           mean(fly.Dynamics.Frame_Body.RH.Dynamics.Force_Total(1, start_idx:end_idx))) / weight;
        Force_Y_mean(p) = (mean(fly.Dynamics.Frame_Body.LH.Dynamics.Force_Total(3, start_idx:end_idx)) - ...
                           mean(fly.Dynamics.Frame_Body.RH.Dynamics.Force_Total(3, start_idx:end_idx))) / weight;
        Force_Z_mean(p) = (mean(fly.Dynamics.Frame_Body.LH.Dynamics.Force_Total(2, start_idx:end_idx)) + ...
                           mean(fly.Dynamics.Frame_Body.RH.Dynamics.Force_Total(2, start_idx:end_idx))) / weight;
    end

    % Store forces based on the fly's state
    if Fly_Master(k).State == "Pre Cut"
        All_Fly_Forces_X_Pre = [All_Fly_Forces_X_Pre; Force_X_mean];
        All_Fly_Forces_Y_Pre = [All_Fly_Forces_Y_Pre; Force_Y_mean];
        All_Fly_Forces_Z_Pre = [All_Fly_Forces_Z_Pre; Force_Z_mean];
    elseif Fly_Master(k).State == "Post Cut"
        All_Fly_Forces_X_Post = [All_Fly_Forces_X_Post; Force_X_mean];
        All_Fly_Forces_Y_Post = [All_Fly_Forces_Y_Post; Force_Y_mean];
        All_Fly_Forces_Z_Post = [All_Fly_Forces_Z_Post; Force_Z_mean];
    end
end

% Plot all forces grouped by component with Pre and Post Cut states
figure;

% Plot Force X
subplot(3, 1, 1);
hold on;
h_pre = []; % Handles for Pre Cut
h_post = []; % Handles for Post Cut

for i = 1:size(All_Fly_Forces_X_Pre, 1)
    data = All_Fly_Forces_X_Pre(i, :);
    data(data == 0) = NaN; % Replace zeros with NaN to avoid plotting them
    h_pre = [h_pre, plot(data, 'r', 'LineWidth', 1.5)]; % Pre Cut
end
for i = 1:size(All_Fly_Forces_X_Post, 1)
    data = All_Fly_Forces_X_Post(i, :);
    data(data == 0) = NaN; % Replace zeros with NaN to avoid plotting them
    h_post = [h_post, plot(data, 'r--', 'LineWidth', 1.5)]; % Post Cut
end
xlabel('Period Index');
ylabel('Side Force (F_x/mg)')
legend([h_pre(1), h_post(1)], 'Pre Cut', 'Post Cut'); % Use the first handles for legend
grid on;
hold off;

% Plot Force Y
subplot(3, 1, 2);
hold on;
h_pre = []; % Reset handles for Pre Cut
h_post = []; % Reset handles for Post Cut

for i = 1:size(All_Fly_Forces_Y_Pre, 1)
    data = All_Fly_Forces_Y_Pre(i, :);
    data(data == 0) = NaN; % Replace zeros with NaN to avoid plotting them
    h_pre = [h_pre, plot(data, 'g', 'LineWidth', 1.5)]; % Pre Cut
end
for i = 1:size(All_Fly_Forces_Y_Post, 1)
    data = All_Fly_Forces_Y_Post(i, :);
    data(data == 0) = NaN; % Replace zeros with NaN to avoid plotting them
    h_post = [h_post, plot(data, 'g--', 'LineWidth', 1.5)]; % Post Cut
end
ylabel('Forward Force (F_y/mg)')
grid on;
hold off;

% Plot Force Z
subplot(3, 1, 3);
hold on;
h_pre = []; % Reset handles for Pre Cut
h_post = []; % Reset handles for Post Cut

for i = 1:size(All_Fly_Forces_Z_Pre, 1)
    data = All_Fly_Forces_Z_Pre(i, :);
    data(data == 0) = NaN; % Replace zeros with NaN to avoid plotting them
    h_pre = [h_pre, plot(data, 'b', 'LineWidth', 1.5)]; % Pre Cut
end
for i = 1:size(All_Fly_Forces_Z_Post, 1)
    data = All_Fly_Forces_Z_Post(i, :);
    data(data == 0) = NaN; % Replace zeros with NaN to avoid plotting them
    h_post = [h_post, plot(data, 'b--', 'LineWidth', 1.5)]; % Post Cut
end
ylabel('Vertical Force (F_z/mg)')
grid on;
hold off;

% Overall adjustments
sgtitle('Forces Grouped by Component (Pre Cut vs. Post Cut)');



