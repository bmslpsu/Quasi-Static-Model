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

% % Save the figure to the Forces folder
% figureFilePath = fullfile(forcesFolderPath, ['Normalized_Forces_Flys.png']);
% saveas(gcf, figureFilePath);
% 
% % Save the figure to the Forces folder
% figureFilePath = fullfile(forcesFolderPath, ['Normalized_Forces_Flys.fig']);
% saveas(gcf, figureFilePath);

% % Close the figure after saving
% close(gcf);

%% Mean vector plot - Force
% Define the origin for each vector (set to [0, 0, 0])
originX = 0; % X-coordinates of the origin
originY = 0; % Y-coordinates of the origin
originZ = 0; % Z-coordinates of the origin

% Custom distinct colors (manually chosen for high contrast)
colors = [...
    0.85, 0.33, 0.10; % Red-orange
    0.47, 0.67, 0.19; % Green
    0.30, 0.75, 0.93; % Light blue
    0.93, 0.69, 0.13; % Yellow-orange
    0.64, 0.08, 0.18; % Maroon
    0.49, 0.18, 0.56; % Purple
    0.00, 0.45, 0.74; % Blue
    0.25, 0.25, 0.25; % Gray
    0.94, 0.39, 0.39; % Salmon
    0.10, 0.60, 0.40]; % Teal

num_colors = size(colors, 1); % Number of available colors

% Initialize handles and legend labels for "Pre Cut" only
h_pre = []; % Handles for "Pre Cut"
legend_labels_pre = {}; % Labels for "Pre Cut"

% 3D Vector Plot
figure;
hold on;
i = 1; % Color index



for k = 1:length(Fly_Master)
    if Fly_Master(k).Fly_Num == 23 || Fly_Master(k).Fly_Num == 24 || Fly_Master(k).Fly_Num == 5
        % Ensure the color index wraps around if there are more than num_colors flies
        color_idx = mod(i-1, num_colors) + 1; % Cycles through 1 to num_colors

        % Calculate the endpoint of the vector
        endX = Force_X_mean(k);
        endY = Force_Y_mean(k);
        endZ = Force_Z_mean(k);

        if Fly_Master(k).State == "Pre Cut"
            % Plot solid line for "Pre Cut"
            h_pre(end+1) = plot3([originX, endX], [originY, endY], [originZ, endZ], ...
                'Color', colors(color_idx, :), 'LineStyle', '-', ...
                'LineWidth', 1.5);
            % Add legend label for "Pre Cut"
            legend_labels_pre{end+1} = ['Fly ' num2str(Fly_Master(k).Fly_Num)];

        elseif Fly_Master(k).State == "Post Cut"
            % Plot dashed line for "Post Cut" without adding to the legend
            plot3([originX, endX], [originY, endY], [originZ, endZ], ...
                'Color', colors(color_idx, :), 'LineStyle', '--', ...
                'LineWidth', 1.5);
            i = i + 1; % Increment color index

            % elseif Fly_Master(k).State == "Steady State"
            %     % Plot dotted line for "Steady State" without adding to the legend
            %     plot3([originX(k), endX], [originY(k), endY], [originZ(k), endZ], ...
            %         'Color', colors(color_idx, :), 'LineStyle', ':', ...
            %         'LineWidth', 1.5);
            %     i = i + 1; % Increment color index
        end
    end
end


% Add title and axis labels
xlabel('Sideward (F/mg)');
ylabel('Forward (F/mg)');
zlabel('Upward (F/mg)');
axis equal;
% grid on;
view([0 0]);


% % Set axis limits
% xmax = max(Force_X_mean); % Maximum x value
% zmax = max(Force_Z_mean); % Maximum z value
xlim([0, .5]);
zlim([-.75, 1]);
% %ylim([0, 0.5])


% Add legend for "Pre Cut" vectors
if ~isempty(h_pre) % Only add legend if there are vectors
    legend(h_pre, legend_labels_pre, 'Location', 'best');
else
    warning('No vectors were plotted, so no legend is displayed.');
end

%% Mean vector plot - Torque
% Define the origin for each vector (set to [0, 0, 0])
originX = 0; % X-coordinates of the origin
originY = 0; % Y-coordinates of the origin
originZ = 0; % Z-coordinates of the origin

% Custom distinct colors (manually chosen for high contrast)
colors = [...
    0.85, 0.33, 0.10; % Red-orange
    0.47, 0.67, 0.19; % Green
    0.30, 0.75, 0.93; % Light blue
    0.93, 0.69, 0.13; % Yellow-orange
    0.64, 0.08, 0.18; % Maroon
    0.49, 0.18, 0.56; % Purple
    0.00, 0.45, 0.74; % Blue
    0.25, 0.25, 0.25; % Gray
    0.94, 0.39, 0.39; % Salmon
    0.10, 0.60, 0.40]; % Teal

num_colors = size(colors, 1); % Number of available colors

% Initialize handles and legend labels for "Pre Cut" only
h_pre = []; % Handles for "Pre Cut"
legend_labels_pre = {}; % Labels for "Pre Cut"

% 3D Vector Plot
figure;
hold on;
i = 1; % Color index



for k = 1:length(Fly_Master)
    if Fly_Master(k).Fly_Num == 23 || Fly_Master(k).Fly_Num == 24 || Fly_Master(k).Fly_Num == 5
        % Ensure the color index wraps around if there are more than num_colors flies
        color_idx = mod(i-1, num_colors) + 1; % Cycles through 1 to num_colors

        % Calculate the endpoint of the vector
        endX = Moment_Roll_mean(k);
        endY = Moment_Pitch_mean(k);
        endZ = Moment_Yaw_mean(k);

        if Fly_Master(k).State == "Pre Cut"
            % Plot solid line for "Pre Cut"
            h_pre(end+1) = plot3([originX, endX], [originY, endY], [originZ, endZ], ...
                'Color', colors(color_idx, :), 'LineStyle', '-', ...
                'LineWidth', 1.5);
            % Add legend label for "Pre Cut"
            legend_labels_pre{end+1} = ['Fly ' num2str(Fly_Master(k).Fly_Num)];

        elseif Fly_Master(k).State == "Post Cut"
            % Plot dashed line for "Post Cut" without adding to the legend
            plot3([originX, endX], [originY, endY], [originZ, endZ], ...
                'Color', colors(color_idx, :), 'LineStyle', '--', ...
                'LineWidth', 1.5);
            i = i + 1; % Increment color index

            % elseif Fly_Master(k).State == "Steady State"
            %     % Plot dotted line for "Steady State" without adding to the legend
            %     plot3([originX(k), endX], [originY(k), endY], [originZ(k), endZ], ...
            %         'Color', colors(color_idx, :), 'LineStyle', ':', ...
            %         'LineWidth', 1.5);
            %     i = i + 1; % Increment color index
        end
    end
end


% Add title and axis labels
xlabel('Roll (T/mgl)');
ylabel('Pitch (T/mgl)');
zlabel('Yaw (T/mgl)');
axis equal;
% grid on;
% view([0 0]);
view([90, 0]); % View along the x-axis (yz-plane)


% % Set axis limits
% xmax = max(Force_X_mean); % Maximum x value
% zmax = max(Force_Z_mean); % Maximum z value
%xlim([-1, 1]);
zlim([-1.25, 1]);
ylim([0, 2])

title ("Torque Vecotr before and after (- -) damage" )

% Add legend for "Pre Cut" vectors
if ~isempty(h_pre) % Only add legend if there are vectors
    legend(h_pre, legend_labels_pre, 'Location', 'best');
else
    warning('No vectors were plotted, so no legend is displayed.');
end

%% Mean vector plot (Non-orgin) - Force
% Custom distinct colors (manually chosen for high contrast)
colors = [... 
    0.85, 0.33, 0.10; % Red-orange
    0.47, 0.67, 0.19; % Green
    0.30, 0.75, 0.93; % Light blue
    0.93, 0.69, 0.13; % Yellow-orange
    0.64, 0.08, 0.18; % Maroon
    0.49, 0.18, 0.56; % Purple
    0.00, 0.45, 0.74; % Blue
    0.25, 0.25, 0.25; % Gray
    0.94, 0.39, 0.39; % Salmon
    0.10, 0.60, 0.40]; % Teal

num_colors = size(colors, 1); % Number of available colors

% Initialize variables for origin and vector endpoints
originX = [];
originY = [];
originZ = [];
endX = [];
endY = [];
endZ = [];
vector_handles = [];
legend_labels = {};

% Loop through the dataset and calculate vectors
for k = 1:length(Fly_Master)
    % if Fly_Master(k).Fly_Num == 23 || Fly_Master(k).Fly_Num == 24 || Fly_Master(k).Fly_Num == 5
        % Find the matching Pre-Cut and Post-Cut states for the same Fly_Num
        if Fly_Master(k).State == "Pre Cut"
            % Find the corresponding Post-Cut state for the same Fly_Num
            matchIdx = find([Fly_Master.Fly_Num] == Fly_Master(k).Fly_Num & ...
                            strcmp({Fly_Master.State}, "Post Cut"), 1);
            if ~isempty(matchIdx)
                % Origin is the Pre-Cut force components
                originX(end+1) = Force_X_mean(k);
                originY(end+1) = Force_Y_mean(k);
                originZ(end+1) = Force_Z_mean(k);

                % End is the Post-Cut force components
                endX(end+1) = Force_X_mean(matchIdx);
                endY(end+1) = Force_Y_mean(matchIdx);
                endZ(end+1) = Force_Z_mean(matchIdx);

                % Save legend label for this fly
                legend_labels{end+1} = ['Fly ' num2str(Fly_Master(k).Fly_Num)];
            end
        end
    % end
end

% 3D Vector Plot
figure;
hold on;

% Plot vectors
for i = 1:length(originX)
    % Ensure the color index wraps around
    color_idx = mod(i-1, num_colors) + 1;

    % Plot vector from Pre-Cut to Post-Cut
    vector_handles(end+1) = plot3([originX(i), endX(i)], [originY(i), endY(i)], [originZ(i), endZ(i)], ...
                                   'Color', colors(color_idx, :), 'LineWidth', 1.5);
end

% Plot markers and collect handles for legend
h_pre_cut = scatter3(originX(1), originY(1), originZ(1), 50, 'k', 'filled', 'o', ...
                     'DisplayName', 'Pre Cut (Dot)');
h_post_cut = scatter3(endX(1), endY(1), endZ(1), 100, 'k', 'filled', '^', ...
                      'DisplayName', 'Post Cut (Triangle)');

% Plot markers for individual vectors
for i = 1:length(originX)
    % Ensure the color index wraps around
    color_idx = mod(i-1, num_colors) + 1;

    % Plot solid dot at the start (Pre-Cut)
    scatter3(originX(i), originY(i), originZ(i), 50, colors(color_idx, :), 'filled', 'o');

    % Plot solid triangle at the end (Post-Cut)
    scatter3(endX(i), endY(i), endZ(i), 100, colors(color_idx, :), 'filled', '^');
end

% Add title and axis labels
xlabel('Sideward (F/mg)');
ylabel('Forward (F/mg)');
zlabel('Upward (F/mg)');
grid on;

% Adjust view and axis limits
view([0 0]);
%view([90, 0]); % View along the x-axis (yz-plane)

% Combine legend handles and labels
legend_handles = [vector_handles, h_pre_cut, h_post_cut];
legend_labels_combined = [legend_labels, "Pre Cut (Dot)", "Post Cut (Triangle)"];

% Add legend
legend(legend_handles, legend_labels_combined, 'Location', 'eastoutside');

%% Mean vector plot (Non-orgin) - Torque
% Custom distinct colors (manually chosen for high contrast)
colors = [... 
    0.85, 0.33, 0.10; % Red-orange
    0.47, 0.67, 0.19; % Green
    0.30, 0.75, 0.93; % Light blue
    0.93, 0.69, 0.13; % Yellow-orange
    0.64, 0.08, 0.18; % Maroon
    0.49, 0.18, 0.56; % Purple
    0.00, 0.45, 0.74; % Blue
    0.25, 0.25, 0.25; % Gray
    0.94, 0.39, 0.39; % Salmon
    0.10, 0.60, 0.40]; % Teal

num_colors = size(colors, 1); % Number of available colors

% Initialize variables for origin and vector endpoints
originX = [];
originY = [];
originZ = [];
endX = [];
endY = [];
endZ = [];
vector_handles = [];
legend_labels = {};

% Loop through the dataset and calculate vectors
for k = 1:length(Fly_Master)
    % if Fly_Master(k).Fly_Num == 23 || Fly_Master(k).Fly_Num == 24 || Fly_Master(k).Fly_Num == 5
        % Find the matching Pre-Cut and Post-Cut states for the same Fly_Num
        if Fly_Master(k).State == "Pre Cut"
            % Find the corresponding Post-Cut state for the same Fly_Num
            matchIdx = find([Fly_Master.Fly_Num] == Fly_Master(k).Fly_Num & ...
                            strcmp({Fly_Master.State}, "Post Cut"), 1);
            if ~isempty(matchIdx)
                % Origin is the Pre-Cut torque components
                originX(end+1) = Moment_Roll_mean(k);
                originY(end+1) = Moment_Pitch_mean(k);
                originZ(end+1) = Moment_Yaw_mean(k);

                % End is the Post-Cut torque components
                endX(end+1) = Moment_Roll_mean(matchIdx);
                endY(end+1) = Moment_Pitch_mean(matchIdx);
                endZ(end+1) = Moment_Yaw_mean(matchIdx);

                % Save legend label for this fly
                legend_labels{end+1} = ['Fly ' num2str(Fly_Master(k).Fly_Num)];
            end
        end
    % end
end

% 3D Vector Plot
figure;
hold on;

% Plot vectors
for i = 1:length(originX)
    % Ensure the color index wraps around
    color_idx = mod(i-1, num_colors) + 1;

    % Plot vector from Pre-Cut to Post-Cut
    vector_handles(end+1) = plot3([originX(i), endX(i)], [originY(i), endY(i)], [originZ(i), endZ(i)], ...
                                   'Color', colors(color_idx, :), 'LineWidth', 1.5);
end

% Plot markers and collect handles for legend
h_pre_cut = scatter3(originX(1), originY(1), originZ(1), 50, 'k', 'filled', 'o', ...
                     'DisplayName', 'Pre Cut (Dot)');
h_post_cut = scatter3(endX(1), endY(1), endZ(1), 100, 'k', 'filled', '^', ...
                      'DisplayName', 'Post Cut (Triangle)');

% Plot markers for individual vectors
for i = 1:length(originX)
    % Ensure the color index wraps around
    color_idx = mod(i-1, num_colors) + 1;

    % Plot solid dot at the start (Pre-Cut)
    scatter3(originX(i), originY(i), originZ(i), 50, colors(color_idx, :), 'filled', 'o');

    % Plot solid triangle at the end (Post-Cut)
    scatter3(endX(i), endY(i), endZ(i), 100, colors(color_idx, :), 'filled', '^');
end

% Add title and axis labels
xlabel('Roll (T/mgl)');
ylabel('Pitch (T/mgl)');
zlabel('Yaw (T/mgl)');
grid on;

% Adjust view and axis limits
view([0 0]);
%view([90, 0]); % View along the x-axis (yz-plane)

% Combine legend handles and labels
legend_handles = [vector_handles, h_pre_cut, h_post_cut];
legend_labels_combined = [legend_labels, "Pre Cut (Dot)", "Post Cut (Triangle)"];

% Add legend
legend(legend_handles, legend_labels_combined, 'Location', 'eastoutside');

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
    fly_nums = Fly_Master(1, fly_numbers(fly_idx));
    phi = fly_nums.Fly.Kinematics_LH.phi; % Stroke angle (phi)

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
        Force_X_mean(p) = (mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Dynamics.Force_Total(1, start_idx:end_idx)) + ...
                           mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Dynamics.Force_Total(1, start_idx:end_idx))) / fly_nums.Fly.total.weight;

        Force_Y_mean(p) = (mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Dynamics.Force_Total(3, start_idx:end_idx)) - ...
                           mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Dynamics.Force_Total(3, start_idx:end_idx))) / fly_nums.Fly.total.weight;

        Force_Z_mean(p) = (mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Dynamics.Force_Total(2, start_idx:end_idx)) + ...
                           mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Dynamics.Force_Total(2, start_idx:end_idx))) / fly_nums.Fly.total.weight;
    end

    % Define time intervals for each period
    period_times = fly_nums.Fly.time(Period_Index);

    % Interpolate mean forces over the entire time vector
    time_vector = fly_nums.Fly.time;
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

if strcmp(fly_nums.State, 'Pre Cut') % Check if it's pre-cut
    color_pos = 'b'; % Blue for positive
    color_neg = 'b'; % Blue for negative
else
    color_pos = 'r'; % Orange (use 'r' for red; MATLAB doesn't have orange) for positive
    color_neg = 'r'; % Orange for negative
end
    
    
    % Plot forces over time
subplot(3, 1, 1);
hold on;

plot(time_vector / 8000, abs(Force_X_pos), '-', 'LineWidth', 1.5, 'Color', color_pos, 'DisplayName', [fly_nums.State, ' Positive']);
plot(time_vector / 8000, abs(Force_X_neg), '--', 'LineWidth', 1.5, 'Color', color_neg, 'DisplayName', [fly_nums.State, ' Negative']);
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
sgtitle(['Forces Over Time: Fly: ', num2str(fly_nums.Fly_Num)]);



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



