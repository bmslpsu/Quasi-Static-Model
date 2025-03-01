% Vector Plots

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

meanX_pre = [];
meanY_pre = [];
meanZ_pre = [];
meanX_post = [];
meanY_post = [];
meanZ_post = [];



for k = 1:length(Fly_Master)
    %if Fly_Master(k).Fly_Num == 23 || Fly_Master(k).Fly_Num == 24 || Fly_Master(k).Fly_Num == 5
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
            meanX_pre(end+1) = endX;
            meanY_pre(end+1) = endY;
            meanZ_pre(end+1) = endZ;

        elseif Fly_Master(k).State == "Post Cut"
            % Plot dashed line for "Post Cut" without adding to the legend
            plot3([originX, endX], [originY, endY], [originZ, endZ], ...
                'Color', colors(color_idx, :), 'LineStyle', '--', ...
                'LineWidth', 1.5);
            i = i + 1; % Increment color index
            meanX_post(end+1) = endX;
            meanY_post(end+1) = endY;
            meanZ_post(end+1) = endZ;


            % elseif Fly_Master(k).State == "Steady State"
            %     % Plot dotted line for "Steady State" without adding to the legend
            %     plot3([originX(k), endX], [originY(k), endY], [originZ(k), endZ], ...
            %         'Color', colors(color_idx, :), 'LineStyle', ':', ...
            %         'LineWidth', 1.5);
            %     i = i + 1; % Increment color index
        end
    %end
        
end


        h_pre(end+1) = plot3([originX, mean(meanX_pre)], [originY, mean(meanY_pre)], [originZ, mean(meanZ_pre)], ...
            'Color', 'k', 'LineStyle', '-', ...
            'LineWidth', 1.5);
        % Add legend label for "Pre Cut"
        legend_labels_pre{end+1} = 'mean';

        plot3([originX, mean(meanX_post)], [originY, mean(meanY_post)], [originZ, mean(meanZ_post)], ...
                'Color', colors(color_idx, :), 'LineStyle', '--', ...
                'LineWidth', 1.5);


% Add title and axis labels
xlabel('Sideward (F/mg)');
ylabel('Forward (F/mg)');
zlabel('Upward (F/mg)');
axis equal;
% grid on;
view(3);


% % Set axis limits
% xmax = max(Force_X_mean); % Maximum x value
% zmax = max(Force_Z_mean); % Maximum z value
xlim([0, .5]);
zlim([-.75, 1]);
% %ylim([0, 0.5])


% Add legend for "Pre Cut" vectors
legend(h_pre, legend_labels_pre, 'Location', 'best');


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

meanX_pre = [];
meanY_pre = [];
meanZ_pre = [];
meanX_post = [];
meanY_post = [];
meanZ_post = [];




for k = 1:length(Fly_Master)
    %if Fly_Master(k).Fly_Num == 23 || Fly_Master(k).Fly_Num == 24 || Fly_Master(k).Fly_Num == 5
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
            meanX_pre(end+1) = endX;
            meanY_pre(end+1) = endY;
            meanZ_pre(end+1) = endZ;

        elseif Fly_Master(k).State == "Post Cut"
            % Plot dashed line for "Post Cut" without adding to the legend
            plot3([originX, endX], [originY, endY], [originZ, endZ], ...
                'Color', colors(color_idx, :), 'LineStyle', '--', ...
                'LineWidth', 1.5);
            i = i + 1; % Increment color index

            meanX_post(end+1) = endX;
            meanY_post(end+1) = endY;
            meanZ_post(end+1) = endZ;

            % elseif Fly_Master(k).State == "Steady State"
            %     % Plot dotted line for "Steady State" without adding to the legend
            %     plot3([originX(k), endX], [originY(k), endY], [originZ(k), endZ], ...
            %         'Color', colors(color_idx, :), 'LineStyle', ':', ...
            %         'LineWidth', 1.5);
            %     i = i + 1; % Increment color index
        end
    %end
end

        h_pre(end+1) = plot3([originX, mean(meanX_pre)], [originY, mean(meanY_pre)], [originZ, mean(meanZ_pre)], ...
            'Color', 'k', 'LineStyle', '-', ...
            'LineWidth', 1.5);
        % Add legend label for "Pre Cut"
        legend_labels_pre{end+1} = 'mean';

        plot3([originX, mean(meanX_post)], [originY, mean(meanY_post)], [originZ, mean(meanZ_post)], ...
                'Color', colors(color_idx, :), 'LineStyle', '--', ...
                'LineWidth', 1.5);



% Add title and axis labels
xlabel('Roll (T/mgl)');
ylabel('Pitch (T/mgl)');
zlabel('Yaw (T/mgl)');
%axis equal;
% grid on;
% view([0 0]);
view(3);


% % Set axis limits
% xmax = max(Force_X_mean); % Maximum x value
% zmax = max(Force_Z_mean); % Maximum z value
%xlim([-1, 1]);
%zlim([-5, 5]);
%ylim([0, 2])

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

meanX_pre = [];
meanY_pre = [];
meanZ_pre = [];
meanX_post = [];
meanY_post = [];
meanZ_post = [];

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

                meanX_pre(end+1) = Force_X_mean(k);
                meanY_pre(end+1) = Force_Y_mean(k);
                meanZ_pre(end+1) = Force_Z_mean(k);

                % End is the Post-Cut force components
                endX(end+1) = Force_X_mean(matchIdx);
                endY(end+1) = Force_Y_mean(matchIdx);
                endZ(end+1) = Force_Z_mean(matchIdx);

                meanX_post(end+1) = Force_X_mean(matchIdx);
                meanY_post(end+1) = Force_Y_mean(matchIdx);
                meanZ_post(end+1) = Force_Z_mean(matchIdx);

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
vector_handles(end+1) = plot3([mean(meanX_pre), mean(meanX_post)], [mean(meanY_pre), mean(meanY_post)], [mean(meanZ_pre), mean(meanZ_post)], ...
    'Color', 'k', 'LineWidth', 1.5);
legend_labels{end+1} = 'Mean';

% Plot markers and collect handles for legend
h_pre_cut = scatter3(mean(meanX_pre), mean(meanY_pre), mean(meanZ_pre), 50, 'k', 'filled', 'o', ...
    'DisplayName', 'Pre Cut (Dot)');
h_post_cut = scatter3(mean(meanX_post), mean(meanY_post), mean(meanZ_post), 100, 'k', 'filled', '^', ...
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
%view(3);
%view([0 0]);
%view([90, 0]); % View along the x-axis (yz-plane)
view([0, 90]);

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

meanX_pre = [];
meanY_pre = [];
meanZ_pre = [];
meanX_post = [];
meanY_post = [];
meanZ_post = [];

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

                meanX_pre(end+1) = Moment_Roll_mean(k);
                meanY_pre(end+1) = Moment_Pitch_mean(k);
                meanZ_pre(end+1) = Moment_Yaw_mean(k);

                % End is the Post-Cut torque components
                endX(end+1) = Moment_Roll_mean(matchIdx);
                endY(end+1) = Moment_Pitch_mean(matchIdx);
                endZ(end+1) = Moment_Yaw_mean(matchIdx);

                meanX_post(end+1) = Moment_Roll_mean(matchIdx);
                meanY_post(end+1) = Moment_Pitch_mean(matchIdx);
                meanZ_post(end+1) = Moment_Yaw_mean(matchIdx);

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

vector_handles(end+1) = plot3([mean(meanX_pre), mean(meanX_post)], [mean(meanY_pre), mean(meanY_post)], [mean(meanZ_pre), mean(meanZ_post)], ...
    'Color', 'k', 'LineWidth', 1.5);
legend_labels{end+1} = 'Mean';

% Plot markers and collect handles for legend
h_pre_cut = scatter3(mean(meanX_pre), mean(meanY_pre), mean(meanZ_pre), 50, 'k', 'filled', 'o', ...
    'DisplayName', 'Pre Cut (Dot)');
h_post_cut = scatter3(mean(meanX_post), mean(meanY_post), mean(meanZ_post), 100, 'k', 'filled', '^', ...
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
%view(3);
view([0 0]);
%view([90, 0]); % View along the x-axis (yz-plane)
%view([0, 90]);

% Combine legend handles and labels
legend_handles = [vector_handles, h_pre_cut, h_post_cut];
legend_labels_combined = [legend_labels, "Pre Cut (Dot)", "Post Cut (Triangle)"];

% Add legend
legend(legend_handles, legend_labels_combined, 'Location', 'eastoutside');

