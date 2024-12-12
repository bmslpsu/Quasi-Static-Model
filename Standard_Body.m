function [Body_Shape, Joint] = Standard_Body(plot_num)
%% Standard Body Model

%% Load data
load(['Data_Sets' filesep 'Dataset_S1.mat'], 'body_model')

%% Extract data for current dataset
% Rotate body data 90 degrees counterclockwise
Body_Shape.Body_x = -body_model.y_mod; % x' = -y
Body_Shape.Body_y = body_model.x_mod;  % y' = x
Body_Shape.Body_z = body_model.z_mod;

%% Rotate Joint
Joint = [ -body_model.Joint_right(2), body_model.Joint_right(1), body_model.Joint_right(3) ];

%% Plot Body
if plot_num == 0
    return
end

% Determine the number of datasets (assuming each is a line in space)
num_datasets = size(Body_Shape.Body_x, 2); % Assuming data is stored in columns

% Loop through each dataset and plot filled polygons between consecutive lines
for dataset_index = 1:num_datasets
    % Extract data for current dataset
    Body_x_data = Body_Shape.Body_x(:, dataset_index);
    Body_y_data = Body_Shape.Body_y(:, dataset_index);
    Body_z_data = Body_Shape.Body_z(:, dataset_index);

    % Plot the line for the current dataset
    plot3(plot_num, Body_x_data, Body_y_data, Body_z_data, 'k', 'LineWidth', 1);

    % Fill between consecutive lines if not the last dataset
    if dataset_index < num_datasets
        % Extract next dataset
        next_body_x_data = Body_Shape.Body_x(:, dataset_index + 1);
        next_body_y_data = Body_Shape.Body_y(:, dataset_index + 1);
        next_body_z_data = Body_Shape.Body_z(:, dataset_index + 1);

        % Ensure correct concatenation for filling
        fill3(plot_num, ...
              [Body_x_data; flipud(next_body_x_data)], ... % x-coordinates
              [Body_y_data; flipud(next_body_y_data)], ... % y-coordinates
              [Body_z_data; flipud(next_body_z_data)], ... % z-coordinates
              'b', 'FaceAlpha', 0.9, 'EdgeColor', 'none');
    end
end


end
