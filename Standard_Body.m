function [Body_Shape] = Standard_Body(plot_num)
%% Standard Body Model

%% Load data

load(['Data_Sets' filesep 'Dataset_S1.mat'], 'body_model')

%% Left Wing
% % Extract data for current dataset
Body_Shape.Body_x = body_model.x_mod;
Body_Shape.Body_y = body_model.y_mod;
Body_Shape.Body_z = body_model.z_mod;

%% Plot Body

if plot_num == 0
    return
end


% Determine the number of datasets (assuming each is a line in space)
num_datasets = size(body_model.x_mod, 2); % Assuming data is stored in columns

% Loop through each dataset and plot filled polygons between consecutive lines
for dataset_index = 1:num_datasets
    % Extract data for current dataset
    Body_x_data = body_model.x_mod(:, dataset_index);
    Body_y_data = body_model.y_mod(:, dataset_index);
    Body_z_data = body_model.z_mod(:, dataset_index);

    % Plot the line for the current dataset
    plot3(plot_num, Body_x_data, Body_y_data, Body_z_data, 'k', 'LineWidth', 1);

    % Fill between consecutive lines if not the last dataset
    if dataset_index < num_datasets
        % Extract next dataset
        next_body_x_data = body_model.x_mod(:, dataset_index + 1);
        next_body_y_data = body_model.y_mod(:, dataset_index + 1);
        next_body_z_data = body_model.z_mod(:, dataset_index + 1);

        % Plot filled polygon between current and next dataset
        fill3(plot_num, [Body_x_data; next_body_x_data], ...
              [Body_y_data; next_body_y_data], ...
              [Body_z_data; next_body_z_data], ...
              'b', 'FaceAlpha', .9, 'EdgeColor', 'none');


    end
end

end
