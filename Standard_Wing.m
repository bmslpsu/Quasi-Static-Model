function [Wing_left_x_data, Wing_left_y_data, Wing_left_z_data, Wing_right_x_data, Wing_right_y_data, Wing_right_z_data, lhWingLength_Initital, lhChordLength_Initital, rhWingLength_Initital, rhChordLength_Initital] = Standard_Wing(lhWingLength, lhChordLength, lhSpanwiseCut, lhChordwiseCut, rhWingLength, rhChordLength, rhSpanwiseCut, rhChordwiseCut)
%% Standard Wing Model

%% Load data

load(['Data_Sets' filesep 'Dataset_S1.mat'], 'wing_model')

%% Left Wing
% Extract data for current dataset
Wing_left_x_data = wing_model.x_mod_R(:, 1);
Wing_left_y_data = wing_model.y_mod_R(:, 1);
Wing_left_z_data = wing_model.z_mod_R(:, 1);

% Apply the 90-degree counterclockwise rotation to left wing
Wing_left_x_data_rot = -Wing_left_y_data;
Wing_left_y_data_rot = Wing_left_x_data;
Wing_left_x_data = Wing_left_x_data_rot;
Wing_left_y_data = Wing_left_y_data_rot;

% Rotate to calculate as a right wing
Wing_left_x_data = -Wing_left_x_data;

[Wing_left_x_data, Wing_left_y_data, Wing_left_z_data, lhChordLength_Initital, lhWingLength_Initital] = wing_mod(Wing_left_x_data, Wing_left_y_data, Wing_left_z_data, lhWingLength, lhChordLength, lhSpanwiseCut, lhChordwiseCut);


%% Right Wing
% Extract data for current dataset
Wing_right_x_data = wing_model.x_mod_L(:, 1);
Wing_right_y_data = wing_model.y_mod_L(:, 1);
Wing_right_z_data = wing_model.z_mod_L(:, 1);

% Apply the 90-degree counterclockwise rotation to right wing
Wing_right_x_data_rot = -Wing_right_y_data;
Wing_right_y_data_rot = Wing_right_x_data;
Wing_right_x_data = Wing_right_x_data_rot;
Wing_right_y_data = Wing_right_y_data_rot;

[Wing_right_x_data, Wing_right_y_data, Wing_right_z_data, rhChordLength_Initital, rhWingLength_Initital] = wing_mod(Wing_right_x_data, Wing_right_y_data, Wing_right_z_data, rhWingLength, rhChordLength, rhSpanwiseCut, rhChordwiseCut);
end

function [Wing_x_data, Wing_y_data, Wing_z_data, ChordLength_Initital, WingLength_Initital] = wing_mod(Wing_x_data, Wing_y_data, Wing_z_data, WingLength, ChordLength, SpanwiseCut, ChordwiseCut)

% Identify the midpoint (chord line) to split into top and bottom
[~, x_max_index] = max(Wing_x_data);  % Assuming the minimum x value is the leading edge

% Split the coordinates into top and bottom
top_x = Wing_x_data(x_max_index:end);
top_y = Wing_y_data(x_max_index:end);
top_z = Wing_z_data(x_max_index:end);
bottom_x = Wing_x_data(1:x_max_index);
bottom_y = Wing_y_data(1:x_max_index);
bottom_z = Wing_z_data(1:x_max_index);

% Define the original parameter t for interpolation
t_top = linspace(0, 1, length(top_x));
t_bottom = linspace(0, 1, length(bottom_x));
t_new = linspace(0, 1, 21); % 20 elements

% Interpolate top and bottom parts separately
interp_top_x = interp1(t_top, top_x, t_new, 'cubic');
interp_top_y = interp1(t_top, top_y, t_new, 'cubic');
interp_top_z = interp1(t_top, top_z, t_new, 'cubic');
interp_bottom_x = interp1(t_bottom, bottom_x, t_new, 'cubic');
interp_bottom_y = interp1(t_bottom, bottom_y, t_new, 'cubic');
interp_bottom_z = interp1(t_bottom, bottom_z, t_new, 'cubic');

% Combine the top and bottom parts
Wing_x_data = [interp_top_x, interp_bottom_x(2:end)];
Wing_y_data = [interp_top_y, interp_bottom_y(2:end)];
Wing_z_data = [interp_top_z, interp_bottom_z(2:end)];

% Scaling and bounds
x_min = min(Wing_x_data);
y_min = min(Wing_y_data);
z_min = min(Wing_z_data);
x_max = max(Wing_x_data);
y_max = max(Wing_y_data);
z_max = max(Wing_z_data);

ChordLength_Initital = round(y_max - y_min, 2);  % Adjusted for new orientation
WingLength_Initital = round(x_max - x_min, 2);  % Adjusted for new orientation

% Chord Scaling
Chord_Scale = ChordLength / (y_max - y_min);  % Adjusted for new orientation
Wing_y_data = Wing_y_data * Chord_Scale;  % Apply scale to y (chord direction)
y_min = min(Wing_y_data);
y_max = max(Wing_y_data);

% Wing Length Scaling
Span_Scale = WingLength / (x_max - x_min);  % Adjusted for new orientation
Wing_x_data = Wing_x_data * Span_Scale;  % Apply scale to x (span direction)
x_min = min(Wing_x_data);
x_max = max(Wing_x_data);

% Chordwise Cut
Cut_spot = y_max - (y_max - y_min) * ChordwiseCut / 100;  % Adjusted for new orientation
Wing_y_data(Wing_y_data < Cut_spot) = Cut_spot;

% Spanwise Cut
Cut_spot = x_min + (x_max - x_min) * SpanwiseCut / 100;  % Adjusted for new orientation
indexToKeep = Wing_x_data <= Cut_spot * 1.0000000000001;

% Filter x, y, and z arrays using the logical index
Wing_x_data = Wing_x_data(indexToKeep);
Wing_y_data = Wing_y_data(indexToKeep);
Wing_z_data = Wing_z_data(indexToKeep);

% Remove the middle index if the total number of points is even
if mod(length(Wing_x_data), 2) == 0
    middleIndex = floor(length(Wing_x_data) / 2);
    Wing_x_data(middleIndex) = [];
    Wing_y_data(middleIndex) = [];
    Wing_z_data(middleIndex) = [];
end
end

