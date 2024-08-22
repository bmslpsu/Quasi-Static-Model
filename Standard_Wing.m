function [Wing_left_x_data, Wing_left_y_data, Wing_left_z_data, Wing_right_x_data, Wing_right_y_data, Wing_right_z_data, lhWingLength_Initital, lhChordLength_Initital, rhWingLength_Initital, rhChordLength_Initital] = Standard_Wing(lhWingLength, lhChordLength, lhSpanwiseCut, lhChordwiseCut, rhWingLength, rhChordLength, rhSpanwiseCut, rhChordwiseCut)
%% Standard Wing Model

%% Load data

load('Data_Sets\Dataset_S1.mat', 'wing_model')

%% Left Wing
% Extract data for current dataset
Wing_left_x_data = wing_model.x_mod_R(:, 1);
Wing_left_y_data = wing_model.y_mod_R(:, 1);
Wing_left_z_data = wing_model.z_mod_R(:, 1);

% Identify the midpoint (chord line) to split into top and bottom
[~ , y_max_index] = max(Wing_left_y_data);  % Assuming the max x value is the leading edge

% Split the coordinates into top and bottom
top_x = Wing_left_x_data(1:y_max_index);
top_y = Wing_left_y_data(1:y_max_index);
top_z = Wing_left_z_data(1:y_max_index);
bottom_x = Wing_left_x_data(y_max_index:end);
bottom_y = Wing_left_y_data(y_max_index:end);
bottom_z = Wing_left_z_data(y_max_index:end);

% Define the original parameter t for interpolation
t_top = linspace(0, 1, length(top_x));
t_bottom = linspace(0, 1, length(bottom_x));
t_new = linspace(0, 1, 21); % 101 points for top and bottom

% Interpolate top and bottom parts separately
interp_top_y = interp1(t_top, top_y, t_new, 'cubic');
interp_top_x = interp1(top_y, top_x, interp_top_y, 'cubic');
interp_top_z = interp1(t_top, top_z, t_new, 'cubic');
interp_bottom_y = interp1(t_bottom, bottom_y, t_new, 'cubic');
interp_bottom_x = interp1(bottom_y, bottom_x, interp_bottom_y, 'cubic');
interp_bottom_z = interp1(t_bottom, bottom_z, t_new, 'cubic');

% Combine the top and bottom parts
Wing_left_x_data = [interp_top_x, interp_bottom_x(2:end)];
Wing_left_y_data = [interp_top_y, interp_bottom_y(2:end)];
Wing_left_z_data = [interp_top_z, interp_bottom_z(2:end)];

x_min = min(Wing_left_x_data);
y_min = min(Wing_left_y_data);
z_min = min(Wing_left_z_data);
x_max = max(Wing_left_x_data);
y_max = max(Wing_left_y_data);
z_max = max(Wing_left_z_data);

lhChordLength_Initital = round(x_max-x_min,2);
lhWingLength_Initital = round(y_max-y_min,2);

%Chord Scaling
Chord_Scale = lhChordLength/(x_max-x_min);
Wing_left_x_data = Wing_left_x_data*Chord_Scale;
x_min = min(Wing_left_x_data);
x_max = max(Wing_left_x_data);

%Wing Length Scaling
Span_Scale = lhWingLength/(y_max-y_min);
Wing_left_y_data = Wing_left_y_data*Span_Scale;
y_min = min(Wing_left_y_data);
y_max = max(Wing_left_y_data);

%Chordwise Cut
Cut_spot = x_max-(x_max-x_min)*lhChordwiseCut/100;

for i=1:length(Wing_left_x_data)
    Wing_left_x_data(i);
    if Cut_spot>Wing_left_x_data(i)
        Wing_left_x_data(i) = Cut_spot;
    end
end

%Spanwise Cut
Cut_spot = y_min+(y_max-y_min)*lhSpanwiseCut/100;


indexToKeep = Wing_left_y_data <= Cut_spot*1.00000001;

% Filter x, y, and z arrays using the logical index
Wing_left_x_data = Wing_left_x_data(indexToKeep);
Wing_left_y_data = Wing_left_y_data(indexToKeep);
Wing_left_z_data = Wing_left_z_data(indexToKeep);

%% Right Wing
% Extract data for current dataset
Wing_right_x_data = wing_model.x_mod_L(:, 1);
Wing_right_y_data = wing_model.y_mod_L(:, 1);
Wing_right_z_data = wing_model.z_mod_L(:, 1);

% Identify the midpoint (chord line) to split into top and bottom
[~ , y_max_index] = min(Wing_right_y_data);  % Assuming the min y value is the leading edge

% Split the coordinates into top and bottom
top_x = Wing_right_x_data(1:y_max_index);
top_y = Wing_right_y_data(1:y_max_index);
top_z = Wing_right_z_data(1:y_max_index);
bottom_x = Wing_right_x_data(y_max_index:end);
bottom_y = Wing_right_y_data(y_max_index:end);
bottom_z = Wing_right_z_data(y_max_index:end);

% Define the original parameter t for interpolation
t_top = linspace(0, 1, length(top_x));
t_bottom = linspace(0, 1, length(bottom_x));
t_new = linspace(0, 1, 21); % 101 points for top and bottom

% Interpolate top and bottom parts separately
interp_top_y = interp1(t_top, top_y, t_new, 'cubic');
interp_top_x = interp1(top_y, top_x, interp_top_y, 'cubic');
interp_top_z = interp1(t_top, top_z, t_new, 'cubic');
interp_bottom_y = interp1(t_bottom, bottom_y, t_new, 'cubic');
interp_bottom_x = interp1(bottom_y, bottom_x, interp_bottom_y, 'cubic');
interp_bottom_z = interp1(t_bottom, bottom_z, t_new, 'cubic');



% Combine the top and bottom parts
Wing_right_x_data = [interp_top_x, interp_bottom_x(2:end)];
Wing_right_y_data = [interp_top_y, interp_bottom_y(2:end)];
Wing_right_z_data = [interp_top_z, interp_bottom_z(2:end)];

x_min = min(Wing_right_x_data);
y_min = min(Wing_right_y_data);
z_min = min(Wing_right_z_data);
x_max = max(Wing_right_x_data);
y_max = max(Wing_right_y_data);
z_max = max(Wing_right_z_data);

rhChordLength_Initital = round(x_max-x_min,2);
rhWingLength_Initital = round(y_max-y_min,2);

%Chord Scaling
Chord_Scale = rhChordLength/(x_max-x_min);%Chord Scaling
Wing_right_x_data = Wing_right_x_data*Chord_Scale;
x_min = min(Wing_right_x_data);
x_max = max(Wing_right_x_data);

%Wing Length Scaling
Span_Scale = rhWingLength/(y_max-y_min);
Wing_right_y_data = Wing_right_y_data*Span_Scale;
y_min = min(Wing_right_y_data);
y_max = max(Wing_right_y_data);

%Chordwise Cut
Cut_spot = x_max-(x_max-x_min)*rhChordwiseCut/100;

for i=1:length(Wing_right_x_data)
    Wing_right_x_data(i);
    if Cut_spot>Wing_right_x_data(i)
        Wing_right_x_data(i) = Cut_spot;
    end
end

%Spanwise Cut
Cut_spot = y_max-(y_max-y_min)*rhSpanwiseCut/100;

indexToKeep = Wing_right_y_data >= Cut_spot*1.00000001;

% Filter x, y, and z arrays using the logical index
Wing_right_x_data = Wing_right_x_data(indexToKeep);
Wing_right_y_data = Wing_right_y_data(indexToKeep);
Wing_right_z_data = Wing_right_z_data(indexToKeep);


end
