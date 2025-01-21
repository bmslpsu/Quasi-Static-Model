function [c, n, wing_length, del_r] = Wing_Characteristics(Wing_shape)
    %% Preamble
    % Wing_Characteristics - Calculate the wing lench, chord, step size, 
    %       and Quasi-Steady Steps
    % 
    %
    % Inputs:
    %   Wing_shape  -   WIng cordinates in x,y,z
    %
    % Outputs:
    %   tbd - need to list
    %
    % Frame:
    %   x-axis is along the length of the wing (Root to Tip)
    %   y-axis is perpendicular to the surface of the wing
    %   z-axis is along the chord of the wing starting and is parallel to the abdomen of the fly

    %% Wing Length
    x_positions = Wing_shape.Wing_x(Wing_shape.Wing_tip_index:Wing_shape.Wing_root_index);
    wing_length = abs(x_positions(end) - x_positions(1));

    %% Chord Length
    y_positions_1 = Wing_shape.Wing_y(Wing_shape.Wing_tip_index:Wing_shape.Wing_root_index);
    y_positions_2 = Wing_shape.Wing_y(end:-1:Wing_shape.Wing_root_index);   
    c = abs(y_positions_2 - y_positions_1); % Chord lengths in mm
    %I believe that if the wing is cut, the chord doesn't actually change
    %but this only impacts cut wings

    %% Quasi-Steady Steps
    n = length(x_positions) - 1;

    %% Step size
    del_r = wing_length/n;

end