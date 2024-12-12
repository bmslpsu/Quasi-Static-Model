function [c, n, wing_length, del_r] = Wing_Characteristics(Wing_shape)


    %% Wing Chord and postion data
    x_positions = Wing_shape.Wing_x(Wing_shape.Wing_tip_index:Wing_shape.Wing_root_index);
    wing_length = abs(x_positions(end) - x_positions(1));


    y_positions_1 = Wing_shape.Wing_y(Wing_shape.Wing_tip_index:Wing_shape.Wing_root_index);
    y_positions_2 = Wing_shape.Wing_y(end:-1:Wing_shape.Wing_root_index);
    

    c = abs(y_positions_2 - y_positions_1); % Chord lengths in meters
    %I believe that if the wing is cut, the chord doesn't actually change
    %but this only impacts cut wings



    %% Array Intialization
    n = length(x_positions) - 1;

    %% Step size
    del_r = wing_length/n;

end