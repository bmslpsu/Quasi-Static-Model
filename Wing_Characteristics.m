function [c, n, wing_length, del_r] = Wing_Characteristics(Wing_shape)


    %% Wing Chord and postion data
    y_positions = Wing_shape.Wing_y(1:Wing_shape.Wing_tip_index);
    wing_length = abs(y_positions(end) - y_positions(1));


    x_positions_1 = Wing_shape.Wing_x(1:Wing_shape.Wing_tip_index);
    x_positions_2 = Wing_shape.Wing_x(end:-1:Wing_shape.Wing_tip_index);
    

    c = abs(x_positions_2 - x_positions_1); % Chord lengths in meters
    %I believe that if the wing is cut, the chord doesn't actually change
    %but this only impacts cut wings



    %% Array Intialization
    n = length(y_positions) - 1;

    %% Step size
    del_r = wing_length/n;

end