function Shape = Wing_Shape_Init(Shape,Wing_x_data,Wing_y_data,Wing_z_data)
    % LH wing
    Shape.Wing_x = Wing_x_data;
    Shape.Wing_y = Wing_y_data;
    Shape.Wing_z = Wing_z_data;
    Shape.Wing_root_index = find(Wing_x_data == min(Wing_x_data));
    Shape.Wing_root = [Wing_x_data(Shape.Wing_root_index), Wing_y_data(Shape.Wing_root_index)];
    
    Min_1 = find(Wing_x_data == max(Wing_x_data),1,"first");
    Min_2 = find(Wing_x_data == max(Wing_x_data),1,"last");   
    if Min_1 ~= Min_2
        Min_12 = Min_1;
    else
        Min_12 = round(Min_1 + (Min_2-Min_1)/2);
    end

    Shape.Wing_tip_index = Min_12;
    Shape.Wing_tip = [Wing_x_data(Min_12), Wing_y_data(Min_12)];
end