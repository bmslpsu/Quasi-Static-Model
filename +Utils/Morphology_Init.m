function Morphology = Morphology_Init(Wing_Shape_lh, Wing_Shape_rh, Fly_Data)
    import Utils.Standard_Wing
    import Utils.Standard_Body
    import Utils.mass_and_inertia
    import Utils.Wing_Shape_Init

    % Init the structure
    Morphology = struct();

    % Load default body
    [Morphology.Body.Body_shape, Morphology.Body.Joint] = Standard_Body(0);

    % Construct wings
    [~, ~, ~, ~, ~, ~, lhWingLength_Max, lhChordLength_Max,...
        rhWingLength_Max, rhChordLength_Max] ...
        = Standard_Wing(0, 0, 100, 100, 0, 0, 100, 100);

    [Wing_left_x_data, Wing_left_y_data, Wing_left_z_data,...
        Wing_right_x_data, Wing_right_y_data, Wing_right_z_data]...
        = Standard_Wing(lhWingLength_Max, lhChordLength_Max,...
        Fly_Data.Span_Cut_LH, Fly_Data.Chord_Cut_LH, rhWingLength_Max, ...
        rhChordLength_Max, Fly_Data.Span_Cut_RH, Fly_Data.Chord_Cut_RH);

    Morphology.Wing_LH.wing_shape = Wing_Shape_Init(Wing_Shape_lh,Wing_left_x_data,Wing_left_y_data,Wing_left_z_data);
    Morphology.Wing_RH.wing_shape = Wing_Shape_Init(Wing_Shape_rh,Wing_right_x_data,Wing_right_y_data,Wing_right_z_data);

    % Add mass and inertia
    Morphology = mass_and_inertia(...
        Morphology.Wing_LH.wing_shape,Morphology.Wing_RH.wing_shape,...
        Morphology.Body.Body_shape,Morphology);

    % legacy feature: CG is considered the joint for robotic flies
    Morphology.total.CG = Morphology.Body.Joint;

    % Extract info from Fly_Data
    Morphology.Body.Body_angle          = Fly_Data.Body_Angle;
    Morphology.Wing_LH.Wing_Plane_angle = Fly_Data.Wing_Plane_Angle_LH;
    Morphology.Wing_RH.Wing_Plane_angle = Fly_Data.Wing_Plane_Angle_RH;

end