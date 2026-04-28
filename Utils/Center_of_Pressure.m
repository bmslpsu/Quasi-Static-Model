function element = Center_of_Pressure(Kinematics, Morphology)
    %% Preamble
    % Calculate the center of pressure (COP) location for each wing element
    % in a flapping wing model.
    %
    % Inputs:
    %   Kinematics -    Structure containing wing kinematic properties
    %   Morphology -    Structure containing morphological properties
    %
    % Outputs:
    %   element - Struct array containing:
    %             - COP_Distance:    Distance of COP from the wing root
    %             - COP_Wing_Frame:  COP location in the body frame
    %             - COP_Body_Frame:  COP location in the wing frame
    %
    % Frame (Wing):
    %   x-axis is along the length of the wing (Root to Tip)
    %   y-axis is perpendicular to the surface of the wing
    %   z-axis is along the chord of the wing and is parallel to the abdomen of the fly

    %% 1. Precomputations  

    % Required Kinematic Parameters
    psi     = Kinematics.psi;
    R_wb    = Kinematics.R_wb;
    N       = Kinematics.N;

    % Required Morphology Parameters
    N_elements  = Morphology.n;
    c           = Morphology.c;
    wing_length = Morphology.wing_length;
    wing_shape  = Morphology.wing_shape;

    % Preallocate arrays
    COP_Wing_Frame  = zeros(3, N);      % COP vector in moving frame
    COP_Body_Frame  = zeros(3, N);      % COP vector in stationary frame

   
    delz = wing_length / N_elements;    % Length of each wing element

    %% 2. Calculation
    % Loop through each time step
    for j = 1:N_elements
        
        % Compute COP vectors for all timesteps in moving frame
        % Source: 2016 Wang (1, 2, & part of 3)
        % Source: 2008 Dickson (part of 3)
        COP_Wing_Frame(1, :) = delz / 2 + delz * (j - 1);                                       % x-component (Root to Tip direction)
        COP_Wing_Frame(2, :) = 0;                                                               % y-component (Perpendicular to wing surface)
        COP_Wing_Frame(3, :) = -(wing_shape.Wing_y(wing_shape.Wing_root_index - 1 + j) - ...
                          wing_shape.Wing_root(2)) + c(j) * (0.82 * abs(psi) / pi + 0.05);      % z-component (Chord direction, based on Dickson 2008)

        % Rotate vectors using R_inv for each timestep
        for i = 1:N
            COP_Body_Frame(:, i) = R_wb(:, :, i) * COP_Wing_Frame(:, i);
        end

        % Computate distance from root to COP
        COP_Distance = vecnorm(COP_Wing_Frame);

        % Store results for element j
        element(j).COP_Distance     = COP_Distance;      % COP distance from root (norms)
        element(j).COP_Body_Frame   = COP_Body_Frame;    % COP in body frame
        element(j).COP_Wing_Frame   = COP_Wing_Frame;    % COP in wing frame
    end
end
