function element = FindCOP(psi, n, c, R_inv2, wing_length, wing_shape)
    %% Starting message
    disp('Location of COP Calculation - Start');

    % For the wing
    % x-axis is along the length of the wing (Root to Tip)
    % y-axis is perpendicular to the surface of the wing
    % z-axis is along the chord of the wing starting and is parallel to the abdomen of the fly
    
    %% Parameters and Precomputations
    delz = wing_length / n;             % Length of each wing element

    % Preallocate arrays
    N = length(psi);
    x_cp = zeros(n, N);                 % x-coordinate of COP for each element and timestep
    element(n).Distance_COP = [];       % Preallocate struct array for n elements

    % Preallocate vectors for efficiency
    r_cp_vec = zeros(3, N);    % COP vector in moving frame
    r_cp = zeros(3, N);        % COP vector in stationary frame

    %% Main Computation Loop
    for j = 1:n
        
        % Compute vectors for all timesteps in one go (vectorized)
        r_cp_vec(1, :) = delz / 2 + delz * (j - 1);                 % x-component of the COP
        r_cp_vec(2, :) = 0;                                         % y-component is zero
        r_cp_vec(3, :) = -(wing_shape.Wing_y(wing_shape.Wing_root_index - 1 + j) - wing_shape.Wing_root(2)) + c(j) * (0.82 * abs(psi) / pi + 0.05);      % z-component (along the chord) % Based on paper: 2008 Dickson Integrative...

        % Rotate vectors using R_inv2 for all timesteps
        for i = 1:N
            r_cp(:, i) = R_inv2(:, :, i) * r_cp_vec(:, i);
        end

        % Store results for element j
        element(j).Distance_COP = vecnorm(r_cp, 2, 1);  % Compute COP distances (norms)
        element(j).location_cop = r_cp;                 % COP in body frame
        element(j).locationInMovingFrame = r_cp_vec;    % COP in wing frame
    end

    %% Ending message
    disp('Location of COP Calculation - End');
end
