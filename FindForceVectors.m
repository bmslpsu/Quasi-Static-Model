function element = FindForceVectors(element, R_inv2, gamma_f, phi_f, phi_dotf, psi_f, psi_dotf)

    for j = 1:length(element)

        disp(['Calculating vector force for element ' num2str(j) '/' num2str(length(element))])

        % Find the minimum length of force arrays to avoid indexing errors
        N = min([length(element(j).force_Lift), length(element(j).force_Drag), length(element(j).force_AddedMass), length(element(j).force_Rotation)]);

        % Array Initialization
        f_lift_vec = zeros(3, N);
        f_drag_vec = zeros(3, N);
        f_addedMass_vec = zeros(3, N);
        f_Rot_vec = zeros(3, N);

        for i = 1:N  % Loop until the smallest force array length

            % Define the vector of the added mass and rotational force
            e_WingNormal = [cosd(90 - psi_f(i)); sind(90 - psi_f(i)); 0];

            % Calculate the force vectors by rotating them into the proper frame
            % Force is always directly up
            f_lift_vec(:, i)        = R_inv2(:, :, i) * (element(j).force_Lift(i)        * [0;   1;   0]);

            % Force is always opposite the velocity direction (This needs recalculated if using the "norm" velocity)
            f_drag_vec(:, i)        = R_inv2(:, :, i) * (element(j).force_Drag(i)        * [1;   0;   0]                                 .*[sign(phi_dotf(i));       1;  1]);

            % Force is normal to the wing and opposite the direction of angle of attack
            f_addedMass_vec(:, i)   = R_inv2(:, :, i) * (element(j).force_AddedMass(i)   * e_WingNormal                                  .*[-sign(psi_dotf(i));    1;  1]);

            % Force is normal to the wing and in the velocity direction  (This needs recalculated if using the "norm" velocity)
            f_Rot_vec(:, i)         = R_inv2(:, :, i) * (element(j).force_Rotation(i)    * e_WingNormal                                  .*[-sign(phi_dotf(i));      1;  1]);

        end

        % Store the force vectors in the element structure
        element(j).force_lift_vec = f_lift_vec; %.* [0;   0;   0];
        element(j).force_drag_vec = f_drag_vec; %.* [0;   0;   0];
        element(j).force_AM_vec = f_addedMass_vec; %.* [0;   0;   0];
        element(j).force_Rot_vec = f_Rot_vec; %.* [0;   0;   0];
    end

    disp('Calculations for elements are done')
end
