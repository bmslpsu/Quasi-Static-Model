function [element, Rot_force] = RotationalForce(element, alpha_dotf, del_r, c, rho, alpha_f, time, alpha_dot_dotf)
    % del_r (m)
    % c (m)
    %beta_dotf (deg/s) converted to rad/s in the function for force
    %element: the struct that has the information for each element in the wing
    %throughout a wing stroke

    %% C_R Calculation
    C_r = 1.55;

    %% Element wise Calculation
    for j = 1:length(element)

        disp(['Rotation force calculation for element ' num2str(j) '/' num2str(length(element))]);

        % Ensure the loop runs for the minimum length to avoid size mismatch
        N = min([length(element(j).linear_vel_norm), length(alpha_dotf), length(alpha_dot_dotf)]);

        for i = 1:N-1  % Subtracting 1 to account for velocity differences due to 'diff'
            %element(j).force_Rotation(i) = C_r * rho * (c(j))^2 * del_r * element(j).linear_vel(1,i) * deg2rad(alpha_dotf(i));
            element(j).force_Rotation(i) = C_r * rho * (c(j))^2 * del_r * element(j).linear_vel_norm(i) * deg2rad(alpha_dotf(i));
            
        end
    end

    disp('Rotational force calculated for the entire wing')

    %% Entire wing Calculation
    % Initialize array for storing rotational force
    Rot_force = zeros(1, length(element(1).force_Rotation));

    % Ensure we sum forces for the correct number of elements
    N = length(Rot_force);  % Length of force_Rotation array

    for j = 1:N
        Rot_wing_temp = 0;

        for i = 1:length(element)
            Rot_wing_temp = Rot_wing_temp + element(i).force_Rotation(j);
        end

        Rot_force(j) = Rot_wing_temp;
    end

    %%  Plot
    figure
    hold on
    plot(Rot_force)
    title('Rotational force for the whole wing throughout a stroke')
    hold off

end
