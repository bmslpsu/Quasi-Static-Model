function [C_L, C_D, element, Lift_force, Drag_force, Force_Translational] = LiftAndDragForces(element, psi_f, del_r, rho, c, phi_dotf)
    % del_r (mm)
    % c (mm)

    % This function will find only the magnitude, not the direction.
    % Angle of attack is in degrees (Dickinson 2002).

    %% C_L and C_D calculation
    % Find the minimum length to avoid indexing errors
    N = min([length(element(1).linear_vel), length(psi_f), length(phi_dotf)]);

    % Array initialization
    C_L = zeros(1, N);
    C_D = zeros(1, N);

    for i = 1:N
        C_L(i) = 0.225 + 1.58 * sind(2.13 * abs(psi_f(i)) - 7.2);
        C_D(i) = 1.92 - 1.55 * cosd(2.04 * abs(psi_f(i)) - 9.82);
    end

    %% Element-wise Calculation
    % figure
    % hold on
    
    for j = 1:length(element)
        
        disp(['Lift and Drag force calculation for element ' num2str(j) '/' num2str(length(element))])

        
        for i = 1:N
            % element(j).force_Drag(i) = 0.5 * rho * del_r * c(j) * C_D(i) * element(j).linear_vel(1,i).^2;
            % element(j).force_Lift(i) = 0.5 * rho * del_r * c(j) * C_L(i) * element(j).linear_vel(1,i).^2;
            element(j).force_Drag(i) = 0.5 * rho * del_r * c(j) * C_D(i) * element(j).linear_vel_norm(i).^2;
            element(j).force_Lift(i) = 0.5 * rho * del_r * c(j) * C_L(i) * element(j).linear_vel_norm(i).^2;
            % test(j).test2(i) = element(j).linear_vel(1,i).^2;
        end
        % plot(test(j).test2(:))
    end
    % hold off
    

    disp('Lift and Drag force calculated for the entire wing')

    %% Entire wing Calculation
    % Array initialization
    Lift_force = zeros(1, N);
    Drag_force = zeros(1, N);

    for j = 1:N
        Lift_wing_temp = 0;
        Drag_wing_temp = 0;

        for i = 1:length(element)
            Lift_wing_temp = Lift_wing_temp + element(i).force_Lift(j);
            Drag_wing_temp = Drag_wing_temp + element(i).force_Drag(j);
        end

        Lift_force(j) = Lift_wing_temp;
        Drag_force(j) = Drag_wing_temp;
    end

    Force_Translational = sqrt(Lift_force.^2 + Drag_force.^2);

    %% Optional Plots
    % figure 
    % hold on
    % plot(C_L)
    % plot(C_D)
    % title('Lift coefficient for an element')
    % legend(["C_L" "C_D"])
    % hold off
    
    figure 
    hold on
    plot(Lift_force)
    plot(Drag_force)
    title('Lift and Drag forces for the whole wing throughout a stroke')
    legend(["Lift" "Drag"])
    hold off


    figure
    hold on
    plot(Force_Translational)
    title('Translational force for the whole wing throughout a stroke')
    hold off

end
