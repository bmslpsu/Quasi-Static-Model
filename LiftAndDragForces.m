function [C_L, C_D, element, Lift_force, Drag_force, Force_Translational] = LiftAndDragForces(element, psi_f, del_r, rho , c)
    % del_r (mm)
    % c (mm)

    %this function will find only the magnitude not the direction
    % Angle of attack is in degrees (Dickinson 2002)

    %% C_L and C_D calculation
    % Array initalization
    C_L = zeros(1,length(element(1).linear_vel));
    C_D = zeros(1,length(element(1).linear_vel));

    for i=1:length(element(1).linear_vel_norm)
        C_L(i)=0.225+1.58*sind(2.13*abs(psi_f(i))-7.2);
        C_D(i)=1.92-1.55*cosd(2.04*abs(psi_f(i))-9.82);
    end
 
    %% Element wise Calcuation
    for j=1:length(element)
        
        disp(['Lift and Drag force calculation for element ' num2str(j) '/' num2str(length(element))])

        for i=1:length(element(j).linear_vel_norm)
            element(j).force_Drag(i) = 0.5*c(j)*rho*element(j).linear_vel_norm(:,i)^2*C_D(i)*del_r*-sign(psi_f(i));
            element(j).force_Lift(i) = 0.5*c(j)*rho*element(j).linear_vel_norm(:,i)^2*C_L(i)*del_r;
        end
    end

    disp('Lift and Drag force caculated for the entire wing')

    %% Entire wing Calcuation
    % Array initalization
    Lift_force = zeros(1,length(element(1).force_Lift));
    Drag_force = zeros(1,length(element(1).force_Lift));

    for j=1:length(element(1).force_Lift)
        Lift_wing_temp = 0;
        Drag_wing_temp = 0;

        for i=1:length(element)
            Lift_wing_temp=Lift_wing_temp + element(i).force_Lift(j);
            Drag_wing_temp=Drag_wing_temp + element(i).force_Drag(j);
        end

        Lift_force(j) = Lift_wing_temp;
        Drag_force(j) = Drag_wing_temp;
    end

    Force_Translational = sqrt(Lift_force.^2+Drag_force.^2);

    %% Plots
    % figure 
    % hold on
    % plot(C_L)
    % plot(C_D)
    % title('lift coef for an element')
    % legend(["C_L" "C_D"])
    % hold off
    %
    % figure 
    % hold on
    % plot(Lift_force)
    % plot(Drag_force)
    % title('Lift and Drag force for the whole wing throughout a stroke')
    % legend(["Lift" "Drag"])
    % hold off
    % 
    % figure
    % hold on
    % plot(Force_Translational)
    % title('Translation force for the whole wing throughout a stroke')
    % hold off

end