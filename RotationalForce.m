function [element, Rot_force] = RotationalForce(element, alpha_dotf, del_r, c, rho, alpha_f, time)
    % del_r (m)
    % c (m)
    %beta_dotf (deg/s) converted to rad/s in the function for force
    %element: the struct that has the information for each element in the wing
    %throughout a wing stroke

    %% C_R Calculation
    C_r = 1.55;

    %% Rotational Accelertion
    alpha_ddotf = diff(alpha_dotf) / (time(2) - time(1));

    %% Element wise Calcuation
    for j=1:length(element)

        disp(['Rotation force caclulation for element' num2str(j) '/' num2str(length(element))]);

        for i=1:length(element(j).linear_vel_norm)-1
             element(j).force_Rotation(i)  = C_r*rho*(c(j))^2*del_r*element(j).linear_vel_norm(i)*((alpha_dotf(i))*pi/180)*-sign(alpha_ddotf(i));
        end
    end

    disp('Rotational force caculated for the entire wing')
    
    %% Entire wing Calcuation
    % Array intilization
    Rot_force=zeros(1,length(element(1).force_Lift));

    for j=1:length(element(1).force_Rotation)
        Rot_wing_temp = 0;

        for i=1:length(element)
            Rot_wing_temp = Rot_wing_temp + element(i).force_Rotation(j);
        end

        Rot_force(j) = Rot_wing_temp;
    end

    %% Plots
    % figure
    % hold on
    % plot(Rot_force)
    % title('Rotational for the whole wing throughout a stroke')
    % hold off

end