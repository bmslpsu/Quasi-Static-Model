function [element, AM_wing_force, AM_wing_torque] = Force_AddedMass(element, Kinematics, del_r, c, rho, weight)
    %% Preamble
    % This function will find only the magnitude, not the direction.
    
    % Units
    % del_r (mm)
    % c (mm)
    
    %% Starting message
    disp('Added Mass Force Calculation - Start')

    %% Array sizing
    N = length(Kinematics.psi);

    %% Array initialization
    AM_wing_force = zeros(3, N);
    AM_wing_torque = zeros(3, N);

    %% Entire wing Calculation
    for j = 1:N
        AM_wing_temp = [0;0;0];
        AM_wing_torque_temp = [0;0;0];
            
        for i = 1:length(element)
            %% Added Mass
            % Element-wise Calculation
            part1 = (rho * pi * ((c(i))^2) / 4) * del_r;
            part2 = (dot(element(i).linear_vel(:,j), element(i).linear_acc(:,j)) * sin(abs(Kinematics.psi(j)))) / element(i).linear_vel_norm(j);
            part3 = element(i).linear_vel_norm(j) * deg2rad(Kinematics.psi_d(j)) * cos(abs(Kinematics.psi(j)));
            element(i).force_AddedMass(j) = part1 * (part2 + part3);
            
            % Force Direction
            Added_Mass_Direction = -[0; 1; 0]; % Force is normal to the wing and opposite the direction of angle of attack % Same as Zafar's model and 2008 Dickson

            %Force for entire Element
            Force_element = element(i).force_AddedMass(j).*Added_Mass_Direction;

            %Force for entire Element
            AM_wing_temp = AM_wing_temp + Force_element;

            %Torque for entire Element
            AM_wing_torque_temp = AM_wing_torque_temp + cross(element(i).locationInMovingFrame(1:3,j),Force_element);

        end
        
        %% Force over entire kinematics (time)
        AM_wing_force(:,j) = AM_wing_temp;

        %% Torques over entire kinematics (time)
        AM_wing_torque(:,j) = AM_wing_torque_temp;
    end

    %% Ending message
    disp('Added Mass Force Calculation - End')

    %% Plots
    % Plot for the X-direction
    % figure
    % hold on
    % plot(AM_wing_force(1,:)./weight, 'b') % X-direction in blue
    % plot(ones(1, length(AM_wing_force(1,:))) * mean(AM_wing_force(1,:)./weight), 'b--') % Mean X as dashed line
    % title(['AM wing force - X direction, stroke ,1'])
    % legend(["AM wing force X", "Mean AM wing force X"])
    % hold off
    % 
    % % Plot for the Y-direction
    % figure
    % hold on
    % plot(AM_wing_force(2,:)./weight, 'b') % Z-direction in red
    % plot(ones(1, length(AM_wing_force(2,:))) * mean(AM_wing_force(2,:)./weight), 'b--') % Mean Z as dashed line
    % title(['AM wing force - Z direction, stroke ,2'])
    % legend(["AM wing force Z", "Mean AM wing force Z"])
    % hold off
    % 
    % % Plot for the Z-direction
    % figure
    % hold on
    % plot(AM_wing_force(3,:)./weight, 'b') % Y-direction in green
    % plot(ones(1, length(AM_wing_force(3,:))) * mean(AM_wing_force(3,:)./weight), 'b--') % Mean Y as dashed line
    % title(['AM wing force - Y direction, stroke ,3'])
    % legend(["AM wing force Y", "Mean AM wing force Y"])
    % hold off
end
