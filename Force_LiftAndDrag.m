function [element, Lift_force, Drag_force, Lift_torque, Drag_torque] = Force_LiftAndDrag(element, Kinematics, del_r, rho, c, weight)
    %% Preamble
    % This function will find only the magnitude, not the direction.
    
    % Units
    % del_r (mm)
    % c (mm)
    
    %% Starting message
    disp('Lift and Drag Force Calculation - Start')
  
    %% Array sizing
    N = length(Kinematics.psi);

    %% Array initialization
    Lift_force = zeros(3, N);
    Drag_force = zeros(3, N);
    Lift_torque = zeros(3, N);
    Drag_torque = zeros(3, N);

    %% C_L and C_D calculation
    % Angle of attack is in degrees (Dickinson 2002).

    C_L = 0.225 + 1.58 * sin(deg2rad(2.13 * abs(rad2deg(Kinematics.psi)) - 7.2));
    C_D = 1.92 - 1.55 * cos(deg2rad(2.04 * rad2deg(Kinematics.psi) - 9.82));

    %% Entire wing Calculation
    for j = 1:N
        Lift_wing_temp = [0;0;0];
        Drag_wing_temp = [0;0;0];
        Lift_wing_torque_temp = [0;0;0];
        Drag_wing_torque_temp = [0;0;0];

        for i = 1:length(element)
            %% Lift
            % Element-wise Calculation
            element(i).force_Lift(j) = 0.5 * rho * del_r * c(i) * C_L(j) * element(i).linear_vel_norm(j).^2;
            
            % Force Direction
            Lift_Direction = rotx(sign(element(i).linear_vel(3))*pi/2)*element(i).linear_vel_direction(:,j)*sign(Kinematics.phi_d(j)); % Force is always normal (up) to the velocity direction % Same as Zafar's model and 2008 Dickson
            
            % Element force
            Force_element = element(i).force_Lift(j).*Lift_Direction;
            
            %Force for entire Element
            Lift_wing_temp = Lift_wing_temp + Force_element;
            
            %Torque for entire Element
            Lift_wing_torque_temp = Lift_wing_torque_temp + cross(element(i).locationInMovingFrame(1:3,j),Force_element);

            %% Drag
            % Element-wise Calculation
            element(i).force_Drag(j) = 0.5 * rho * del_r * c(i) * C_D(j) * element(i).linear_vel_norm(j).^2;
            
            % Force Direction % Force is always opposite the velocity direction
            Drag_Direction = element(i).linear_vel_direction(:,j)*sign(Kinematics.phi_d(j)); % Force is always opposite the velocity direction % Same as Zafar's model and 2008 Dickson
            
            % Element force
            Force_element = element(i).force_Drag(j).*Drag_Direction;
            
            %Force for entire Element
            Drag_wing_temp = Drag_wing_temp + Force_element;
            
            %Torque for entire Element
            Drag_wing_torque_temp = Drag_wing_torque_temp + cross(element(i).locationInMovingFrame(1:3,j),Force_element);

        end

        %% Forces over entire kinematics (time)
        Lift_force(:,j) = Lift_wing_temp;
        Drag_force(:,j) = Drag_wing_temp;

        %% Torques over entire kinematics (time)
        Lift_torque(:,j) = Lift_wing_torque_temp;
        Drag_torque(:,j) = Drag_wing_torque_temp;

    end

    %% Ending Message
    disp('Lift and Drag Force Calculation - End')
    %% Optional Plots
    % % figure
    % % hold on
    % % plot(C_L)
    % % plot(C_D)
    % % title('Lift coefficient for an element')
    % % legend(["C_L" "C_D"])
    % % hold off
    % % 
    % First figure (stroke 1)
    % figure
    % hold on
    % plot(Lift_force(1,:)./weight, 'b')
    % plot(Drag_force(1,:)./weight, 'r')
    % sum_forces_1 = (Lift_force(1,:)./weight) + (Drag_force(1,:)./weight);
    % plot(sum_forces_1, 'k') % Plot the sum in black
    % plot(ones(1, length(Lift_force(1,:))) * mean(Lift_force(1,:)./weight), 'b--') % Mean lift as dashed line
    % plot(ones(1, length(Drag_force(1,:))) * mean(Drag_force(1,:)./weight), 'r--') % Mean drag as dashed line
    % plot(ones(1, length(sum_forces_1)) * mean(sum_forces_1), 'k--') % Mean of sum as dashed line
    % title('Lift, Drag forces, and their Sum for the whole wing throughout a stroke, 1')
    % legend(["Lift", "Drag", "Sum", "Mean Lift", "Mean Drag", "Mean Sum"])
    % hold off
    % 
    % % Second figure (stroke 2)
    % figure
    % hold on
    % plot(Lift_force(2,:)./weight, 'b')
    % plot(Drag_force(2,:)./weight, 'r')
    % sum_forces_2 = (Lift_force(2,:)./weight) + (Drag_force(2,:)./weight);
    % plot(sum_forces_2, 'k') % Plot the sum in black
    % plot(ones(1, length(Lift_force(2,:))) * mean(Lift_force(2,:)./weight), 'b--') % Mean lift as dashed line
    % plot(ones(1, length(Drag_force(2,:))) * mean(Drag_force(2,:)./weight), 'r--') % Mean drag as dashed line
    % plot(ones(1, length(sum_forces_2)) * mean(sum_forces_2), 'k--') % Mean of sum as dashed line
    % title('Lift, Drag forces, and their Sum for the whole wing throughout a stroke, 2')
    % legend(["Lift", "Drag", "Sum", "Mean Lift", "Mean Drag", "Mean Sum"])
    % hold off
    % 
    % % Third figure (stroke 3)
    % figure
    % hold on
    % plot(Lift_force(3,:)./weight, 'b')
    % plot(Drag_force(3,:)./weight, 'r')
    % sum_forces_3 = (Lift_force(3,:)./weight) + (Drag_force(3,:)./weight);
    % plot(sum_forces_3, 'k') % Plot the sum in black
    % plot(ones(1, length(Lift_force(3,:))) * mean(Lift_force(3,:)./weight), 'b--') % Mean lift as dashed line
    % plot(ones(1, length(Drag_force(3,:))) * mean(Drag_force(3,:)./weight), 'r--') % Mean drag as dashed line
    % plot(ones(1, length(sum_forces_3)) * mean(sum_forces_3), 'k--') % Mean of sum as dashed line
    % title('Lift, Drag forces, and their Sum for the whole wing throughout a stroke, 3')
    % legend(["Lift", "Drag", "Sum", "Mean Lift", "Mean Drag", "Mean Sum"])
    % hold off
    % 
    % 
    % figure
    % hold on
    % plot(Force_Translational'./weight)
    % title('Translational force for the whole wing throughout a stroke')
    % hold off

end
