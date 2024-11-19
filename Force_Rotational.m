function [element, Rot_wing_force, Rot_wing_torque] = Force_Rotational(element, Kinematics, del_r, c, rho, weight)
    %% Preamble
    % This function will find only the magnitude, not the direction.
    
    % Units
    % del_r (mm)
    % c (mm)

    %% Starting message
    disp('Rotation Force Calculation - Start') 

    %% Array sizing
    N = length(Kinematics.psi_d);

    %% Array initialization
    Rot_wing_force = zeros(3, N);
    Rot_wing_torque = zeros(3, N);
    
    %% C_R calculation
    C_r = 1.55;

    %% Entire wing Calculation
    for j = 1:N
        Rot_wing_temp = [0;0;0];
        Rot_wing_torque_temp = [0;0;0];

        for i = 1:length(element)
            % Element wise Calculation
            element(i).force_Rotation(j) = C_r * rho * (c(i))^2 * del_r * element(i).linear_vel_norm(j) * Kinematics.psi_d(j);

            % Force Direction
            Rotation_Direction = [0; 1; 0];  %Force is normal to the wing and in the velocity direction % Same as Zafar's model and 2008 Dickson

            %Force for entire Element
            Force_element = element(i).force_Rotation(j).*Rotation_Direction;

            %Force for entire Element
            Rot_wing_temp = Rot_wing_temp + Force_element;

            %Torque for entire Element
            Rot_wing_torque_temp = Rot_wing_torque_temp + cross(element(i).locationInMovingFrame(1:3,j),Force_element); 
        
        end

        %% Forces over entire kinematics (time)
        Rot_wing_force(:,j) = Rot_wing_temp;

        %% Torques over entire kinematics (time)
        Rot_wing_torque(:,j) = Rot_wing_torque_temp;
    end
        
    %% Ending message
    disp('Rotation Force Calculation - End') 
    %%  Plot
    % Plot for the X-direction
    % figure
    % hold on
    % plot(Rot_force(1,:)./weight, 'b') % X-direction in blue
    % plot(ones(1, length(Rot_force(1,:))) * mean(Rot_force(1,:)./weight), 'b--') % Mean X as dashed line
    % title(['ROT wing force - X direction, stroke ,1'])
    % legend(["ROT wing force X", "Mean ROT wing force X"])
    % hold off
    % 
    % % Plot for the Y-direction
    % figure
    % hold on
    % plot(Rot_force(2,:)./weight, 'b') % Z-direction in red
    % plot(ones(1, length(Rot_force(2,:))) * mean(Rot_force(2,:)./weight), 'b--') % Mean Z as dashed line
    % title(['ROT wing force - Z direction, stroke ,2'])
    % legend(["ROT wing force Z", "Mean ROT wing force Z"])
    % hold off
    % 
    % % Plot for the Z-direction
    % figure
    % hold on
    % plot(Rot_force(3,:)./weight, 'b') % Y-direction in green
    % plot(ones(1, length(Rot_force(3,:))) * mean(Rot_force(3,:)./weight), 'b--') % Mean Y as dashed line
    % title(['ROT wing force - Y direction, stroke ,3'])
    % legend(["ROT wing force Y", "Mean ROT wing force Y"])
    % hold off

end
