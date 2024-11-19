function [forces_torque] = Torque_Forces(Fly, element, wing, R_inv2, force, side, fly_num)
    %% Preamble
    % This function will find both the magnitude and the direction.

    %% Starting message
    disp('Force Torque Calculation - Start') 

    %% Array sizing
    N = length(Fly.force_total.Force_Body_LH.Lift_torque);

    %% Array initialization
    Force_torque = zeros(3, N);
    total_cop = zeros(3, N);
    

    %% Total COP Location


    for j = 1:N
        total_cop_temp = [0; 0; 0];
        Total_force_temp = [0; 0; 0];
        Total_force_temp_2 = [0; 0; 0];
        for i = 1:length(element)
            Total_force_temp = element(i).force_Lift(j)+element(i).force_Drag(j)+element(i).force_Rotation(j)+element(i).force_AddedMass(j);
            Total_force_temp_2 = Total_force_temp_2 + Total_force_temp;
            total_cop_temp = total_cop_temp + Total_force_temp .* element(i).locationInMovingFrame(1:3,j);
        end
        total_cop_temp_hold(:,j)  = total_cop_temp;
        Total_force_temp_2_hold(:,j)  = Total_force_temp_2;

        total_cop(:,j) =  total_cop_temp./Total_force_temp_2;

    end

    threshold_1 = 1.5; % Your chosen threshold for force validation
    threshold_2 = 0.5; % Your chosen threshold for force validation
    for i=1:3
        for j=1:N
            if j>1 & j<N-1
                value =(abs(total_cop_temp_hold(i,j)/Total_force_temp_2_hold(i,j))/(abs(total_cop_temp_hold(i,j-1)/Total_force_temp_2_hold(i,j-1))));
                if value>threshold_1 || value<threshold_2
                    bad_data(i,j)=1;
                else
                    bad_data(i,j)=0;
                end
            end
        end
    end

    for i=1:3
        for j=1:N
            if j>1 && j<N-1
                if bad_data(i,j)==1
                    % Find all indices where the value equals 0
                    indicesOfZero = find(bad_data(i,:) == 0);

                    % Find the next value that equals 0 after the currentIndex
                    nextIndex = indicesOfZero(find(indicesOfZero > j, 1));

                    % Handle the case for projecting based on j-2 and j-1 values
                    if j > 2
                        % If j-2 exists, use both j-1 and j-2 for projection
                        if isempty(nextIndex) % If no next valid value
                            % Project using the previous two values
                            total_cop(i, j) = (2 * total_cop(i, j - 1) - total_cop(i, j - 2));
                        else
                            % Use the average of j-1, j-2, and nextIndex values
                            total_cop(i, j) = (total_cop(i, j - 1) + total_cop(i, j - 2) + total_cop(i, nextIndex)) / 3;
                        end
                    else
                        % If j-2 does not exist (e.g., j is 2 or less)
                        if isempty(nextIndex) % If no next valid value
                            % Project using just the previous value
                            total_cop(i, j) = total_cop(i, j - 1);
                        else
                            % Use the average of j-1 and nextIndex values
                            total_cop(i, j) = (total_cop(i, j - 1) + total_cop(i, nextIndex)) / 2;
                        end
                    end
                end
            end
        end
    end

   

    [b, a] = butter(2, 0.1, 'low');
    for j=1:3
        total_cop_but(j, :) = filtfilt(b, a, total_cop(j, :));
    end
    
    
    % figure
    % hold on
    % plot(total_cop','b')
    % plot(total_cop_but','m')
    % title([num2str(side) ' ' num2str(fly_num)])
    % hold off

    total_cop=total_cop_but;


    %% CG Delta Caluclation
    % figure
    % hold on
    for j = 1:N
        %% Torques over entire kinematics (time)
        Rot_mat = R_inv2(:, :, j);
        % CG_test(:,j)=total_cop(:,j);
        CG_Delta = (Fly.body.Joint.*[1;1;1] + Rot_mat * total_cop(:,j)) - Fly.total.CG';
        % % scatter(j,CG_test(1,j),'.','b')
        % scatter(j,CG_Delta(1),'x','b')
        % % scatter(j,CG_test(2,j),'.','m')
        % scatter(j,CG_Delta(2),'x','m')
        % % scatter(j,CG_test(3,j),'.','r')
        % scatter(j,CG_Delta(3),'x','r')
        Force_torque(:,j) = cross(CG_Delta,force.force_total_vec(:,j));
    end
    % hold off
    forces_torque = Force_torque;

    % figure
    % plot(forces_torque');
    % title([num2str(side) ' ' num2str(fly_num)])
    
        
    %% Ending message
    disp('Force Torque Calculation - End') 

end