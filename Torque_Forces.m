function [forces_torque] = Torque_Forces(Fly, element, R_inv2, force)
    %% Preamble
    disp('Force Torque Calculation - Start');

    %% Initialization
    N = length(Fly.force_total.Force_Body_LH.Lift_torque);
    Force_torque = zeros(3, N);

    %% Compute Total Center of Pressure (COP)
    [total_cop, bad_data] = computeCOP(N, element);

    %% Handle Bad Data in COP
    total_cop = handleBadData(total_cop, bad_data);

    %% Smooth COP using Butterworth Filter
    [b, a] = butter(2, 0.1, 'low');
    for j = 1:3
        total_cop(j, :) = filtfilt(b, a, total_cop(j, :));
    end

    %% Compute Torques
    for j = 1:N
        CG_joint = [0; 0; 0]; %rotx(deg2rad(32.5)) * (Fly.body.Joint' .* [1; 1; -1]);
        CG_wing = CG_joint + R_inv2(:, :, j) * total_cop(:, j);
        CG_body = [0; 0; 0]; %rotx(deg2rad(-32.5)) * Fly.total.CG';
        CG_Delta = CG_body - CG_wing;
        Force_torque(:, j) = cross(CG_Delta, force.force_total_vec(:, j));
    end

    %% Output Results
    forces_torque = Force_torque;
    disp('Force Torque Calculation - End');
end

function [total_cop, bad_data] = computeCOP(N, element)
    total_cop = zeros(3, N);
    total_cop_temp_hold = zeros(3, N);
    Total_force_temp_2_hold = zeros(3, N);
    bad_data = zeros(3, N);

    for j = 1:N
        total_cop_temp = zeros(3, 1);
        Total_force_temp_2 = zeros(3, 1);
        for i = 1:length(element)
            Total_force_temp = sum([
                element(i).force_Lift(j), 
                element(i).force_Drag(j), 
                element(i).force_Rotation(j), 
                element(i).force_AddedMass(j)
            ]);
            Total_force_temp_2 = Total_force_temp_2 + Total_force_temp;
            total_cop_temp = total_cop_temp + Total_force_temp .* element(i).locationInMovingFrame(1:3, j);
        end
        total_cop_temp_hold(:, j) = total_cop_temp;
        Total_force_temp_2_hold(:, j) = Total_force_temp_2;
        total_cop(:, j) = total_cop_temp ./ Total_force_temp_2;
    end

    % Identify bad data points based on thresholds
    threshold_1 = 1.5;
    threshold_2 = 0.5;
    for i = 1:3
        for j = 2:N-1
            value = abs(total_cop_temp_hold(i, j) / Total_force_temp_2_hold(i, j)) / ...
                    abs(total_cop_temp_hold(i, j-1) / Total_force_temp_2_hold(i, j-1));
            bad_data(i, j) = value > threshold_1 || value < threshold_2;
        end
    end
end

function total_cop = handleBadData(total_cop, bad_data)
    for i = 1:3
        for j = 2:size(total_cop, 2)-1
            if bad_data(i, j)
                indicesOfZero = find(bad_data(i, :) == 0);
                nextIndex = indicesOfZero(find(indicesOfZero > j, 1));

                if j > 2
                    if isempty(nextIndex)
                        total_cop(i, j) = 2 * total_cop(i, j-1) - total_cop(i, j-2);
                    else
                        total_cop(i, j) = mean([total_cop(i, j-1), total_cop(i, j-2), total_cop(i, nextIndex)]);
                    end
                else
                    if isempty(nextIndex)
                        total_cop(i, j) = total_cop(i, j-1);
                    else
                        total_cop(i, j) = mean([total_cop(i, j-1), total_cop(i, nextIndex)]);
                    end
                end
            end
        end
    end
end
