clear Force_Lift_X_mean Force_Drag_X_mean Force_AM_X_mean Force_Rot_X_mean Force_total_norm_mean Force_Lift_norm_mean Force_Drag_norm_mean Force_AM_norm_mean Force_Rot_norm_mean

% Define fly numbers
fly_numbers = [13:14];

% Initialize storage for results
results = struct();

% Initialize a figure for combined plotting
figure;

% Loop through each fly number
for fly_idx = 1:length(fly_numbers)

    clear Period_Index

    % Access Fly data from the structured array
    fly_nums = Fly_Master(1, fly_numbers(fly_idx));
    phi = fly_nums.Fly.Kinematics.LH.phi; % Stroke angle (phi)

    % Find each period via peaks
    [peaks, peakIndices] = findpeaks(phi);

    % Initialize period indices
    j = 1;
    Period_Index(1) = 1;

    i = 2;
    while j < length(phi)
        % Get the last held value
        last_value = phi(1);

        % Search for the next value after the current peak within ±0.1 of last_value
        for k = peakIndices(i-1):length(phi)
            if abs(phi(k) - last_value) <= 0.2
                Period_Index(i) = k; % Store the index
                j = k; % Update the loop index
                break;
            end
        end

        i = i + 1; % Move to the next period index
        if j > peakIndices(end) || i > length(peakIndices)
            break;
        end
    end

    % Initialize arrays for storing mean forces
    Force_total_X_mean = zeros(1, length(Period_Index) - 1);
    Force_total_Y_mean = zeros(1, length(Period_Index) - 1);
    Force_total_Z_mean = zeros(1, length(Period_Index) - 1);

    % Loop through each period to calculate mean forces
    for p = 1:(length(Period_Index) - 1)
        % Extract the start and end indices for the current period
        start_idx = Period_Index(p);
        end_idx = Period_Index(p + 1) - 1; % Exclude the endpoint of the next period

        % Mean calculations for Left Hand (LH) wing
        force_total_LH_mean_x = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_Total(1, start_idx:end_idx));
        force_total_LH_mean_y = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_Total(2, start_idx:end_idx));
        force_total_LH_mean_z = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_Total(3, start_idx:end_idx));

        % Mean calculations for Right Hand (RH) wing
        force_total_RH_mean_x = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_Total(1, start_idx:end_idx));
        force_total_RH_mean_y = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_Total(2, start_idx:end_idx));
        force_total_RH_mean_z = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_Total(3, start_idx:end_idx));

        % Force components mean calculations normalized by the fly's total weight
        Force_total_X_mean(p) = (force_total_LH_mean_x + force_total_RH_mean_x) / fly_nums.Fly.Morphology.total.weight;
        Force_total_Y_mean(p) = (force_total_LH_mean_y + force_total_RH_mean_y) / fly_nums.Fly.Morphology.total.weight;
        Force_total_Z_mean(p) = (force_total_LH_mean_z + force_total_RH_mean_z) / fly_nums.Fly.Morphology.total.weight;

        % LIFT FORCES
        force_lift_LH_mean_x = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_Lift(1, start_idx:end_idx));
        force_lift_LH_mean_y = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_Lift(2, start_idx:end_idx));
        force_lift_LH_mean_z = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_Lift(3, start_idx:end_idx));

        force_lift_RH_mean_x = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_Lift(1, start_idx:end_idx));
        force_lift_RH_mean_y = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_Lift(2, start_idx:end_idx));
        force_lift_RH_mean_z = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_Lift(3, start_idx:end_idx));

        Force_Lift_X_mean(p) = (force_lift_LH_mean_x + force_lift_RH_mean_x) / fly_nums.Fly.Morphology.total.weight;
        Force_Lift_Y_mean(p) = (force_lift_LH_mean_y - force_lift_RH_mean_y) / fly_nums.Fly.Morphology.total.weight;
        Force_Lift_Z_mean(p) = (force_lift_LH_mean_z + force_lift_RH_mean_z) / fly_nums.Fly.Morphology.total.weight;

        % DRAG FORCES
        force_drag_LH_mean_x = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_Drag(1, start_idx:end_idx));
        force_drag_LH_mean_y = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_Drag(2, start_idx:end_idx));
        force_drag_LH_mean_z = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_Drag(3, start_idx:end_idx));

        force_drag_RH_mean_x = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_Drag(1, start_idx:end_idx));
        force_drag_RH_mean_y = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_Drag(2, start_idx:end_idx));
        force_drag_RH_mean_z = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_Drag(3, start_idx:end_idx));

        Force_Drag_X_mean(p) = (force_drag_LH_mean_x + force_drag_RH_mean_x) / fly_nums.Fly.Morphology.total.weight;
        Force_Drag_Y_mean(p) = (force_drag_LH_mean_y + force_drag_RH_mean_y) / fly_nums.Fly.Morphology.total.weight;
        Force_Drag_Z_mean(p) = (force_drag_LH_mean_z + force_drag_RH_mean_z) / fly_nums.Fly.Morphology.total.weight;
        
        % ADDED MASS FORCES
        force_AM_LH_mean_x = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_AM(1, start_idx:end_idx));
        force_AM_LH_mean_y = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_AM(2, start_idx:end_idx));
        force_AM_LH_mean_z = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_AM(3, start_idx:end_idx));

        force_AM_RH_mean_x = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_AM(1, start_idx:end_idx));
        force_AM_RH_mean_y = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_AM(2, start_idx:end_idx));
        force_AM_RH_mean_z = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_AM(3, start_idx:end_idx));

        Force_AM_X_mean(p) = (force_AM_LH_mean_x + force_AM_RH_mean_x) / fly_nums.Fly.Morphology.total.weight;
        Force_AM_Y_mean(p) = (force_AM_LH_mean_y + force_AM_RH_mean_y) / fly_nums.Fly.Morphology.total.weight;
        Force_AM_Z_mean(p) = (force_AM_LH_mean_z + force_AM_RH_mean_z) / fly_nums.Fly.Morphology.total.weight;

        % ROTATIONAL FORCES
        force_Rot_LH_mean_x = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_Rotation(1, start_idx:end_idx));
        force_Rot_LH_mean_y = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_Rotation(2, start_idx:end_idx));
        force_Rot_LH_mean_z = mean(fly_nums.Fly.Dynamics.Frame_Body.LH.Force_Rotation(3, start_idx:end_idx));

        force_Rot_RH_mean_x = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_Rotation(1, start_idx:end_idx));
        force_Rot_RH_mean_y = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_Rotation(2, start_idx:end_idx));
        force_Rot_RH_mean_z = mean(fly_nums.Fly.Dynamics.Frame_Body.RH.Force_Rotation(3, start_idx:end_idx));

        Force_Rot_X_mean(p) = (force_Rot_LH_mean_x + force_Rot_RH_mean_x) / fly_nums.Fly.Morphology.total.weight;
        Force_Rot_Y_mean(p) = (force_Rot_LH_mean_y + force_Rot_RH_mean_y) / fly_nums.Fly.Morphology.total.weight;
        Force_Rot_Z_mean(p) = (force_Rot_LH_mean_z + force_Rot_RH_mean_z) / fly_nums.Fly.Morphology.total.weight;

        % Norm calculations for total forces
        Force_total_norm_mean(p) = sqrt(Force_X_mean(p).^2 + Force_Y_mean(p).^2 + Force_Z_mean(p).^2);

        % Norm calculations for lift forces
        Force_Lift_norm_mean(p) = sqrt(Force_Lift_X_mean(p).^2 + Force_Lift_Y_mean(p).^2 + Force_Lift_Z_mean(p).^2);

        % Norm calculations for drag forces
        Force_Drag_norm_mean(p) = sqrt(Force_Drag_X_mean(p).^2 + Force_Drag_Y_mean(p).^2 + Force_Drag_Z_mean(p).^2);

        % Norm calculations for added mass forces
        Force_AM_norm_mean(p) = sqrt(Force_AM_X_mean(p).^2 + Force_AM_Y_mean(p).^2 + Force_AM_Z_mean(p).^2);

        % Norm calculations for rotational forces
        Force_Rot_norm_mean(p) = sqrt(Force_Rot_X_mean(p).^2 + Force_Rot_Y_mean(p).^2 + Force_Rot_Z_mean(p).^2);


    end

    % Define time intervals for each period
    period_times = fly_nums.Fly.time(Period_Index);

    % Interpolate mean forces over the entire time vector
    time_vector = fly_nums.Fly.time;
    Force_Lift_norm_mean_interp = interp1(period_times(1:end-1), Force_Lift_norm_mean, time_vector, 'previous', 'extrap');
    Force_Drag_norm_mean_interp = interp1(period_times(1:end-1), Force_Drag_norm_mean, time_vector, 'previous', 'extrap');
    Force_AM_norm_mean_interp = interp1(period_times(1:end-1), Force_AM_norm_mean, time_vector, 'previous', 'extrap');
    Force_Rot_norm_mean_interp = interp1(period_times(1:end-1), Force_Rot_norm_mean, time_vector, 'previous', 'extrap');

    % Mask positive and negative values
    Force_Lift_norm_mean_interp_pos = Force_Lift_norm_mean_interp;
    Force_Lift_norm_mean_interp_neg = Force_Lift_norm_mean_interp;
    Force_Lift_norm_mean_interp_pos(Force_Lift_norm_mean_interp <= 0) = NaN;
    Force_Lift_norm_mean_interp_neg(Force_Lift_norm_mean_interp >= 0) = NaN;

    Force_Drag_norm_mean_interp_pos = Force_Drag_norm_mean_interp;
    Force_Drag_norm_mean_interp_neg = Force_Drag_norm_mean_interp;
    Force_Drag_norm_mean_interp_pos(Force_Drag_norm_mean_interp <= 0) = NaN;
    Force_Drag_norm_mean_interp_neg(Force_Drag_norm_mean_interp >= 0) = NaN;

    Force_AM_norm_mean_interp_pos = Force_AM_norm_mean_interp;
    Force_AM_norm_mean_interp_neg = Force_AM_norm_mean_interp;
    Force_AM_norm_mean_interp_pos(Force_AM_norm_mean_interp <= 0) = NaN;
    Force_AM_norm_mean_interp_neg(Force_AM_norm_mean_interp >= 0) = NaN;

    Force_Rot_norm_mean_interp_pos = Force_Rot_norm_mean_interp;
    Force_Rot_norm_mean_interp_neg = Force_Rot_norm_mean_interp;
    Force_Rot_norm_mean_interp_pos(Force_Rot_norm_mean_interp <= 0) = NaN;
    Force_Rot_norm_mean_interp_neg(Force_Rot_norm_mean_interp >= 0) = NaN;

if strcmp(fly_nums.State, 'Pre Cut') % Check if it's pre-cut
    color_pos = 'b'; % Blue for positive
    color_neg = 'b'; % Blue for negative
elseif strcmp(fly_nums.State, 'Post Cut') % Check if it's pre-cut
    color_pos = 'r'; % Orange (use 'r' for red; MATLAB doesn't have orange) for positive
    color_neg = 'r'; % Orange for negative
else
    color_pos = 'g'; % Orange (use 'r' for red; MATLAB doesn't have orange) for positive
    color_neg = 'g'; % Orange for negative
end
    
%% Plot forces over time
subplot(4, 1, 1);
hold on;

plot(time_vector / 8000, abs(Force_Lift_norm_mean_interp_pos), '-', 'LineWidth', 1.5, 'Color', color_pos, 'DisplayName', [fly_nums.State]);

ylabel('Lift Force (F/mg)');
title('Forces Over Time');
legend('Pre-Cut', 'Post-Cut', 'Steady State');
grid on;
ylim([0 1])

subplot(4, 1, 2);
hold on;
plot(time_vector / 8000, abs(Force_Drag_norm_mean_interp_pos), '-', 'LineWidth', 1.5, 'Color', color_pos);
plot(time_vector / 8000, abs(Force_Drag_norm_mean_interp_neg), '--', 'LineWidth', 1.5, 'Color', color_neg);
ylabel('Drag Force (F/mg)');
ylim([0 1])
grid on;

subplot(4, 1, 3);
hold on;
plot(time_vector / 8000, abs(Force_AM_norm_mean_interp_pos), '-', 'LineWidth', 1.5, 'Color', color_pos);
plot(time_vector / 8000, abs(Force_AM_norm_mean_interp_neg), '--', 'LineWidth', 1.5, 'Color', color_neg);
ylabel('Added Mass Force (F/mg)');
grid on;
ylim([0 1])

subplot(4, 1, 4);
hold on;
plot(time_vector / 8000, abs(Force_Rot_norm_mean_interp_pos), '-', 'LineWidth', 1.5, 'Color', color_pos);
plot(time_vector / 8000, abs(Force_Rot_norm_mean_interp_neg), '--', 'LineWidth', 1.5, 'Color', color_neg);
xlabel('Time (s)');
ylabel('Rotational Force (F/mg)');
grid on;
ylim([0 1])




end

% Overall plot adjustments
sgtitle(['Forces Over Time: Fly: ', num2str(fly_nums.Fly_Num)]);

%%


