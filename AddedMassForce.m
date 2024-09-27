function [element, AM_wing_force] = AddedMassForce(element, psi_dotf, psi_f, del_r, c, time, rho, phi_dotf)
    % del_r (mm)
    % c (mm)

    %% Time adjustment
    adjusted_psi_f = psi_f(2:end); % Truncate psi_f to match linear_acc size
    adjusted_psi_dotf = psi_dotf(2:end); % Truncate psi_dotf to match linear_acc size

    % % Initialize arrays for storing the added mass force parts
    % part_1_total = zeros(length(linear_acc(1,:)), length(element));
    % part_2_total = zeros(length(linear_acc(1,:)), length(element));
    % part_3_total = zeros(length(linear_acc(1,:)), length(element));
    % total = zeros(length(linear_acc(1,:)), length(element));

    %% Element-wise Calculation
    for j = 1:length(element)
        disp(['Added mass force calculation for element ' num2str(j) '/' num2str(length(element))])

        % Find the minimum length to avoid indexing issues
        N = min([length(element(j).linear_vel), length(element(j).linear_acc), ...
                 length(adjusted_psi_f), length(element(j).linear_vel_norm)]);

        for i = 1:N  % Loop for the minimum length of the arrays
            % Calculate each part of the force equation
            part1 = (rho * pi * ((c(j))^2) / 4) * del_r;
            %part2 = (dot(element(j).linear_vel(:,i), element(j).linear_acc(:,i)) * sind(abs(adjusted_psi_f(i)))) / element(j).linear_vel(1,i);
            part2 = (dot(element(j).linear_vel(:,i), element(j).linear_acc(:,i)) * sind(abs(adjusted_psi_f(i)))) / element(j).linear_vel_norm(i);
            %part3 = element(j).linear_vel(1,i) * deg2rad(adjusted_psi_dotf(i)) * cosd(abs(adjusted_psi_f(i)));
            part3 = element(j).linear_vel_norm(i) * deg2rad(adjusted_psi_dotf(i)) * cosd(abs(adjusted_psi_f(i)));
            
            

            % Calculate the total added mass force for this element and time step
            element(j).force_AddedMass(i) = part1 * (part2 + part3);

            % % % Store intermediate parts for plotting later
            part_1_total(i,j) = part2;
            part_2_total(i,j) = part2;
            part_3_total(i,j) = part2;
            total(i,j) = part1 * (part2 + part3);
        end
    end

    % % % Sum across elements for total forces at each time step
    part_1_total_total = sum(part_1_total, 2);
    part_2_total_total = sum(part_2_total, 2);
    part_3_total_total = sum(part_3_total, 2);
    total_total = sum(total, 2);
    % 
    % % Plot results
    figure;
    hold on;
    % plot(part_1_total_total, 'DisplayName', 'Part 1 Total');
    % plot(part_2_total_total, 'DisplayName', 'Part 2 Total');
    % plot(part_3_total_total, 'DisplayName', 'Part 3 Total');
    plot(total_total, 'DisplayName', 'Total');
    legend();
    hold off;

    disp('Added mass force calculated for entire wing')

    %% Entire wing Calculation
    AM_wing_force = zeros(1, length(element(1).force_AddedMass));

    for j = 1:length(element(1).force_AddedMass)
        AM_wing_temp = 0;

        for i = 1:length(element)
            AM_wing_temp = AM_wing_temp + element(i).force_AddedMass(j);
        end
        AM_wing_force(j) = AM_wing_temp;
    end
end
