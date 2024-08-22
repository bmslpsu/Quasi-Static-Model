function [element, AM_wing_force] = AddedMassForce(element, psi_dotf, psi_f, del_r, c, time, rho)
    % del_r (m)
    % c (m)

    %% Linear acceleration calculation
    for j = 1:length(element)
        
        % Calculate linear acceleration
        linear_acc = diff((element(j).linear_vel)')' / (time(2) - time(1));
        adjusted_psi_f = psi_f(2:end);

        % Butterworth filter for acceleration
        [b, a] = butter(4, 0.1, 'low');
        linear_acc_filt = filtfilt(b, a, linear_acc')';

        % Store the acceleration
        linear_acc_all(:,:,j) = linear_acc;
        linear_acc_filt_all(:,:,j) = linear_acc_filt;
       
    end


    %% Element wise Calcuation
    for j = 1:length(element)

        disp(['Added mass force calculation for element ' num2str(j) '/' num2str(length(element))])

        for i = 1:length(linear_acc(1,:))
            % split into parts
            part1 = -sign(linear_acc(i)) * (rho * pi * ((c(j))^2) / 4) * del_r;
            part2 = (dot(element(j).linear_vel(:,i), linear_acc_all(:,i,j)) * sind(abs(psi_f(i)))) / element(j).linear_vel_norm(i);
            part3 = element(j).linear_vel_norm(i) * ((psi_dotf(i))*pi/180) * cosd(abs(psi_f(i)));

            element(j).force_AddedMass(i) = part1 * (part2 + part3);

        end
    end

    disp('Added mass force calculated for entire wing')

    %% Entire wing Calcuation
    % Array initalization
    AM_wing_force = zeros(1, length(element(1).force_AddedMass));

    for j = 1:length(element(1).force_AddedMass)
        AM_wing_temp = 0;

        for i = 1:length(element)
            AM_wing_temp = AM_wing_temp + element(i).force_AddedMass(j);
        end
        AM_wing_force(j) = AM_wing_temp;
    end

    %% Plots
    % figure
    % hold on
    % plot(AM_wing_force)
    % title('Added Mass force for the whole wing throughout a stroke')
    % hold off

end