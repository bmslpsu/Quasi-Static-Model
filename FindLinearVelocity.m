function element = FindLinearVelocity(element, omega, alpha)

% Finds the linear velocity and linear acceleration of each element throughout a full wing stroke

% Truncate to match alpha size
N = length(alpha);  % Number of elements in alpha

% Initialize arrays for storing linear velocity and acceleration
v_linear = zeros(3, N);
v_linear_Norm = zeros(1, N);
a_linear = zeros(3, N);
a_linear_Norm = zeros(1, N);
a_tangential = zeros(3, N);
a_centripetal = zeros(3, N);

for j = 1:length(element)
    disp(['Calculating linear velocity and acceleration for element ' num2str(j) '/' num2str(length(element))])

    for i = 1:N-2
        % Position vector of the center of pressure (or any reference point)
        r = element(j).location_cop(1:3,i);

        %% Linear Velocity
        % Velocity: omega x r
        v_linear(1:3,i) = cross(deg2rad(omega(1:3,i)), r);
        v_linear_Norm(i) = norm(v_linear(1:3,i));
        %v_linear_Norm(i) =10000;

        %% Linear Acceleration
        % Tangential acceleration: alpha x r
        a_tangential(1:3,i) = cross(deg2rad(alpha(1:3,i)), r);

        % Centripetal acceleration: omega x v_linear)
        a_centripetal(1:3,i) = cross(deg2rad(omega(1:3,i)), v_linear(1:3,i));

        % Total linear acceleration: tangential + centripetal
        a_linear(1:3,i) = a_tangential(1:3,i) + a_centripetal(1:3,i);
        a_linear_Norm(i) = norm(a_linear(1:3,i));
    end

    %% Filtering
    [b, a] = butter(2, 0.2, 'low');

    v_linear_Norm = filtfilt(b, a, v_linear_Norm);
    a_linear_Norm = filtfilt(b, a, a_linear_Norm);


    %% Store results in the element structure
    % Linear velocity and its norm
    element(j).linear_vel = v_linear;
    element(j).linear_vel_norm = v_linear_Norm;

    % Linear acceleration and its norm
    element(j).linear_acc = a_linear;
    element(j).linear_acc_norm = a_linear_Norm;

end

disp('Done calculating linear velocity and acceleration for elements')

%% Plots
figure
hold on
plot(v_linear')
title("v linear")
 legend(["X" "Y" "Z"])
hold off

figure
hold on
plot(v_linear_Norm)
title("v linear norm")
hold off

% figure
% hold on
% plot(a_linear')
% title("a linear")
% hold off
% 
% figure
% hold on
% plot(a_linear_Norm)
% title("a linear norm")
% hold off

end