function element = FindLinearVelocity(element, omega, alpha)
%% Preamble
% Finds the linear velocity and linear acceleration of each element throughout a full wing stroke

%% Starting message
disp('Linear Acceleration Calculation - Start')

%% Array sizing
N = length(alpha);  % Number of elements in alpha

%% Array initialization
v_linear = zeros(3, N);
v_linear_Norm = zeros(1, N);
v_linear_direction = zeros(1, N);
a_linear = zeros(3, N);
a_linear_Norm = zeros(1, N);
a_tangential = zeros(3, N);
a_centripetal = zeros(3, N);

%% Calculation
for j = 1:N
    for i = 1:length(element)
        % Position vector of the center of pressure (or any reference point)
        r = element(i).locationInMovingFrame(1:3,j);

        %% Linear Velocity
        % Velocity: omega x r
        v_linear = cross(omega(1:3,j), r');
        v_linear_Norm = norm(v_linear);

        % Direction is evaluated based on unit vector
        v_linear_direction = v_linear./v_linear_Norm;


        %% Linear Acceleration
        % Tangential acceleration: alpha x r
        a_tangential = cross(alpha(1:3,j), r');

        % Centripetal acceleration: omega x v_linear)
        a_centripetal = cross(omega(1:3,j), v_linear);

        % Total linear acceleration: tangential + centripetal
        a_linear = a_tangential + a_centripetal;
        a_linear_Norm = norm(a_linear);

        %% Store results in the element structure
        % Linear velocity and its norm
        element(i).linear_vel(:,j) = v_linear;
        element(i).linear_vel_norm(j) = v_linear_Norm;
        element(i).linear_vel_direction(:,j) = v_linear_direction;

        % Linear acceleration and its norm
        element(i).linear_acc(:,j)  = a_linear;
        element(i).linear_acc_norm(j) = a_linear_Norm;
    end
end

%% Starting message
disp('Linear Acceleration Calculation - End')

end