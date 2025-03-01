function element = Kin_Linear(Kinematics, element)
%% Preamble
% Finds the linear velocity and linear acceleration of each element
% throughout a full wing stroke.
%
% Inputs:
%   Kinematics -    Structure containing wing kinematic properties
%   element    -    Structure containing wing element data
%
% Outputs:
%   element - Struct array containing:
%             - linear velocity
%             - linear acceleration

%% 1. Precomputations

% Required Kinematic Parameters
omega   = Kinematics.omega;
alpha   = Kinematics.alpha;
N       = Kinematics.N;

%% 2. Calculation
% Loop through each time step
for j = 1:N
    % Loop through each wing element
    for i = 1:length(element)

        % Position vector of the center of pressure
        r = element(i).COP_Wing_Frame(:, j);

        %% 2.1 Linear Velocity
        % Compute linear velocity using cross product: v = omega × r
        v_linear = cross(omega(:, j), r(:));

        % Compute linear velocity magnitude
        v_linear_Norm = vecnorm(v_linear);

        %% 2.2 Linear Acceleration
        % Compute tangential acceleration: a_tangential = alpha × r
        a_tangential    = cross(alpha(:, j), r(:));

        % Compute centripetal acceleration: a_centripetal = omega × v_linear
        a_centripetal   = cross(omega(:, j), v_linear);

        % Compute total linear acceleration: a_linear = a_tangential + a_centripetal
        a_linear        = a_tangential + a_centripetal;
        a_linear_Norm   = vecnorm(a_linear);

        %% 2.3 Store results in the element structure
        % Store linear velocity and its magnitude
        element(i).linear_vel(:, j)                     = v_linear;
        element(i).linear_vel_norm(j)                   = v_linear_Norm;

        % Store linear acceleration and its magnitude
        element(i).linear_acc(:, j)                     = a_linear;
        element(i).linear_acc_norm(j)                   = a_linear_Norm;
    end
end
end

