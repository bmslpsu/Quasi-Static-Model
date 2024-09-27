% Define the initial position of the wing tip (adjust as needed)
initial_wing_tip = [1; 0; 0];  % Assume the wing tip starts at (1, 0, 0)

% Your 3x3x115 matrix data (replace 'your_rotation_matrices' with your actual variable name)
rotation_matrices = R_inv2;  % This should be your 3x3x115 matrix representing rotations

% Initialize a matrix to store the wing tip positions over time
wing_tip_positions = zeros(115, 3);

% Loop over time steps to calculate the wing tip position at each time step
for t = 1:115
    % Extract the rotation matrix at time step t
    R = rotation_matrices(:,:,t);  % 3x3 rotation matrix at time t
    
    % Calculate the new wing tip position by applying the rotation matrix
    wing_tip_positions(t, :) = (R * initial_wing_tip)';  % Transpose to store as row
end

% Plot the wing tip trajectory in 3D
figure;
plot3(wing_tip_positions(:, 1), wing_tip_positions(:, 2), wing_tip_positions(:, 3), '-o');
grid on;
xlabel('X Position');
ylabel('Y Position');
zlabel('Z Position');
title('Wing Tip Movement Over Time');
