% Define the initial unit vectors (x, y, z axes)
unitX = [1 0 0];
unitY = [0 1 0];
unitZ = [0 0 1];

% Create a figure for the 3D motion and angles composite
figure;
subplot(1,2,1);  % Left: 3D path and rotation
hold on;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);

% Set plot limits to accommodate motion through space
xlim([-5 5]);
ylim([-5 5]);
zlim([-5 5]);

% Define a motion path for the origin to move through space (e.g., a spiral)
t = linspace(0, 10, 115);  % Time steps corresponding to R_inv2
path_x = cos(t);  % Example x-coordinate path
path_y = sin(t);  % Example y-coordinate path
path_z = linspace(0, 2, 115);  % Example z-coordinate path

% Initialize arrays to store the trace of the origin
trace_x = [];
trace_y = [];
trace_z = [];

% Prepare to store angles over time
stroke_angles = zeros(1, 115);
deviations = zeros(1, 115);
rotations = zeros(1, 115);

% Loop through 115 time steps of R_inv2
for i = 1:115
    % Access the i-th rotation matrix from R_inv2
    R = R_inv2(:,:,i);  % Assuming R_inv2(:,:,i) gives the rotation matrix at time step i

    % Rotate the unit vectors using R_inv2
    newUnitX = (R * unitX')';
    newUnitY = (R * unitY')';
    newUnitZ = (R * unitZ')';

    % Get the moving origin at time step i
    origin = [path_x(i), path_y(i), path_z(i)];

    % Append the current origin to the trace arrays
    trace_x = [trace_x, origin(1)];
    trace_y = [trace_y, origin(2)];
    trace_z = [trace_z, origin(3)];

    % Clear the current quivers
    cla;

    % Plot the trace of the origin's path
    plot3(trace_x, trace_y, trace_z, 'k-', 'LineWidth', 1.5);  % Traced path as a black line

    % Replot the rotated and translated vectors at the new origin
    quiver3(origin(1), origin(2), origin(3), newUnitX(1), newUnitX(2), newUnitX(3), 'r', 'LineWidth', 2);
    quiver3(origin(1), origin(2), origin(3), newUnitY(1), newUnitY(2), newUnitY(3), 'g', 'LineWidth', 2);
    quiver3(origin(1), origin(2), origin(3), newUnitZ(1), newUnitZ(2), newUnitZ(3), 'b', 'LineWidth', 2);

    % Extract stroke angle (φ), deviation (θ), and rotation (ψ) from R_inv2
    % Stroke angle φ: Angle in the horizontal XY plane (azimuth)
    stroke_angles(i) = atan2(newUnitX(2), newUnitX(1));  % φ = atan2(Y, X)

    % Deviation θ: Angle from the XY plane (elevation)
    deviations(i) = asin(newUnitX(3));  % θ = arcsin(Z)

    % Rotation ψ: Rotation around the Z-axis
    rotations(i) = atan2(newUnitZ(2), newUnitZ(1));  % ψ = atan2(Z_y, Z_x)

    % Set plot limits and axis properties to show movement through space
    xlim([-5 5]);
    ylim([-5 5]);
    zlim([-5 5]);

    % Pause to create animation effect
    pause(0.05);  % Adjust the pause duration for smoother animation
end

% Plot the composite of stroke angle, deviation, and rotation
subplot(1,2,2);  % Right: Composite plot of angles over time
hold on;
plot(t, rad2deg(stroke_angles), 'r', 'LineWidth', 1.5);  % Stroke angle φ in degrees
plot(t, rad2deg(deviations), 'g', 'LineWidth', 1.5);     % Deviation θ in degrees
plot(t, rad2deg(rotations), 'b', 'LineWidth', 1.5);      % Rotation ψ in degrees
xlabel('Time');
ylabel('Angle (degrees)');
legend('Stroke Angle (φ)', 'Deviation (θ)', 'Rotation (ψ)');
grid on;

hold off;
