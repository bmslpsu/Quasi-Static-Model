function [Fly]  = Phsycial_Characteristics(Wing_Shape_lh, Wing_Shape_rh, Body_Shape, Fly)

%% Get standard data
[metrics, fly_body, fly_wing] = get_metrics();

% Wing properties
rho = fly_wing.density;      % Density (kg/m^3)

%% Head
%Calculate Mass, CG, Intertia, and surface area

fly_body.head_x = Body_Shape.Body_x(15:21,:);
fly_body.head_y = Body_Shape.Body_y(15:21,:);
fly_body.head_z = Body_Shape.Body_z(15:21,:);

% Create a matrix of the points


points = [fly_body.head_x(:), fly_body.head_y(:), fly_body.head_z(:)];

% Create the Delaunay triangulation
dt = delaunayTriangulation(points);

% Get the convex hull
[~, volume] = convexHull(dt);

mass = volume*fly_body.density;

CG = [(max(fly_body.head_x(:)) + min(fly_body.head_x(:)))/2, (max(fly_body.head_y(:)) + min(fly_body.head_y(:)))/2, (max(fly_body.head_z(:)) + min(fly_body.head_z(:)))/2];

% Calculate inertia tensor for a convex hull of points
points_centered = points - CG;
Ixx = sum(points_centered(:, 2).^2 + points_centered(:, 3).^2);
Iyy = sum(points_centered(:, 1).^2 + points_centered(:, 3).^2);
Izz = sum(points_centered(:, 1).^2 + points_centered(:, 2).^2);
Ixy = -sum(points_centered(:, 1) .* points_centered(:, 2));
Ixz = -sum(points_centered(:, 1) .* points_centered(:, 3));
Iyz = -sum(points_centered(:, 2) .* points_centered(:, 3));
inertia = mass * [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz];

%Place in head structure
head.volume = volume;
head.mass = mass;
head.CG = CG;
head.inertia = inertia;


%% Thorax
%Calculate Mass, CG, Intertia, and surface area

fly_body.thorax_x = Body_Shape.Body_x(10:15,:);
fly_body.thorax_y = Body_Shape.Body_y(10:15,:);
fly_body.thorax_z = Body_Shape.Body_z(10:15,:);

% Create a matrix of the points
points = [fly_body.thorax_x(:), fly_body.thorax_y(:), fly_body.thorax_z(:)];

% Create the Delaunay triangulation
dt = delaunayTriangulation(points);

% Get the convex hull
[~, volume] = convexHull(dt);

mass = volume*fly_body.density;

CG = [(max(fly_body.thorax_x(:)) + min(fly_body.thorax_x(:)))/2, (max(fly_body.thorax_y(:)) + min(fly_body.thorax_y(:)))/2, (max(fly_body.thorax_z(:)) + min(fly_body.thorax_z(:)))/2];

% Calculate inertia tensor for a convex hull of points
points_centered = points - CG;
Ixx = sum(points_centered(:, 2).^2 + points_centered(:, 3).^2);
Iyy = sum(points_centered(:, 1).^2 + points_centered(:, 3).^2);
Izz = sum(points_centered(:, 1).^2 + points_centered(:, 2).^2);
Ixy = -sum(points_centered(:, 1) .* points_centered(:, 2));
Ixz = -sum(points_centered(:, 1) .* points_centered(:, 3));
Iyz = -sum(points_centered(:, 2) .* points_centered(:, 3));
inertia = mass * [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz];

%Place in thorax structure
thorax.volume = volume;
thorax.mass = mass;
thorax.CG = CG;
thorax.inertia = inertia;

%% Abdomen
%Calculate Mass, CG, Intertia, and surface area

fly_body.abdomen_x = Body_Shape.Body_x(1:10,:);
fly_body.abdomen_y = Body_Shape.Body_y(1:10,:);
fly_body.abdomen_z = Body_Shape.Body_z(1:10,:);


% Create a matrix of the points
points = [fly_body.abdomen_x(:), fly_body.abdomen_y(:), fly_body.abdomen_z(:)];

% Create the Delaunay triangulation
dt = delaunayTriangulation(points);

% Get the convex hull
[~, volume] = convexHull(dt);

mass = volume*fly_body.density;

CG = [(max(fly_body.abdomen_x(:)) + min(fly_body.abdomen_x(:)))/2, (max(fly_body.abdomen_y(:)) + min(fly_body.abdomen_y(:)))/2, (max(fly_body.abdomen_z(:)) + min(fly_body.abdomen_z(:)))/2];

% Calculate inertia tensor for a convex hull of points
points_centered = points - CG;
Ixx = sum(points_centered(:, 2).^2 + points_centered(:, 3).^2);
Iyy = sum(points_centered(:, 1).^2 + points_centered(:, 3).^2);
Izz = sum(points_centered(:, 1).^2 + points_centered(:, 2).^2);
Ixy = -sum(points_centered(:, 1) .* points_centered(:, 2));
Ixz = -sum(points_centered(:, 1) .* points_centered(:, 3));
Iyz = -sum(points_centered(:, 2) .* points_centered(:, 3));
inertia = mass * [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz];

%Place in abdomen structure
abdomen.volume = volume;
abdomen.mass = mass;
abdomen.CG = CG;
abdomen.inertia = inertia;

%% Total Body
% Sum head, thorax, and abdomen

Fly.body.volume = head.volume + thorax.volume + abdomen.volume;
Fly.body.mass = head.mass + thorax.mass + abdomen.mass;
Fly.body.CG = (head.mass*head.CG + thorax.mass*thorax.CG + abdomen.mass*abdomen.CG) / Fly.body.mass;
Fly.body.inertia = head.inertia  + thorax.inertia  + abdomen.inertia;

%% Wing_LH
%Calculate Mass, CG, Intertia, and surface area

Fly.wing_LH = wing_values(Wing_Shape_lh.Wing_x, Wing_Shape_lh.Wing_y, Wing_Shape_lh.Wing_z, fly_wing.thickness, Fly.wing_LH, rho, Wing_Shape_lh.Wing_tip_index, Wing_Shape_lh.Wing_root_index);

%% Wing_RH
%Calculate Mass, CG, Intertia, and surface area

Fly.wing_RH = wing_values(Wing_Shape_rh.Wing_x, Wing_Shape_rh.Wing_y, Wing_Shape_rh.Wing_z, fly_wing.thickness, Fly.wing_RH, rho, Wing_Shape_rh.Wing_tip_index, Wing_Shape_rh.Wing_root_index);

%% Fly Total
Fly.total.mass = Fly.body.mass + Fly.wing_LH.mass + Fly.wing_RH.mass;
Fly.total.weight = Fly.total.mass * metrics.gravity; %(g*mm/s^2)
Fly.total.CG = (Fly.body.mass*Fly.body.CG + Fly.wing_LH.mass*Fly.wing_LH.CG + Fly.wing_RH.mass*Fly.wing_RH.CG)/Fly.total.mass;

% Calculates the 3rd moment of area ratio
Fly.total.S_3_Ratio = Fly.wing_LH.S_3/Fly.wing_RH.S_3;

% Calculates the 2nd moment of area ratio
Fly.total.S_2_Ratio = Fly.wing_LH.S_2/Fly.wing_RH.S_2;

end

function Fly_vars = wing_values(x,y,z,z_thickness, Fly_vars, rho, tip_index, root_index)
% Calculate area
Area = polyarea(x, y);

% Calculate volume assuming uniform thickness
volume = Area * z_thickness;

% Calculate center of gravity
% Use the same formula for centroid as in 2D case
sum_x = sum(x(1:end-1) .* y(2:end) - y(1:end-1) .* x(2:end));
sum_y = sum((x(1:end-1) + x(2:end)) .* (x(1:end-1) .* y(2:end) - x(2:end) .* y(1:end-1)));
X_CG = sum_x / (6 * Area);
Y_CG = sum_y / (6 * Area);
Z_CG = mean(z);   % need to add portion where z axis is respected
CG = [X_CG, Y_CG, Z_CG];

% Step 1: Calculate the CoM
x_com = sum(x) / length(x); % x-coordinate of the CoM
y_com = sum(y) / length(y); % y-coordinate of the CoM
z_com = sum(z) / length(z); % z-coordinate of the CoM

% Shift coordinates to the CoM frame
x_shifted = x - x_com;
y_shifted = y - y_com;
z_shifted = 0;

% Step 2: Calculate moments of inertia in the CoM frame
Ixx_com = rho * z_thickness * sum(y_shifted.^2 + z_shifted.^2); % Moment about x-axis
Iyy_com = rho * z_thickness * sum(x_shifted.^2 + z_shifted.^2); % Moment about y-axis
Izz_com = rho * z_thickness * sum(x_shifted.^2 + y_shifted.^2); % Moment about z-axis

% Step 3: Calculate products of inertia in the CoM frame
Ixy_com = -rho * z_thickness * sum(x_shifted .* y_shifted); % Product of inertia xy
Ixz_com = -rho * z_thickness * sum(x_shifted .* z_shifted); % Product of inertia xz
Iyz_com = -rho * z_thickness * sum(y_shifted .* z_shifted); % Product of inertia yz

% Step 4: Parallel axis theorem (if needed)
% Mass of the plate
mass = volume*rho;

% Distances from the original frame to the CoM frame
dx = min(x_shifted);
dxi = find(x_shifted==min(x_shifted));
dy = y(dxi(1));
dz = 0;

% Apply the parallel axis theorem
Ixx = Ixx_com + mass * (dy^2 + dz^2);
Iyy = Iyy_com + mass * (dx^2 + dz^2);
Izz = Izz_com + mass * (dx^2 + dy^2);

% Products of inertia (unchanged by parallel axis theorem for CoM offset)
Ixy = Ixy_com - mass * dx * dy;
Ixz = Ixz_com - mass * dx * dz;
Iyz = Iyz_com - mass * dy * dz;

inertia = [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz];

% Wing Length
x_positions = x(tip_index:root_index);
wing_length = abs(x_positions(end) - x_positions(1));

% Chord Length
y_positions_1 = y(tip_index:root_index);
y_positions_2 = y(end:-1:root_index);
c = abs(y_positions_2 - y_positions_1); % Chord lengths in mm
%I believe that if the wing is cut, the chord doesn't actually change
%but this only impacts cut wings

% Quasi-Steady Steps
n = length(x_positions) - 1;

% Step size
del_r = wing_length/n;

% Calculate the second moment of area (S_2)
S_2 = 0;
for i = 1:n
    r = (i-1) * del_r;
    S_2 = S_2 + c(i) * r^2 * del_r;
end

% Calculate the third moment of area (S_3)
S_3 = 0;
for i = 1:n
    r = (i-1) * del_r;
    S_3 = S_3 + c(i) * r^3 * del_r;
end

%Place in wing_lh structure
Fly_vars.area = Area;
Fly_vars.volume = volume;
Fly_vars.CG = CG;
Fly_vars.mass = mass;
Fly_vars.inertia = inertia;
Fly_vars.c = c;
Fly_vars.n = n;
Fly_vars.wing_length = wing_length;
Fly_vars.del_r = del_r;
Fly_vars.S_2 = S_2;
Fly_vars.S_3 = S_3;

end