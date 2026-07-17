function [Fly]  = mass_and_inertia(Wing_Shape_lh, Wing_Shape_rh, Body_Shape, Fly)
import Utils.get_metrics

%% Get standard data
[metrics, fly_Body, fly_wing] = get_metrics();

% Wing properties
rho = fly_wing.density;      % Density (kg/m^3)

%% Head
%Calculate Mass, CG, Intertia, and surface area

fly_Body.head_x = Body_Shape.Body_x(15:21,:);
fly_Body.head_y = Body_Shape.Body_y(15:21,:);
fly_Body.head_z = Body_Shape.Body_z(15:21,:);

% Create a matrix of the points


points = [fly_Body.head_x(:), fly_Body.head_y(:), fly_Body.head_z(:)];

% Create the Delaunay triangulation
dt = delaunayTriangulation(points);

% Get the convex hull
[~, volume] = convexHull(dt);

mass = volume*fly_Body.density;

CG = [(max(fly_Body.head_x(:)) + min(fly_Body.head_x(:)))/2, (max(fly_Body.head_y(:)) + min(fly_Body.head_y(:)))/2, (max(fly_Body.head_z(:)) + min(fly_Body.head_z(:)))/2];

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

fly_Body.thorax_x = Body_Shape.Body_x(10:15,:);
fly_Body.thorax_y = Body_Shape.Body_y(10:15,:);
fly_Body.thorax_z = Body_Shape.Body_z(10:15,:);

% Create a matrix of the points
points = [fly_Body.thorax_x(:), fly_Body.thorax_y(:), fly_Body.thorax_z(:)];

% Create the Delaunay triangulation
dt = delaunayTriangulation(points);

% Get the convex hull
[~, volume] = convexHull(dt);

mass = volume*fly_Body.density; % kg

CG = [(max(fly_Body.thorax_x(:)) + min(fly_Body.thorax_x(:)))/2, (max(fly_Body.thorax_y(:)) + min(fly_Body.thorax_y(:)))/2, (max(fly_Body.thorax_z(:)) + min(fly_Body.thorax_z(:)))/2];

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

fly_Body.abdomen_x = Body_Shape.Body_x(1:10,:);
fly_Body.abdomen_y = Body_Shape.Body_y(1:10,:);
fly_Body.abdomen_z = Body_Shape.Body_z(1:10,:);


% Create a matrix of the points
points = [fly_Body.abdomen_x(:), fly_Body.abdomen_y(:), fly_Body.abdomen_z(:)];

% Create the Delaunay triangulation
dt = delaunayTriangulation(points);

% Get the convex hull
[~, volume] = convexHull(dt);

mass = volume*fly_Body.density;

CG = [(max(fly_Body.abdomen_x(:)) + min(fly_Body.abdomen_x(:)))/2, (max(fly_Body.abdomen_y(:)) + min(fly_Body.abdomen_y(:)))/2, (max(fly_Body.abdomen_z(:)) + min(fly_Body.abdomen_z(:)))/2];

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

Fly.Body.volume = head.volume + thorax.volume + abdomen.volume;
Fly.Body.mass = head.mass + thorax.mass + abdomen.mass;
Fly.Body.CG = (head.mass*head.CG + thorax.mass*thorax.CG + abdomen.mass*abdomen.CG) / Fly.Body.mass;
Fly.Body.inertia = head.inertia  + thorax.inertia  + abdomen.inertia;

%% Wing_LH
%Calculate Mass, CG, Intertia, and surface area

Fly.Wing_LH = wing_values(Wing_Shape_lh.Wing_x, Wing_Shape_lh.Wing_y, Wing_Shape_lh.Wing_z, fly_wing.thickness, Fly.Wing_LH, rho, Wing_Shape_lh.Wing_tip_index, Wing_Shape_lh.Wing_root_index);

%% Wing_RH
%Calculate Mass, CG, Intertia, and surface area

Fly.Wing_RH = wing_values(Wing_Shape_rh.Wing_x, Wing_Shape_rh.Wing_y, Wing_Shape_rh.Wing_z, fly_wing.thickness, Fly.Wing_RH, rho, Wing_Shape_rh.Wing_tip_index, Wing_Shape_rh.Wing_root_index);

%% Fly Total
Fly.total.mass = Fly.Body.mass + Fly.Wing_LH.mass + Fly.Wing_RH.mass;
Fly.total.weight = Fly.total.mass * metrics.gravity; %(g*mm/s^2)
Fly.total.CG = (Fly.Body.mass*Fly.Body.CG + Fly.Wing_LH.mass*Fly.Wing_LH.CG + Fly.Wing_RH.mass*Fly.Wing_RH.CG)/Fly.total.mass;

% Calculates the 3rd moment of area ratio
Fly.total.S_3_Ratio = Fly.Wing_LH.S_3/Fly.Wing_RH.S_3;

% Calculates the 2nd moment of area ratio
Fly.total.S_2_Ratio = Fly.Wing_LH.S_2/Fly.Wing_RH.S_2;

end

function Fly_vars = wing_values(x,y,z,z_thickness, Fly_vars, rho, tip_index, root_index)
import Utils.inertial_tensor

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

% Mass of the plate
mass = volume*rho;

inertia = inertial_tensor([x',y'],z_thickness,rho);

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