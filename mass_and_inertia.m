function [Fly]  = mass_and_inertia(Wing_Shape_lh, Wing_Shape_rh, Body_Shape, Fly)

%% Get standard data
[metrics, fly_body, fly_wing] = get_metrics();


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
%Calculate Mass, CG, Intertia, and surface area??

% Calculate area using the shoelace formula
n = length(Wing_Shape_lh.Wing_x);
x = Wing_Shape_lh.Wing_x;
y = Wing_Shape_lh.Wing_y;
z = Wing_Shape_lh.Wing_z;
z_thickness = fly_wing.thickness; 


% Calculate area
Area = polyarea(x, y);

% Calculate volume assuming uniform thickness
volume = Area * z_thickness;

mass = volume*fly_wing.density;

% Calculate center of gravity
% Use the same formula for centroid as in 2D case
sum_x = sum(x(1:end-1) .* y(2:end) - y(1:end-1) .* x(2:end));
sum_y = sum((x(1:end-1) + x(2:end)) .* (x(1:end-1) .* y(2:end) - x(2:end) .* y(1:end-1)));
X_CG = sum_x / (6 * Area);
Y_CG = sum_y / (6 * Area);
Z_CG = mean(z);   % need to add portion where z axis is respected
CG = [X_CG, Y_CG, Z_CG];

% Calculate inertia tensor
Ixx = (1/12) * fly_wing.density * z_thickness * sum(y.^2 + z.^2);
Iyy = (1/12) * fly_wing.density * z_thickness * sum(x.^2 + z.^2);
Izz = (1/12) * fly_wing.density * z_thickness * sum(x.^2 + y.^2);
Ixy = -(1/24) * fly_wing.density * z_thickness * sum(x .* y);
Ixz = -(1/24) * fly_wing.density * z_thickness * sum(x .* z);
Iyz = -(1/24) * fly_wing.density * z_thickness * sum(y .* z);
inertia = [...
    Ixx, Ixy, Ixz;...
    Ixy, Iyy, Iyz;...
    Ixz, Iyz, Izz];

%Place in wing_lh structure
Fly.wing_LH.area = Area;
Fly.wing_LH.volume = volume;
Fly.wing_LH.CG = CG;
Fly.wing_LH.mass = mass;
Fly.wing_LH.inertia = inertia;

%% Wing_RH
%Calculate Mass, CG, Intertia, and surface area??

% Calculate area using the shoelace formula
n = length(Wing_Shape_rh.Wing_x);
x = Wing_Shape_rh.Wing_x;
y = Wing_Shape_rh.Wing_y;
z = Wing_Shape_rh.Wing_z;
z_thickness = fly_wing.thickness; 

% Calculate area
Area = polyarea(x, y);

% Calculate volume assuming uniform thickness
volume = Area * z_thickness;

mass = volume*fly_wing.density;

% Calculate center of gravity
% Use the same formula for centroid as in 2D case
sum_x = -sum(x(1:end-1) .* y(2:end) - y(1:end-1) .* x(2:end));
sum_y = sum((x(1:end-1) + x(2:end)) .* (x(1:end-1) .* y(2:end) - x(2:end) .* y(1:end-1)));
X_CG = sum_x / (6 * Area);
Y_CG = sum_y / (6 * Area);
Z_CG = mean(z);   % need to add portion where z axis is respected
CG = [X_CG, Y_CG, Z_CG];

% Calculate inertia tensor
Ixx = (1/12) * fly_wing.density * z_thickness * sum(y.^2 + z.^2);
Iyy = (1/12) * fly_wing.density * z_thickness * sum(x.^2 + z.^2);
Izz = (1/12) * fly_wing.density * z_thickness * sum(x.^2 + y.^2);
Ixy = -(1/24) * fly_wing.density * z_thickness * sum(x .* abs(y));
Ixz = -(1/24) * fly_wing.density * z_thickness * sum(x .* z);
Iyz = -(1/24) * fly_wing.density * z_thickness * sum(abs(y) .* z);
inertia = [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz];

%Place in wing_rh structure
Fly.wing_RH.area = Area;
Fly.wing_RH.volume = volume;
Fly.wing_RH.CG = CG;
Fly.wing_RH.mass = mass;
Fly.wing_RH.inertia = inertia;


%% Define Joint Locations
% Head
% Abdomen
% Joint LH
% Joint RH

% % %% Mass Matrix
% % 
% % % Initialize matrices
% % Mb = zeros(6, 6);
% % MwL = zeros(6, 6);
% % MwR = zeros(6, 6);
% % 
% % % Compute Mb
% % Mb(1, 1) = body.mass;
% % Mb(1, 5) = body.mass * body.CG(3);
% % Mb(1, 6) = body.mass * body.CG(2) * -1;
% % Mb(2, 2) = body.mass;
% % Mb(2, 4) = body.mass * body.CG(3) * -1;
% % Mb(2, 6) = body.mass * body.CG(1);
% % Mb(3, 3) = body.mass;
% % Mb(3, 4) = body.mass * body.CG(2);
% % Mb(3, 5) = body.mass * body.CG(1) * -1;
% % Mb(4, 2) = body.mass * body.CG(3) * -1;
% % Mb(4, 3) = body.mass * body.CG(2);
% % Mb(4, 4) = body.inertia(1, 1);
% % Mb(4, 5) = body.inertia(1, 2);
% % Mb(4, 6) = body.inertia(1, 3);
% % Mb(5, 1) = body.mass * body.CG(3);
% % Mb(5, 3) = body.mass * body.CG(1) * -1;
% % Mb(5, 4) = body.inertia(2, 1);
% % Mb(5, 5) = body.inertia(2, 2);
% % Mb(5, 6) = body.inertia(2, 3);
% % Mb(6, 1) = body.mass * body.CG(2) * -1;
% % Mb(6, 2) = body.mass * body.CG(1);
% % Mb(6, 4) = body.inertia(3, 1);
% % Mb(6, 5) = body.inertia(3, 2);
% % Mb(6, 6) = body.inertia(3, 3);
% % 
% % body.mb = Mb;
% % 
% % % Compute MwL 
% % MwL(1, 1) = wing_lh.mass;
% % MwL(1, 5) = wing_lh.CG(3) * wing_lh.mass;
% % MwL(1, 6) = wing_lh.CG(2) * wing_lh.mass * -1;
% % MwL(2, 2) = wing_lh.mass;
% % MwL(2, 4) = wing_lh.CG(3) * wing_lh.mass * -1;
% % MwL(2, 6) = wing_lh.CG(1) * wing_lh.mass;
% % MwL(3, 3) = wing_lh.mass;
% % MwL(3, 4) = wing_lh.CG(2) * wing_lh.mass;
% % MwL(3, 5) = wing_lh.CG(1) * wing_lh.mass * -1;
% % MwL(4, 2) = wing_lh.CG(3) * wing_lh.mass * -1;
% % MwL(4, 3) = wing_lh.CG(2) * wing_lh.mass;
% % MwL(4, 4) = wing_lh.inertia(1, 1);
% % MwL(4, 5) = wing_lh.inertia(1, 2);
% % MwL(4, 6) = wing_lh.inertia(1, 3);
% % MwL(5, 1) = wing_lh.CG(3) * wing_lh.mass;
% % MwL(5, 3) = wing_lh.CG(1) * wing_lh.mass * -1;
% % MwL(5, 4) = wing_lh.inertia(2, 1);
% % MwL(5, 5) = wing_lh.inertia(2, 2);
% % MwL(5, 6) = wing_lh.inertia(2, 3);
% % MwL(6, 1) = wing_lh.CG(2) * wing_lh.mass * -1;
% % MwL(6, 2) = wing_lh.CG(1) * wing_lh.mass;
% % MwL(6, 4) = wing_lh.inertia(3, 1);
% % MwL(6, 5) = wing_lh.inertia(3, 2);
% % MwL(6, 6) = wing_lh.inertia(3, 3);
% % 
% % wing_lh.MwL = MwL;
% % 
% % % Compute MwR
% % MwR(1, 1) = wing_rh.mass;
% % MwR(1, 5) = wing_rh.CG(3) * wing_rh.mass;
% % MwR(1, 6) = wing_rh.CG(2) * wing_rh.mass * -1;
% % MwR(2, 2) = wing_rh.mass;
% % MwR(2, 4) = wing_rh.CG(3) * wing_rh.mass * -1;
% % MwR(2, 6) = wing_rh.CG(1) * wing_rh.mass;
% % MwR(3, 3) = wing_rh.mass;
% % MwR(3, 4) = wing_rh.CG(2) * wing_rh.mass;
% % MwR(3, 5) = wing_rh.CG(1) * wing_rh.mass * -1;
% % MwR(4, 2) = wing_rh.CG(3) * wing_rh.mass * -1;
% % MwR(4, 3) = wing_rh.CG(2) * wing_rh.mass;
% % MwR(4, 4) = wing_rh.inertia(1, 1);
% % MwR(4, 5) = wing_rh.inertia(1, 2);
% % MwR(4, 6) = wing_rh.inertia(1, 3);
% % MwR(5, 1) = wing_rh.CG(3) * wing_rh.mass;
% % MwR(5, 3) = wing_rh.CG(1) * wing_rh.mass * -1;
% % MwR(5, 4) = wing_rh.inertia(2, 1);
% % MwR(5, 5) = wing_rh.inertia(2, 2);
% % MwR(5, 6) = wing_rh.inertia(2, 3);
% % MwR(6, 1) = wing_rh.CG(2) * wing_rh.mass * -1;
% % MwR(6, 2) = wing_rh.CG(1) * wing_rh.mass;
% % MwR(6, 4) = wing_rh.inertia(3, 1);
% % MwR(6, 5) = wing_rh.inertia(3, 2);
% % MwR(6, 6) = wing_rh.inertia(3, 3);
% % 
% % wing_rh.MwR = MwR;

%% Fly Total
Fly.total.mass = Fly.body.mass + Fly.wing_LH.mass + Fly.wing_RH.mass;
Fly.total.weight = Fly.total.mass * metrics.gravity; %(g*mm/s^2)
Fly.total.CG = (Fly.body.mass*Fly.body.CG + Fly.wing_LH.mass*Fly.wing_LH.CG + Fly.wing_RH.mass*Fly.wing_RH.CG)/Fly.total.mass;

end