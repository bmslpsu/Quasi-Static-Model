function inertia = inertial_tensor(xy,z_thickness,rho)
% INERTIAL_TENSOR - Compute the inertial tensor of a wing
%
% Inputs:
%   XY - 2D wing hull coordinates with respect to the wing hinge, mm (n,3)
%   RHO - constant wing density, g/mm^3 (1,1)
%   Z_THICKNESS - constant wing thickness, mm (1,1)
%
% Ouputs:
%   INERTIA - inertial tensor, g-mm^2 (3,3)

% Allocate variables
Ixx = 0;
Iyy = 0;
Izz = 0;
Ixy = 0;
Ixz = 0;
Iyz = 0;
mass= 0;
n_pts = length(xy);
[x_com,y_com] = centroid(polyshape(xy(:,1),xy(:,2)));

% example: Ixx = \int y^2+z^2 dm
% we'll approximate this with a trapezoidal Riemann summation
for idx_1 = 1:n_pts
    % calculate the next index, but be sure to wrap around
    idx_2 = mod(idx_1,n_pts)+1;
    
    % get the current point, next point, and midpoint with respect to the
    % center of mass
    pt_1 = xy(idx_1,:) - [x_com,y_com];
    pt_2 = xy(idx_2,:) - [x_com,y_com];
    pt_12= (pt_2 - pt_1)./ 2;

    % calculate pt_0 (com) pt_1 pt_2 triangular area
    w = norm(pt_2-pt_1);
    h = norm(pt_12);
    A = 0.5*w*h;

    % calculate finite mass element dm
    dm = rho * z_thickness * A;

    % accumulate inertia (Trapezoidal Riemann Sum)
    x1 = pt_1(1); y1 = pt_1(2); z1 = 0;
    x2 = pt_2(1); y2 = pt_2(2); z2 = 0;
    Ixx = Ixx + 0.5*(y1^2 + z1^2 + y2^2 + z2^2) * dm;
    Iyy = Iyy + 0.5*(x1^2 + z1^2 + x2^2 + z2^2) * dm;
    Izz = Izz + (x1^2 + y1^2 + x2^2 + y2^2) * dm;
    Ixy = Ixy + (x1*y1 + x2*y2) * dm;
    Ixz = Ixz + (x1*z1 + x2*z2) * dm;
    Iyz = Iyz + (y1*z1 + y2*z2) * dm;
    
    % accumulate mass
    mass = mass + dm;
end

% Apply Parallel Axis Theorem to shift the frame from COM to wing joint. 
% We assume that the joint lies on the leftmost coordinate on the x-axis.
origin_idx = find(xy(:,1)==min(xy(:,1)), 1);
com_origin = [x_com,y_com] - xy(origin_idx,:);
x = com_origin(1); y = com_origin(2); z = 0;

Ixx = Ixx + mass*(y^2+z^2);
Iyy = Iyy + mass*(x^2+z^2);
Izz = Izz + mass*(x^2+y^2);
Ixy = Ixy + mass*(x*y);
Ixz = Ixz + mass*(x*z);
Iyz = Iyz + mass*(y*z);

% return inertia
inertia = [...
    Ixx, -Ixy,-Ixz;
    -Ixy, Iyy,-Iyz;
    -Ixz,-Iyz, Izz...
    ];

end