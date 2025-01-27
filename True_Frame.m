function [Dynamics] = True_Frame(Dynamics);
% For the Body
% z-axis is up
% y-axis is to forward
% x-axis is to the side (right positive)

%Sideslip
Dynamics.Force_Lift(3, :)       = -Dynamics.Force_Lift(3, :);
Dynamics.Force_Drag(3, :)       = -Dynamics.Force_Drag(3, :);
Dynamics.Force_Rotation(3, :)   = -Dynamics.Force_Rotation(3, :);
Dynamics.Force_AM(3, :)         = -Dynamics.Force_AM(3, :);
Dynamics.Force_Total(3, :)      = -Dynamics.Force_Total(3, :);

%Roll
Dynamics.Torque_Lift(1, :)            = -Dynamics.Torque_Lift(1, :);
Dynamics.Torque_Drag(1, :)            = -Dynamics.Torque_Drag(1, :);
Dynamics.Torque_AM(1, :)              = -Dynamics.Torque_AM(1, :);
Dynamics.Torque_Rotation(1, :)        = -Dynamics.Torque_Rotation(1, :);
Dynamics.Torque_Inertia(1, :)         = -Dynamics.Torque_Inertia(1, :);
Dynamics.Torque_Due_to_Forces(1, :)   = -Dynamics.Torque_Due_to_Forces(1, :);
Dynamics.Torque_Total(1, :)           = -Dynamics.Torque_Total(1, :);

%Yaw
Dynamics.Torque_Lift(2, :)            = -Dynamics.Torque_Lift(2, :);
Dynamics.Torque_Drag(2, :)            = -Dynamics.Torque_Drag(2, :);
Dynamics.Torque_AM(2, :)              = -Dynamics.Torque_AM(2, :);
Dynamics.Torque_Rotation(2, :)        = -Dynamics.Torque_Rotation(2, :);
Dynamics.Torque_Inertia(2, :)         = -Dynamics.Torque_Inertia(2, :);
Dynamics.Torque_Due_to_Forces(2, :)   = -Dynamics.Torque_Due_to_Forces(2, :);
Dynamics.Torque_Total(2, :)           = -Dynamics.Torque_Total(2, :);

end
