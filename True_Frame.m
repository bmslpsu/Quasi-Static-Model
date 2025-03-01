function [Dynamics] = True_Frame(Dynamics)
% For the Body
% z-axis is up
% y-axis is to forward
% x-axis is to the side (right positive)

%Sideslip
Dynamics.Force_Lift(1, :)           = -Dynamics.Force_Lift(1, :);
Dynamics.Force_Drag(1, :)           = -Dynamics.Force_Drag(1, :);
Dynamics.Force_Rotation(1, :)       = -Dynamics.Force_Rotation(1, :);
Dynamics.Force_AM(1, :)             = -Dynamics.Force_AM(1, :);
Dynamics.Force_Total(1, :)          = -Dynamics.Force_Total(1, :);

%Roll
Dynamics.Torque_Lift(2, :)          = -Dynamics.Torque_Lift(2, :);
Dynamics.Torque_Drag(2, :)          = -Dynamics.Torque_Drag(2, :);
Dynamics.Torque_AM(2, :)            = -Dynamics.Torque_AM(2, :);
Dynamics.Torque_Rotation(2, :)      = -Dynamics.Torque_Rotation(2, :);
Dynamics.Torque_Inertia(2, :)       = -Dynamics.Torque_Inertia(2, :);
Dynamics.Torque_Total(2, :)         = -Dynamics.Torque_Total(2, :);

%Yaw
Dynamics.Torque_Lift(3, :)          = -Dynamics.Torque_Lift(3, :);
Dynamics.Torque_Drag(3, :)          = -Dynamics.Torque_Drag(3, :);
Dynamics.Torque_AM(3, :)            = -Dynamics.Torque_AM(3, :);
Dynamics.Torque_Rotation(3, :)      = -Dynamics.Torque_Rotation(3, :);
Dynamics.Torque_Inertia(3, :)       = -Dynamics.Torque_Inertia(3, :);
Dynamics.Torque_Total(3, :)         = -Dynamics.Torque_Total(3, :);

end
