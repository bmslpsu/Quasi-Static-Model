function [Fly] = Dyn(Kinematics_LH, Kinematics_RH, Fly)
%% Variable Decleration
Wing_Element_LH = struct();
Wing_Element_RH = struct();

%% Find the Location of the Center of Pressure for each Wing Element
% Calculates the center of pressure of each wing element 
Wing_Element_LH = FindCOP(Kinematics_LH.psi, Fly.wing_LH.n, Fly.wing_LH.c, Kinematics_LH.R_inv2, Fly.wing_LH.wing_length);
Wing_Element_RH = FindCOP(Kinematics_RH.psi, Fly.wing_RH.n, Fly.wing_RH.c, Kinematics_RH.R_inv2, Fly.wing_RH.wing_length);

%% Find Linear Velocity of each Element for each Time Step
% Calculates the linear velocity of each element based on the magnitude of
% the angluar velocity
Wing_Element_LH = FindLinearVelocity(Wing_Element_LH, Kinematics_LH.omega, Kinematics_LH.alpha);
Wing_Element_RH = FindLinearVelocity(Wing_Element_RH, Kinematics_RH.omega, Kinematics_RH.alpha);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the Lift and Drag Forces Acting on Each Wing
[Wing_Element_LH, Fly.force_components.lh.Lift_force, Fly.force_components.lh.Drag_force, Fly.force_components.lh.Lift_torque, Fly.force_components.lh.Drag_torque] = Force_LiftAndDrag(Wing_Element_LH, Kinematics_LH, Fly.wing_LH.del_r, metrics.airDensity, Fly.wing_LH.c,  Fly.total.weight);
[Wing_Element_RH, Fly.force_components.rh.Lift_force, Fly.force_components.rh.Drag_force, Fly.force_components.rh.Lift_torque, Fly.force_components.rh.Drag_torque] = Force_LiftAndDrag(Wing_Element_RH, Kinematics_RH, Fly.wing_RH.del_r, metrics.airDensity, Fly.wing_RH.c,  Fly.total.weight);

%% Find the Rotational Force acting on Each Wing
[Wing_Element_LH, Fly.force_components.lh.Rot_force, Fly.force_components.lh.Rot_torque] = Force_Rotational(Wing_Element_LH, Kinematics_LH, Fly.wing_LH.del_r, Fly.wing_LH.c, metrics.airDensity, Fly.total.weight);
[Wing_Element_RH, Fly.force_components.rh.Rot_force, Fly.force_components.rh.Rot_torque] = Force_Rotational(Wing_Element_RH, Kinematics_RH, Fly.wing_RH.del_r, Fly.wing_RH.c, metrics.airDensity, Fly.total.weight);

%% Find Added Mass Force acting on Each Wing
[Wing_Element_LH, Fly.force_components.lh.AM_force, Fly.force_components.lh.AM_torque] = Force_AddedMass(Wing_Element_LH,  Kinematics_LH, Fly.wing_LH.del_r, Fly.wing_LH.c, metrics.airDensity, Fly.total.weight);
[Wing_Element_RH, Fly.force_components.rh.AM_force, Fly.force_components.rh.AM_torque] = Force_AddedMass(Wing_Element_RH,  Kinematics_RH, Fly.wing_RH.del_r, Fly.wing_RH.c, metrics.airDensity, Fly.total.weight);

%% Find Force Directions
Fly.force_total.Force_Body_LH = FindForceVectors(Fly.force_components.lh, Kinematics_LH.R_inv2);
Fly.force_total.Force_Body_RH = FindForceVectors(Fly.force_components.rh, Kinematics_RH.R_inv2);

%% Torque due to Inertia
[Wing_Element_LH, Fly.force_components.lh.Inertia_torque] = Torque_Inertia(Wing_Element_LH,  Kinematics_LH, Fly.wing_LH.inertia);
[Wing_Element_RH, Fly.force_components.rh.Inertia_torque] = Torque_Inertia(Wing_Element_RH,  Kinematics_RH, Fly.wing_RH.inertia);

%% Torque due to Force offset
Fly.force_total.Force_Body_LH.torque_forces_vec = Torque_Forces(Fly, Wing_Element_LH, Fly.wing_LH, Kinematics_LH.R_inv2, Fly.force_total.Force_Body_LH,1, fly_num);
Fly.force_total.Force_Body_RH.torque_forces_vec = Torque_Forces(Fly, Wing_Element_RH, Fly.wing_RH, Kinematics_RH.R_inv2, Fly.force_total.Force_Body_RH,2, fly_num);

%% Find Torque Directions
Fly.force_total.Force_Body_LH = FindTorqueVectors(Fly.force_components.lh, Kinematics_LH.R_inv2, Fly.force_total.Force_Body_LH);
Fly.force_total.Force_Body_RH = FindTorqueVectors(Fly.force_components.rh, Kinematics_RH.R_inv2, Fly.force_total.Force_Body_RH);

end