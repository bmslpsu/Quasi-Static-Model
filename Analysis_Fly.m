function [Fly] = Analysis_Fly(LH_Chord_Cut, LH_Span_Cut, RH_Chord_Cut, RH_Span_Cut, LH_Stroke_Amplitude, RH_Stroke_Amplitude, fly_num, FilteredAngleL, FilteredAngleR, period) 
%% Note on the Axese

% For the wing
% x-axis is along the length of the wing (Chord to Tip)
% y-axis is perpendicular to the surface of the wing
% z-axis is along the chord of the wing starting and is parallel to the abdomen of the fly

% For the body
% z-axis is up
% y-axis is forward
% x-axis is the side (right positive)

%% Runtime
tic

%% Sig Figs
digits(4); % sets decimal point accuracy

%% Standard Constants
[metrics, ~, ~] = get_metrics();

%% Variable Decleration
%Creates structures to manage data throughout the program
Wing_Shape_LH = struct();
Wing_Shape_RH = struct();
Body_Shape = struct();
Fly = struct();


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Time Set Up
%Store time in Fly structure
Fly.time = period;
dt=.000125;

%% Kinematic data
Kinematics_LH = Kin(FilteredAngleL(period,2), FilteredAngleL(period,1).*LH_Stroke_Amplitude/100, FilteredAngleL(period,3), 0, dt);
Kinematics_RH = Kin(FilteredAngleR(period,2), FilteredAngleR(period,1).*RH_Stroke_Amplitude/100, FilteredAngleR(period,3), 0, dt);
Fly.Kinematics_LH = Kinematics_LH;
Fly.Kinematics_RH = Kinematics_RH;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Wing and Boddy Selection

% LH and RH wing uploader
[Fly.wing_LH.wing_shape, Fly.wing_RH.wing_shape, Fly.body.body_shape, Fly.body.Joint] = wingPlotGUI(Wing_Shape_LH, Wing_Shape_RH, Body_Shape,true,LH_Span_Cut,LH_Chord_Cut,RH_Span_Cut,RH_Chord_Cut);

% Body and Wing physical Analyis
[Fly] = mass_and_inertia(Fly.wing_LH.wing_shape,Fly.wing_RH.wing_shape, Fly.body.body_shape, Fly);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the Location of the Center of Pressure for each Wing Element
% Calculates the center of pressure of each wing element 
Wing_Element_LH = FindCOP(Kinematics_LH.psi, Fly.wing_LH.n, Fly.wing_LH.c, Kinematics_LH.R_inv2, Fly.wing_LH.wing_length, Fly.wing_LH.wing_shape);
Wing_Element_RH = FindCOP(Kinematics_RH.psi, Fly.wing_RH.n, Fly.wing_RH.c, Kinematics_RH.R_inv2, Fly.wing_RH.wing_length, Fly.wing_RH.wing_shape);

%% Find Linear Velocity of each Element for each Time Step
% Calculates the linear velocity of each element based on the magnitude of
% the angluar velocity
Wing_Element_LH = FindLinearVelocity(Wing_Element_LH, Kinematics_LH.omega, Kinematics_LH.alpha);
Wing_Element_RH = FindLinearVelocity(Wing_Element_RH, Kinematics_RH.omega, Kinematics_RH.alpha);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the Lift, Drag, Rotation, and Added Mass Forces Acting on Each Wing
[Wing_Element_LH, Fly.force_components.lh] = Force_Components(Wing_Element_LH, Kinematics_LH, Fly.wing_LH.del_r, metrics.airDensity, Fly.wing_LH.c);
[Wing_Element_RH, Fly.force_components.rh] = Force_Components(Wing_Element_RH, Kinematics_RH, Fly.wing_RH.del_r, metrics.airDensity, Fly.wing_RH.c);

%% Torque due to Inertia
[Wing_Element_LH, Fly.force_components.lh.Inertia_torque] = Torque_Inertia(Wing_Element_LH,  Kinematics_LH, Fly.wing_LH.inertia, Kinematics_LH.R_inv2);
[Wing_Element_RH, Fly.force_components.rh.Inertia_torque] = Torque_Inertia(Wing_Element_RH,  Kinematics_RH, Fly.wing_RH.inertia, Kinematics_RH.R_inv2);

%% Find Force Directions
ang_wing_plane = 0;
Fly.force_total.Force_Body_LH = FindForceVectors(Fly.force_components.lh, Kinematics_LH.R_inv2, ang_wing_plane);
Fly.force_total.Force_Body_RH = FindForceVectors(Fly.force_components.rh, Kinematics_RH.R_inv2, ang_wing_plane);

%% Torque due to Force offset
Fly.force_total.Force_Body_LH.torque_forces_vec = Torque_Forces(Fly, Wing_Element_LH, Kinematics_LH.R_inv2, Fly.force_total.Force_Body_LH);
Fly.force_total.Force_Body_RH.torque_forces_vec = Torque_Forces(Fly, Wing_Element_RH, Kinematics_RH.R_inv2, Fly.force_total.Force_Body_RH);

%% Find Torque Directions
Fly.force_total.Force_Body_LH = FindTorqueVectors(Fly.force_components.lh, Kinematics_LH.R_inv2, Fly.force_total.Force_Body_LH);
Fly.force_total.Force_Body_RH = FindTorqueVectors(Fly.force_components.rh, Kinematics_RH.R_inv2, Fly.force_total.Force_Body_RH);

%% End of Code Timer
toc 
end