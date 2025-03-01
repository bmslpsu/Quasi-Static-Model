function [Fly] = Analysis_Fly(LH_Chord_Cut, LH_Span_Cut, RH_Chord_Cut, RH_Span_Cut, LH_Stroke_Amplitude, RH_Stroke_Amplitude, fly_num, FilteredAngleL, FilteredAngleR, period, ang_wing_plane_LH, ang_wing_plane_RH) 
%% Note on the Axese

% For the wing
% x-axis is along the length of the wing (Chord to Tip)
% y-axis is perpendicular to the surface of the wing
% z-axis is along the chord of the wing starting and is parallel to the abdomen of the fly

% For the body
% z-axis is up (2)
% y-axis is forward (3)
% x-axis is the side (right positive) (1)

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
Kinematics.LH = Kin(FilteredAngleL(period,2), FilteredAngleL(period,1).*LH_Stroke_Amplitude/100, FilteredAngleL(period,3), 0, dt);
Kinematics.RH = Kin(FilteredAngleR(period,2), FilteredAngleR(period,1).*RH_Stroke_Amplitude/100, FilteredAngleR(period,3), 0, dt);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Wing and Boddy Selection

% LH and RH wing uploader
[Morphology.Wing_LH.wing_shape, Morphology.Wing_RH.wing_shape, Morphology.Body.Morphology.Body_shape, Morphology.Body.Joint] = wingPlotGUI(Wing_Shape_LH, Wing_Shape_RH, Body_Shape,true,LH_Span_Cut,LH_Chord_Cut,RH_Span_Cut,RH_Chord_Cut);

% Body and Wing physical Analyis
[Morphology] = mass_and_inertia(Morphology.Wing_LH.wing_shape,Morphology.Wing_RH.wing_shape, Morphology.Body.Morphology.Body_shape, Morphology);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the Location of the Center of Pressure for each Wing Element
% Calculates the center of pressure of each wing element 
Wing_Element_LH = Center_of_Pressure(Kinematics.LH.psi, Morphology.Wing_LH.n, Morphology.Wing_LH.c, Kinematics.LH.R_inv2, Morphology.Wing_LH.wing_length, Morphology.Wing_LH.wing_shape);
Wing_Element_RH = Center_of_Pressure(Kinematics.RH.psi, Morphology.Wing_RH.n, Morphology.Wing_RH.c, Kinematics.RH.R_inv2, Morphology.Wing_RH.wing_length, Morphology.Wing_RH.wing_shape);

%% Find the Linear Velocity of each Element for each Time Step
% Calculates the linear velocity of each element based on the magnitude of
% the angluar velocity
Wing_Element_LH = Linear_Kinematics(Wing_Element_LH, Kinematics.LH.omega, Kinematics.LH.alpha);
Wing_Element_RH = Linear_Kinematics(Wing_Element_RH, Kinematics.RH.omega, Kinematics.RH.alpha);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the Lift, Drag, Rotation, and Added Mass Forces and Torques Acting on Each Wing
[Wing_Element_LH, Dynamics.Frame_Wing.LH] = Force_Components(Wing_Element_LH, Kinematics.LH, Morphology.Wing_LH.del_r, metrics.airDensity, Morphology.Wing_LH.c);
[Wing_Element_RH, Dynamics.Frame_Wing.RH] = Force_Components(Wing_Element_RH, Kinematics.RH, Morphology.Wing_RH.del_r, metrics.airDensity, Morphology.Wing_RH.c);

%% Find Force Directions
Dynamics.Frame_Body.LH = Force_Body_Frame(Dynamics.Frame_Wing.LH, Kinematics.LH.R_inv2, ang_wing_plane_LH);
Dynamics.Frame_Body.RH = Force_Body_Frame(Dynamics.Frame_Wing.RH, Kinematics.RH.R_inv2, ang_wing_plane_RH);

%% Find the Torque due to Force offset
Dynamics.Frame_Body.LH.Torque_Due_to_Forces = Torque_Forces(Dynamics, Wing_Element_LH, Kinematics.LH.R_inv2, Dynamics.Frame_Wing.LH, Morphology);
Dynamics.Frame_Body.RH.Torque_Due_to_Forces = Torque_Forces(Dynamics, Wing_Element_RH, Kinematics.RH.R_inv2, Dynamics.Frame_Wing.RH, Morphology);

%% Find the Torque due to Inertia
[Wing_Element_LH, Dynamics.Frame_Wing.LH.Torque_Inertia] = Torque_Inertia(Wing_Element_LH,  Kinematics.LH, Morphology.Wing_LH.inertia, Kinematics.LH.R_inv2);
[Wing_Element_RH, Dynamics.Frame_Wing.RH.Torque_Inertia] = Torque_Inertia(Wing_Element_RH,  Kinematics.RH, Morphology.Wing_RH.inertia, Kinematics.RH.R_inv2);

%% Find Torque Directions
Dynamics.Frame_Body.LH = Torque_Body_Frame(Dynamics.Frame_Wing.LH, Kinematics.LH.R_inv2, Dynamics.Frame_Body.LH);
Dynamics.Frame_Body.RH = Torque_Body_Frame(Dynamics.Frame_Wing.RH, Kinematics.RH.R_inv2, Dynamics.Frame_Body.RH);

%% Rotate Forces and Torque from Calcualted Frame (LH) to True Frame (RH)
[Dynamics.Frame_Body.RH] = True_Frame(Dynamics.Frame_Body.RH);

%% Store in Structure
Fly.Kinematics  = Kinematics;
Fly.Morphology  = Morphology;
Fly.Dynamics    = Dynamics;
%% End of Code Timer
toc 
end