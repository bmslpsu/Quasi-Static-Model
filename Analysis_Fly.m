function [Fly] = Analysis_Fly(LH_Chord_Cut, LH_Span_Cut, RH_Chord_Cut, RH_Span_Cut, LH_Stroke_Amplitude, RH_Stroke_Amplitude, fly_num, FilteredAngleL, FilteredAngleR, period, ang_wing_plane_LH, ang_wing_plane_RH) 
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
%% Wing and Boddy Selection

% LH and RH wing uploader
[Morphology.Wing_LH.wing_shape, Morphology.Wing_RH.wing_shape, Morphology.Body.Morphology.Body_shape, Morphology.Body.Joint] = wingPlotGUI(Wing_Shape_LH, Wing_Shape_RH, Body_Shape,true,LH_Span_Cut,LH_Chord_Cut,RH_Span_Cut,RH_Chord_Cut);

% Body and Wing physical Analyis
[Morphology] = mass_and_inertia(Morphology.Wing_LH.wing_shape,Morphology.Wing_RH.wing_shape, Morphology.Body.Morphology.Body_shape, Morphology);

Morphology.Body.Body_angle = 45;

Morphology.Wing_LH.Wing_Plane_angle = ang_wing_plane_LH;
Morphology.Wing_RH.Wing_Plane_angle = ang_wing_plane_RH;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Time Set Up
%Store time in Fly structure
Fly.time = period;
dt=.000125;

%% Kinematic data
Kinematics.LH = Kin(FilteredAngleL(period,2), FilteredAngleL(period,1).*LH_Stroke_Amplitude/100, FilteredAngleL(period,3), 0, dt);
Kinematics.RH = Kin(FilteredAngleR(period,2), FilteredAngleR(period,1).*RH_Stroke_Amplitude/100, FilteredAngleR(period,3), 0, dt);

%Store time in Kinematics structure
Kinematics.time = period;
Kinematics.LH.N = length(period);
Kinematics.RH.N = length(period);

%% Find the Location of the Center of Pressure for each Wing Element
% Calculates the center of pressure of each wing element 
Wing_Element_LH = Center_of_Pressure(Kinematics.LH, Morphology.Wing_LH);
Wing_Element_RH = Center_of_Pressure(Kinematics.RH, Morphology.Wing_RH);

%% Find the Linear Velocity of each Element for each Time Step
% Calculates the linear velocity of each element based on the magnitude of
% the angluar velocity
Wing_Element_LH = Kin_Linear(Kinematics.LH, Wing_Element_LH);
Wing_Element_RH = Kin_Linear(Kinematics.RH, Wing_Element_RH);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the Lift, Drag, Rotation, and Added Mass Forces and Torques Acting on Each Wing
[Wing_Element_LH, Dynamics.Frame_Wing.LH, Dynamics.Frame_Body.LH ] = Dynamic_Components(Kinematics.LH, Wing_Element_LH, Morphology.Wing_LH, Morphology, metrics.airDensity);
[Wing_Element_RH, Dynamics.Frame_Wing.RH, Dynamics.Frame_Body.RH ] = Dynamic_Components(Kinematics.RH, Wing_Element_RH, Morphology.Wing_RH, Morphology, metrics.airDensity);

%% Rotate Forces and Torque from Calcualted Frame (LH) to True Frame (RH)
[Dynamics.Frame_Body.RH] = True_Frame(Dynamics.Frame_Body.RH);

%% Store in Structure
Fly.Kinematics  = Kinematics;
Fly.Morphology  = Morphology;
Fly.Dynamics    = Dynamics;
%% End of Code Timer
toc 
end