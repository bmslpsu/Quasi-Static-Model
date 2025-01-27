%function [Fly] = Analysis_Robot(LH_Chord_Cut, LH_Span_Cut, RH_Chord_Cut, RH_Span_Cut, LH_Stroke_Amplitude, RH_Stroke_Amplitude) 
%% Note on the Axese

% For the wing
% x-axis is along the length of the wing (Root to Tip)
% y-axis is perpendicular to the surface of the wing
% z-axis is along the chord of the wing starting and is parallel to the abdomen of the fly

% For the Body
% z-axis is up
% y-axis is to forward
% x-axis is to the side (right positive)

% %% Uncomment to Clear Everything
clear all
clc
%close all
warning off

%% Mandatory features if not running as a function
LH_Span_Cut = 100;
LH_Chord_Cut = 100;
RH_Span_Cut = 100;
RH_Chord_Cut = 100;

LH_Stroke_Amplitude = 100;
RH_Stroke_Amplitude = 100;

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
%% Load Wing Kinematic Data

% Loads previously generated data wing kinematics come from
% Flies compensate for unilateral wing damage through modular
% adjustments of wing and Morphology.Body kinematics, Figure 1d
% Supplemental Material: Dataset S2
% (Michael H. Dickinson et. al.) 2017
load(['Data_Sets' filesep 'Dataset_S2.mat'], 'Deviation_IntactWing', 'Rotation_IntactWing', 'Stroke_IntactWing', 'Deviation_CutWing', 'Rotation_CutWing', 'Stroke_CutWing', 'Time_norm')


%% Time Set Up
% This section sets the time scale up and the kinematics to one period

[~, peak_index] = findpeaks(-Stroke_IntactWing(:,1));

Period = peak_index(1):peak_index(2);
time = Time_norm(Period)/250;
dt=diff(time(1:2));

%Store time in Fly structure
Kinematics.time = time;

%% Kinematic data
Kinematics.LH = Kin(Rotation_CutWing(Period,1), Stroke_CutWing(Period,1).*LH_Stroke_Amplitude/100, Deviation_CutWing(Period,1), 0, dt);
Kinematics.RH = Kin(Rotation_IntactWing(Period,1), Stroke_IntactWing(Period,1).*RH_Stroke_Amplitude/100, Deviation_IntactWing(Period,1), 0, dt);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Wing and Body Selection

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
ang_wing_plane = 0;
Dynamics.Frame_Body.LH = Force_Body_Frame(Dynamics.Frame_Wing.LH, Kinematics.LH.R_inv2, ang_wing_plane);
Dynamics.Frame_Body.RH = Force_Body_Frame(Dynamics.Frame_Wing.RH, Kinematics.RH.R_inv2, ang_wing_plane);

%% Find the Torque due to Force offset
Dynamics.Frame_Body.LH.Torque_Due_to_Forces = Torque_Forces(Dynamics, Wing_Element_LH, Kinematics.LH.R_inv2, Dynamics.Frame_Wing.LH);
Dynamics.Frame_Body.RH.Torque_Due_to_Forces = Torque_Forces(Dynamics, Wing_Element_RH, Kinematics.RH.R_inv2, Dynamics.Frame_Wing.RH);

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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% End of Code Timer
toc

%end