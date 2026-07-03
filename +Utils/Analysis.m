function [Fly] = Analysis(Fly_Data, FilteredAngleL, FilteredAngleR, period, dt)
import Utils.get_metrics
import Utils.Init_Morphology
import Utils.Init_Kinematics
import Utils.Center_of_Pressure
import Utils.Kin_Linear
import Utils.Dynamic_Components
import Utils.True_Frame

%% Note on the Axes

% For the wing
% x-axis is along the length of the wing (Root to Tip)
% y-axis is perpendicular to the surface of the wing
% z-axis is along the chord of the wing starting and is parallel to the 
% abdomen of the fly

% For the Body
% x-axis is to the side (right positive)
% y-axis is to forward
% z-axis is up

%% Start of Code Timer
tic

%% Sig Figs
digits(6); % sets decimal point accuracy

%% Standard Constants
[metrics, ~, ~] = get_metrics();

%% Variable Declaration
%Creates structures to manage data throughout the program
Wing_Shape_LH = struct();
Wing_Shape_RH = struct();
Fly = struct();

%% Time Set Up %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Fly.time = 0:dt:dt*(period-1);

%% Wing and Body Selection %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LH and RH wing uploader
Morphology = Init_Morphology(Wing_Shape_LH,Wing_Shape_RH,Fly_Data);

%% Kinematic data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sampLH = Fly_Data.Stroke_Amplitude_LH/100;
sampRH = Fly_Data.Stroke_Amplitude_RH/100;
Kinematics = Init_Kinematics(...
    [FilteredAngleL(:,1).*sampLH, FilteredAngleR(:,1).*sampRH],...
    [FilteredAngleL(:,2), FilteredAngleR(:,2)],...
    [FilteredAngleL(:,3), FilteredAngleR(:,3)],period,dt);

%% Find the Location of the Center of Pressure for each Wing Element %%%%%%
% Calculates the center of pressure of each wing element
Wing_Element_LH = Center_of_Pressure(Kinematics.LH, Morphology.Wing_LH);
Wing_Element_RH = Center_of_Pressure(Kinematics.RH, Morphology.Wing_RH);

%% Find the Linear Velocity of each Element for each Time Step %%%%%%%%%%%%
% Calculates the linear velocity of each element based on the magnitude of
% the angluar velocity
Wing_Element_LH = Kin_Linear(Kinematics.LH, Wing_Element_LH);
Wing_Element_RH = Kin_Linear(Kinematics.RH, Wing_Element_RH);

%% Find the Forces and Torques Acting on Each Wing %%%%%%%%%%%%%%%%%%%%%%%%
% TODO: Rewrite into a proper Dynamics constructor
[Wing_Element_LH, Dynamics.Frame_Wing.LH, Dynamics.Frame_Body.LH ] = ...
    Dynamic_Components(Kinematics.LH, Wing_Element_LH, ...
    Morphology.Wing_LH, Morphology, metrics.airDensity);
[Wing_Element_RH, Dynamics.Frame_Wing.RH, Dynamics.Frame_Body.RH ] = ...
    Dynamic_Components(Kinematics.RH, Wing_Element_RH, ...
    Morphology.Wing_RH, Morphology, metrics.airDensity);

%% Rotate Forces and Torque from Calcualted Frame (LH) to True Frame (RH) %
[Dynamics.Frame_Body.RH] = True_Frame(Dynamics.Frame_Body.RH);

%% Store in Structure %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: Unpack Fly, every Fly_Master element should just be a struct of
% Kinematics, Morphology, Dynamics, time, Fly_Num, and Attributes
Fly.Kinematics  = Kinematics;
Fly.Morphology  = Morphology;
Fly.Dynamics    = Dynamics;

%% End of Code Timer %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
toc

end