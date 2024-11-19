function [Fly] = Analyis_Fly(LH_Chord_Cut, LH_Span_Cut, RH_Chord_Cut, RH_Span_Cut, LH_Stroke_Amplitude, RH_Stroke_Amplitude, fly_num, FilteredAngleL, FilteredAngleR, period) 
%% Note on the Axese

% For the wing
% x-axis is along the length of the wing (Chord to Tip)
% y-axis is perpendicular to the surface of the wing
% z-axis is along the chord of the wing starting and is parallel to the abdomen of the fly

% For the body
% z-axis is up
% y-axis is to forward
% x-axis is to the side (right positive)

%% Runtime
current_time = datetime;
tic

%% Sig Figs
digits(4); % sets decimal point accuracy

%% Standard Constants
[metrics, ~, ~] = get_metrics();

%% Variable Decleration
%Creates structures to manage data throughout the program
Wing_Element_LH = struct();
Wing_Element_RH = struct();
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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Wing and Boddy Selection

% LH and RH wing uploader
[Fly.wing_LH.wing_shape, Fly.wing_RH.wing_shape, Fly.body.body_shape, Fly.body.Joint] = wingPlotGUI(Wing_Shape_LH, Wing_Shape_RH, Body_Shape,true,LH_Span_Cut,LH_Chord_Cut,RH_Span_Cut,RH_Chord_Cut);

% Stores the data in the "Fly" structure for future analysis 
[Fly] = mass_and_inertia(Fly.wing_LH.wing_shape,Fly.wing_RH.wing_shape, Fly.body.body_shape, Fly);

[Fly.wing_LH.c, Fly.wing_LH.n, Fly.wing_LH.wing_length, Fly.wing_LH.del_r] = Wing_Characteristics(Fly.wing_LH.wing_shape);
[Fly.wing_RH.c, Fly.wing_RH.n, Fly.wing_RH.wing_length, Fly.wing_RH.del_r] = Wing_Characteristics(Fly.wing_RH.wing_shape);

%% Find the Second and Third moment of area
% Calculates the 3rd moment of area
Fly.wing_LH.S_3 = thirdMomentOfArea(Fly.wing_LH.wing_length, Fly.wing_LH.c, Fly.wing_LH.n);
Fly.wing_RH.S_3 = thirdMomentOfArea(Fly.wing_RH.wing_length, Fly.wing_RH.c, Fly.wing_RH.n);
Fly.total.S_3_Ratio = Fly.wing_LH.S_3/Fly.wing_RH.S_3;

% Calculates the 2nd moment of area
Fly.wing_LH.S_2 = secondMomentOfArea(Fly.wing_LH.wing_length, Fly.wing_LH.c, Fly.wing_LH.n);
Fly.wing_RH.S_2 = secondMomentOfArea(Fly.wing_RH.wing_length, Fly.wing_RH.c, Fly.wing_RH.n);
Fly.total.S_2_Ratio = Fly.wing_LH.S_2/Fly.wing_RH.S_2;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Fly = Dyn(Kinematics_LH, Kinematics_RH, Fly);

%% End of Code Timer
toc
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Functions---------------------------------------------------------------

function S_2 = secondMomentOfArea(wing_length, c, n_elements)
    % Calculate the second moment of area (S_2)
    % Inputs:
    %   wing_length - Total length of the wing
    %   c - Array of chord lengths at each discrete point
    %   n_elements - Number of discrete points
    
    % Distance between points (assuming uniform spacing)
    Delta_r = wing_length / (n_elements - 1);
    
    % Calculate the second moment of area (S_2)
    S_2 = 0;
    for i = 1:n_elements
        r = (i-1) * Delta_r;
        S_2 = S_2 + c(i) * r^2 * Delta_r;
    end
end

function S_3 = thirdMomentOfArea(wing_length, c, n_elements)
    % Calculate the third moment of area (S_3)
    % Inputs:
    %   wing_length - Total length of the wing
    %   c - Array of chord lengths at each discrete point
    %   n_elements - Number of discrete points
    
    % Distance between points (assuming uniform spacing)
    Delta_r = wing_length / (n_elements - 1);
    
    % Calculate the third moment of area (S_3)
    S_3 = 0;
    for i = 1:n_elements
        r = (i-1) * Delta_r;
        S_3 = S_3 + c(i) * r^3 * Delta_r;
    end
end



