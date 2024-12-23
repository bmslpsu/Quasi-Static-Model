
%% Improvement Ideas
% 1. Calculate accelerations in main code ad pass the values rather than in
% the forces section
% 2. Save the data from the "filtered" kinematics for kinematics 1 to remove the complexity
% 3. Collect functions into one code


%% Uncomment to Clear Everything
clear all
clc
close all
warning off


%% Runtime
current_time = datetime;

%% Uncomment this section to run batch 
%i = [50 70 80 90 95 100];
i = [100 100 100 100 100 100];
Stroke_Amplitude = [75 80 85 90 95 100];
%i=100;

% Chordwise cut
for t=1:length(i)
    Fly_Master(t).Fly  = Main_Program_lite(i(t), 100, 100, 100,Stroke_Amplitude(t));
    Fly_Master(t).chord_cut = i(t);
    Fly_Master(t).span_cut = 100;
    Fly_Master(t).Stroke_Amplitude = Stroke_Amplitude(t);
end
% % 
% % % 
% % % % Spanewise cut (I recomned only doing chord cuts for now)
% % % for t=1:length(i) 
% % %     Fly_Master(length(i)+t).Fly = Main_Program_Run(100, i(t), 100, 100);
% % %     Fly_Master(length(i)+t).chord_cut = 100;
% % %     Fly_Master(length(i)+t).span_cut = i(t);
% % % end
% % 
% % 
% % %% Uncomment this section to run only one wing section
% LH_Chord_Cut = 50;
% LH_Span_Cut = 100;
% RH_Chord_Cut = 100;
% RH_Span_Cut = 100;
% 
%Main_Program_lite(LH_Chord_Cut, LH_Span_Cut, RH_Chord_Cut, RH_Span_Cut) 


%% Uncomment this section to save data

% % Open a dialog box for the user to select a location and specify a file name
% [filename, pathname] = uiputfile('Fly_Master.mat', 'Save data as');
% 
% % Check if the user canceled the operation
% if isequal(filename, 0) || isequal(pathname, 0)
%     disp('User canceled the operation.');
% else
%     % Construct the full file path
%     fullpath = fullfile(pathname, filename);
% 
%     % Save the data to the selected location
%     save(fullpath, 'Fly_Master');  
% end
% 
% save('C:\Users\jacob\OneDrive - The Pennsylvania State University\Research\Code\Main Code\Outputs\Fly_Master_Inverted_paper.mat', 'Fly_Master');


% Filter data
% Butter filter to compare to actual collected data
[b, a] = butter(2, 0.25, 'low');


for i=1:length(Fly_Master)


    for j=1:3
        Force_lh_f(j,:,i)=filtfilt(b, a, Fly_Master(i).Fly.force_total.Force_Body_lh.force_total_vec(j,:));
        Force_rh_f(j,:,i)=filtfilt(b, a, Fly_Master(i).Fly.force_total.Force_Body_rh.force_total_vec(j,:));
        Force_lh_f_mean(i,j) = mean(Force_lh_f(j,:,i));
        Force_rh_f_mean(i,j) = mean(Force_rh_f(j,:,i));
    end
end

time = Fly_Master(1).Fly.time(1:113)/Fly_Master(1).Fly.time(113) * 100;

% figure
% hold on
% for i=1
%     plot(time,Fly_Master(i).Fly.force_total.Force_Body_lh.force_total_vec(2,:)/Fly_Master(i).Fly.total.weight,"r")
%     plot(time,Force_lh_f(2,:,i)/Fly_Master(i).Fly.total.weight,"k--")
%     plot(time,mean(Fly_Master(i).Fly.force_total.Force_Body_lh.force_total_vec(2,:))*ones(112)/Fly_Master(i).Fly.total.weight,"r")
%     plot(time,mean(Force_lh_f(2,:,i))*ones(112)/Fly_Master(i).Fly.total.weight,"k--")
%     legend(["Unfiltered" "Filtered"])
%     ylabel("Normalized Z Forces (F_z/mg)")
%     xlabel('Wingbeat Cylce (%)')
%     title("Filter: butter(2, 0.25, 'low')")
% end
% hold off

fly_num = 1;

figure
hold on
plot(time,Force_lh_f(2,:,fly_num)/Fly_Master(fly_num).Fly.total.weight,"r")
plot(time,Force_rh_f(2,:,fly_num)/Fly_Master(fly_num).Fly.total.weight,"b")
plot(time,(Force_lh_f(2,:,fly_num)+Force_rh_f(2,:,fly_num))/Fly_Master(fly_num).Fly.total.weight,"m")
plot(time,mean(Force_rh_f(2,:,fly_num))*ones(113)/Fly_Master(fly_num).Fly.total.weight,"r--")
plot(time,mean(Force_rh_f(2,:,fly_num))*ones(113)/Fly_Master(fly_num).Fly.total.weight,"b--")
plot(time,mean(Force_lh_f(2,:,fly_num)+Force_rh_f(2,:,fly_num))*ones(113)/Fly_Master(fly_num).Fly.total.weight,"m--")
legend(["Damaged" "Intact" "Total"])
ylabel("Normalized Z Forces (F_z/mg)")
xlabel('Wingbeat Cylce (%)')
hold off


%% Uncomment this section to see the compiled data from the batch run




for i=1:length(Fly_Master)
    S_3_Ratio(i) = Fly_Master(i).Fly.total.S_3_Ratio;
    S_2_Ratio(i) = Fly_Master(i).Fly.total.S_2_Ratio;
    Force_X_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_lh.force_total_vec(1,:)) + mean(Fly_Master(i).Fly.force_total.Force_Body_rh.force_total_vec(1,:)))/Fly_Master(i).Fly.total.weight;
    Force_Y_mean(i) = (-mean(Fly_Master(i).Fly.force_total.Force_Body_lh.force_total_vec(3,:)) + mean(Fly_Master(i).Fly.force_total.Force_Body_rh.force_total_vec(3,:)))/Fly_Master(i).Fly.total.weight;
    Force_Z_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_lh.force_total_vec(2,:)) + mean(Fly_Master(i).Fly.force_total.Force_Body_rh.force_total_vec(2,:)))/Fly_Master(i).Fly.total.weight;
    Force_f_X_mean_t(i) = (Force_lh_f_mean(i,1) + Force_rh_f_mean(i,1))/Fly_Master(i).Fly.total.weight;
    Force_f_Y_mean_t(i) = (-Force_lh_f_mean(i,3) + Force_rh_f_mean(i,3))/Fly_Master(i).Fly.total.weight;
    Force_f_Z_mean_t(i) = (Force_lh_f_mean(i,2) + Force_rh_f_mean(i,2))/Fly_Master(i).Fly.total.weight;
    % Moment_Roll_mean(i) = (mean(Fly_Master(i).Fly.force_total.Moments_Body_lh.moment_roll(:))/Fly_Master(i).Fly.wing_lh.wing_length - mean(Fly_Master(i).Fly.force_total.Moments_Body_rh.moment_roll(:))/Fly_Master(i).Fly.wing_rh.wing_length)/Fly_Master(i).Fly.total.weight;
    % Moment_Pitch_mean(i) = (mean(Fly_Master(i).Fly.force_total.Moments_Body_lh.moment_pitch(:))/Fly_Master(i).Fly.wing_lh.wing_length + mean(Fly_Master(i).Fly.force_total.Moments_Body_rh.moment_pitch(:))/Fly_Master(i).Fly.wing_rh.wing_length)/Fly_Master(i).Fly.total.weight;
    % Moment_Yaw_mean(i) = (mean(Fly_Master(i).Fly.force_total.Moments_Body_lh.moment_yaw(:))/Fly_Master(i).Fly.wing_lh.wing_length - mean(Fly_Master(i).Fly.force_total.Moments_Body_rh.moment_yaw(:))/Fly_Master(i).Fly.wing_rh.wing_length)/Fly_Master(i).Fly.total.weight;
end

figure
hold on
plot(Stroke_Amplitude,Force_X_mean,'Color',[1, 0.5, 0])
plot(Stroke_Amplitude,Force_Y_mean,'Color',"g")
plot(Stroke_Amplitude,Force_Z_mean,'Color',"b")
plot(Stroke_Amplitude, Force_f_X_mean_t, 'Color', [1, 0.5, 0], 'LineStyle', '--')
plot(Stroke_Amplitude,Force_f_Y_mean_t,"g--")
plot(Stroke_Amplitude,Force_f_Z_mean_t,"b--")
legend(["X" "Y" "Z" "X_f_i_l_t_e_r_e_d" "Y_f_i_l_t_e_r_e_d" "Z_f_i_l_t_e_r_e_d"])
ylabel("Normalized Forces (F/mg)")
xlabel("Second moment of area Ration S_2")
% axis([.5 1 0 1])
hold off

% figure
% hold on
% plot(S_3_Ratio,Moment_Roll_mean,'Color',[1, 0.5, 0])
% plot(S_3_Ratio,Moment_Pitch_mean,'Color',"g")
% plot(S_3_Ratio,Moment_Yaw_mean,'Color',"b")
% legend(["Roll" "Pitch" "Yaw"])
% ylabel("Normalized Torques (T/mgl)")
% xlabel("Third moment of area Ration S_3")
% axis([.5 1 0 1])
% hold off

% Run Time End
Duration = datetime-current_time

return

% % function [Fly] = Main_Program_Run(LH_Chord_Cut, LH_Span_Cut, RH_Chord_Cut, RH_Span_Cut) 

%% Note on the Axese

% For the wing
% z-axis is along the length of the wing
% y-axis is perpendicular to the surface of the wing
% x-axis is along the chord of the wing starting at the wing base and
% is parallel to the abdomen of the fly

% For the body
% z-axis is up
% x-axis is to forward
% y-axis is to the side (right positive)
 

%% Clear Everything
close all
warning off
tic
%% Sig Figs
digits(4); % sets decimal point accuracy

%% Standard Constants
[metrics, ~, ~] = get_metrics();
rho = metrics.airDensity;
g = metrics.gravity;

%% Variable Decleration
%Creates structures to manage data throughout the program
Wing_Element_lh = struct();
Wing_Element_rh = struct();
Force_Body_lh = struct();
Force_Body_rh = struct();
Wing_Shape_lh = struct();
Wing_Shape_rh = struct();
Body_Shape = struct();
Fly = struct();

%% Choose Wing Kinematics

    Kinematic_Selection = 2; % This is temporary for data testing

    % % prompt = {['Enter 1 for Integrative Model of Drosophila Flight, Figure 9 (2008) or' newline ...
    % %            'Enter 2 for :']};
    % % dlgtitle = 'Kienmeatics Selection';
    % % dims = [1 75];
    % % definput = {'2'};
    % % answer = inputdlg(prompt, dlgtitle, dims, definput);
    % % 
    % % % Check if the user provided input
    % % if ~isempty(answer)
    % %     % Convert the input to a number
    % %     Kinematic_Selection = str2double(answer{1});
    % % 
    % %     % Validate the input
    % %     if isnan(Kinematic_Selection) || ~ismember(Kinematic_Selection, [1, 2])
    % %         errordlg('Invalid input. Please enter either 1 or 2.', 'Error');
    % %     else
    % %         % Handle the valid input
    % %         if Kinematic_Selection == 1
    % %             msgbox('You selected Option 1', 'Selection');
    % %         elseif Kinematic_Selection == 2
    % %             msgbox('You selected Option 2', 'Selection');
    % %         end
    % %     end
    % % else
    % %     % Handle case where input dialog was cancelled
    % %     msgbox('No selection made.', 'Selection');
    % % end

%% Loads Wing Kinematic Data
% This section will load the data chosen above

switch Kinematic_Selection
    case 1
        % Loads previously generated data wing kinematics come from
        % Integrative Model of Drosophila Flight, Figure 9
        % (William B. Dickson, Andrew D. Straw, and Michael H. Dickinson) 2008
        load(['Data_Sets' filesep 'Fly_Angles.mat'])
    case 2
        % Loads previously generated data wing kinematics come from
        % Flies compensate for unilateral wing damage through modular
        % adjustments of wing and body kinematics, Figure 1d
        % Supplemental Material: Dataset S2
        % (Michael H. Dickinson et. al.) 2017
        load(['Data_Sets' filesep 'Dataset_S2.mat'], 'Deviation_IntactWing', 'Rotation_IntactWing', 'Stroke_IntactWing', 'Time_norm')
        % Deviation_IntactWing = -Deviation_IntactWing;
        % Rotation_IntactWing = -Rotation_IntactWing;
        % Stroke_IntactWing = -Stroke_IntactWing;     
    case 3
        % This will be a custom data uploaded in the future
    otherwise
        msgbox("An Error occured")
        return
end

%% Time Set Up
% This section sets the time scale up and the kinematics to one period

switch Kinematic_Selection
    case 1
        time=time_1/1000; % adjusts time to miliseconds

        % Adjusts time scale for one period of time
        step = 43;
        step_div = 5;
        time = time(1+step:round(length(time)*1/step_div)+step);
        filtered_AoA = filtered_AoA(1+step:round(length(filtered_AoA)*1/step_div)+step);
        filtered_stroke_angle_2 = filtered_stroke_angle_2(1+step:round(length(filtered_stroke_angle_2)*1/step_div)+step);
        filtered_dev_angle_2 = filtered_dev_angle_2(1+step:round(length(filtered_dev_angle_2)*1/step_div)+step);
    case 2
        % Adjusts time scale for one period of time
        time = Time_norm(1:115)/250;
        AoA = Rotation_IntactWing(1:115,1);
        stroke_angle = Stroke_IntactWing(1:115,1);
        dev_angle = Deviation_IntactWing(1:115,1);
        %62:177 is an alternate period
    case 3
        % To be added later
    otherwise
        msgbox("An Error occured")
        return
end

%% Extract/Filter Wing Kinematic Data

switch Kinematic_Selection
    case 1
        % This data is orignally from a image and requires filtering for
        % clean processing
        filter = true;
        [phi_f, alpha_f, gamma_f, phi_original, alpha_original, gamma_original] = ExtractAngles(time, 100-rad2deg(filtered_AoA), rad2deg(filtered_stroke_angle_2)-60, rad2deg(filtered_dev_angle_2)-90, Plot_kinematics, filter);
    case 2
        % This data comes directly from a dataset and does not requiring
        % filtering. In this case the "filtered" data and original data are
        % the same
        filter = false;
        [phi_f, alpha_f, gamma_f, phi_original, alpha_original, gamma_original] = ExtractAngles(time, AoA, stroke_angle, dev_angle, Plot_kinematics, filter);
    case 3
        % To be added later
    otherwise
        msgbox("An Error occured")
        return
end

%% Find the Angular Velocity
% Calculates the angular velocity using a data diffrential function. THis
% also will filter the data if the orgianl is from an image
[phi_dotf, alpha_dotf, gamma_dotf, phi_dot_dotf, alpha_dot_dotf, gamma_dot_dotf] = GetEulerAngleVelocity(time, phi_f, alpha_f, gamma_f, phi_original, alpha_original, gamma_original, Plot_kinematic_Velocity, filter);

%% Find Moving Vectors in Stationary Frame
% Calculate the vectors from the origin of the body frame and the
% assoicated rotation matrix. 
% Right now this does not diffrentiate from left and right wing
[ex11, ey11, ez11, R_inv2, R_2] = Find_vectors(phi_f, alpha_f, gamma_f);

%% Find Angular Velocity of each Wing with respect to Stationary Frame
% This finds the magnitude of the angluar velocity with respect to the body
% frame
% omega_mag is in deg
% omega is in deg

[omega, omega_mag, alpha, alpha_mag] = GetWingAngVel(ex11, ey11, ez11, phi_dotf, alpha_dotf, gamma_dotf, phi_dot_dotf, alpha_dot_dotf, gamma_dot_dotf);

%% Wing Selection
% LH and RH wing uploader

Wing_Upload_Type = 2;
switch Wing_Upload_Type
    case 1 %Standard wing using GUI
        [Wing_Shape_lh, Wing_Shape_rh, Body_Shape] = wingPlotGUI(Wing_Shape_lh, Wing_Shape_rh, Body_Shape,false,0,0,0,0);
    case 2 %Standard wing using intial values
        [Wing_Shape_lh, Wing_Shape_rh, Body_Shape] = wingPlotGUI(Wing_Shape_lh, Wing_Shape_rh, Body_Shape,true,LH_Span_Cut,LH_Chord_Cut,RH_Span_Cut,RH_Chord_Cut);
    case 3 %Custom wing
        % wing uploader
    otherwise
        msgbox("An Error has occured!")
end

% Stores the data in the "Fly" structure for future analysis 
[Fly.wing_lh, Fly.wing_rh, Fly.body] = mass_and_inertia(Wing_Shape_lh,Wing_Shape_rh, Body_Shape);
Fly.total.mass = Fly.wing_lh.mass + Fly.wing_rh.mass + Fly.body.mass;
% % Fly.total.mass = .001; % Ideal Body weight for troubleshooting
Fly.total.weight = Fly.total.mass * g; 

Fly.wing_lh.wing_shape = Wing_Shape_lh;
Fly.wing_rh.wing_shape = Wing_Shape_rh;
Fly.body.body_shape = Body_Shape;
Fly.time = time;

[Fly.wing_lh.c, Fly.wing_lh.n, Fly.wing_lh.wing_length, Fly.wing_lh.del_r] = Wing_Characteristics(Fly.wing_lh.wing_shape);
[Fly.wing_rh.c, Fly.wing_rh.n, Fly.wing_rh.wing_length, Fly.wing_rh.del_r] = Wing_Characteristics(Fly.wing_rh.wing_shape);
%% Find the Second and Third moment of area
% Calculates the 3rd moment of area
Fly.wing_lh.S_3 = thirdMomentOfArea(Fly.wing_lh.wing_length, Fly.wing_lh.c, Fly.wing_lh.n);
Fly.wing_rh.S_3 = thirdMomentOfArea(Fly.wing_rh.wing_length, Fly.wing_rh.c, Fly.wing_rh.n);
Fly.total.S_3_Ratio = Fly.wing_lh.S_3/Fly.wing_rh.S_3;

% Calculates the 2nd moment of area
Fly.wing_lh.S_2 = secondMomentOfArea(Fly.wing_lh.wing_length, Fly.wing_lh.c, Fly.wing_lh.n);
Fly.wing_rh.S_2 = secondMomentOfArea(Fly.wing_rh.wing_length, Fly.wing_rh.c, Fly.wing_rh.n);
Fly.total.S_2_Ratio = Fly.wing_lh.S_2/Fly.wing_rh.S_2;

%% Find the Location of the Center of Pressure for each Wing Element
% Calculates the center of pressure of each wing element 
Wing_Element_lh = FindDistanceOfCOP(alpha_f, Fly.wing_lh.n, Fly.wing_lh.c, R_inv2, Fly.wing_lh.wing_length);
Wing_Element_rh = FindDistanceOfCOP(alpha_f, Fly.wing_rh.n, Fly.wing_rh.c, R_inv2, Fly.wing_rh.wing_length);

%% Find Linear Velocity of each Element for each Time Step
% Calculates the linear velocity of each element based on the magnitude of
% the angluar velocity
Wing_Element_lh = FindLinearVelocity(Wing_Element_lh, omega, alpha); %omega instead of phi_f
Wing_Element_rh = FindLinearVelocity(Wing_Element_rh, omega, alpha); %omega instead of phi_f

%% Find the Lift and Drag Forces Acting on Each Wing
[Fly.wing_lh.C_L, Fly.wing_lh.C_D, Wing_Element_lh, Fly.force_components.Lift_force_lh, Fly.force_components.Drag_force_lh, Fly.force_components.Force_Translation_lh] = LiftAndDragForces(Wing_Element_lh, alpha_f, Fly.wing_lh.del_r, rho, Fly.wing_lh.c, phi_dotf);
[Fly.wing_rh.C_L, Fly.wing_rh.C_D, Wing_Element_rh, Fly.force_components.Lift_force_rh, Fly.force_components.Drag_force_rh, Fly.force_components.Force_Translation_rh] = LiftAndDragForces(Wing_Element_rh, alpha_f, Fly.wing_rh.del_r, rho, Fly.wing_rh.c, phi_dotf);

%% Find the Rotational Force acting on Each Wing
[Wing_Element_lh, Fly.force_components.Rot_force_lh] = RotationalForce(Wing_Element_lh, alpha_dotf, Fly.wing_lh.del_r, Fly.wing_lh.c, rho, alpha_f, time, alpha_dot_dotf);
[Wing_Element_rh, Fly.force_components.Rot_force_rh] = RotationalForce(Wing_Element_rh, alpha_dotf, Fly.wing_rh.del_r, Fly.wing_rh.c, rho, alpha_f, time, alpha_dot_dotf);

%% Find Added Mass Force acting on Each Wing
[Wing_Element_lh, Fly.force_components.Am_force_lh] = AddedMassForce(Wing_Element_lh, alpha_dotf, alpha_f, Fly.wing_lh.del_r, Fly.wing_lh.c, time, rho, phi_dotf);
[Wing_Element_rh, Fly.force_components.Am_force_rh] = AddedMassForce(Wing_Element_rh, alpha_dotf, alpha_f, Fly.wing_rh.del_r, Fly.wing_rh.c, time, rho, phi_dotf);

%% Find Force Directions
Wing_Element_lh = FindForceVectors(Wing_Element_lh, R_inv2, gamma_f, phi_f, phi_dotf, alpha_f, alpha_dotf);
Wing_Element_rh = FindForceVectors(Wing_Element_rh, R_inv2, gamma_f, phi_f, phi_dotf, alpha_f, alpha_dotf);

%% Find the Total Forces in x, y, and z Directions
Fly.force_total.Force_Body_lh = Find_forces_XYZ(Wing_Element_lh, Force_Body_lh);
Fly.force_total.Force_Body_rh = Find_forces_XYZ(Wing_Element_rh, Force_Body_rh);
%% Find the Total Moments in x, y, and z Directions
% Not Complete
Fly.force_total.Moments_Body_lh = Find_moments_XYZ(Wing_Element_lh, Force_Body_lh);
Fly.force_total.Moments_Body_rh = Find_moments_XYZ(Wing_Element_rh, Force_Body_rh);

%% End of Code Timer
toc

% % end

%% Functions---------------------------------------------------------------
%--------------------------------------------------------------------------

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

function force_Body = Find_forces_XYZ(element, force_Body)

    force_lift_vec = 0;
    force_drag_vec = 0;
    force_Rot_vec = 0;
    force_AM_vec = 0;
    force_Total = 0; % the total force in x y and z directions for each wing
    
    for j=1:length(element)
        force_lift_vec = force_lift_vec + element(j).force_lift_vec;
        force_drag_vec = force_drag_vec + element(j).force_drag_vec;
        force_Rot_vec = force_Rot_vec + element(j).force_Rot_vec;
        force_AM_vec = force_AM_vec + element(j).force_AM_vec;
        force_Total = force_Total + element(j).force_Rot_vec + element(j).force_AM_vec + element(j).force_lift_vec + element(j).force_drag_vec;
    end

    force_Body.force_lift_vec = force_lift_vec;
    force_Body.force_drag_vec = force_drag_vec;
    force_Body.force_Rot_vec = force_Rot_vec;
    force_Body.force_AM_vec = force_AM_vec;
    force_Body.force_Total = force_Total;
    
end

function force_Body = Find_moments_XYZ(element, force_Body)

    moment_roll = 0;
    moment_yaw = 0;
    moment_pitch = 0;
    
    for j=1:length(element)
        force_Total = element(j).force_Rot_vec + element(j).force_AM_vec + element(j).force_lift_vec + element(j).force_drag_vec;
        moment_roll = moment_roll + force_Total(3,:)*element(j).Distance_COP(1) - force_Total(2,:)*element(j).Distance_COP(1);
        moment_yaw = moment_yaw + force_Total(2,:)*element(j).Distance_COP(1) - force_Total(1,:)*element(j).Distance_COP(1);
        moment_pitch = moment_pitch + force_Total(1,:)*element(j).Distance_COP(1) - force_Total(3,:)*element(j).Distance_COP(1);
        
    end

    force_Body.moment_roll = moment_roll;
    force_Body.moment_yaw = moment_yaw;
    force_Body.moment_pitch = moment_pitch;
    
end


