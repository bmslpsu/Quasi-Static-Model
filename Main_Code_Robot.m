%% Uncomment to Clear Everything
clear all
clc
%close all
warning off


%% Runtime
current_time = datetime;

%% Wing Damage Values
Wing_damage_LH = [50 70 80 90 95 100];
Wing_damage_RH = [100 100 100 100 100 100];
Stroke_Amplitude_LH = [100 100 100 100 100 100];
Stroke_Amplitude_RH = [100 100 100 100 100 100];


%% Stroke Amplitude Values
% Wing_damage_LH = [100 100 100 100 100 100];
% Wing_damage_RH = [100 100 100 100 100 100];
% Stroke_Amplitude_LH = [75 80 85 90 95 100];
% Stroke_Amplitude_RH = [100 100 100 100 100 100];

%% Program Runner

% Chordwise cut
for i=1:length(Stroke_Amplitude_LH)
    Fly_Master(i).Fly  = Analysis_Robot(Wing_damage_LH(i), Wing_damage_RH(i), 100, 100,Stroke_Amplitude_LH(i),Stroke_Amplitude_RH(i), i);
    Fly_Master(i).chord_cut_LH = Wing_damage_LH(i);
    Fly_Master(i).chord_cut_RH = Wing_damage_RH(i);
    Fly_Master(i).span_cut = 100;
    Fly_Master(i).Stroke_Amplitude = Stroke_Amplitude_LH(i);
    Fly_Master(i).Stroke_Amplitude = Stroke_Amplitude_RH(i);
end

% % Spanewise cut (I recomned only doing chord cuts for now)
% for t=1:length(i) 
%     Fly_Master(length(i)+t).Fly = Main_Program_Run(100, i(t), 100, 100);
%     Fly_Master(length(i)+t).chord_cut = 100;
%     Fly_Master(length(i)+t).span_cut = i(t);
% end



%% Save data

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

%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Force means

for i=1:length(Fly_Master)
    S_2_Ratio(i) = Fly_Master(i).Fly.total.S_2_Ratio;
    Force_X_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_LH.force_total_vec(1,:)) + mean(Fly_Master(i).Fly.force_total.Force_Body_RH.force_total_vec(1,:)))/Fly_Master(i).Fly.total.weight;
    Force_Y_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_LH.force_total_vec(3,:)) - mean(Fly_Master(i).Fly.force_total.Force_Body_RH.force_total_vec(3,:)))/Fly_Master(i).Fly.total.weight;
    Force_Z_mean(i) = (mean(Fly_Master(i).Fly.force_total.Force_Body_LH.force_total_vec(2,:)) + mean(Fly_Master(i).Fly.force_total.Force_Body_RH.force_total_vec(2,:)))/Fly_Master(i).Fly.total.weight;
end

%% S_2 versus force
figure
hold on
plot(S_2_Ratio,Force_X_mean,'Color',[1, 0.5, 0])
plot(S_2_Ratio,Force_Y_mean,'Color',"g")
plot(S_2_Ratio,Force_Z_mean,'Color',"b")

legend(["X" "Y" "Z"])
ylabel("Normalized Forces (F/mg)")
xlabel("Second moment of area Ration S_2")
% axis([.5 1 0 1])
hold off

%% Stroke Amplitude versus force
figure
hold on
plot(Stroke_Amplitude_LH,Force_X_mean,'Color',[1, 0.5, 0])
plot(Stroke_Amplitude_LH,Force_Y_mean,'Color',"g")
plot(Stroke_Amplitude_LH,Force_Z_mean,'Color',"b")

legend(["X" "Y" "Z"])
ylabel("Normalized Forces (F/mg)")
xlabel("Stroke Amplitude")
% axis([.5 1 0 1])
hold off

%% Torques means

for i=1:length(Fly_Master)
    S_3_Ratio(i) = Fly_Master(i).Fly.total.S_3_Ratio;
    Moment_Roll_mean(i) = mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(1,:) - Fly_Master(i).Fly.force_total.Force_Body_RH.torque_total_vec(1,:)) / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length+Fly_Master(i).Fly.wing_RH.wing_length)/2));
    Moment_Pitch_mean(i) = mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(3,:) + Fly_Master(i).Fly.force_total.Force_Body_RH.torque_total_vec(3,:)) / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length+Fly_Master(i).Fly.wing_RH.wing_length)/2));
    Moment_Yaw_mean(i) = mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(2,:) - Fly_Master(i).Fly.force_total.Force_Body_RH.torque_total_vec(2,:)) / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length+Fly_Master(i).Fly.wing_RH.wing_length)/2));
    %Moment_Yaw_mean(i) = mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(2,:)  / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length))));
    % Moment_Yaw_mean(i) = mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(1,:) - Fly_Master(i).Fly.force_total.Force_Body_RH.torque_total_vec(1,:)) / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length+Fly_Master(i).Fly.wing_RH.wing_length)/2));
    % Moment_Roll_mean(i) = -mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(3,:) - Fly_Master(i).Fly.force_total.Force_Body_RH.torque_total_vec(3,:)) / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length+Fly_Master(i).Fly.wing_RH.wing_length)/2));
    % Moment_Pitch_mean(i) = -mean((Fly_Master(i).Fly.force_total.Force_Body_LH.torque_total_vec(2,:) - Fly_Master(i).Fly.force_total.Force_Body_RH.torque_total_vec(2,:)) / (Fly_Master(i).Fly.total.weight * (Fly_Master(i).Fly.wing_LH.wing_length+Fly_Master(i).Fly.wing_RH.wing_length)/2));
end

%% S_3 versus torque
figure
hold on
plot(S_3_Ratio,Moment_Roll_mean,'Color',[1, 0.5, 0])
plot(S_3_Ratio,Moment_Pitch_mean,'Color',"g")
plot(S_3_Ratio,Moment_Yaw_mean,'Color',"b")
legend(["Roll" "Pitch" "Yaw"])
ylabel("Normalized Torques (T/mgl)")
xlabel("Third moment of area Ration S_3")
%axis([.5 1 0 1])
hold off

%% Stroke Amplitude versus torque
figure
hold on
plot(Stroke_Amplitude_LH,Moment_Roll_mean,'Color',[1, 0.5, 0])
plot(Stroke_Amplitude_LH,Moment_Pitch_mean,'Color',"g")
plot(Stroke_Amplitude_LH,Moment_Yaw_mean,'Color',"b")
legend(["Roll" "Pitch" "Yaw"])
ylabel("Normalized Torques (T/mgl)")
xlabel("Stroke Amplitude")
%axis([.5 1 0 1])
hold off
%% Run Time End
Duration = datetime-current_time