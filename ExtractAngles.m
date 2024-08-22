function [phi_f, alpha_f, gamma_f,phi,alpha,gamma] = ExtractAngles(time, alpha_input, phi_input, gamma_input, Plot_kinematics, filter)

    %This function filters the angles to smooth them out if required. Note:
    %whether it is filtered or not the out put is the "filtered" variables.
    %This may be imporved in the future for readability.

    phi = phi_input; %stroke angle
    alpha = alpha_input; %rotation angle
    gamma = gamma_input; %deviation angle
    
    %% filtering the position data due to noise by me

    if filter == true 
        [b, a] = butter(2, 0.15,'low');
    
        phi_f=filtfilt(b, a, phi);
        alpha_f=filtfilt(b, a, alpha);
        gamma_f=filtfilt(b, a, gamma);

    else
        
        phi_f = phi;
        alpha_f = alpha;
        gamma_f = gamma;

    end
            
    %% plots
    % these figures show a comparison between filtered and unfiltered data

    if Plot_kinematics == true

        if filter == true

            figure
            hold on
            plot(time,phi_f)
            plot(time,phi)
            legend(["Filtered" "Unfiltered"])
            title("Input Data: Phi (Stroke Angle)")
            hold off
        
            figure
            hold on
            plot(time,alpha_f)
            plot(time,alpha)
            legend(["Filtered" "Unfiltered"])
            title("Input Data: Psi (Rotation Angle)")
            hold off
        
            figure
            hold on
            plot(time,gamma_f)
            plot(time,gamma)
            legend(["Filtered" "Unfiltered"])
            title("Input Data: Beta (Deviation Angle)")
            hold off
    
            figure
            hold on
            plot(time,phi)
            plot(time,alpha)
            plot(time,gamma)
            legend(["Phi (Stroke Angle)" "Psi (Rotation Angle)" "Beta (Deviation Angle)"])
            title("Unfiltered Angles")
            hold off
            
            figure
            hold on
            plot(time,phi_f)
            plot(time,alpha_f)
            plot(time,gamma_f)
            legend(["Phi (Stroke Angle)" "Psi (Rotation Angle)" "Beta (Deviation Angle)"])
            title("Filtered Angles")
            hold off

            figure
            hold on
            plot(time,phi_f)
            plot(time,alpha_f)
            plot(time,gamma_f)
            legend(["Phi (Stroke Angle)" "Psi (Rotation Angle)" "Beta (Deviation Angle)"])
            title("Filtered Angles")
            hold off

        else
        
            figure
            hold on
            plot(time,phi_f)
            title("Input Data: Phi (Stroke Angle)")
            hold off
        
            figure
            hold on
            plot(time,alpha_f)
            title("Input Data: Psi (Rotation Angle)")
            hold off
        
            figure
            hold on
            plot(time,gamma_f)
            title("Input Data: Beta (Deviation Angle)")
            hold off

        end

            figure
            hold on
            plot(time,phi)
            plot(time,alpha)
            plot(time,gamma)
            legend(["Phi (Stroke Angle)" "Psi (Rotation Angle)" "Beta (Deviation Angle)"])
            title("Original Angles")
            hold off



    end

end