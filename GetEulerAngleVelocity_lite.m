function [phi_dotf, psi_dotf, beta_dotf, phi_dot_dotf, psi_dot_dotf, beta_dot_dotf] = GetEulerAngleVelocity(time, phi_f, alpha_f, gamma_f, phi, alpha, gamma, Plot_kinematic_Velocity, filter)

    %This function finds the Euler velocity of the euler angles. Note:
    %whether it is filtered or not the out put is the "filtered" variables.
    %This may be imporved in the future for readability.

    %% filtering also to be done on the derivative of the angle
    % This is not needed

    % [b, a] = butter(3, 0.18,'low');
    % phi_dotf=filtfilt(b, a, phi_dotf);
    % psi_dotf=filtfilt(b, a, psi_dotf);
    % beta_dotf=filtfilt(b, a, beta_dotf);
    
    
    %% angular velocities

    phi_dot = diff(phi)/(time(2)-time(1));
    psi_dot = diff(alpha)/(time(2)-time(1));
    beta_dot = diff(gamma)/(time(2)-time(1));

    phi_dotf = diff(phi_f)/(time(2)-time(1));
    psi_dotf = diff(alpha_f)/(time(2)-time(1));
    beta_dotf = diff(gamma_f)/(time(2)-time(1));

    %% angular accelerations

    phi_dot_dot = diff(phi_dot)/(time(2)-time(1));
    psi_dot_dot = diff(psi_dot)/(time(2)-time(1));
    beta_dot_dot = diff(beta_dot)/(time(2)-time(1));

    phi_dot_dotf = diff(phi_dotf)/(time(2)-time(1));
    psi_dot_dotf = diff(psi_dotf)/(time(2)-time(1));
    beta_dot_dotf = diff(beta_dotf)/(time(2)-time(1));


    %% Plots

    if Plot_kinematic_Velocity == true

        if filter == true

            figure
            hold on
            plot(phi_dotf)
            plot(phi_dot)
            legend(["Phi_d_o_t_f" "Phi_d_o_t"])
            hold off
    
            figure
            hold on
            plot(psi_dotf)
            plot(psi_dot)
            legend(["Alpha_d_o_t_f" "Alpha_d_o_t"])
            hold off
    
            figure
            hold on
            plot(beta_dotf)
            plot(beta_dot)
            legend(["Gamma_d_o_t_f" "Gamma_d_o_t"])
            hold off

        else

            figure
            hold on
            plot(phi_dotf)
            legend(["Phi_d_o_t"])
            hold off
        
            figure
            hold on
            plot(psi_dotf)
            legend(["Alpha_d_o_t"])
            hold off
        
            figure
            hold on
            plot(beta_dot)
            legend(["Gamma_d_o_t"])
            hold off

        end


    end

end