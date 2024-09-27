function [omega, omega_mag, alpha, alpha_mag] = GetWingAngVel(ex1, ey1, ez1, phi_dotf, alpha_dotf, gamma_dotf, phi_dot_dotf, alpha_dot_dotf, gamma_dot_dotf)

    %% Angular velocity in vector form  
    disp('Calculating the angular velocity in vector form')
    
    omega = zeros(3,length(phi_dotf));
    
    for i=1:length(phi_dotf)
        omega(1:3,i) = phi_dotf(i)*ez1(1:3,i) + gamma_dotf(i)*ey1(1:3,i) + alpha_dotf(i)*ex1(1:3,i);
    end
    
    disp('Done with vector ang velolcity')
    
    %% Magnitude of ang vel in deg/s
    disp('Calculating magnitude of angular velocity')

    omega_mag = vecnorm(omega);

    [b, a] = butter(2, 0.2, 'low');

    omega_mag = filtfilt(b, a, omega_mag);
    
    disp('Finished calulating ang vel magnitude')

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Angular acceleration in vector form
    disp('Calculating angular acceleration in vector form')

    alpha = zeros(3,length(phi_dot_dotf));

    for i=1:length(phi_dot_dotf)
        alpha(1:3,i) = phi_dot_dotf(i)*ez1(1:3,i) + gamma_dot_dotf(i)*ey1(1:3,i) + alpha_dot_dotf(i)*ex1(1:3,i);
    end

    disp('Done with angular acceleration')

    %% Magnitude of ang accel in deg/s 
    disp('Calculating magnitude of angular acceleration')

    alpha_mag = vecnorm(alpha);

    alpha_mag= filtfilt(b, a, alpha_mag);
      
    disp('Finished calulating ang accel magnitude')

end