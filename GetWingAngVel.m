function [omega, omega_mag, omega_rad]  =GetWingAngVel(ex1, ey1, ez1, phi_dotf, alpha_dotf, gamma_dotf, R_inv2)

    %% angular velocity in vector form
    % omega (deg)
    % omega_mag(deg)
    % omega_rad (rad)
    
    disp('Calculating the angular velocity in vector form')
    
    omega = zeros(3,length(phi_dotf));
    
    for i=1:length(phi_dotf)
        % Original
        % omega(1:3,i) = phi_dotf(i)*ey1(1:3,i) + alpha_dotf(i)*ez1(1:3,i) + gamma_dotf(i)*ex1(1:3,i);
        omega(1:3,i) = phi_dotf(i)*ez1(1:3,i) + alpha_dotf(i)*ey1(1:3,i) + gamma_dotf(i)*ex1(1:3,i);
    end
    
    disp('Done with vector ang vel')
    
    %% Magnitude of ang vel in deg/s
    
    disp('Calculating magnitude of angular velocity')

    omega_mag = zeros(1,length(omega));
    
    for i=1:length(omega)
        omega_mag(i) = norm(omega(1:3,i));
    end
    
    disp('Finished calulating ang vel magnitude')
    
    omega_rad = deg2rad(omega_mag); % magnitude in radians
end