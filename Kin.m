function Kinematics = Kin(Rotation, Stroke, Deviation, rad_or_deg, dt)
    
    %% Angular Postion (Kinematic data)
    if rad_or_deg == 0 %deg = 0 & rad = 1
        Kinematics.psi = deg2rad(Rotation);
        Kinematics.phi = deg2rad(Stroke);
        Kinematics.beta = deg2rad(Deviation);
    else
        Kinematics.psi = Rotation;
        Kinematics.phi = Stroke;
        Kinematics.beta = Deviation;
    end
    
    %% Angular Velocity
    % Calculates the angular velocity using a data diffrential function.
    Kinematics.phi_d = deriv(Kinematics.phi,dt);
    Kinematics.psi_d = deriv(Kinematics.psi,dt);
    Kinematics.beta_d = deriv(Kinematics.beta,dt);
    
    %% Angular Acceleration
    % Calculates the angular acceleration using a data diffrential function.
    Kinematics.phi_dd = deriv(Kinematics.phi_d,dt);
    Kinematics.psi_dd = deriv(Kinematics.psi_d,dt);
    Kinematics.beta_dd = deriv(Kinematics.beta_d,dt);
    
    %% Rotation Matricies in Stationary Frame
    % Calculate the vectors from the origin of the body frame and the
    % assoicated rotation matrix.
    [ex1, ey1, ez1, Kinematics] = Rotation_Matrix(Kinematics);

    %% Find Angular Velocity of each Wing with respect to Stationary Frame
    % This finds the magnitude of the angluar velocity with respect to the body frame
    [Kinematics] = Omega_and_Alpha(ex1, ey1, ez1, Kinematics);


end

%% Functions

function dx = deriv(x,dt)
    dx = zeros(size(x));
    dx_temp = diff(x)*(1/dt);
    
    dx(1,:) = dx_temp(1,:);
    dx(end,:) = dx_temp(end,:);
    dx(2:end-1,:) = 0.5*(dx_temp(1:end-1,:)+2*dx_temp(2:end,:));
end

function [ex1, ey1, ez1, kinematics] = Rotation_Matrix(kinematics)
    %this function finds the vectors in hopes of increasing the speed of
    %the code
    
    %% Euler Transfermation
    [R_inv, Rz_inv, Ry_inv, Rx_inv] = EulerRotation();
    
    %Array size
    N = length(kinematics.phi);

    % define the initial vectors and transformation
    ex1=zeros(3,3,N);
    ey1=zeros(3,3,N);
    ez1=zeros(3,3,N);
    
    % calculate it for each instance
    for i=1:N
    
        beta_sym = kinematics.beta(i);
        psi_sym = kinematics.psi(i);
        phi_sym = kinematics.phi(i);
    
        ex1(:,:,i)=vpa(subs(Rz_inv));
        ey1(:,:,i)=vpa(subs(Rz_inv*Ry_inv));
        ez1(:,:,i)=vpa(subs(Rz_inv*Ry_inv*Rx_inv));
    
        kinematics.R_inv2(:,:,i)=vpa(subs(R_inv));
    
    end
end

function [R_inv, Rz_inv, Ry_inv, Rx_inv] = EulerRotation()
    %this function finds the rotation matrix from the fly stationary wing axis
    %to the moving wing axis.
    
    %R is the rotation from the stationary to the moving. Therefore finding its
    %inverse is required
    
    % important note: as the wing is always moving, we cannot have one rotation
    % matrix. To avoid having many matricies, i used a symbolic representation
    % for this rotation. R_ivn; the output is a symbolic matrix which is
    % evaluated for every datapoint of the euler angles when needed. however,
    % it is not saved
    
    syms beta_sym phi_sym psi_sym
    
    Rx = [1 0 0;
        0 cos(psi_sym) -sin(psi_sym);
        0 sin(psi_sym) cos(psi_sym)];
    
    Ry = [cos(beta_sym) 0 sin(beta_sym);
        0 1 0;
        -sin(beta_sym) 0 cos(beta_sym)];
    
    Rz = [cos(phi_sym) -sin(phi_sym) 0;
        sin(phi_sym) cos(phi_sym) 0;
        0 0 1];
    
    Rx_inv = transpose(Rx);
    Ry_inv = transpose(Ry);
    Rz_inv = transpose(Rz);
    
    
    R=Rx*Ry*Rz; % complete rotation from the stationary wing base frame to the moving wing frame
    
    R_inv=inv(R);
end

function [Kinematics] = Omega_and_Alpha(ex1, ey1, ez1, Kinematics)
    % This finds the magnitude of the angluar velocity with respect to the body frame

    % coordinate system (body)
    % x-axis is to the side (right positive)
    % y-axis is to forward
    % z-axis is up  

    % Unit Vectors in body coordinate system
    ex=[1;0;0];
    ey=[0;1;0];
    ez=[0;0;1];

    %% Angular velocity in vector form  
    disp('Calculating the angular velocity in vector form')

    % Array size
    N = length(Kinematics.phi_d);

    Kinematics.omega = zeros(3,N);
    
    for i=1:N
        Kinematics.omega(1:3,i) = ez1(3,3,i)*Kinematics.phi_d(i)*ez + ey1(3,3,i)*Kinematics.beta_d(i)*ey + ex1(3,3,i)*Kinematics.psi_d(i)*ex;
    end
   
    disp('Done with vector ang velolcity')
    
    %% Magnitude of ang vel in deg/s
    disp('Calculating magnitude of angular velocity')

    omega_mag = vecnorm(Kinematics.omega);

    [b, a] = butter(2, 0.2, 'low');

    Kinematics.omega_mag = filtfilt(b, a, omega_mag);
    
    disp('Finished calulating ang vel magnitude')

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Angular acceleration in vector form
    disp('Calculating angular acceleration in vector form')

    Kinematics.alpha = zeros(3,N);

    for i=1:N
        Kinematics.alpha(1:3,i) = ez1(3,3,i)*Kinematics.phi_dd(i)*ez + ey1(3,3,i)*Kinematics.beta_dd(i)*ey + ex1(3,3,i)*Kinematics.psi_dd(i)*ex;
    end

    disp('Done with angular acceleration')

    %% Magnitude of ang accel in deg/s 
    disp('Calculating magnitude of angular acceleration')

    alpha_mag = vecnorm(Kinematics.alpha);

    Kinematics.alpha_mag= filtfilt(b, a, alpha_mag);
      
    disp('Finished calulating ang accel magnitude')

end