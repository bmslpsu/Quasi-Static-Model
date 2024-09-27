function [ex1, ey1, ez1, R_inv2, R_2]=Find_vectors(phi_f,alpha_f,gamma_f)
    %this function finds the vectors in hopes of increasing the speed of
    %the code

    %% Stationary wing reference frame
    ex=[1;0;0];
    ey=[0;1;0];
    ez=[0;0;1];

    %% Euler Transfermation
    [R_inv, R, Rz_inv, Ry_inv, Rx_inv] = EulerRotation();

    % define the initial vectors and transformation
    ex1=zeros(3,length(phi_f));
    ey1=zeros(3,length(phi_f));
    ez1=zeros(3,length(phi_f));
    R_inv2=zeros(size(R_inv,1),size(R_inv,2),length(phi_f));
    R_2=zeros(size(R_inv,1),size(R_inv,2),length(phi_f));

    % calculate it for each instance
    for i=1:length(phi_f)

        gamma_sym = gamma_f(i);
        alpha_sym = alpha_f(i);
        phi_sym = phi_f(i);

        % ex1(1:3,i)=vpa(subs(R_inv*ex));
        % ey1(1:3,i)=vpa(subs(R_inv*ey));
        % ez1(1:3,i)=vpa(subs(R_inv*ez));

        ex1(1:3,i)=vpa(subs(Rz_inv*ex));
        ey1(1:3,i)=vpa(subs(Rz_inv*Ry_inv*ey));
        ez1(1:3,i)=vpa(subs(Rz_inv*Ry_inv*Rx_inv*ez));


        R_inv2(:,:,i)=vpa(subs(R_inv));
        R_2(:,:,i)=double(vpa(subs(R)));

    end
end

function [R_inv, R, Rz_inv, Ry_inv, Rx_inv] = EulerRotation()
%% rotations
    %this function finds the rotation matrix from the fly stationary wing axis
    %to the moving wing axis.
    
    %R is the rotation from the stationary to the moving. Therefore finding its
    %inverse is required
    
    % important note: as the wing is always moving, we cannot have one rotation
    % matrix. To avoid having many matricies, i used a symbolic representation
    % for this rotation. R_ivn; the output is a symbolic matrix which is
    % evaluated for every datapoint of the euler angles when needed. however,
    % it is not saved
    
    syms gamma_sym phi_sym alpha_sym
    
    % Original
    % Rx = [1 0 0;
    %       0 cosd(gamma_sym) -sind(gamma_sym);
    %       0 sind(gamma_sym) cosd(gamma_sym)];
    % 
    % Ry = [cosd(phi_sym) 0 sind(phi_sym);
    %       0 1 0;
    %           -sind(phi_sym) 0 cosd(phi_sym)];
    % 
    % Rz = [cosd(alpha_sym) -sind(alpha_sym) 0;
    %       sind(alpha_sym) cosd(alpha_sym) 0;
    %       0 0 1];

    Rx = [1 0 0;
          0 cosd(alpha_sym) -sind(alpha_sym);
          0 sind(alpha_sym) cosd(alpha_sym)];

    Ry = [cosd(gamma_sym) 0 sind(gamma_sym);
          0 1 0;
          -sind(gamma_sym) 0 cosd(gamma_sym)];

    Rz = [cosd(phi_sym) -sind(phi_sym) 0;
          sind(phi_sym) cosd(phi_sym) 0;
          0 0 1];

    Rx_inv = transpose(Rx);
    Ry_inv = transpose(Ry);
    Rz_inv = transpose(Rz);
    
    
    R=Rz*Ry*Rx; % complete rotation from the stationary wing base frame to the moving wing frame
        
    R_inv=inv(R);

end
