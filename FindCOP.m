function element=FindCOP(psi,n,c,R_inv2,wing_length)
    %% Starting message
    disp('Location of COP Calculation - Start')
    
    %% Find the location of center of pressure

    delz = wing_length / n;                  % Precompute delz
    psi = abs(psi);         % Precompute radians once

    x_cp = zeros(n, length(psi));          % Preallocate x_cp array
    element(n).Distance_COP = [];            % Preallocate struct array

    % Preallocate vectors for efficiency
    r_cp_vec = zeros(3, length(psi));      
    r_cp = zeros(3, length(psi));

    % figure;
    % hold on;

    for j = 1:n
        % Calculate x_cp for the current element
        x_cp(j, :) = c(j) * (0.82 * psi / pi + 0.05);

        % Iterate over each time step
        for i = 1:length(x_cp(j,:))
            % Vector to center of pressure for element j
            r_cp_vec(:, i) = [delz/2 + delz*(j - 1); 0; x_cp(j, i)];
            %r_cp_vec(:, i) = [delz/2 + delz*(j - 1); 0; 0]; %Zafar model

            % Rotate the vector using the inverse rotation matrix R_inv2
            r_cp(:, i) = R_inv2(:, :, i) * r_cp_vec(:, i);

            % Store the distance of COP
            element(j).Distance_COP(i) = norm(r_cp(:, i));
        end
        % plot(r_cp(1,:)')

        % Store location data for the element
        element(j).location_cop = r_cp;
        element(j).locationInMovingFrame = r_cp_vec;
    end

    % hold off;

    % figure
    % plot(x_cp(:,:)')
    % disp('Complete for entire wing');
    
    %% Ending message
    disp('Location of COP Calculation - End')
    
    %% test plot for the center of pressure

    % figure
    % hold on
    % for j=1:length(element)
    %     plot3(element(j).location_cop(1,1),element(j).location_cop(2,1),element(j).location_cop(3,1),'*');
    %     plot3(element(j).location_cop(1,end),element(j).location_cop(2,end),element(j).location_cop(3,end),'x');
    %     plot3(element(j).location_cop(1,:),element(j).location_cop(2,:),element(j).location_cop(3,:));  
    % end
    % legend(["Start" "End"])
    % title({'Location of centor of pressure for all element throughout', 'a wingstroke in the statonary wing frame'})
    % view([-40.1999996505678 27.6000011477619]);
    % grid on;
    % hold off

end