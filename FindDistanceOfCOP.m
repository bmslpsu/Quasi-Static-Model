function element=FindDistanceOfCOP(psi_f,n,c,R_inv2,wing_length)
    %% Find the location of center of pressure

    delz=wing_length/n;


    % % Array initialization
    % r_cpp(k,:) = zeros(3,length(x_cp(k,:))-1);
    % r_cp(k,:)  = zeros(3,length(x_cp(k,:))-1);

    % finds the distance vector to each center of pressure for a each element
    for j=1:n
        disp(['Caclulating COP for element ' num2str(j) '/' num2str(n)]);

        x_cp(j,:) = c(j)*(0.82*abs(deg2rad(psi_f))/pi+0.05); %location of center of pressure in x-axis

        for i=1:length(x_cp(j,:))-1

            r_cpp(1:3,i) = [-x_cp(j,i);  0; delz/2+delz*(j-1)];
            r_cp(1:3,i) = vpa(subs(R_inv2(:,:,i)*r_cpp(1:3,i)));

            element(j).Distance_COP(i) = double(norm(r_cp(:,i)));

        end

        element(j).location_cop = double(r_cp);
        element(j).locationInMovingFrame = double(r_cpp);
        
    end

    disp('Complete for entire wing')
    
    %% test plot for the center of pressure

    % % figure
    % % hold on
    % % for j=1:length(element)
    % %     plot3(element(j).location_cop(1,1),element(j).location_cop(2,1),element(j).location_cop(3,1),'*');
    % %     plot3(element(j).location_cop(1,end),element(j).location_cop(2,end),element(j).location_cop(3,end),'x');
    % %     plot3(element(j).location_cop(1,:),element(j).location_cop(2,:),element(j).location_cop(3,:));  
    % % end
    % % legend(["Start" "End"])
    % % title({'Location of centor of pressure for all element throughout', 'a wingstroke in the statonary wing frame'})
    % % view([-40.1999996505678 27.6000011477619]);
    % % grid on;
    % % hold off

end