figure
hold on
plot(psi/(max(psi)))
plot(psi_dot/(max(psi_dot)))
plot(psi_dot_dot/(max(psi_dot_dot)))
legend(["\psi" "\psi_d_o_t" "\psi_d_o_t_ _d_o_t"])
title("\psi Analyis")
hold off

figure
hold on
plot(alpha_f)
plot(alpha_dotf)
plot(alpha_dot_dotf)
legend(["\alpha" "\alpha_d_o_t" "\alpha_d_o_t_ _d_o_t"])
title("\alpha Analyis")
hold off

%%
figure
hold on
plot(beta/(max(beta)))
plot(beta_dot/(max(beta_dot)))
plot(beta_dot_dot/(max(beta_dot_dot)))
legend(["\beta" "\beta_d_o_t" "\beta_d_o_t_ _d_o_t"])
title("\beta Analyis")
hold off

%%
figure
hold on
plot(phi/(max(phi)))
plot(phi_dot/(max(phi_dot)))
plot(phi_dot_dot/(max(phi_dot_dot)))
legend(["\phi" "\phi_d_o_t" "\phi_d_o_t_ _d_o_t"])
title("\phi Analyis")
hold off


%%
figure
hold on
plot(alpha_f)
plot(gamma_f)
plot(phi_f)
legend(["\alpha" "\gamma" "\phi"])
title("Kinematic Analyis")
hold off

%%
figure
hold on
plot(ex11')
legend(["X" "Y" "Z"])
title("ex11 Analyis")
hold off

figure
hold on
plot(ey11')
legend(["X" "Y" "Z"])
title("ey11 Analyis")
hold off

figure
hold on
plot(ez11')
legend(["X" "Y" "Z"])
title("ez11 Analyis")
hold off

figure
hold on
plot(omega')
legend(["X" "Y" "Z"])
title("\omega Analyis")
hold off

figure
hold on
plot(alpha')
legend(["X" "Y" "Z"])
title("\alpha Analyis")
hold off


figure
hold on
plot(omega_mag/max(omega_mag))
plot(alpha_mag/max(alpha_mag))
legend(["\omega" "\alpha"])
title("\omega & \alpha Magnitude Analyis")
hold off

figure
hold on
plot(omega_mag/max(omega_mag))
plot(omega_mag_test/max(omega_mag_test),"--")
plot(omega_mag_test2/max(omega_mag_test2),":")
legend(["Original" "Test" "Filtered:"])
title("\omega Magnitude Analyis")
hold off

figure
hold on
plot(alpha_mag/max(alpha_mag))
plot(alpha_mag_test/max(alpha_mag_test),"--")
legend(["Original" "Test"])
title("\alpha Magnitude Analyis")
hold off

[b, a] = butter(2, 0.2, 'low');
omega_mag_test2 = filtfilt(b, a, omega_mag_test);

figure
hold on
plot(Wing_Element_lh(4).location_cop(1:3,:)')
legend(["X" "Y" "Z"])
title("Location of COP")
hold off

%%
figure
hold on
plot((Wing_Element_lh(20).linear_vel(:,:)'))
legend(["X" "Y" "Z"])
title("Linear Velocity")
hold off

%%
figure
hold on
plot((Wing_Element_lh(20).linear_vel_norm(:)').^2)
title("Linear Velocity")
hold off

%%
figure
hold on
plot(Wing_Element_lh(20).linear_vel_direction')
title("Linear Velocity Direction")
hold off