figure
hold on
plot(alpha_f/(max(alpha_f)))
plot(alpha_dotf/(max(alpha_dotf)))
plot(alpha_dot_dotf/(max(alpha_dot_dotf)))
legend(["\alpha" "\alpha_d_o_t" "\alpha_d_o_t_ _d_o_t"])
title("\alpha Analyis")
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
plot(gamma_f/(max(gamma_f)))
plot(gamma_dotf/(max(gamma_dotf)))
plot(gamma_dot_dotf/(max(gamma_dot_dotf)))
legend(["\gamma" "\gamma_d_o_t" "\gamma_d_o_t_ _d_o_t"])
title("\gamma Analyis")
hold off

%%
figure
hold on
plot(phi_f/(max(phi_f)))
plot(phi_dotf/(max(phi_dotf)))
plot(phi_dot_dotf/(max(phi_dot_dotf)))
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
plot((Wing_Element_lh(4).linear_vel(:,:)').^2)
legend(["X" "Y" "Z"])
title("Linear Velocity")
hold off

%%
figure
hold on
plot((Wing_Element_lh(4).linear_vel_norm(:)').^2)
title("Linear Velocity")
hold off
