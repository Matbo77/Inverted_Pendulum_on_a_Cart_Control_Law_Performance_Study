%% Plot  non-lin
% toutes les commandes
figure();
subplot(2,1,1)
plot(T(1:end-p),180/pi*theta_nonlin(1:end-1),'LineWidth',2)
grid()
hold on
plot(T(1:end-p),180/pi*theta_SF(1:end-1)','LineWidth',2)
plot(T(1:end-p),180/pi*theta_nonlin_LQR2(1:end-1),'LineWidth',2,'linestyle','--')
plot(T(1:end-p),180/pi*theta_nonlin_LQRI(1:end-1),'LineWidth',2)
plot(T(1:end-p),180/pi*theta_OSF(1:end-1),'LineWidth',2)
plot(T(1:end-p),180/pi*theta_COSF(1:end-1),'LineWidth',2)
%plot(T(1:end-p),180/pi*thetades(1:end-p))
hold off
legend('\theta_{MPC} (°)','\theta_{SF}','\theta_{LQR}','\theta_{LQRI}','\theta_{OSFMPC}','\theta_{COSFMPC}') %,'\theta des'
xlabel('Time (s)')
ylabel("Angle \theta (°)")

title(titre)
%title('')

% subplot(3,1,2)
% plot(T(1:end-p),x_nonlin(1:end-1),'LineWidth',2)
% hold on
% %plot(T(1:end-p),x_LQR(1:end)','LineWidth',2)
% plot(T(1:end-p),x_nonlin_LQR2(1:end-1),'LineWidth',2)
% plot(T(1:end-p),x_OSF(1:end-1),'LineWidth',2)
% %plot(T(1:end-p),xdes(1:end-p))
% hold off
% grid()
% legend('x_{MPC}','x_{LQR2}','x_{OSFMPC}') %'x des'
% xlabel('time (s)')

subplot(2,1,2)
plot(T(1:end-p),list_Uopt_nonlin,'LineWidth',2)
hold on
plot(T(1:end-p),list_U_SF,'LineWidth',2)
plot(T(1:end-p),Uopt_nonlin_LQR2,'LineWidth',2,'linestyle','--')
plot(T(1:end-p),Uopt_nonlin_LQRI,'LineWidth',2)
plot(T(1:end-p),list_Uopt_OSF,'LineWidth',2)
plot(T(1:end-p),list_Uopt_COSF,'LineWidth',2)
hold off
grid
legend('Uopt_{MPC}','U_{SF}','Uopt_{LQR}','Uopt_{LQRI}','Uopt_{OSF-MPC}','Uopt_{COSF-MPC}')
xlabel('Time (s)')
ylabel("Control input U")
title('Commandes')


%{

figure();
subplot(2,1,1)
plot(T(1:end-p),180/pi*theta_nonlin(1:end-1),'LineWidth',2)
grid()
hold on
%plot(T(1:end-p),180/pi*theta_LQR(1:end)','LineWidth',2)
plot(T(1:end-p),180/pi*theta_nonlin_LQR2(1:end-1),'LineWidth',2)
plot(T(1:end-p),180/pi*theta_OSF(1:end-1),'LineWidth',2)
plot(T(1:end-p),180/pi*thetades(1:end-p))
hold off
legend({'\theta_{MPC}','\theta_{LQR2}','\theta OSFMPC','\theta des'},'FontSize',11) %'\theta des'
ylabel('angles (en °)')
title('Suivi de consigne \theta')


subplot(2,1,2)
plot(T(1:end-p),list_Uopt_nonlin,'LineWidth',2)
hold on
%plot(T(1:end-p),Uopt_nonlin__LQR,'LineWidth',2)
plot(T(1:end-p),Uopt_nonlin_LQR2,'LineWidth',2)
plot(T(1:end-p),list_Uopt_OSF,'LineWidth',2)
hold off
grid
legend('Uopt_{MPC}','Uopt_{LQR2}','Uopt_{OSFMPC}')
xlabel('time (s)')



figure()
plot(T(1:end-p),x_nonlin(1:end-1),'LineWidth',2)
hold on
%plot(T(1:end-p),x_LQR(1:end)','LineWidth',2)
plot(T(1:end-p),x_nonlin_LQR2(1:end-1),'LineWidth',2)
plot(T(1:end-p),x_OSF(1:end-1),'LineWidth',2)
%plot(T(1:end-p),xdes(1:end-p))
hold off
ylabel('position (m)')
grid()
legend('x_{MPC}','x_{LQR2}','x OSFMPC') %'x des'
xlabel('time (s)')


%}
