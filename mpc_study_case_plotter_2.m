
taille = length(list_Uopt_nonlin);
% e_x_COSF = x_COSF(1:end-1) - xdes(1:taille);
% e_x_OSF = x_OSF(1:end-1) - xdes(1:taille);
% e_x_MPC = x_nonlin(1:end-1) - xdes(1:taille);
% e_x_LQR2 = x_nonlin_LQR2(1:end-1) - xdes(1:taille);

e_t_COSF = theta_COSF(1:end-1) - thetades(1:taille);
e_t_OSF = theta_OSF(1:end-1) - thetades(1:taille);
e_t_MPC = theta_nonlin(1:end-1) - thetades(1:taille);
e_t_LQR = theta_nonlin_LQR2(1:end-1) - thetades(1:taille);
e_t_LQRI = theta_nonlin_LQRI(1:end-1) - thetades(1:taille);

e_t_feed_lin = theta_feed_lin(1:end-1) - thetades(1:taille);
e_t_NMPC = theta_NMPC(1:end-1) - thetades(1:taille);

##e_t_COSF(end)
##e_t_OSF(end)
##e_t_MPC(end)
##e_t_LQR(end) % LQR recursif
##e_t_LQRI(end)
%%

figure();
% subplot(2,1,1)
plot(T(1:end-p),180/pi*e_t_MPC(1:end),'LineWidth',2)
grid()
hold on
%plot(T(1:end-p),theta_LQR(1:end)','LineWidth',2)
plot(T(1:end-p),180/pi*e_t_LQR(1:end),'LineWidth',2,'linestyle','--')
plot(T(1:end-p),180/pi*e_t_LQRI(1:end),'LineWidth',2)
plot(T(1:end-p),180/pi*e_t_OSF(1:end),'LineWidth',2)
plot(T(1:end-p),180/pi*e_t_COSF(1:end),'LineWidth',2)
plot(T(1:end-p),180/pi*e_t_feed_lin(1:end),'LineWidth',2)
plot(T(1:end-p),180/pi*e_t_NMPC(1:end),'LineWidth',2)
hold off
legend('e_{\theta,MPC}','e_{\theta,LQR}','e_{\theta,LQRI}','e_{\theta,OSF-MPC}','e_{\theta,COSF-MPC}','e_{\theta,feed lin}','e_{\theta,NMPC}','fontsize',13) %'\theta des'
xlabel('Time (s)')
ylabel('Tracking error e_{\theta} (in °)')
%title('Comparatif erreurs de suivi')
title({'Comparison tracking error for different controllers',plot_subtitle},'fontsize',16)
% subplot(2,1,2)
% plot(T(1:end-p),e_x_MPC(1:end),'LineWidth',2)
% hold on
% %plot(T(1:end-p),x_LQR(1:end)','LineWidth',2)
% plot(T(1:end-p),e_x_LQR2(1:end),'LineWidth',2)
%
% plot(T(1:end-p),e_x_OSF(1:end),'LineWidth',2)
% hold off
% grid()
% legend('x_{MPC}','x_{LQR2}','x OSFMPC') %'x des'
% xlabel('time (s)')
