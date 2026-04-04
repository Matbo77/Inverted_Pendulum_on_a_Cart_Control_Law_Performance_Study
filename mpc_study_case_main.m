
close all
%clear

% Packages loading for Octave
pkg load control
pkg load optim

%% PB constants
m_pend = 0.2; %
M = 2.3;
l_pend = 0.2; %0.3
g = 9.81;
psi = 5*10^(-5);
phi = 5*10^(-3);
A = [ 0 1 0 0; 0 -1/M*psi -m_pend*g/M 0 ; 0 0 0 1 ; 0 1/(M*l_pend)*psi g/(M*l_pend)*(m_pend+M) -1/(m_pend*l_pend^2)*phi];
B = [0 1/M 0 -1/(M*l_pend)]';
[n,~] = size(A);

C = [0 0 1 0];
% X = [ x tilde ; dx ; theta tilde ; dtheta ]
[Ny,c] = size(C);
rank(ctrb(A,B));
rank(ctrb(A',C')); %rank(obsv(A,C))
D = zeros(Ny,1);
ss_c = ss(A,B,C,D);
tf_c = tf(ss_c);
Te = 0.01; % eig(A) : 0,0,-6,5.8     Te = 0.01
liste_param = [M m_pend l_pend psi phi Te];
ss_d = c2d(ss_c,Te);
Ad = ss_d.A; % = expm(A*Te)
Bd = ss_d.B; % = A\(expm(A*Te)-eye(4))*B
Cd = ss_d.C;
Dd = ss_d.D;
rank(ctrb(Ad,Bd));

normalize_angle = @(angle)  mod(angle+pi,2*pi)-pi; %between, [-pi,pi]

[Ny,~] = size(Cd);
[~,Nu] = size(Bd);

% CI
x_0 = 0.0;
theta_0 = 40*pi/180; %40 %60
X0 = [x_0 ; 0 ; theta_0 ; 0];
t_f = 6;
T = 0:Te:t_f;
run mpc_study_case_traj.m
p = 50; %% --> prediction horizon
% 50*Te = 0.5 s

% J = (Yref(1:l*p)-Yp)'*Gamma_y*(Yref(1:l*p)-Yp) + Uopt'*Gamma_u*Uopt
Gamma_y = 100; %10^(3)*eye(l*p);
Gamma_u = 0.01;

last_element = Cd;
Sx = [];
Su = zeros(Ny*p,p);
for k=1:p
    last_element = last_element*Ad; %Cd*Ad^(k)
    Sx = [Sx ; last_element];
    for i=1:k
        Su(Ny*(k-1)+1:Ny*k,i) = Cd*Ad^(k-i)*Bd;
    end
end

%% Main

%% System : Linear
titre = 'Asservissement du système linéarisé';
linear = 1; %use on the linear system
allow_pert = 0; % pas de perturbation
t_f = 2; % temps final
run mpc_study_case_nl.m
run mpc_study_case_plotter_nl.m

%% System : Non-linear without perturbation
linear = 0; % use non linear
allow_pert = 0;
t_f = 2;
plot_subtitle = 'for the nonlinear system';
run mpc_study_case_nl.m
% Retour d'Etat diverge en cas de grosse incertitudes paramètriques
figure();
subplot(2,1,1)
plot(T(1:end-p),180/pi*theta_nonlin(1:end-1),'LineWidth',2)
grid()
hold on
plot(T(1:end-p),180/pi*theta_nonlin_LQR2(1:end-1),'LineWidth',2,'linestyle','--')
plot(T(1:end-p),180/pi*theta_nonlin_LQRI(1:end-1),'LineWidth',2)
plot(T(1:end-p),180/pi*theta_OSF(1:end-1),'LineWidth',2)
plot(T(1:end-p),180/pi*theta_COSF(1:end-1),'LineWidth',2)
plot(T(1:end-p),180/pi*theta_feed_lin(1:end-1),'LineWidth',2)
%plot(T(1:end-p),180/pi*thetades(1:end-p))
hold off
legend('\theta_{MPC} ( )','\theta_{LQR}','\theta_{LQRI}','\theta_{OSFMPC}','\theta_{COSFMPC}','\theta_{feed lin}') %,'\theta des'
xlabel('Time (s)')
ylabel("Angle (in °)")
title('Asservissement système non-linéaire')
%title('')

subplot(2,1,2)
plot(T(1:end-p),list_Uopt_nonlin,'LineWidth',2)
hold on
plot(T(1:end-p),Uopt_nonlin_LQR2,'LineWidth',2,'linestyle','--')
plot(T(1:end-p),Uopt_nonlin_LQRI,'LineWidth',2)
plot(T(1:end-p),list_Uopt_OSF,'LineWidth',2)
plot(T(1:end-p),list_Uopt_COSF,'LineWidth',2)
plot(T(1:end-p),list_U_feed_lin,'LineWidth',2)
hold off
grid
legend('Uopt_{MPC}','Uopt_{LQR}','Uopt_{LQRI}','Uopt_{OSFMPC}','Uopt_{COSFMPC}','U_{feed lin}')
ylabel("Control input U")
xlabel('Time (s)')

run mpc_study_case_plotter_2.m

%% System : Non-linear with perturbation
linear = 0;
allow_pert = 1;
plot_subtitle = 'for the nonlinear perturbed system';
t_f = 4.5;
run mpc_study_case_nl.m

figure();
subplot(2,1,1)
plot(T(1:end-p),180/pi*theta_nonlin(1:end-1),'LineWidth',2)
grid()
hold on
plot(T(1:end-p),180/pi*theta_nonlin_LQR2(1:end-1),'LineWidth',2,'linestyle','--')
plot(T(1:end-p),180/pi*theta_nonlin_LQRI(1:end-1),'LineWidth',2)
plot(T(1:end-p),180/pi*theta_OSF(1:end-1),'LineWidth',2)
plot(T(1:end-p),180/pi*theta_COSF(1:end-1),'LineWidth',2)
plot(T(1:end-p),180/pi*theta_feed_lin(1:end-1),'LineWidth',2)
%plot(T(1:end-p),180/pi*thetades(1:end-p))
hold off
legend('\theta_{MPC}','\theta_{LQR}','\theta_{LQRI}','\theta_{OSFMPC}','\theta_{COSFMPC}','\theta_{feed lin}') %,'\theta des'
xlabel('Time (s)')
ylabel("Angle (in °)")
title('Asservissement système non-linéaire perturbé')
%title('')

subplot(2,1,2)
plot(T(1:end-p),list_Uopt_nonlin,'LineWidth',2)
hold on
plot(T(1:end-p),Uopt_nonlin_LQR2,'LineWidth',2,'linestyle','--')
plot(T(1:end-p),Uopt_nonlin_LQRI,'LineWidth',2)
plot(T(1:end-p),list_Uopt_OSF,'LineWidth',2)
plot(T(1:end-p),list_Uopt_COSF,'LineWidth',2)
plot(T(1:end-p),list_U_feed_lin,'LineWidth',2)
hold off
grid
legend('Uopt_{MPC}','Uopt_{LQR}','Uopt_{LQRI}','Uopt_{OSFMPC}','Uopt_{COSFMPC}','U_{feed lin}')
ylabel("Control input U")
xlabel('Time (s)')

run mpc_study_case_plotter_2.m

% print("Tracking_error_nonlin_perturbed4.png" ,"-S700,600")


% Plot x position

##figure();
##plot(T(1:end-p),x_nonlin(1:end-1),'LineWidth',2)
##grid()
##hold on
##plot(T(1:end-p),x_nonlin_LQR2(1:end-1),'LineWidth',2,'linestyle','--')
##plot(T(1:end-p),x_nonlin_LQRI(1:end-1),'LineWidth',2)
##plot(T(1:end-p),x_OSF(1:end-1),'LineWidth',2)
##plot(T(1:end-p),x_COSF(1:end-1),'LineWidth',2)
##hold off
##legend('x_{MPC}','x_{LQR}','x_{LQRI}','x_{OSFMPC}','x_{COSFMPC}') %,'\theta des'
##xlabel('Time (s)')
##ylabel("Position x (m)")
##title('Asservissement système non-linéaire perturbé')


N_start = 10;
Nsim = N-p+1;

% WIth perturbaiton and parameter uncertainties
RMSE_SF = sqrt(mean((theta_SF(N_start:Nsim)-thetades(N_start:Nsim)).^2))

RMSE_LQR = sqrt(mean((theta_nonlin_LQR2(N_start:Nsim)-thetades(N_start:Nsim)).^2))
RMSE_LQRI = sqrt(mean((theta_nonlin_LQRI(N_start:Nsim)-thetades(N_start:Nsim)).^2))
RMSE_SSMPC = sqrt(mean((theta_nonlin(N_start:Nsim)-thetades(N_start:Nsim)).^2))
RMSE_OSF = sqrt(mean((theta_OSF(N_start:Nsim)-thetades(N_start:Nsim)).^2))
RMSE_COSF = sqrt(mean((theta_COSF(N_start:Nsim)-thetades(N_start:Nsim)).^2))

RMSE_feed_lin = sqrt(mean((theta_feed_lin(N_start:Nsim)-thetades(N_start:Nsim)).^2))

%RMSE_NMPC =










