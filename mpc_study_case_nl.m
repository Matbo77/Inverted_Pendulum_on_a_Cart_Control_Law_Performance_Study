%% Non-linear syst
%run mpc_study_case.m
T = 0:Te:t_f; N = length(T); run mpc_study_case_traj.m;
%p = 50; % prediciton horizon
Xk = X0;
%Xk2 = X0;
Xk_RK4 = [X0];
%Xk_RK2 = [X0];
Gamma_y = 100; %10^(3)*eye(l*p);
Gamma_u = 1;
% linear = 0; %use on the linear system
% allow_pert = 0;

% Uncertainties on dynamical prediction model
% négliger frottement phi = psi = 0
% imprécision masses / uncertainties on weight
m_pend_imp = 0*m_pend; % masse négligé complétement
M_imp = 0.7*M; %0.9
l_pend_imp = 0.7*l_pend; %1.3

if allow_pert==1
    A_incert = [ 0 1 0 0; 0 0 -m_pend_imp*g/M_imp 0 ; 0 0 0 1 ; 0 0 g/(M_imp*l_pend_imp)*(m_pend_imp+M_imp) 0];
    B_incert = [0 1/M_imp 0 -1/(M_imp*l_pend_imp)]';
else
    A_incert = A;
    B_incert = B;
end
ss_incert = ss(A_incert,B_incert,C,zeros(Ny,1));
ssd_incert = c2d(ss_incert,Te);
Ad_incert = ssd_incert.A; %expm(A_sans_frot*Te)
Bd_incert = ssd_incert.B;

% Constant perturbation on theta acceleration
% Perturbation sur a_theta
% from time t_pert on
t_pert = 2.5; % Te 1.2;
pert_theta = zeros(4,N);
%pert_theta(3,round(t_pert/Te):end) = 1*pi/180;
pert_theta(4,round(t_pert/Te):end) = allow_pert*5*pi/180;

%% State feedback SF
rank(ctrb(A,B))
t5des = 0.4; %entre 0.3 et 0.5s
%poles_des_disc = [exp(-6.3/t5des*Te) exp(-0/t5des*Te) exp(-6.31/t5des*Te)  exp(-6.32*Te) ]; %continous to discrete
poles_des_disc = [exp(-7.76/t5des*Te) exp(-7.77/t5des*Te) exp(-7.78/t5des*Te)  exp(-7.75*Te) ];
%P = poly([-7.76/t5des,-7.77/t5des,-7.78/t5des,-7.75/t5des]);
Kd_SF = place(Ad_incert,Bd_incert,poles_des_disc);

%1/(Cd*(eye(n)-Ad+Bd*Kd_SF)^(-1)*Bd)
%eig(Ad_incert-Bd_incert*Kd_SF)
ss_d_SF = ss(Ad_incert-Bd_incert*Kd_SF, [0 0 1 0]',Cd,zeros(1,1),Te);
tf_d_SF = tf(ss_d_SF);
%zero(tf_d_SF)
%figure();step(ss_d_SF);grid()

Xk = X0;
x_SF = [X0(1)];
theta_SF = [X0(3)];
N = length(T);
list_U_SF= [];

for j=1:(N-p)

    uk = -Kd_SF*Xk;
    if linear==1
        Xk = Ad*Xk + Bd*uk + pert_theta(:,j); % state update lin
    else
        Xk = RK4(Xk,uk,Te) + pert_theta(:,j); % state update non-lin + pert
    end
    list_U_SF= [list_U_SF ; uk];
    theta_SF = [theta_SF ; Xk(3)];
    x_SF= [x_SF ; Xk(1)];
end

% MPC sur non lin avec incertitudes prediction
% dans fonction de coût mettre commande et pas débit de commande
%p =40;
last_element = Cd;
Sx = [];
Su = zeros(Ny*p,p);
for k=1:p
    last_element = last_element*Ad_incert; %Cd*Ad^(k)
    Sx = [Sx ; last_element];
    for i=1:k
        Su(Ny*(k-1)+1:Ny*k,i) = Cd*Ad_incert^(k-i)*Bd_incert;
    end
end

Gamma_y = 400; %10^(3)*eye(l*p);
Gamma_u = 0.1;
Xk = X0;
x_nonlin = [X0(1)];
theta_nonlin = [X0(3)];
N = length(T);
H = Su'*Gamma_y*Su + Gamma_u*eye(p);
list_Uopt_nonlin = [];
facteur_y = 10;
%Yref = [Yref ; ones(6,1)*Yref(end-1:end)%extend Yref
for j=1:(N-p)
    %grad = -Su'*Gamma_y*Yref(Ny*(j-1)+1:Ny*(j-1)+Ny*p)'+Su'*Gamma_y*Sx*Xk;
%
    if j> round(0.35/Te) %  aux instants finaux, on augmente le facteur pénalisant l'erreur ey
         %grad = Su'*(facteur_y*Gamma_y)*(Sx*Xk-Yref(Ny*(j-1)+1:Ny*(j-1)+Ny*p));
         %H = Su'*(facteur_y*Gamma_y)*Su + Gamma_u*eye(p);
         grad = Su'*Gamma_y*(Sx*Xk -Yref(Ny*(j-1)+1:Ny*(j-1)+Ny*p));
    else
        grad = Su'*Gamma_y*(Sx*Xk -Yref(Ny*(j-1)+1:Ny*(j-1)+Ny*p));

    %grad = S_Delta_u'*Gamma_y*(S_Delta_x*zk - Yref(Ny*(j-1)+1:Ny*(j-1)+Ny*p))
    end

    Uopt = -H^(-1)*grad;
    uk = Uopt(1); % we only keep the first input
    if linear==1
        Xk = Ad*Xk + Bd*uk + pert_theta(:,j); % state update
    else
        Xk = RK4(Xk,uk,Te) + pert_theta(:,j); % state update non-lin + pert
    end
    list_Uopt_nonlin = [list_Uopt_nonlin ; uk];
    theta_nonlin = [theta_nonlin ; Xk(3)];
    x_nonlin = [x_nonlin ; Xk(1)];
end

%% LQR recursif (horizon fini)
% u(k) = - K(k)xk
% sum[(y-ydes)'Q(y-ydes)+u'Ru] = sum[x_e'Q_ex_e + u'Ru]
% x_e = [x ; dx ; theta ; dtheta ; theta_des]
% y = [e_theta]
Ades = eye(Ny);
Ae = [Ad_incert, zeros(n,Ny) ; zeros(Ny,n), Ades]; %augmentation state with desired output
Be = [Bd_incert ; zeros(Ny,1)];
%Ce = [1 0 0 0 -1 0 ; 0 0 1 0 0 -1];
Ce = [0 0 1 0 -1]; %e_theta = theta - theta des

Q = 400; %4000 % Si Q grand syst plus rapide
Qf = 600; %4000
R = 0.1;
Qe = Ce'*Q*Ce; % Q
N_h = N-p;% 5/Te-1; %horizon
Xk_LQR2 = [X0 ; Yref(1) ]; %; Yref(2)
theta_nonlin_LQR2 = Xk_LQR2(3);
x_nonlin_LQR2 = Xk_LQR2(1);

P_LQR = Ce'*Qf*Ce;
K_opt2 = zeros(N_h,n+Ny);
Qe_k = [kron(ones(1,N_h-9),Qe), kron(ones(1,9),P_LQR)]; %different weight Qe matrix
for k=1:N_h
    K_opt2(N_h+1-k,:) = (R + Be'*P_LQR*Be)^(-1)*Be'*P_LQR*Ae;
    Qek = Qe_k(1:n+Ny,(n+Ny)*(N_h-k)+1:(n+Ny)*(N_h+1-k));
    P_LQR = Ae'*P_LQR*Ae - Ae'*P_LQR*Be*(R + Be'*P_LQR*Be)^(-1)*Be'*P_LQR*Ae + Qek ;

end

Uopt_nonlin_LQR2 = [];
for k=1:N_h
    Uopt_nonlin_LQR2(end+1) = - K_opt2(k,:)*Xk_LQR2; %N_h+1-k
    %Xk_LQR2 =  Ae*Xk_LQR2 + Be*Uopt_LQR2(end); %state update
    uk = Uopt_nonlin_LQR2(end);
    Xk = Xk_LQR2(1:4);
    if linear==1
        Xk = Ad*Xk + Bd*uk + pert_theta(:,j); % state update
    else
        Xk = RK4(Xk,uk,Te) + pert_theta(:,j); % state update non-lin + pert
    end
    Xk_LQR2(1:4) = Xk;
    theta_nonlin_LQR2 = [theta_nonlin_LQR2 ; Xk_LQR2(3)];
    x_nonlin_LQR2 = [x_nonlin_LQR2 ; Xk_LQR2(1)];
    %dtheta_LQR2 = [dtheta_LQR2 ; Xk_LQR2(4)];
end


%% LQRI Integral recursif (horizon fini)
% u(k) = - K.xk + KI.(thetades-theta)
% sum[(y-ydes)'Q(y-ydes)+u'Ru] = sum[x_e'Q_ex_e + u'Ru]
% x_e = [x ; dx ; theta ; dtheta ; theta - thetades ] %; e_theta
% y = [theta]
%Ades = eye(Ny);
Adi = [Ad_incert, zeros(n,Ny) ; Cd, 1]; %augmentation state with desired output
%Adi = Ad_incert;
Bdi = [Bd_incert ; 0]; % [Bd_incert ; 1]
%Bdi = Bd_incert;
%Ce = [1 0 0 0 -1 0 ; 0 0 1 0 0 -1];
Cdi = [Cd 1]; %theta
%Cdi = Cd;
Cd_out = eye(n); %n+1
Cd_er = [ 0 0 0 0 1];
rank(ctrb(Adi,Bdi));
rank(ctrb(Adi',Cdi'));
q = 0.1; %*eye(n+1) %100 1000
Qi =  Cdi'*q*Cdi; %q*eye(n+1);Cd_er'*q*Cd_er; %; %Cdi'*Q*Cdi;

%K_global_LQRI = dlqr(Adi,Bdi,Qi,R);
%K_LQRI = K_global_LQRI(1:n)
%Ki_LQRI = K_global_LQRI(n+1)

N_h = N-p;% 5/Te-1; %horizon
Xk_LQRI = [X0; Cd*X0- thetades(1)]; %; Yref(2); thetades(1) - Cd*X0
theta_nonlin_LQRI = Xk_LQRI(3);
x_nonlin_LQRI = Xk_LQRI(1);
%int_e_t_LQRI = Xk_LQRI(n+1);
Uopt_nonlin_LQRI = [];
int_err_theta = 0;

Q = 400; %4000
Qf = 600; %4000
R = 0.1;
Qe = Ce'*Q*Ce;
P_LQR = Cdi'*Qf*Cdi;
K_opt_LQRI = zeros(N_h,n+Ny);
Qe_k = [kron(ones(1,N_h-1),Qe), kron(ones(1,1),P_LQR)]; %different weight Qe matrix
for k=1:N_h %N_h+1-
    K_opt_LQRI(k,:) = -(R + Bdi'*P_LQR*Bdi)^(-1)*Bdi'*P_LQR*Adi; %attention signe
    Qek = Qe_k(1:n+Ny,(n+Ny)*(N_h-k)+1:(n+Ny)*(N_h+1-k));
    %Qek = Qf;
    P_LQR = Adi'*P_LQR*Adi - Adi'*P_LQR*Bdi*(R + Bdi'*P_LQR*Bdi)^(-1)*Bdi'*P_LQR*Adi + Qek ;
end
Delta_xk = zeros;
uk = 0; % u before
Xk = X0;
for k=1:N_h
    %Delta_u = - K_opt_LQRI(k,:)*Xk_LQR2;
    Uopt_nonlin_LQRI(end+1) = uk + K_opt_LQRI(k,1:n)*Xk_LQRI(1:n) + K_opt_LQRI(k,n+1)*Xk_LQRI(n+1);
    %Uopt_nonlin_LQRI(end+1) = - K_LQRI*Xk_LQRI(1:n) - 1*Ki_LQRI*Xk_LQRI(n+1); % + thetades
    %Uopt_nonlin_LQRI(end+1) = - K_opt2(k,:)*Xk_LQRI - 0.01*int_err_theta;
    %Xk_LQR2 =  Ae*Xk_LQR2 + Be*Uopt_LQR2(end); %state update
    uk = Uopt_nonlin_LQRI(end);
    Xk_before = Xk;
    if linear==1
        Xk = Ad*Xk + Bd*uk + pert_theta(:,j); % state update linear
    else
        Xk = RK4(Xk,uk,Te) + pert_theta(:,j); % state update non-lin + pert
    end
    Xk_LQRI(1:n) = Xk-Xk_before;
    Xk_LQRI(n+1) = Cd*Xk -thetades(k); %update tracking error
    %Xk_LQRI(n+1) = Xk_LQRI(n+1) + thetades(k) - Cdi*Xk_LQRI; %update integral error
    theta_nonlin_LQRI = [theta_nonlin_LQRI ; Xk(3)];
    x_nonlin_LQRI = [x_nonlin_LQRI ; Xk(1)];
    int_err_theta = int_err_theta + thetades(k) - Xk(3);
    %int_e_t_LQRI = [int_e_t_LQRI ; Xk_LQRI(n+1)];
end

%% MPC off-set free (OSF MPC) sur non lin avec incertitude prediction

Phi = [Ad_incert zeros(4,Ny) ; Cd*Ad_incert eye(Ny)];
Lambda = [Bd_incert  ; Cd*Bd_incert];
Xi = [ zeros(Ny,4) eye(Ny)];
m = p; % -->> horizon de commande

last_element = Xi;
S_Delta_x = []; %Delta y
S_Delta_u = zeros(Ny*p,Nu*m);
for k=1:p
    last_element = last_element*Phi; %Cd*Ad^(k)
    S_Delta_x = [S_Delta_x ; last_element];
    for i=1:k
        if k <= m
        %condition on m
            S_Delta_u(Ny*(k-1)+1:Ny*k,i) = Xi*Phi^(k-i)*Lambda;
        end
    end
end

Gamma_y = 70; %10^(3)*eye(l*p);
Gamma_u = 0.1;
uk = 0; % u initiale
Xk = X0;
zk = [zeros(4,1); Cd*X0];  % z_k = [Delta_Xk ; y_k ]
x_OSF = [X0(1)];
theta_OSF= [X0(3)];
N = length(T);
Tri_inf = zeros(m,m);
for j=1:m
    Tri_inf(j,1:j) = ones(1,j);
end
%H = (S_Delta_u')*Gamma_y*S_Delta_u + (Tri_inf')^(-1)*Gamma_u*(Tri_inf)^(-1)*eye(m);
H = S_Delta_u'*Gamma_y*S_Delta_u + Gamma_u*eye(m);
list_Uopt_OSF = [];
%facteur_y = 10^(1);

for j=1:(N-p)
    %grad = -Su'*Gamma_y*Yref(Ny*(j-1)+1:Ny*(j-1)+Ny*p)'+Su'*Gamma_y*Sx*Xk;

    %if j> N-p-round(2/Te) %  aux instants finaux, on augmente le facteur pénalisant l'erreur ey
        %grad = -S_Delta_u'*(facteur_y*Gamma_y)*Yref(Ny*(j-1)+1:Ny*(j-1)+Ny*p)+S_Delta_u'*(facteur_y*Gamma_y)*Sx*Xk;
        %H = S_Delta_u'*(facteur_y*Gamma_y)*S_Delta_u + Gamma_u*eye(p);
    %else
    grad = S_Delta_u'*Gamma_y*(S_Delta_x*zk - Yref(Ny*(j-1)+1:Ny*(j-1)+Ny*p));
    %end

    Uopt = -H^(-1)*grad; %Delta U opt
    uk_m1 = uk;
    Delta_uk = Uopt(1); % we only keep the first input
    uk = uk_m1+ Delta_uk;
    list_Uopt_OSF = [list_Uopt_OSF ; uk];
    %cumsum(Uopt)

    Xk_m1 = Xk;
    if linear==1
        Xk = Ad*Xk + Bd*uk + pert_theta(:,j); % state update
    else
        Xk = RK4(Xk,uk,Te) + pert_theta(:,j); % state update non-lin + pert
    end
    zk(1:4) = Xk - Xk_m1; %  state update delta x(k) = xk - xk-1
    zk(5:4+Ny) = Cd*Xk;
    theta_OSF = [theta_OSF ; Xk(3)];
    x_OSF = [x_OSF; Xk(1)];
end



%% Constrained off-set free MPC (OSF MPC)
% % Inequality constraints A_ineq.Delta U < b_ineq
% % quadprog : min 0.5*Delta U'.H.Delta U + f'.Delta U

%m = p; %horizon de commande
Gamma_y = 50; %100 150 %10^(3)*eye(l*p);
Gamma_u = 0.1; %0.1
uk = 0; % u initiale
Xk = X0;
zk = [zeros(4,1); Cd*X0];  % z_k = [Delta_Xk ; y_k ]
x_COSF = [X0(1)];
theta_COSF = [X0(3)];
N = length(T);
% Tri_inf = zeros(m,m);
% for j=1:m
%     Tri_inf(j,1:j) = ones(1,j);
% end
%H = (S_Delta_u')*Gamma_y*S_Delta_u + (Tri_inf' )^(-1)*Gamma_u*(Tri_inf)^(-1)*eye(m); % Hessian not totally symetric -> H-H' très faible arrondi
H = (S_Delta_u')*Gamma_y*S_Delta_u + Gamma_u*eye(m);
list_Uopt_COSF = [];
u_0 = 0;

u_max=80.3; %max command
u_min = -80.3;
% A_ineq = [];
% b_ineq = [];
A_ineq = [  Tri_inf ; -Tri_inf]; %A_ineq;
b_ineq = [(u_max - u_0)*ones(m,1); (-u_min + u_0)*ones(m,1)]; %b_ineq ;

Cx = [1 0 0 0]; %just select pos x
last_element = Cx;
list_AB = [Cx*Bd_incert]; %Cx*Bd_incert;
Sx_Delta = [];
Su_Delta = zeros(p,m);
sum_A = zeros(Ny,n);

for k=1:p
    if k~=1
        list_AB = [ list_AB, list_AB(1:end,end) + last_element*Bd_incert]; % [CxB, CxB + CxBA, CxB + CxBA + CxBA^2 ...
    end
    last_element = last_element*Ad_incert; %Cd*Ad^(k)
    sum_A = sum_A + last_element;
    Sx_Delta = [Sx_Delta ; sum_A];
    for i=1:k
        if (k-i+1<=m)
            Su_Delta(k,k-i+1) = sum(list_AB(1:i));
        end
    end
end
% x_max = 50;
% x_min = -50;
% delta_X_0 = zeros(n,1);
% xk = Cx*X0;
% A_ineq = [A_ineq ; Su_Delta ; -Su_Delta];
% b_ineq = [b_ineq ; x_max*ones(p,1) - Sx_Delta*delta_X_0 - ones(p,1)*xk; - x_min*ones(p,1) + Sx_Delta*delta_X_0 + ones(p,1)*xk ];
%borne sur le débit
lb = []; %lower bound
ub = [];

%contrainte xmax et xmin juste pour les h états prédits suivants
% h_constraints = p;
% X_max = [x_max*ones(h_constraints,1) ; 10*x_max*ones(p-h_constraints,1)];
% X_min = [x_min*ones(h_constraints,1) ; 10*x_max*ones(p-h_constraints,1)];
% -u_max < u < u_max
% x < x_max
%A_ineq = []; %S_Delta_u;
%b_ineq = []; % theta_max*ones(p,1)-S_Delta_x *Xk;

% Optimization loop
for j=1:(N-p)

    grad = S_Delta_u'*Gamma_y*(S_Delta_x*zk - Yref(Ny*(j-1)+1:Ny*(j-1)+Ny*p));

    Uopt = quadprog(2*H,2*grad,A_ineq,b_ineq,[],[],lb,ub);
    %max(Uopt)
    uk_m1 = uk;
    Delta_uk = Uopt(1); % we only keep the first input
    uk = uk_m1+ Delta_uk;
    list_Uopt_COSF = [list_Uopt_COSF ; uk];
    %cumsum(Uopt)

    Xk_m1 = Xk;
    if linear==1
        Xk = Ad*Xk + Bd*uk + pert_theta(:,j); % state update lin
    else
        Xk = RK4(Xk,uk,Te) + pert_theta(:,j); % state update non-lin + pert
    end
    zk(1:4) = Xk - Xk_m1; %  state update delta x(k) = xk - xk-1
    zk(5:4+Ny) = Cd*Xk;
    xk = Cx*Xk;
    delta_Xk = zk(1:4);

    %b_ineq = [(u_max - uk)*ones(m,1); (-u_min + uk)*ones(m,1) ; X_max - Sx_Delta*delta_Xk - ones(p,1)*xk ; - X_min + Sx_Delta*delta_Xk + ones(p,1)*xk ]; % update constraints
    %b_ineq = [X_max - Sx_Delta*delta_Xk - ones(p,1)*xk ; - X_min + Sx_Delta*delta_Xk + ones(p,1)*xk ]; % update constraints
    b_ineq = [(u_max - uk)*ones(m,1); (-u_min + uk)*ones(m,1)];

    theta_COSF = [theta_COSF ; Xk(3)];
    x_COSF = [x_COSF; Xk(1)];
end


%run mpc_study_case_plotter_nl.m
% PLot constained MPC
% figure();
% subplot(3,1,1)
% plot(T(1:end-p),180/pi*theta_COSF(1:end-1),'LineWidth',2)
% hold on
% plot(T(1:end-p),180/pi*thetades(1:end-p))
% hold off
% grid()
% legend('\theta_{COSFMPC} (°)','\theta des') %'\theta des'
% xlabel('time (s)')
% subplot(3,1,2)
% plot(T(1:end-p),x_COSF(1:end-1),'LineWidth',2)
% grid()
% legend('x_{COSFMPC}') %'\theta des'
% xlabel('time (s)')
% subplot(3,1,3)
% plot(T(1:end-p),list_Uopt_COSF,'LineWidth',2)
% grid
% legend('Uopt_{COSFMPC}')
% xlabel('time (s)')


