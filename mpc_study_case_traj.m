

% Traj builder

%Yref = zeros(length(T)) ; %[ x_ref(k) ; theta_ref(k) ]
Yref = [];
%f_T = 1/(length(T)-1);
for k=1:length(T)
    %Yref = [Yref ; 20*pi/180*cos(2*pi/2*T(k)) ];
    %Yref(:,k) = [ 1 ; 0*pi/180 ]; % step
    %Yref = [Yref ; 1 ; 0*pi/180 ]; % step
    %Yref = [Yref ; 80*pi/180*cos(pi*f_T*(k-1))]; % test
    %Yref = [Yref ; X0(3) + (-2*X0(3))*f_T*(k-1)]; % ramp traj
    Yref = [Yref ; 0*pi/180 ]; %step theta
end
% figure();plot(Yref)
% xdes = Yref(1:2:end,1);
% thetades = Yref(2:2:end,1);
thetades = Yref(1:end,1);

% X0(3)

Xref = 0*pi/180*ones(1,length(T));

% y_1 = Cd*x_1 = Cd*(Ad*x_0 + Bd*u_0)
% y_1 - Cd*Ad*x_0 = Cd*Bd*u_0
% (Cd*Bd)'*[y_1 - Cd*Ad*x_0] = (Cd*Bd)'*Cd*Bd*u_0
% u0_test = (Bd'*Cd'*Cd*Bd)^(-1)*(Cd*Bd)'*(Yref(1:Ny)-Cd*Ad*X0);
% X1 = Ad * X0 + Bd * u0_test;
% u1_test = (Bd'*Cd'*Cd*Bd)^(-1)*(Cd*Bd)'*(Yref(1:Ny)-Cd*Ad*X1);
% X2 = Ad * X1 + Bd * u1_test;
