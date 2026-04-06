function Xk_horizon = propagate_dynamic(X0,Uk,N_NMPC,Te)
 %


Xk_horizon = [];

% if ...
% Xk_horizon = X0;

Xk = X0;
for k=1:N_NMPC

    Xk = RK4(Xk,Uk(:,k),Te);
    Xk_horizon = [Xk_horizon, Xk];

end



end
