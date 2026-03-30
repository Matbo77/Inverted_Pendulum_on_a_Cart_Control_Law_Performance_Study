function dXk = f_dyn(t,Xk,uk)
    % Xk = [ x_k ; dx_k ; theta_k ; dtheta_k]
    % dXk = f(t,Xk,uk)
    % parameter t (time) just to be used in ode45
    
    % Const
    m = 0.2;
    M = 2.3;
    l_pend = 0.2;
    g = 9.81;
    psi = 5*10^(-5);
    phi = 5*10^(-3);
    
    x_k = Xk(1);
    dx_k = Xk(2);
    theta_k = Xk(3);
    dtheta_k = Xk(4);
    
    D = l_pend*(M + m*(sin(theta_k))^2);
    
    dXk = [ dx_k ; 
        l_pend/D*(m*l_pend*sin(theta_k)*dtheta_k^2 - m*g*sin(theta_k)*cos(theta_k) - phi*dx_k + uk);
        dtheta_k ;
        -cos(theta_k)/D*(m*l_pend*sin(theta_k)*dtheta_k^2-m*g*sin(theta_k)*cos(theta_k)-psi*dx_k+uk)+g/l_pend*sin(theta_k)-1/(m*l_pend^2)*phi*dtheta_k];
    
end