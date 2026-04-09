function dXk = f_dyn(t,Xk,uk)
    % non-linear continuous dynamic of the inverted pendulum on a cart
    % Xk = [ x_k ; dx_k ; theta_k ; dtheta_k]
    % dXk = f(t,Xk,uk)
    % uk : F force applied to the cart
    % parameter t (time) just to be used in ode45


    % Const(real) physical parameters
    m_pend = 0.2; # pendulum weight
    M = 2.3; # cart weight
    l_pend = 0.2; % pendulum rod length
    g = 9.81; # gravitationnal acceleration on earth
    psi = 5*10^(-5); % translation friction coefficient
    phi = 5*10^(-3); % rotation fluid friction coefficient

    x_k = Xk(1);
    dx_k = Xk(2);
    theta_k = Xk(3);
    dtheta_k = Xk(4);

    D_theta = l_pend*(M + m_pend*(sin(theta_k))^2);

##    dXk = [ dx_k ;
##        l_pend/D_theta*(m_pend*l_pend*sin(theta_k)*dtheta_k^2 - m_pend*g*sin(theta_k)*cos(theta_k) - phi*dx_k + uk);
##        dtheta_k ;
##        -cos(theta_k)/D_theta*(m_pend*l_pend*sin(theta_k)*dtheta_k^2-m_pend*g*sin(theta_k)*cos(theta_k)-psi*dx_k + uk)+g/l_pend*sin(theta_k)-1/(m_pend*l_pend^2)*phi*dtheta_k];


    dXk = [ dx_k ;
    1/D_theta*( l_pend*uk - l_pend*psi*dx_k -  m_pend*l_pend^2*sin(theta_k)*dtheta_k^2 - m_pend*g*l_pend*sin(theta_k)*cos(theta_k) - phi*dtheta_k*cos(theta_k)) ;
    dtheta_k ;
    1/D_theta*( -(M+m_pend)*g*sin(theta_k) - (M+m_pend)/(m_pend*l_pend)*phi*dtheta_k  + cos(theta_k)*uk - psi*dx_k*cos(theta_k) - m_pend*l_pend*dtheta_k^2*sin(theta_k)*cos(theta_k)) ];

end
