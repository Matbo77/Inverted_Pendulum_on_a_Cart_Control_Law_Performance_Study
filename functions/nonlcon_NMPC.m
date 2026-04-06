function  [c, ceq] = nonlcon_NMPC(XU,X0_init,Nw,Te)
 % compute nonlinear equality and inequality constraints of the NMPC problem
 % XU = [ X_1 X_2 , ... X_N ; U_0, U_1, ..., U_{N-1}]
 % size (n + n_u * Nw] )
 % Nw : prediction window size

  c = [];
  %ceq = [];

  n = size(X0_init,1);
  Xk = XU(1:n,:);
  Uk = XU(n+1:end,:);
  % Xk_plus = XU(1:n,k+1);

  ceq =  Xk(:,1) - RK4(X0_init,Uk(:,1),Te);

  for k=1:Nw-1

    ceq = [ceq ; Xk(:,k+1) - RK4(Xk(:,k),Uk(:,k+1),Te) ];

  endfor





end
