function Jcost = cost_NMPC_tracking(XU,Xref,C,Q,R,Nw)
% compute the cost of the NMPC tracking problem
% XU = [ X_1 X_2 , ... X_N ; U_0, U_1, ..., U_{N-1}]
% size (n + n_u * Nw] )


  Jcost = 0;
  n = size(C,2);

  Xk = XU(1:n,:);
  Uk = XU(n+1:end,:);

  for k =1:Nw


    Jcost += (C*Xk(:,k)-Xref(:,k))'*Q*(C*Xk(:,k)-Xref(:,k)) + Uk(:,k)'*R*Uk(:,k);

  endfor



end
