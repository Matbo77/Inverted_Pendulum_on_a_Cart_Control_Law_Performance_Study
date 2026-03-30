function Xk_1 = RK4(Xk,uk,Te)
% Xk : state at k
% uk : command at k
% Te : sampling period
% Xk_1 : state at k+1 
k_1 = f_dyn(0,Xk,uk);
k_2 = f_dyn(0,Xk + Te/2*k_1,uk);
k_3 = f_dyn(0,Xk + Te/2*k_2,uk);
k_4 = f_dyn(0,Xk + Te*k_3,uk);
Xk_1 = Xk + Te/6*(k_1 + 2*k_2 + 2*k_3 + k_4);

end