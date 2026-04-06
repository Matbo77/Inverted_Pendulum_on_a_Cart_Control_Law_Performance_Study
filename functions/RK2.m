function Xk_1 = RK2(Xk,uk,Te)

k_1 = f_dyn(0,Xk,uk);
k_2 = f_dyn(0,Xk + Te/2*k_1,uk);

Xk_1 = Xk + Te*k_2;

end
