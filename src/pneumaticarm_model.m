 %VectorXd state_derivative(statevector.size());
 function state_derivative_ = pneumaticarm_model(state_vector_, control_vector_)
 state_derivative_ = zeros(size(state_vector_))
    PI =pi;     
    lo = 0.185;
    alphao = 20.0*PI/180;
    epsilono = 0.15;
    k = 1.25;
    ro = 0.0085;
    R = 0.015;
    m = 3.0;
    I = 0.0036;
    link_l = 0.12;
    I = 0.25*(m*(2*link_l)^2)/3;
    time_constant = 0.1;
    velocity_constant = 0.15;
    a = 3/((tan(alphao))^2);
    b = 1/((sin(alphao))^2);
               
    K1 = 1e5*(PI*(ro^2))*R*( a*((1 - k*epsilono)^ 2) - b);
    K2 = 1e5*(PI*(ro^2))*R*2*a*(1 - k*epsilono)*k*R/lo;
    Tmax = 5*K1;
    fk = 0.01*Tmax;
    fs = fk/10;
    P_m = 2.5;
%     double fadd;// (fs -fk)*( state_vector_[2]*exp(-R*state_vector_[1]/velocity_constant) + state_vector_[3]*exp(-R*state_vector_[1]/velocity_constant) )*state_vector_[1];
%     if(state_vector_[1] >= 0)
%         fadd = (fs-fk)*exp(-R*abs(state_vector_[1])/velocity_constant);
%     else
%         fadd = -1*(fs-fk)*exp(-R*abs(state_vector_[1])/velocity_constant);

    state_derivative_(1) = state_vector_(2);
    

    state_derivative_(2) = (K1/I)*(2*control_vector_(1)) - (K2/I)*(2*P_m)*state_vector_(1) -(m*GRAVITY*link_l/I)*sin(state_vector_(1)) -(fk/I)*state_vector_(2); %- (fadd/I)*state_vector_(2);
%%
%% A B matrix linearized at 0 , 0
t1 = (-2*K2*P_m/I)-(m*GRAVITY*link_l/I) % -2.89*1e3%
t2 =  (-fs/I) %   -30.26%
A = [0 1; t1 t2 ];
B = [0 (K1/I)*2]';
C = [1 0]; D = 0;
pneumatic_ss = ss(A,B,C,D)
step(pneumatic_ss)
