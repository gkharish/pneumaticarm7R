#include "pneumaticarm_2linkmodel.h"
#include <math.h>

#define pi M_PI




// Authors: Ganesh Kumar


/* ******** It is a trial program for modelling a Pneumatic muscle and 
   ******** controlling it in closeloop under xenomai realtime kernel ********/

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <iostream>
#include <cmath>
//#include <native/task.h>
//#include <native/timer.h>
#include <time.h>
#include <string.h>
#include <sstream>
#include <debug.hh>

#include "pneumaticarm_model.hh"
// Joint 1 parameters
// Joint 2 parameters
double j2lo = 0.23;
double j2alphao = 20*PI/180;
double j2k = 1.1;
double j2ro = 0.012;
double j2R = 0.009;
double j2m = 5.20;
double j2link_l = 0.67;
double j2fv = 3.0;
double j2Pmax = 3.0e5;
// Joint 3 parameters
double j3lo = 0.185;
double j3alphao = 20*PI/180;
double j3k = 1.25;
double j3ro = 0.0085;
double j3R = 0.015;
double j3m = 2.6;
double j3link_l = 0.32;
double j3fv = 0.25;
double pi =PI;
PneumaticarmModel::PneumaticarmModel()
{
    state_vector_.resize(8);
    state_derivative_.resize(8);
    control_vector_.resize(2);

}

/* Setting Number of Joints or degree of freedom*/
void PneumaticarmModel::setProblemDimension (int n)
{
    nDOF_ = 8;
}
        
/* Initialization or setting the parameters */
void PneumaticarmModel::setParameters (double lo, 
                                    double alphao, 
                                    double k, 
                                    double ro, 
                                    double R, 
                                    double m,
                                    double link_l, 
                                    double fv,
                                    double Pmax) 
{
    n_ = 8; 
    lo_ = lo ; //0.185;
    alphao_ = alphao; //20.0*PI/180;
    //double epsilono = 0.15;
    k_ = k; // 1.25;
    ro_ = ro; //0.0085;
    // Parameters Joint
    R_ = R; //0.015;
    m_ = m; //2.6;
    link_l_ = link_l; //0.32;
    g = GRAVITY;
    //double time_constant = 0.1;
    //double velocity_constant = 0.15;
    I_ = m*link_l*link_l/3; //0.0036;
    fv_ = fv; //0.25;  
    Pmax_ = Pmax;
    /*lo = data.lo ; //0.185;
    alphao = data.alphao; //20.0*PI/180;
    //double epsilono = 0.15;
    k = data.k; // 1.25;
    ro = data.ro; //0.0085;
    // Parameters Joint
    R = data.R; //0.015;
    m = data.m; //2.6;
    link_l = data.link_l; //0.32;
    g = GRAVITY;
    //double time_constant = 0.1;
    //double velocity_constant = 0.15;
    I = m*link_l*link_l/3; //0.0036;
    fv = data.fv; //0.25; */
}
        
 /*PAM system dynamics and Compute state derivatives*/
        
void PneumaticarmModel::computeStateDerivative(double time)
{
    //VectorXd state_derivative(statevector.size());
    //Parameters Muscles
    //double Tmax, fk,fs, a, b, K0, K1, K2, P_m1, P_m2;        
   
    double m = -0.0023;
    double c = 0.0136;

    //R_ = 0.012; //m*state_vector_[2]*1e-5 + c; 
    P1_ = state_vector_[4];
    P2_ = state_vector_[5];
   /* a_ = 3/pow(tan(alphao_), 2);
    b_ = 1/pow(sin(alphao_), 2);
    emax_ = (1/k_)*(1 - sqrt(b_/a_));
    double lreal = lo_ - R_*0.0;
    double cs2 = pow(cos(alphao_),2);
    lb_ = lreal- R_*state_vector_[0];
    epsb_ = (1-(lb_/lo_));
    lt_ = lo_*(1-emax_) + R_*state_vector_[0];
    epst_ = (1-(lt_/lo_));
    double termb1 = (1-cs2*pow(epsb_,2));
    double termt1 = (1-cs2*pow(epst_,2));
    Vb_ = 1e6*(pi*lb_*pow(ro_,2)/(pow((sin(alphao_)),2)))*termb1;
    //Vb = 230;
    wnb_ = 2*pi*380*(1/Vb_);*/
    wnb1_ = 10;
    //Vt_ = 1e6*(pi*lt_*pow(ro_,2)/(pow((sin(alphao_)),2)))*termt1;
    //Vt = 1e6*(pi*lt*ro^2/((sin(alphao))^2))*termt1
    //Vt = 230;
    //wnt_ = 2*pi*380*(1/Vt_);
    wnb2_ = 8;

    F1_ =  pi*pow(ro_,2)*P1_*(a_*pow((1-k_*epsb_),2) - b_);
    F2_ =  pi*pow(ro_,2)*P2_*(a_*pow((1-k_*epst_),2) - b_);
    Torque_ = (F1_ -F2_ )*R_;
    state_derivative_[0] = state_vector_[1];
    state_derivative_[1] =  ((F1_ -F2_ )*R_  - fv_*state_vector_[1] - m_*g*0.5*link_l_*sin(state_vector_[0]))/I_;

    state_derivative_[2] = state_vector_[3];
    state_derivative_[3] = -pow(wnb_,2)*state_vector_[2] - 2*wnb_*1*state_vector_[3] + pow(wnb_,2)*control_vector_[0];
    state_derivative_[4] = state_vector_[5];
    state_derivative_[5] = -pow(wnt_,2)*state_vector_[4] - 2*wnt_*1*state_vector_[5] + pow(wnt_,2)*control_vector_[1];
    //cout << "Vb1: " << P1_  << "F2: " <<Vt_ << "P2: " << wnb_ << endl;       
    //P_m1 = 0.675;
    //P_m2 = 4.0;
/////////// Matlab copy of the model function /////////////////
//%% Delta P Pressure Dynamics
//%%%%%%% 2nd order  %%%%%%%%%%%%%%%
//%wnb2 = wnb1;
state_deriv(4) = x(6);
state_deriv(5) = x(7);
state_deriv(6) = -pow(wnb1,2)*x(4) - 2*wnb1*x(6) + pow(wnb1,2)*Pdes1;
state_deriv(7) = -pow(wnb2,2)*x(5) - 2*wnb2*x(7) + pow(wnb2,2)*Pdes2;

//%% Force calculation
double T1 = Torque_net(state_vector_,joint1_lo,joint1_alphaob,joint1_k,joint1_ro,joint1_R,1,4);
double T2 = Torque_net(state_vector_,joint2_lo,joint2_alphaob,joint2_k,joint2_ro,joint2_R,2,5);
//% T = [T1 T2]';

//%% Mass Inertia Matrix 
double m11_const = link1_I + m1*(link1_lc)^2 + link2_I + m2*(link1_l^2 + link2_lc^2) + mb*(link1_l^2 + link2_l^2);
double m11_var = m2*2*link1_l*link2_lc.*cos(x(2,:)) + mb*2*link1_l*link2_l.*cos(x(2,:));
double m11 = pp(m11_var,m11_const);

m12_const = link2_I + m2*link2_lc^2 + mb*link2_l^2;
m12_var = m2*link1_l*link2_lc.*cos(x(2,:)) + mb*link1_l*link2_l.*cos(x(2,:));
m12 = pp(m12_var,m12_const);
col = size(m12,2);
m22 = link2_I + m2*link2_lc^2 + mb*link2_l^2;
for k=1:col
    M(1:2,1:2,k) = [m11(1,k) m12(1,k);m12(1,k) m22];
end
% sm = size(M)
%M() = [m11 m12;m12 m22];
%% Coriolis Matrix
c1_const = -(m2*link2_lc + mb*link2_l)*link1_l;
c1_var1 = sin(x(2,:));
c1_var2 = 2*x(3,:).*x(4,:) + x(4,:).^2;
c1 = c1_const.*tt(c1_var1,c1_var2);

c2_const = (m2*link2_lc + mb*link2_l)*link1_l;
c2_var1 = sin(x(2,:));
c2_var2 = x(3,:).^2;
c2 = c2_const.*tt(c2_var1,c2_var2);
% C = [c1 c2]';
%% Gravity Matrix
g1 = (m1*link1_lc + m2*link1_l + mb*link1_l).*sin(x(1,:)) + (m2*link2_lc + mb*link2_l).*sin(x(1,:) + x(2,:));
g2 = (m2*link2_lc + mb*link2_l).*sin(x(1,:) + x(2,:));
sg = size(g1,2);
%% viscous friction matrix
tf1 = -fv1.*x(3,:);
tf2 = -fv2.*x(4,:);


for k=1:sg
    T(1:2,1,k) = [T1(1,k);T2(1,k)];
    G(1:2,1,k) = 9.8.*[g1(1,k);g2(1,k)];
    C(1:2,1,k) = [c1(1,k);c2(1,k)];
    Tf(1:2,1,k) = [tf1(1,k);tf2(1,k)];
end
%G = 9.8.*[g1 g2]';
% sg = size(G);
% st =size(T)
% sc =size(C)
%% Joint Dynamics
% Mat = [q1_dotdot, q2_dotdot]'
for k=1:col
  Mat(:,1,k)   = inv(M(:,:,k))*(T(:,:,k)+Tf(:,:,k) - C(:,:,k) - G(:,:,k));
end 
state_deriv(1,:) = x(3,:); %joint_state(2);
state_deriv(2,:) = x(4,:); %joint_state(2);
state_deriv(3,:) = Mat(1,:);
state_deriv(4,:) = Mat(2,:);
%((F_biceps -F_triceps ).*R  - fv.*theta_dot - (m*g*0.5*link_l).*sin(theta))/I;

y = x + dt.*state_deriv;
///////////////////////////////////////////////////////////////


    ODEBUGL("State derivative: "<< state_derivative_[0],0);
}
 
        
/* Numerical Integrator Rungee Kutta */
void PneumaticarmModel::integrateRK4 (double t, double h)
{
    vector<double> st1, st2, st3, st4, state_temp_;
    st1.resize(n_);
    st2.resize(n_);
    st3.resize(n_);
    st4.resize(n_);
    state_temp_.resize(n_);
    for (unsigned int i =0; i <n_; i++)
    {
        state_temp_[i] = state_vector_[i];
    }
    computeStateDerivative (t);
    for (unsigned int i =0; i <n_; i++)
    {
        st1[i] = state_derivative_[i];
        state_vector_[i] = state_temp_[i] + 0.5*h*st1[i];
    }
    ODEBUGL("After St1 inside integraterk4" << state_vector_[0], 4);

    computeStateDerivative (t + (0.5 * h));
    for (unsigned int i =0; i <n_; i++)
    {
        st2[i] = state_derivative_[i];
        state_vector_[i] = state_temp_[i] + 0.5*h*st2[i];
    }
        
   computeStateDerivative (t + (0.5 * h));
   for (unsigned int i =0; i <n_; i++)
   {
        st3[i] = state_derivative_[i];
        state_vector_[i] = state_temp_[i] + h*st3[i];
   }
   
   computeStateDerivative (t + h);
   for (unsigned int i =0; i <n_; i++)
        st4[i] = state_derivative_[i];
  
  
   for (unsigned int i =0; i <n_; i++)
       state_vector_[i]= state_temp_[i] + ( (1/6.0) * h * (st1[i] + 2.0*st2[i] + 2.0*st3[i] + st4[i]) );
   ODEBUGL("State vector: " << state_vector_[0],0);
}
  
/* Numerical Integrator Euler */
/*VectorXd PneumaticarmModel::integrateEuler (double t, VectorXd state, VectorXd u, double h)
{
    VectorXd st = computeStateDerivative (t, state, u);
            
    VectorXd stNew = state + h*st;
    return (stNew);
}*/

void PneumaticarmModel::Set_ControlVector (double value, unsigned int idx)

{
    control_vector_[idx] = value;
    ODEBUGL("Control vector is set" << control_vector_[idx],0);
}

double PneumaticarmModel::Get_ControlVector(unsigned int idx)
{
    return(control_vector_[idx]);
}

double PneumaticarmModel::Get_StateVector(unsigned int idx)
{
    return(state_vector_[idx]);
}

double PneumaticarmModel::Get_PmeanRef()
{
    return(Pmean_ref_);
}


void PneumaticarmModel::Set_StateVector(double value, unsigned int idx)
{
    state_vector_[idx] = value;
}
double PneumaticarmModel::Get_Torque()
{
   return(Torque_);
}
double PneumaticarmModel::Get_TorqueDes()
{
   return(TorqueDes_);
}
PneumaticarmModel::~PneumaticarmModel()
{
}
PneumaticarmModel::Torque_net(double<vector> x,double lo,double alphaob,double k,double ro,double R,unsigned int i,double pmax)
double theta = x(i);
double a_biceps = 3/pow(tan(alphaob),2);
double b_biceps = 1/pow(sin(alphaob),2);
double emax_biceps = (1/k)*(1 - sqrt(b_biceps/a_biceps));
double alphaot = alphaob;
double a_triceps = 3/pow(tan(alphaot),2);
double b_triceps = 1/pow(sin(alphaot),2);
double emax_triceps = (1/k)*(1 - sqrt(b_triceps/a_triceps));

double lb = lo - R*theta;
double epsb = (1-(lb/lo));
double lt = lo*(1-emax_triceps) + R*theta;
double epst = (1-(lt/lo));


double P1 = x(i+4);
double P2 = pmax-x(i+4);;
//P = [P1;P2];
double fbterm = 1e5*pi*ro^2*(a_biceps*pow((1-k*epsb),2) - b_biceps);
double F_biceps =  P1*fbterm;
double ftterm = 1e5*pi*ro^2*(a_triceps*pow((1-k*epst),2) - b_triceps);
double F_triceps = P2*ftterm;
//%F2max = 1*pi*ro^2*4*1e5*(a*pow((1-k*emax),2) - b);
//Fmat = [F_biceps; F_triceps];
double Torqe_pneumatics = (F_biceps -F_triceps )*R;
return (Torque_pneumatics);
}

