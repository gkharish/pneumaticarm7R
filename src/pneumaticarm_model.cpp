
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

double pi =PI;
PneumaticarmModel::PneumaticarmModel()
{
    state_vector_.resize(6);
    state_derivative_.resize(6);
    control_vector_.resize(2);

}

/* Setting Number of Joints or degree of freedom*/
void PneumaticarmModel::setProblemDimension (int n)
{
    nDOF_ = 6;
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
    n_ = 6; 
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
    I_ = m*link_l*link_l/3; //0.0036;
    fv_ = fv; //0.25;  
    Pmax_ = Pmax;

}
        
 /*PAM system dynamics and Compute state derivatives*/
        
void PneumaticarmModel::computeStateDerivative(double time)
{
    //VectorXd state_derivative(statevector.size());
    //Parameters Muscles
    //double Tmax, fk,fs, a, b, K0, K1, K2, P_m1, P_m2;        
   
   
    P1_ = state_vector_[2];
    P2_ = state_vector_[4];
    a_ = 3/pow(tan(alphao_), 2);
    b_ = 1/pow(sin(alphao_), 2);
    emax_ = (1/k_)*(1 - sqrt(b_/a_));
    double lreal = lo_ - R_*0.0;
    double cs2 = pow(alphao_,2);
    lb_ = lreal- R_*state_vector_[0];
    epsb_ = (1-(lb_/lo_));
    lt_ = lo_*(1-emax_) + R_*state_vector_[0];
    epst_ = (1-(lt_/lo_));
    double termb1 = (1-cs2*pow(epsb_,2));
    double termt1 = (1-cs2*pow(epst_,2));
    Vb_ = 1e6*(pi*lb_*pow(ro_,2)/(pow((sin(alphao_)),2)))*termb1;
    //Vb = 230;
    wnb_ = 2*pi*380*(1/Vb_);
    
    Vt_ = 1e6*(pi*lt_*pow(ro_,2)/(pow((sin(alphao_)),2)))*termt1;
    //Vt = 1e6*(pi*lt*ro^2/((sin(alphao))^2))*termt1
    //Vt = 230;
    wnt_ = 2*pi*380*(1/Vt_);

    F1_ =  pi*pow(ro_,2)*P1_*(a_*pow((1-k_*epsb_),2) - b_);
    F2_ =  pi*pow(ro_,2)*P2_*(a_*pow((1-k_*epst_),2) - b_);
    Torque_ = (F1_ -F2_ )*R_;
    state_derivative_[0] = state_vector_[1];
    state_derivative_[1] =  ((F1_ -F2_ )*R_  - fv_*state_vector_[1] - m_*g*0.5*link_l_*sin(state_vector_[0]))/I_;

    state_derivative_[2] = state_vector_[3];
    state_derivative_[3] = -pow(wnb_,2)*state_vector_[2] - 2*wnb_*1*state_vector_[3] + pow(wnb_,2)*control_vector_[0];
    state_derivative_[4] = state_vector_[5];
    state_derivative_[5] = -pow(wnt_,2)*state_vector_[4] - 2*wnt_*1*state_vector_[5] + pow(wnt_,2)*control_vector_[1];
     cout << "Vb: "<< wnb_<< endl;       
    //P_m1 = 0.675;
    //P_m2 = 4.0;

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
double  PneumaticarmModel::InverseModel (vector<double>& reference)
{
    //Parameters Muscles
    //double Tmax, fk,fs, a, b, K0, K1, K2, P_m1, P_m2;        
    double P_meanDes;
    
    double theta, theta_dot, theta_dot2, tor1, tor2, Fmax, t1,t2;
   
    theta = reference[0];//%(t-1)*5*pi/180;         %ref_traj(1);
    theta_dot = reference[1];//%5*pi/180;     %ref_traj(2);
    theta_dot2 = reference[2];
    //theta_dot3 = reference[3];
    //theta_dot4 = reference[4];
    double lreal = lo_ - R_*0.0;
    Fmax = (pi*pow(ro_,2))*(a_- b_)*Pmax_;
    t1 = R_*theta/(lreal*emax_);
    t2 = (I_*theta_dot2 + fv_*theta_dot + m_*g*link_l_*0.5*sin(theta))/(R_*Fmax);
    //cout << "t2:" << t2  << endl;
    P_meanDes = Pmax_*(t1 + t2);
    tor1 = P_meanDes/Pmax_;
    tor2 = R_*theta/(lo_*emax_);
    TorqueDes_ = R_*Fmax*(tor1 -tor2);
    return(P_meanDes*1e-5);
}
       

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
