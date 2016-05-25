
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
   
    double m = -0.0023;
    double c = 0.0136;

    //R_ = 0.012; //m*state_vector_[2]*1e-5 + c; 
    P1_ = state_vector_[2];
    P2_ = state_vector_[4];
    a_ = 3/pow(tan(alphao_), 2);
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
<<<<<<< HEAD
     cout << "Vb: "<< wnb_<< endl;       
=======
    //cout << "Vb1: " << P1_  << "F2: " <<Vt_ << "P2: " << wnb_ << endl;       
>>>>>>> master
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
vector<double>  PneumaticarmModel::InverseModel (vector<double>& reference)
{
    //Parameters Muscles
    //double Tmax, fk,fs, a, b, K0, K1, K2, P_m1, P_m2;        
    double P_meanDes, P_real, P_real_dot, P_real_dot2, P1o, P2o, P1,P2;
    double P1dot, P2dot, P1dot2, P2dot2;
    vector<double>P_num;
    P_num.resize(2);
    
<<<<<<< HEAD
    double theta, theta_dot, theta_dot2, tor1, tor2, Fmax, t1,t2;
   
=======
    double theta, theta_dot, theta_dot2,theta_dot3, theta_dot4, t1dot, t2dot, t1dot2, t2dot2, tor1, tor2, Pmax, Fmax, t1,t2;
    double m = -0.0023;
    double c = 0.0136;
    P1o = 0.0e5;
    P2o = Pmax_;
>>>>>>> master
    theta = reference[0];//%(t-1)*5*pi/180;         %ref_traj(1);
    theta_dot = reference[1];//%5*pi/180;     %ref_traj(2);
    theta_dot2 = reference[2];
    theta_dot3 = reference[3];
    theta_dot4 = reference[4];
    //theta_dot3 = reference[3];
    //theta_dot4 = reference[4]; 
    //R_ = 0.012;//m*state_vector_[2]*1e-5 + c; 
    //cout << "R: " << R_ << endl ;
    double p1 = -0.009338;   //(-0.01208, -0.006597)
    double p2 = 0.01444;
    R_ = p1*theta + p2;
    double lreal = lo_ - R_*0.0;
    Fmax = (pi*pow(ro_,2))*(a_- b_)*Pmax_;
    t1 = R_*theta/(lreal*emax_);
    t2 = (I_*theta_dot2 + fv_*theta_dot + m_*g*link_l_*0.5*sin(theta))/(R_*Fmax);
<<<<<<< HEAD
    //cout << "t2:" << t2  << endl;
    P_meanDes = Pmax_*(t1 + t2);
    tor1 = P_meanDes/Pmax_;
=======
    //cout << "t2:" << t2 << "R:" << R_ << endl;
    P_real = Pmax_*(t1 + t2);
    tor1 = P_real/Pmax;
>>>>>>> master
    tor2 = R_*theta/(lo_*emax_);
    TorqueDes_ = R_*Fmax*(tor1 -tor2);
    t1dot = R_*theta_dot/(lo_*emax_);
    t2dot = (I_*theta_dot3 + fv_*theta_dot2 + theta_dot*m_*g*link_l_*0.5*cos(theta))/(R_*Fmax);

    P_real_dot = Pmax_*(t1dot + t2dot);

    t1dot2 = R_*theta_dot2/(lo_*emax_);
    t2dot2 = (I_*theta_dot4 + fv_*theta_dot3 + theta_dot2*m_*g*link_l_*0.5*cos(theta) - 
                theta_dot*theta_dot*m_*g*link_l_*0.5*sin(theta))/(R_*Fmax);

    P_real_dot2 = Pmax_*(t1dot2 + t2dot2);
    P1 = P1o + P_real;
    P2 = P2o - P_real;

    P1dot = P_real_dot;
    P2dot = -P_real_dot;

    P1dot2 = P_real_dot2;
    P2dot2 = -P_real_dot2;

    a_ = 3/pow(tan(alphao_), 2);
    b_ = 1/pow(sin(alphao_), 2);
    emax_ = (1/k_)*(1 - sqrt(b_/a_));
    double cs2 = pow(cos(alphao_),2);
    double lb_ref, epsb_ref, lt_ref, epst_ref, Vb_ref,Vt_ref, wnb_ref, wnt_ref;
    lb_ref = lreal- R_*theta;
    epsb_ref = (1-(lb_ref/lo_));
    lt_ref = lo_*(1-emax_) + R_*theta;
    epst_ref = (1-(lt_ref/lo_));
    double termb1 = (1-cs2*pow(epsb_ref,2));
    double termt1 = (1-cs2*pow(epst_ref,2));
    Vb_ref = 1e6*(pi*lb_ref*pow(ro_,2)/(pow((sin(alphao_)),2)))*termb1;
    //Vb = 230;
    wnb_ref = 2*pi*380*(1/Vb_ref);
    
    Vt_ref = 1e6*(pi*lt_ref*pow(ro_,2)/(pow((sin(alphao_)),2)))*termt1;
    //Vt = 1e6*(pi*lt*ro^2/((sin(alphao))^2))*termt1
    //Vt = 230;
    wnt_ref = 2*pi*380*(1/Vt_ref);

///////////////////////////////////////////////////
/*t1dot = R*theta_dot/(lo*emax);
t2dot = (I*theta_dot3 + fv*theta_dot2 + theta_dot*m*g*link_l*0.5*cos(theta))/(R*Fmax);

P_real_dot = Pmax*(t1dot + t2dot);

t1dot2 = R*theta_dot2/(lo*emax);
t2dot2 = (I*theta_dot4 + fv*theta_dot3 + theta_dot2*m*g*link_l*0.5*cos(theta)- theta_dot*m*g*link_l*0.5*sin(theta))/(R*Fmax);

P_real_dot2 = Pmax*(t1dot2 + t2dot2);

P1 = P1o + P_real;
P2 = P2o - P_real;

P1dot = P_real_dot;
P2dot = -P_real_dot;

P1dot2 = P_real_dot2;
P2dot2 = -P_real_dot2;

%% Pressure dynamic
P1_num = P(1) + 2*Pdot(1)/wnb + Pdot2(1)/(wnb^2);

P2_num = P(2) + 2*Pdot(2)/wnt + Pdot2(2)/(wnt^2);*/
/////////////////////////////////////////////////   
    P_num[0] = P_real + 2*P1dot/wnb_ref + P1dot2/pow(wnb_ref,2);
    P_num[1] = P2 + 2*P2dot/wnt_ref + P2dot2/pow(wnt_ref,2);
    P_num[0] = P_num[0]*1e-5;
    P_num[1] = P_num[1]*1e-5;
    Pmean_ref_ = P_num[0]; //1dot2*1e-5; //P_num[0];
    return(P_num);
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
