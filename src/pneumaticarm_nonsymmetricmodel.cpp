// Copyright (c) 2015 CNRS
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


PneumaticarmModel::PneumaticarmModel()
{
    state_vector_.resize(2);
    state_derivative_.resize(2);
    control_vector_.resize(2);
}

/* Setting Number of Joints or degree of freedom*/
void PneumaticarmModel::setProblemDimension (int n)
{
    nDOF_ = n;
}
        
/* Initialization or setting the parameters */
void PneumaticarmModel::setParameters (void)
{
            
            
    length_ = 1.0; // m
    mass_ = 1.0;   // kg
    friction_ = 0.1; // kg/s
    pressure_musclebase_ = 2.5; //bar
    n_ = 2; // Number of states in the state space model
    /* M=1; //Mass (kg)
    K = 30000; //stiffness 
    L = 1; //Length of the rod
    g = 9.8;
    I = 1; //Inertia of the primary motor
    J = 1; */ // Inertia of the other m*/
}
        
 /*PAM system dynamics and Compute state derivatives*/
        
void PneumaticarmModel::computeStateDerivative(double time)
{
    //VectorXd state_derivative(statevector.size());
    double Tmax, fk,fs, a, b, K0, K1, K2, P_m1, P_m2;        
    double lo = 0.185;
    double alphao = 20.0*PI/180;
    double epsilono = 0.15;
    double k = 1.25;
    double ro = 0.0085;
    double R = 0.015;
    double m = 2.5;
    double link_l = 0.32;
    double time_constant = 0.1;
    double velocity_constant = 0.15;
    double I = m*link_l*link_l/3; //0.0036;
    double term1, term2, term3, Po_1, Po_2, epsilono1, epsilono2;

    a = 3/pow(tan(alphao), 2);
    b = 1/pow(sin(alphao), 2);
               
    /*K1 = 1e5*(PI*pow(ro,2))*R*( a*(pow(1 - k*epsilono, 2)) - b);
    K2 = 1e5*(PI*pow(ro,2))*R*2*a*(1 - k*epsilono)*k*R/lo;*/
    Po_1 = 0.67*1e5;
    Po_2 = 4.0*1e5;
    epsilono1 = 0.05;
    epsilono2 = 0.25;
    term1 = Po_1*(1 - (2*k*epsilono1));
    term2 = Po_2*(1 - (2*k*epsilono2));
    K0 = a*(term1 - term2) - b*(Po_1 - Po_2);
    term3 = 1 - k*(epsilono1 + epsilono2);
    K1 = 2* (a*term3 - b)*1e5;
    K2 = 2*a*k*R*(Po_1 + Po_2)/lo;
    Tmax = 5*K1;
    fk = 0.1*Tmax*1e-6;
    fs = fk/10;
    //P_m1 = 0.675;
    //P_m2 = 4.0;
    double fadd;// (fs -fk)*( state_vector_[2]*exp(-R*state_vector_[1]/velocity_constant) + state_vector_[3]*exp(-R*state_vector_[1]/velocity_constant) )*state_vector_[1];
    if(state_vector_[1] >= 0)
        fadd = (fs-fk)*exp(-R*abs(state_vector_[1])/velocity_constant);
    else
        fadd = -1*(fs-fk)*exp(-R*abs(state_vector_[1])/velocity_constant);

    state_derivative_[0] = state_vector_[1];
    

   /* state_derivative_[1] = (K1/I)*(2*control_vector_[0] ) 
                            - (K2/I)*(P_m1 + P_m2)*state_vector_[0]
                            -(m*GRAVITY*link_l/(2*I))*sin(state_vector_[0]) 
                            -(fk/I)*state_vector_[1];//- (fadd/I)*state_vector_[1];*/
    
    state_derivative_[1] = (PI*pow(ro,2))*(R/I)*(K0 + K1*control_vector_[0] - K2*state_vector_[0]) 
                                        - (m*GRAVITY*link_l*0.5/I)*sin(state_vector_[0])
                                        - (fk/I)*state_vector_[1];
    //state_derivative_[2] = (1/time_constant)*(-state_vector_[2] + control_vector_[0]);

    //state_derivative_[3] = (1/time_constant)*(-state_vector_[3] + control_vector_[1]);

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

void PneumaticarmModel::Set_StateVector(double value, unsigned int idx)
{
    state_vector_[idx] = value;
}

PneumaticarmModel::~PneumaticarmModel()
{
}