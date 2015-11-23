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

#include "pressuredynamic.hh"


PressureModel::PressureModel()
{
    state_vector_.resize(4);
    state_derivative_.resize(4);
    control_vector_.resize(4);
}

/* Setting Number of Joints or degree of freedom*/
void PressureModel::setProblemDimension (int n)
{
    nDOF_ = n;
}
/* Setting Angular position feedback of the joint*/
void PressureModel::Set_PositionFeedback (double position)
{
    theta = position;
}       
/* Initialization or setting the parameters */
void PressureModel::setParameters (void)
{
            
            
    length_ = 1.0; // m
    mass_ = 1.0;   // kg
    friction_ = 0.1; // kg/s
    pressure_musclebase_ = 2.5; //bar
    n_ = 4; // Number of states in the state space model
    /* M=1; //Mass (kg)
    K = 30000; //stiffness 
    L = 1; //Length of the rod
    g = 9.8;
    I = 1; //Inertia of the primary motor
    J = 1; */ // Inertia of the other m*/
}
        
 /*PAM system dynamics and Compute state derivatives*/
        
void PressureModel::computeStateDerivative(double time)
{
    //VectorXd state_derivative(statevector.size());
    double Tmax, fk,fs, a, b, K0, K1, K2, P_m1, P_m2;        
    double lo = 0.185;
    double alphao = 20.0*PI/180;
    //double epsilono = 0.15;
    double k = 1.25;
    double ro = 0.0085;
    double R = 0.015;
    //double m = 2.5;
    double pi = PI;
    /*double link_l = 0.32;
    double time_constant = 0.1;
    double velocity_constant = 0.15;
    double I = m*link_l*link_l/3; //0.0036;*/
    double term1, term2, term3, Po_1, Po_2, epsilono1, epsilono2;
    double Vb,Vt, wnb, wnt, emax;
    //theta = state_vector_[0];

    a = 3/pow(tan(alphao), 2);
    b = 1/pow(sin(alphao), 2);
               
    /*K1 = 1e5*(PI*pow(ro,2))*R*( a*(pow(1 - k*epsilono, 2)) - b);
    K2 = 1e5*(PI*pow(ro,2))*R*2*a*(1 - k*epsilono)*k*R/lo;*/
    //Po_1 = 0.67*1e5;
    //Po_2 = 4.0*1e5;
    //epsilono1 = 0.05;
    //epsilono2 = 0.25;
   
    emax = (1/k)*(1 - sqrt(b/a));
    //%% Calculate Volume 
    //% biceps agonistic muscle
    double lreal = lo ;//- R*0.25;
    double lb = lreal - R*theta;
    double cs2 = pow(cos(alphao),2);
    double epsb2 = pow((1-(lb/lo)),2);
    double termb1 = (1 - cs2*epsb2);

    Vb = 1e6*(pi*lb*pow(ro,2)/(pow((sin(alphao)),2)))*termb1;
    //Vb = 230;
    wnb = 2*pi*380*(1/Vb);

    //% triceps antagonistic muscle 
    double lt = lo*(1-emax) + R*theta; 
    //double cs2 = pow(cos(alphao),2);
    double epst2 = pow((1-(lt/lo)),2);
    double termt1 = (1 - cs2*epst2);

    Vt = 1e6*(pi*lt*pow(ro,2)/(pow((sin(alphao)),2)))*termt1;
    //Vt = 1e6*(pi*lt*ro^2/((sin(alphao))^2))*termt1
    //Vt = 230;
    wnt = 2*pi*380*(1/Vt);



    state_derivative_[0] = state_vector_[1];
    state_derivative_[1] = -pow(wnb,2)*state_vector_[0] - 2*wnb*1*state_vector_[1] + pow(wnb,2)*control_vector_[0];
    state_derivative_[2] = state_vector_[3];
    state_derivative_[3] = -pow(wnt,2)*state_vector_[2] - 2*wnt*1*state_vector_[3] + pow(wnt,2)*control_vector_[1];
   // pres_deriv(3) = pt_state(2);
   // pres_deriv(4) = -wnt^2*pt_state(1) - 2*wnt*1*pt_state(2) + (wnt^2)*Pdes(2);
    ODEBUGL("State derivative: "<< state_derivative_[0],0);
    }
 
        
/* Numerical Integrator Rungee Kutta */
void PressureModel::integrateRK4 (double t, double h)
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

/*void PressureModel::integrateRK4 (double t, double h)
{
    vector<double> st1, st2, st3, st4;
    st1.resize(n_);
    st2.resize(n_);
    st3.resize(n_);
    st4.resize(n_);
    computeStateDerivative (t);
    for (unsigned int i =0; i <n_; i++)
    {
        st1[i] = state_derivative_[i];
        state_vector_[i] = state_vector_[i] + 0.5*h*st1[i];
    }
    ODEBUGL("After St1 inside integraterk4" << state_vector_[0], 4);

    computeStateDerivative (t + (0.5 * h));
    for (unsigned int i =0; i <n_; i++)
    {
        st2[i] = state_derivative_[i];
        state_vector_[i] = state_vector_[i] + 0.5*h*st2[i];
    }
        
   computeStateDerivative (t + (0.5 * h));
   for (unsigned int i =0; i <n_; i++)
   {
        st3[i] = state_derivative_[i];
        state_vector_[i] = state_vector_[i] + h*st3[i];
   }
   
   computeStateDerivative (t + h);
   for (unsigned int i =0; i <n_; i++)
        st4[i] = state_derivative_[i];
  
  
   for (unsigned int i =0; i <n_; i++)
       state_vector_[i]= state_vector_[i] + ( (1/6.0) * h * (st1[i] + 2.0*st2[i] + 2.0*st3[i] + st4[i]) );
   ODEBUGL("State vector: " << state_vector_[0],0);
}*/
        
        
/* Numerical Integrator Euler */
/*VectorXd PressureModel::integrateEuler (double t, VectorXd state, VectorXd u, double h)
{
    VectorXd st = computeStateDerivative (t, state, u);
            
    VectorXd stNew = state + h*st;
    return (stNew);
}*/
        

void PressureModel::Set_ControlVector (double value, unsigned int idx)

{
    control_vector_[idx] = value;
    ODEBUGL("Control vector is set" << control_vector_[idx],0);
}

double PressureModel::Get_ControlVector(unsigned int idx)
{
    return(control_vector_[idx]);
}

double PressureModel::Get_StateVector(unsigned int idx)
{
    return(state_vector_[idx]);
}

void PressureModel::Set_StateVector(double value, unsigned int idx)
{
    state_vector_[idx] = value;
}

PressureModel::~PressureModel()
{
}
