// Copyright (c) 2015 CNRS
// Authors: Ganesh Kumar


/* ******** It is a trial program for modelling a Pneumatic muscle and 
   ******** controlling it in closeloop under xenomai realtime kernel ********/

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

//#include <native/task.h>
//#include <native/timer.h>
#include <time.h>
#include <string.h>
#include <sstream>
#include <debug.hh>

#include "pneumaticarm_model.hh"

/* Setting Number of Joints or degree of freedom*/
void PnematicarmModel::setProblemDimension (int n)
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
    /* M=1; //Mass (kg)
    K = 30000; //stiffness 
    L = 1; //Length of the rod
    g = 9.8;
    I = 1; //Inertia of the primary motor
    J = 1; */ // Inertia of the other m*/
}
        
 /*PAM system dynamics and Compute state derivatives*/
        
VectorXd PnuematicarmModel::computeStateDerivative(double time, VectorXd statevector, VectorXd control)
{
    double c1,c2,c3,c4;
    VectorXd state_derivative(statevector.size());
    double Tmax, fv, a, b;        
    double lo = 0.185;
    double alphao = 23.0;
    double epsilono = 0.15;
    double k = 1.25;
    double ro = 0.009;
    double R = 0.015;
    double m = 5;
    double I = 1;
    double link_l = 0.18;
    a = 3/pow(tan(alphao), 2);
    b = 1/pow(sin(alphao), 2);
               
    K1 = (PI*pow(ro,2))*R*( a*(pow(1 - k*epsilono, 2)) - b);
    K2 = (PI*pow(ro,2))*R*2*a*(1 - k*epsilono)*k*R/lo;
    Tmax = 5*K1;
    fv = 0.1*Tmax;
    state_derivative(0) = statevector(1);
            
    state_derivative(1) = (K1/I)*(control(0) - control(1)) - (K2/I)*(control(0) + control(1))
                                                -(m*GRAVITY*link_l/I)*sin(statevector(0)) 
                                                -(fv/I)*statevector(1);
            
    return state_derivative;
}
 
        
/* Numerical Integrator Rungee Kutta */
VectorXd PneumatiarmModel::integrateRK4 (double t, VectorXd state, VectorXd u, double h)
{
    VectorXd st1 = computeStateDerivative (t, state, u);
            
    VectorXd st2 = computeStateDerivative (t + (0.5 * h), state + (0.5 * h * st1), u);
            
    VectorXd st3 = computeStateDerivative (t + (0.5 * h), state + (0.5 * h * st2),  u);
            
    VectorXd st4 = computeStateDerivative (t + h, state + (h * st3), u);

    VectorXd stNew = state + ( (1/6.0) * h * (st1 + 2.0*st2 + 2.0*st3 + st4) );

    
    return (stNew);
}
        
        
/* Numerical Integrator Euler */
VectorXd PneumaticarmModel::integrateEuler (double t, VectorXd state, VectorXd u, double h)
{
    VectorXd st = computeStateDerivative (t, state, u);
            
    VectorXd stNew = state + h*st;
    return (stNew);
}
        



