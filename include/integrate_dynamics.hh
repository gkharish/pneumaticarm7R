// Copyright (c) 2015 CNRS
// Authors: Ganesh Kumar


/* ******** It is a trial program for modelling a Pneumatic muscle and 
   ******** controlling it in closeloop under xenomai realtime kernel ********/






#ifndef INTEGRATEDYNAMICS_HH
#define INTEGRATEDYNAMICS_HH

# include <iostream>
#include<Eigen/Core>

using namespace Eigen;

        class IntegrateDynamics
        
        {
            public:
                // Abstract class for system dynamics
                virtual VectorXd integrateRK4 (double, VectorXd, VectorXd, double) = 0;
                virtual VectorXd integrateEuler (double, VectorXd, VectorXd, double) = 0;
        };
    
#endif
