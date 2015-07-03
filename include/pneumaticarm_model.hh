// Copydight (c) 2015 LAAS_CNRS //
// Author: Ganesh Kumar //


/* This is a program to model the pneumatic muscle based joint of the Pneumatic 7R arm */

#ifndef PNEUMATICARMMODEL_HH
#define PNEUMATICARMMODEL_HH

#include <system_dynamics.hh>
#include <integrate_dynamics.hh>

#include <Eigen/Core>
#include <math.h>

#define GRAVITY 9.81
#define PI 3.14

using namespace std;
using namespace Eigen;

class PneumaticarmModel::public SystemDynamics, public IntegrateDynamics
{
 protected:
            double length_;
            double mass_;
            double friction_;
            float pressure_muscle1_, pressure_muscle2_, pressure_musclebase_;
            
            int nDOF_;
            
        public:
                /// Constructor
                PneumaticarmModel () : SystemDynamics(), IntegrateDynamics()
                {
                    
                }
                
                void setProblemDimension (int n);
                void setParameters (void);
                //void setpidcoeff(int p, int i, int d);
                VectorXd computeStateDerivative (double time, VectorXd state, VectorXd control);
                VectorXd integrateRK4 (double time, VectorXd state, VectorXd control, double timeStep);
                VectorXd integrateEuler (double time, VectorXd state, VectorXd control, double timeStep);
                
                //VectorXd getControl (VectorXd statevector, double reference_position, double position);
   };

    

    
#endif

