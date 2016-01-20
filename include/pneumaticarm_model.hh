// Copydight (c) 2015 LAAS_CNRS //
// Author: Ganesh Kumar //


/* This is a program to model the pneumatic muscle based joint of the Pneumatic 7R arm */

#ifndef PNEUMATICARMMODEL_HH
#define PNEUMATICARMMODEL_HH

//#include <Eigen/Dense>
//#include <Eigen/Core>
#include <math.h>
#include <vector>
#define GRAVITY 9.81
#define PI 3.14159265

using namespace std;
//using namespace Eigen;
struct
{
    double lo;
    double alphao;
    double k;
    double ro;
    double R;
    double m;
    double link_l;
    double fv;
    double Pmax;
}musclejointdata;
class PneumaticarmModel
{
 protected:
            double a_, b_, emax_, lb_, lt_, epsb_, epst_, F1_, F2_, P1_, P2_, Pmax_;
            double lo_, alphao_, k_,ro_, R_, m_, link_l_, g, I_, fv_;
            double Torque_, TorqueDes_, Pmean_ref_;
            double wnb_, wnt_, Vb_, Vt_;
            float pressure_muscle1_, pressure_muscle2_, pressure_musclebase_;
            
            int nDOF_;
           unsigned int n_;
            std::vector<double> state_vector_;
            std::vector<double> state_derivative_;
            std::vector<double> control_vector_;
        public:
                /// Constructor
                PneumaticarmModel();   
                virtual  ~PneumaticarmModel();
                void setProblemDimension (int n);
                void setParameters (double lo, 
                                    double alphao, 
                                    double k, 
                                    double ro, 
                                    double R, 
                                    double m,
                                    double l, 
                                    double fv,
                                    double Pmax);
                //void setParameters (musclejointdata data);
                //void setpidcoeff(int p, int i, int d);
                void computeStateDerivative (double time);
                void integrateRK4 (double time, double timeStep);
                vector<double> InverseModel(vector<double>& reference);
                // vector<double> integrateEuler (double time, double timeStep);
                void Set_ControlVector(double value, unsigned int idx);
                void Set_StateVector(double value, unsigned int idx);
                double Get_StateVector(unsigned idx);
                double Get_PmeanRef();
                double Get_ControlVector(unsigned int idx);
                double Get_Torque();
                double Get_TorqueDes();
                //VectorXd getControl (VectorXd statevector, double reference_position, double position);
   };

    

    
#endif

