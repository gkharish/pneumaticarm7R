#ifndef MPCCONTROLLER_HH_
#define MPCCONTROLLER_HH_

#include <iostream>
#include <fstream>
#include <vector>
#include "config.h"

#include "ilqrsolver.h"

/*#include "romeosimpleactuator.h"
#include "romeolinearactuator.h"
#include "costfunctionromeoactuator.h"*/
//#include "pneumaticarm2ordermodel.h"
//#include "pneumaticarmnonlinearmodel.h"
#include "pneumaticarm_2linkmodel.hh"
//#include "pneumaticarmelbowlinear.h"
// #include "pneumaticarmelbowpiecelinear.h"
#include "costfunctionpneumaticarmelbow.h"

#include <time.h>
#include <sys/time.h>


using namespace std;
using namespace Eigen;

class MPCcontroller
{

private:
    struct timeval tbegin,tend;
    double texec;
    stateVec_t xinit,xDes;

    unsigned int T ;
    unsigned int M ;
    double dt;
    unsigned int iterMax;
    double stopCrit ;
    stateVec_t* xList;
    commandVec_t* uList;
    ILQRSolver::traj lastTraj;  // ofstream fichier("resultsMPC.csv",ios::out | ios::trunc);

    /* --- test on romeo actuator --- */
    /*RomeoSimpleActuator romeoActuatorModel(dt);
    RomeoLinearActuator romeoLinearModel(dt);
    CostFunctionRomeoActuator costRomeoActuator;
    ILQRSolver testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator);*/

    /* --- test on pneumatic actuator --- */
   /* PneumaticarmElbowLinear pneumaticarmElbowLinearModel(double dt);
    CostFunctionPneumaticarmElbow costPneumaticArmElbow;
    ILQRSolver iLQRsolverpneumaticarmElbowLinear(PneumaticarmElbowLinear pneumaticarmElbowLinearModel, CostFunctionPneumaticarmElbow costPneumaticArmElbow);*/

public:
    MPCcontroller();
    virtual ~MPCcontroller();
    std::vector<double> GetControl(vector<double>& xstate, vector<double>& reference);
    double GetState();
};

#endif
