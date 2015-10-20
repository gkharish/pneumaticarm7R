#ifndef MPCCONTROLLER_HH_
#define MPCCONTROLLER_HH_

#include <iostream>
#include <fstream>

#include "config.h"

#include "ilqrsolver.h"

/*#include "romeosimpleactuator.h"
#include "romeolinearactuator.h"
#include "costfunctionromeoactuator.h"*/
#include "pneumaticarm2ordermodel.h"
#include "pneumaticarmelbowlinear.h"
#include "pneumaticarmelbowpiecelinear.h"
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
    ILQRSolver::traj lastTraj;

    /* --- test on romeo actuator --- */
    /*RomeoSimpleActuator romeoActuatorModel(dt);
    RomeoLinearActuator romeoLinearModel(dt);
    CostFunctionRomeoActuator costRomeoActuator;
    ILQRSolver testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator);*/

    /* --- test on pneumatic actuator --- */
    PneumaticarmElbowLinear pneumaticarmElbowLinearModel(dt);
    CostFunctionPneumaticarmElbow costPneumaticArmElbow;
    ILQRSolver iLQRsolverpneumaticarmElbowLinear(pneumaticarmElbowLinearModel,costPneumaticArmElbow);

public:
    MPCcontroller();
    virtual ~MPCcontroller();
    double GetControl(double xfeedback, double reference);
    double GetState();
};

#endif
