#include <MPCcontroller.hh>

double dt = 5e-3;
PneumaticarmNonlinearModel pneumaticarmModel(dt);
CostFunctionPneumaticarmElbow costPneumaticArmElbow;
ILQRSolver iLQRsolver(pneumaticarmModel, costPneumaticArmElbow);


MPCcontroller::MPCcontroller()
{
    texec=0.0;
    xinit  <<0.0,0.0, 0.0, 4.0*1e5;
    //xDes  0.0,0.0,0.0, 0.0;

    T = 20;
    //M = 400;
    dt=5e-3;
    iterMax = 20;
    stopCrit = 1e-3;
  
    /* --- test on romeo actuator --- */
    /*RomeoSimpleActuator romeoActuatorModel(dt);
    RomeoLinearActuator romeoLinearModel(dt);
    CostFunctionRomeoActuator costRomeoActuator;
    ILQRSolver testSolverRomeoActuator(romeoActuatorModel,costRomeoActuator);*/

    /* --- test on pneumatic actuator --- */
    /*PneumaticarmElbowLinear pneumaticarmElbowLinearModel(dt);
    CostFunctionPneumaticarmElbow costPneumaticArmElbow;
    ILQRSolver iLQRsolverpneumaticarmElbowLinear(pneumaticarmElbowLinearModel,costPneumaticArmElbow);
    iLQRsolverpneumaticarmElbowLinear.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);*/


    /*ofstream fichier("resultsMPC.csv",ios::out | ios::trunc);
    if(!fichier)
    {
        cerr  "erreur fichier ! " << endl;
        //return 1;
    }
    fichier  T << "," << M << endl;
    fichier  "tau,tauDot,q,qDot,u" << endl;a*/
}

double MPCcontroller::GetControl(vector<double>& xstate, vector<double>& reference)
{
    xinit(0) = xstate[0];
    xinit(1) = xstate[1];
    xinit(2) = xstate[2];
    xinit(3) = xstate[3];
    
    /*xinit(0) = reference[0];//xstate[0];
    xinit(1) = reference[0];//xstate[1];
    xinit(2) = reference[2]; //xstate[2];
    xinit(3) = reference[3]; //xstate[3];*/
    

    xDes(0) = reference[0];
    xDes(1) = reference[1];
    xDes(2) = reference[2];
    xDes(3) = reference[3];
    //xDes(2) = reference[2]*3.14/180;
    //cout  "Reference position" << xDes(1) << endl;

    iLQRsolver.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);

    gettimeofday(&tbegin,NULL);
    /*for(int i=0;i<M;i++)
    {*/
        iLQRsolver.initSolver(xinit,xDes,T);
        iLQRsolver.solveTrajectory();
        lastTraj = iLQRsolver.getLastSolvedTrajectory();
        xList = lastTraj.xList;
        uList = lastTraj.uList;
        //xinit(2) = xList[1](2,0);
        //xinit(3) = xList[1](3,0);
        
        cout  << "mpc position: " << xList[1](0,0) << endl;
       // cout  "mpc control: " << uList[0](0,0);
        // state feedback
        /*for(int j=0;j<T;j++) fichier  xList[j](0,0) << "," << xList[j](1,0) << "," << xList[j](2,0)  << "," << uList[j](0,0) << endl;
        fichier  xList[T](0,0) << "," << xList[T](1,0) << "," << xList[T](2,0)  << "," << 0.0 << endl;
    //}*/
    gettimeofday(&tend,NULL);


    texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec = (double)(tend.tv_usec - tbegin.tv_usec);

    /*cout  "temps d'execution total du solveur ";
    cout  texec/1000000.0 << endl;
    cout  "temps d'execution par pas de MPC ";
    cout  texec/(T*1000000) << endl;*/

//    fichier.close();

    return(uList[0](0,0));

}

double MPCcontroller::GetState()
{
    return(xList[1](0,0));
}

MPCcontroller::~MPCcontroller()
{
  
}


