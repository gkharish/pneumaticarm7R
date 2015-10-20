#include <MPCcontroller.hh>

MPCcontroller::MPCcontroller()
{
    texec=0.0;
    xinit << 0.0,0.0,0.0;
    xDes << 1.0,0.0,0.0;

    T = 40;
    M = 400;
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


    ofstream fichier("resultsMPC.csv",ios::out | ios::trunc);
    if(!fichier)
    {
        cerr << "erreur fichier ! " << endl;
        return 1;
    }
    fichier << T << "," << M << endl;
    fichier << "tau,tauDot,q,qDot,u" << endl;
}

double MPCcontroller::GetControl(double xfeedback, double reference)
{
    xinit(0) = xfeedback;
    xDes(0) = reference;
    iLQRsolverpneumaticarmElbowLinear.FirstInitSolver(xinit,xDes,T,dt,iterMax,stopCrit);

    gettimeofday(&tbegin,NULL);
    /*for(int i=0;i<M;i++)
    {*/
        iLQRsolverpneumaticarmElbowLinear.initSolver(xinit,xDes,T);
        iLQRsolverpneumaticarmElbowLinear.solveTrajectory();
        lastTraj = iLQRsolverpneumaticarmElbowLinear.getLastSolvedTrajectory();
        xList = lastTraj.xList;
        uList = lastTraj.uList;
        xinit = xList[1];
        // state feedback
        for(int j=0;j<T;j++) fichier << xList[j](0,0) << "," << xList[j](1,0) << "," << xList[j](2,0)  << "," << uList[j](0,0) << endl;
        fichier << xList[T](0,0) << "," << xList[T](1,0) << "," << xList[T](2,0)  << "," << 0.0 << endl;
    //}
    gettimeofday(&tend,NULL);


    texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec = (double)(tend.tv_usec - tbegin.tv_usec);

    cout << "temps d'execution total du solveur ";
    cout << texec/1000000.0 << endl;
    cout << "temps d'execution par pas de MPC ";
    cout << texec/(T*1000000) << endl;

//    fichier.close();

    return(uList[0](0,0));

}

double MPCcontroller::GetState()
{
    return(xList[1](0,0));
}

