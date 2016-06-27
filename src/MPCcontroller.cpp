#include <MPCcontroller.hh>

double mdt = 10e-3;
PneumaticarmNonlinearModel pneumaticarmModel(mdt);
CostFunctionPneumaticarmElbow costPneumaticArmElbow;
ILQRSolver iLQRsolver(pneumaticarmModel, costPneumaticArmElbow);


MPCcontroller::MPCcontroller()
{
    texec=0.0;
    xinit_MPC  << 0.0,0.0, 0.0,0.0,0,0,0,0;
    //xDes  0.0,0.0,0.0, 0.0;

    T = 40;
    //M = 400;
    dt = mdt;
    iterMax = 100;
    stopCrit = 1e-3;
    plimit.resize(2);
    //plimit[0] = 3.0;
    //plimit[1] = 4.0;
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

vector<double> MPCcontroller::GetControl(vector<double>& xstate, vector<double>& reference)
{
    vector<double> u;
    u.resize(2);
    //xinit(0) = xstate[0];
    //xinit(1) = xstate[1];
    //xinit(2) = xstate[2];
    //xinit(3) = xstate[3];
    
    /*xinit(0) = reference[0];//xstate[0];
    xinit(1) = reference[0];//xstate[1];
    xinit(2) = reference[2]; //xstate[2];
    xinit(3) = reference[3]; //xstate[3];*/
    

    xDes_MPC(0) = reference[0];
    xDes_MPC(1) = reference[1];
    //xDes(2) = reference[2];
    //xDes(3) = reference[3];
    //xDes(2) = reference[2]*3.14/180;
    //cout  << "Reference position" << xDes(0) << endl;

    iLQRsolver.FirstInitSolver(xinit_MPC,xDes_MPC,T,dt,iterMax,stopCrit);

    gettimeofday(&tbegin,NULL);
    
    /*for(int i=0;i<M;i++)
    {*/
        iLQRsolver.initSolver(xinit_MPC,xDes_MPC);
        iLQRsolver.solveTrajectory();
        lastTraj = iLQRsolver.getLastSolvedTrajectory();
        xList_MPC = lastTraj.xList;
        uList_MPC = lastTraj.uList;
        //Apply limit
        /*for(unsigned int i =0; i<2;i++)
        {
            for(unsigned int j=0;j<T;j++)
            {
                if(uList[j](i,0) >=plimit(i))
                    uList[j](i,0) = plimit(i);
                else if (uList[j](i,0) <= 0.0)
                    uList[j](i,0)= 0.0;
            }
        }*/


        //uList
        xinit_MPC(0) = xList_MPC[1](0,0);
        xinit_MPC(1) = xList_MPC[1](1,0);
        
        //cout  << "mpc position: " << xList[1](1,0) << endl;
        //cout  << "mpc control: " << xList[1](1,0);
        // state feedback
        /*for(int j=0;j<T;j++) fichier  xList[j](0,0) << "," << xList[j](1,0) << "," << xList[j](2,0)  << "," << uList[j](0,0) << endl;
        fichier  xList[T](0,0) << "," << xList[T](1,0) << "," << xList[T](2,0)  << "," << 0.0 << endl;
    //}*/
    //
    gettimeofday(&tend,NULL);


    /*texec=((double)(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;
    texec = (double)(tend.tv_usec - tbegin.tv_usec);

    cout  << "temps d'execution total du solveur ";
    cout  << texec/1000000.0 << endl;
    cout  << "temps d'execution par pas de MPC ";
    cout  << texec/(T*1000000) << endl;*/

//    fichier.close();
    u[0] = uList_MPC[0](0,0);
    u[1] = uList_MPC[0](1,0);
    return(u);

}

double MPCcontroller::GetState()
{
    return(xList_MPC[1](0,0));
}

MPCcontroller::~MPCcontroller()
{
  
}
stateMat_t MPCcontroller::Get_mpc_fx()
{
    return(pneumaticarmModel.getfx());
}
stateR_commandC_t MPCcontroller::Get_mpc_fu()
{
    return(pneumaticarmModel.getfu());
}



