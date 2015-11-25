#include "pneumaticarmnonlinearmodel.h"
#include <math.h>

#define pi M_PI

PneumaticarmNonlinearModel::PneumaticarmNonlinearModel(double& mydt)
{
    stateNb=4;
    commandNb=2;
  
   //////////////////////////////////////////////
   // double lo, alphao, k,ro,R,a,b,emax,lb,lt,epsb,epst;
    lo = 0.185;
    alphao = 23.0*pi/180;
    k = 1.25;
    ro = 0.0085;
    R = 0.015;
    a = 3/pow((tan(alphao)),2);
    b = 1/pow((sin(alphao)),2);
    emax = (1/k)*(1 - sqrt(b/a));
   
  
//Parameters of Joint
    m = 2.6;
    link_l = 0.32;
    g  = 9.81;
    I = m*pow(link_l,2)/3;
    fv = 0.25;
   //////////////////////////////////////////////
    time_constant = 0.17;
    dt = mydt;
    Id.setIdentity();
    
    I = m*link_l*link_l/3;
    A.setZero();
    B.setZero();
    A(0,1) = 1.0;
    A(2,2) = -1/time_constant;
    A(3,3) = -1/time_constant;
    A(1,1) = -fv;
    
    //A10 = dt*(m*g*0.5*link_l/I);
 

////////////////////////////////////////////////////////////////////////////////////////////////////////

  //% jointstate_deriv(1) = theta_dot; %joint_state(2);
//% jointstate_deriv(2) = ((F1 -F2 )*R  - fv*theta_dot - m*g*0.5*link_l*sin(theta))/I;


///////////////////////////////////////////////////////////////////////////////////////////////////////


    /*B <<0.0, 0.0,
        0.0, 0.0,
        1, 0.0,
        0.0, 1;*/
    B(0,0) = 0.0;
    B(0,1) = 0.0;
    B(1,0) = 0.0;
    B(1,1) = 0.0;
    B(2,0) = 1.0;
    B(2,1) = 0.0;
    B(3,0) = 0.0;
    B(3,1) = 1.0;
    Bd = dt*B;

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();

    fxu[0].setZero();
    fxu[0].setZero();
    /*fuBase <<0.0, 0.0,
             0.0, 0.0,
             1, 0.0,
             0.0, 1;*/
    fuBase = B;
    fu = dt* fuBase;
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    QxxCont.setZero();
    QuuCont.setZero();
    QuxCont.setZero();
}


stateVec_t PneumaticarmNonlinearModel::computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    //    result(1,0)-=A10*sin(X(0));
    //result(3,0)+=A33atan*atan(a*X(3,0));
    double co,theta,tb1,tb2,tb3,tt1,tt2_1,tt2,tt3_1,tt3,F1,F2,T,P1,P2;    
    theta = X(0);
    P1 = U(0);
    P2 = U(1);
    /*lb = lo- R*theta;
    epsb = (1-(lb/lo));
    lt = lo*(1-emax) + R*theta;
    epst = (1-(lt/lo));*/

    co = pi*ro*ro*1e5;
    tb1 = co*(a -b)*P1;
    tb2 = co*a*(-2*k)*(R/lo)*P1*theta;
    tb3 = co*a*pow((k*R/lo),2)*P1*pow(theta,2);
    F1 = tb1 + tb2 + tb3;

    tt1 = co*(a -b)*P2;

    tt2_1 = pow((k*emax),2) - 2*k*emax;
    tt2 = co*a*tt2_1*P2;

    tt3_1 = pow((R*theta/lo),2) - 2*emax*(R*theta/lo);

    tt3 = co*a*(pow(k,2)*tt3_1 + 2*k*(R*theta/lo))*P2;

    F2 = tt1 +tt2 +tt3;

    T = (F1 -F2 )*R;
    //F = [F1 F2 T];
    /*A(0,1) = 1.0;
    A(2,2) = -1/time_constant;
    A(3,3) = -1/time_constant;
    A(1,1) = -fv;*/
    
    //A10 = dt*(m*g*0.5*link_l/I);
//%% J(2,1)
    double t1_j21,t2_j21,t3_j21,t4_j21,t1_j23,t2_j23,t3_j23,t1_j24,t2_j24,t3_j24;
    t1_j21  = -m*g*0.5*link_l*cos(theta)/I;
    t2_j21 = 2*co*a*pow(k,2)*emax*(R/lo)*P2;
    t3_j21 = 2*co*a*pow((k*R/lo),2)*(P1 - P2)*theta;
    t4_j21 = -2*co*a*k*(R/lo)*(P1 + P2);
    A(1,0) = t1_j21 + (t2_j21 + t3_j21 + t4_j21)*(R/I);

  
//%% J(2,2)
//Jx(2,2) = -fv;

//%% J(2,3)

    t1_j23 = co*(a-b);
    t2_j23 = -2*co*a*k*(R/lo)*theta;
    t3_j23 = co*a*pow((k*R/lo),2)*pow(theta,2);

    A(1,2) = (t1_j23  + t2_j23 + t3_j23 )*(R/I);

//%% j(2,4)

    t1_j24 =co*(a-b);

    t2_j24 = co*a*(pow((k*emax),2) - 2*k*emax);
    //tt3_1 = (R*theta/lo)^2 - 2*emax*(R*theta/lo);
    t3_j24 = co*a*(pow(k,2)*tt3_1 + 2*k*(R*theta/lo));

    A(1,3) = -(t1_j24 + t2_j24 + t3_j24)*(R/I);
    
    Ad = (A*dt + Id);
    stateVec_t result = Ad*X + Bd*U;
    fx = Ad;
    return result;
}

void PneumaticarmNonlinearModel::computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    //fx = fxBase;
    //fx(1,0) -= A10*cos(X(0));
    //fxx[0](1,0)+= A10*sin(X(0));
    double co,theta,P1,P2;    
    theta = X(0);
    P1 = U(0);
    P2 = U(1);
    co = pi*ro*ro*1e5;
//
    fxx[1](0,0) = (m*g*0.5*link_l*sin(theta)/I) + 2*co*a*pow((k*R/lo),2)*(P1 - P2)*(R/I);
   // fxx[1](2,0) = (-2*co*a*k*(R/lo) + 2*co*a*pow((k*R/lo),2)*theta)*(R/I);
    //fxx[1](3,0) =  (-2*co*a*k*(R/lo) + 2*co*a*pow(k,2)*emax*(R/lo))*(R/I);

    //fxx[3](1,3) = -((2*dt*Jm*R)/(pi*Jl))*Cf0*((2*a*a*a*X(3,0))/((1+(a*a*X(3,0)*X(3,0)))*(1+(a*a*X(3,0)*X(3,0)))));
    //fxx[3](3,3) = +((2*dt*Cf0)/(pi*Jl))*((2*a*a*a*X(3,0))/((1+(a*a*X(3,0)*X(3,0)))*(1+(a*a*X(3,0)*X(3,0)))));
}

stateMat_t PneumaticarmNonlinearModel::computeTensorContxx(const stateVec_t& nextVx)
{
    QxxCont = nextVx[1]*fxx[1];
    return QxxCont;
}

commandMat_t PneumaticarmNonlinearModel::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

commandR_stateC_t PneumaticarmNonlinearModel::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}

/// accessors ///
unsigned int PneumaticarmNonlinearModel::getStateNb()
{
    return stateNb;
}

unsigned int PneumaticarmNonlinearModel::getCommandNb()
{
    return commandNb;
}

stateMat_t& PneumaticarmNonlinearModel::getfx()
{
    return fx;
}

stateTens_t& PneumaticarmNonlinearModel::getfxx()
{
    return fxx;
}

stateR_commandC_t& PneumaticarmNonlinearModel::getfu()
{
    return fu;
}

stateR_commandC_commandD_t& PneumaticarmNonlinearModel::getfuu()
{
    return fuu;
}

stateR_stateC_commandD_t& PneumaticarmNonlinearModel::getfxu()
{
    return fxu;
}

stateR_commandC_stateD_t& PneumaticarmNonlinearModel::getfux()
{
    return fux;
}
