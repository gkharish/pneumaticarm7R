#include "pneumaticarm3ordermodel.h"
#include <math.h>

#define pi M_PI

Pneumaticarm3orderModel::Pneumaticarm3orderModel(double& mydt)
{
    stateNb=3;
    commandNb=1;
    /*k=1000.0;
    R=200.0;
    Jm=138*1e-7;
    Jl=0.1;
    fvm=0.01;
    Cf0=0.1;
    a=10.0;*/
    
    m = 2.7;
    link_l = 0.32;
    
    g = 9.81;
    K1 = 2.1794;
    K2 = 1.2698;
    Pm = 2.5;
    fv = 0.01;
    time_constant = 0.15;
    dt = mydt;
    Id.setIdentity();
    
    I = m*link_l*link_l/3;
    A.setZero();
    A(0,1) = 1.0;
    A(1,0) = -2*K2*Pm/I;
    
    A10 = dt*(m*g*0.5*link_l/I);
    A(1,1) = (-fv/I);
    A(1,2) = 2*K1/I;
    A(2,2) = 1/time_constant;
    /*A(0,1) = 1.0;
    A(2,3) = 1.0;
    A(1,0) = -((k/Jl)+(k/(Jm*R*R)));
    A(1,1) = -(fvm/Jm);
    A(1,3) = -((fvm*k)/Jm);
    A(3,0) = 1.0/Jl;
    Ad = (A*dt+Id);

    A13atan = dt*(2.0*Jm*R/(pi*Jl))*Cf0;
    A33atan = dt*(2.0/(pi*Jl))*Cf0;
*/
    B <<  0.0,
          0.0,
          1/time_constant;

    Ad = (A*dt + Id);
    Bd = dt*B;

    fxBase <<  1,                       dt,         0,
               dt*(-2*K2*Pm/I),     1 -(fv/I)*dt,   0,
               0 ,                       0,         1/time_constant;
    fxx[0].setZero();
    fxx[1].setZero();
    //fxx[2].setZero();
    //fxx[3].setZero();

    fxu[0].setZero();
    fxu[0].setZero();
    fuBase << 0.0,
              0.0;
              1/time_constant;
    fu = dt* fuBase;
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    QxxCont.setZero();
    QuuCont.setZero();
    QuxCont.setZero();
}


stateVec_t Pneumaticarm3orderModel::computeNextState(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    stateVec_t result = Ad*X + Bd*U;
    result(1,0)-=A10*sin(X(0));
    //result(3,0)+=A33atan*atan(a*X(3,0));

    return result;
}

void Pneumaticarm3orderModel::computeAllModelDeriv(double& dt, const stateVec_t& X,const commandVec_t& U)
{
    fx = fxBase;
    fx(1,0) -= A10*cos(X(0));
    fxx[0](1,0)+= A10*sin(X(0));
    //fxx[3](1,3) = -((2*dt*Jm*R)/(pi*Jl))*Cf0*((2*a*a*a*X(3,0))/((1+(a*a*X(3,0)*X(3,0)))*(1+(a*a*X(3,0)*X(3,0)))));
    //fxx[3](3,3) = +((2*dt*Cf0)/(pi*Jl))*((2*a*a*a*X(3,0))/((1+(a*a*X(3,0)*X(3,0)))*(1+(a*a*X(3,0)*X(3,0)))));
}

stateMat_t Pneumaticarm3orderModel::computeTensorContxx(const stateVec_t& nextVx)
{
    QxxCont = nextVx[1]*fxx[1];
    return QxxCont;
}

commandMat_t Pneumaticarm3orderModel::computeTensorContuu(const stateVec_t& nextVx)
{
    return QuuCont;
}

commandR_stateC_t Pneumaticarm3orderModel::computeTensorContux(const stateVec_t& nextVx)
{
    return QuxCont;
}

/// accessors ///
unsigned int Pneumaticarm3orderModel::getStateNb()
{
    return stateNb;
}

unsigned int Pneumaticarm3orderModel::getCommandNb()
{
    return commandNb;
}

stateMat_t& Pneumaticarm3orderModel::getfx()
{
    return fx;
}

stateTens_t& Pneumaticarm3orderModel::getfxx()
{
    return fxx;
}

stateR_commandC_t& Pneumaticarm3orderModel::getfu()
{
    return fu;
}

stateR_commandC_commandD_t& Pneumaticarm3orderModel::getfuu()
{
    return fuu;
}

stateR_stateC_commandD_t& Pneumaticarm3orderModel::getfxu()
{
    return fxu;
}

stateR_commandC_stateD_t& Pneumaticarm3orderModel::getfux()
{
    return fux;
}
