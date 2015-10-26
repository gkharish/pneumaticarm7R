#ifndef PNEUMATICARM2ORDERMODEL_H
#define PNEUMATICARM2ORDERMODEL_H

#include "config.h"

#include "dynamicmodel.h"
#include <Eigen/Dense>

using namespace Eigen;

class Pneumaticarm2orderModel : public DynamicModel
{
public:
    Pneumaticarm2orderModel(double& mydt);
private:
protected:

    // attributes //
public:
private:
    double dt;
    unsigned int stateNb;
    unsigned int commandNb;
    double k;
    double R;
    double Jm;
    double Jl;
    double fvm;
    double Cf0;
    double a;
    
    double m ;
    double link_l ;
   
    double g ;
    double K1 ;
    double K2 ;
    double Pm ;
    double fv ;
    double I;

    stateMat_t Id;
    stateMat_t A;
    stateMat_t Ad;
    stateR_commandC_t B;
    stateR_commandC_t Bd;
    double A13atan,A10;
    double A33atan;
    stateMat_t fx,fxBase;
    stateTens_t fxx;
    stateR_commandC_t fu,fuBase;
    stateR_commandC_commandD_t fuu;
    stateR_stateC_commandD_t fxu;
    stateR_commandC_stateD_t fux;

    stateMat_t QxxCont;
    commandMat_t QuuCont;
    commandR_stateC_t QuxCont;

protected:
    // methods //
public:
    stateVec_t computeNextState(double& dt, const stateVec_t& X, const commandVec_t &U);
    void computeAllModelDeriv(double& dt, const stateVec_t& X, const commandVec_t &U);
    stateMat_t computeTensorContxx(const stateVec_t& nextVx);
    commandMat_t computeTensorContuu(const stateVec_t& nextVx);
    commandR_stateC_t computeTensorContux(const stateVec_t& nextVx);
private:
protected:
        // accessors //
public:
    unsigned int getStateNb();
    unsigned int getCommandNb();
    stateMat_t &getfx();
    stateTens_t& getfxx();
    stateR_commandC_t &getfu();
    stateR_commandC_commandD_t& getfuu();
    stateR_stateC_commandD_t& getfxu();
    stateR_commandC_stateD_t& getfux();

};

#endif // PNEUMATICARM2ORDERMODEL_H
