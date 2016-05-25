/****************************************
 * Basic object to access shared memory *
 * and implement a controller for the   *
 * INSA pneumatic robotic arm.          *
 * CNRS, LAAS                           *
 * O. Stasse                            *
 ****************************************/

#ifndef CONTROLLER_HH_
#define CONTROLLER_HH_

#include <vector>
#include <native/task.h>
#include <cmath>
#include <time.h>
#include <math.h>
//#include <random>
#define PI 3.14159265
#include "shared_memory.hh"
//#include "pneumaticarm_model.hh"
#include "pressuredynamic.hh"
#include "MPCcontroller.hh"
using namespace std;
//using namespace Eigen;
class Controller 
{
public:
  Controller();
  virtual ~Controller();

  /** !\brief Start the control real time thread */
  void StartingRealTimeThread();
  /** \ Set Joint numbe rto be controlled */
  void SetJointNum(unsigned int);
  /** \brief Apply the control law. */
  void ApplyControlLaw();

  /** \brief Default control law.
      Set constant pressure on each muscle. */
  virtual void ComputeControlLaw(long double timestep);
  //PneumaticarmModel *modelp;

  /** \brief Set the user control law,
      i.e. set the desired pressure */
  void SetUserControl(unsigned int idx, double control);

  /** \brief get the user control law,
      i.e. set the desired pressure */
  double GetUserControl(unsigned int idx);

  /** \brief Set if a given muscle may or not be activated. */
  void SetApplyControl(unsigned int idx, bool apply_control);

  /** \brief Getting error */
  double GetErrorNow(unsigned int idx);
  
  /** \brief Getting derivative */
  double GetErrorDerivative(unsigned int idx);
  
  /** \brief Get if a given muscle may is activated. */
  bool GetApplyControl(unsigned int idx);
 /** \ Reset flag to set the control value to zero */
  void ResetControl(bool);
  /** \ Get COntroller Type */
  void SetControllerType(int idx);
  /** \ PID controller design  */
  double  PidController(double error, double error_derivative , int joint_num);
  void  SimulatedPidController(double error, double error_derivative , int joint_num);
  //double GetPidParameter();
  //void SetPidParameter();
  /* MPC controller */
  MPCcontroller mpc_controller;
  std::vector<double> mpc_u;
  std::vector<double> reference_mpc_;
  double  MeanPressure(int);
 /** \ Reference generator function   */
  double GetDesiredPosition(unsigned int idx);
  void  ReferenceGenerator(long double timestep, unsigned int joint_num, unsigned int type);
  void SetStepResponse();
  double GetUpdateDelta(unsigned int idx);
  bool GetJointNum(unsigned int idx);
  protected:
  // Pointer to the shared memory
  double * shmaddr_;
  
  // Semaphore to protect the shared memory.
  Semaphore shm_sem_;
 
  //Model object for elbow joint
  //PneumaticarmModel *modelp;
 
  // Pointer of the graph entity 
  RT_TASK principal_task_;

  // Positions values 
  std::vector<double> positions_;
  std::vector<double> velocity_;
  std::vector<double> acceleration_;
  std::vector<double> position_store_;
  std::vector<double> xstate_;
  std::vector<double> state_mpc_;
  std::vector<double> simulated_positions_;

  // Control values
  std::vector<double> controls_;
  std::vector<double> simulated_controls_; 

  // Binary vector to apply controls.
  std::vector<bool> apply_controls_;

  // User specificied controls.
  std::vector<bool> user_controls_;
 // Initial configuration or zero configuration control
  std::vector<double> initconfig_controls_;
  std::vector<double> simulated_initconfig_controls_;
  // Joint Number 
  std::vector<bool>JOINT_NUM_;
  // User Specified Mean Pressure
  std::vector<double>mean_pressure_;
  //Pid Controller parameter
  std::vector<double> Pid_factor_;
  std::vector<double> P_;
  std::vector<double> D_;
  std::vector<double> delta;
  std::vector<double> simulated_delta_;
  double criterror_, integrated_error_, integrated_simerror_, integrated_Terror_;

  /*! @{ Errors computation */
  /* \brief Current error */
  std::vector<double> error_now_;
  std::vector<double> simulated_error_now_;
  /* \brief Error derivative */
  std::vector<double> error_derivative_;
  std::vector<double> simulated_error_derivative_;
  /*! @} */

  bool reset_control_;
  //ReferenceGenerator parameter
  std::vector<double>ref_init_;
  std::vector<double>ref_final_;
  std::vector<double>ref_traj_;
  std::vector<double>ref_vel_;
  std::vector<double>ref_acl_;
  std::vector<double> ref_jerk_;
  std::vector<double> ref_jerkdot_;
  std::vector<double>reference_;
  std::vector<double>ref_slope_;
  
  //double ref_step_;
  std::vector<double>desired_position_;
  std::vector<unsigned int>ref_type_;
  std::vector<int> loop_reference_traj_;

  
  std::vector<double> error_prev_;
  std::vector<double> simulated_error_prev_;
  /** ! Initialize the shared memory. */
  void InitSharedMemory();

  /** ! Close the shared memory. */
  void CloseSharedMemory();

  /** ! Controller state */
  int CONTROLLER_STATE_;
  int CONTROLLER_TYPE_;
};
#endif /* _PNEUMATIC_ARM_7R_CONTROLLER_HH_ */
