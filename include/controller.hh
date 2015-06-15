/****************************************
 * Basic object to access shared memory *
 * and implement a controller for the   *
 * INSA pneumatic robotic arm.          *
 * CNRS, LAAS                           *
 * O. Stasse                            *
 ****************************************/

#ifndef _PNEUMATIC_ARM_7R_CONTROLLER_HH_
#define _PNEUMATIC_ARM_7R_CONTROLLER_HH_

#include <vector>
#include <native/task.h>
#include <cmath>
using namespace std;

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
  void  PidController(double error, double error_derivative , int joint_num);
  //double GetPidParameter();
  //void SetPidParameter();
  double  MeanPressure(int);
 /** \ Reference generator function   */
  double GetDesiredPosition();
  void  ReferenceGenerator(long double timestep);
  
protected:
  // Pointer to the shared memory
  double * shmaddr_;
  
  // Pointer of the graph entity 
  RT_TASK principal_task_;

  // Positions values 
  std::vector<double> positions_;

  // Control values
  std::vector<double> controls_;

  // Binary vector to apply controls.
  std::vector<bool> apply_controls_;

  // User specificied controls.
  std::vector<bool> user_controls_;
 // Initial configuration or zero configuration control
  std::vector<double> initconfig_controls_;
  // Joint Number 
  std::vector<bool>JOINT_NUM_;
  // User Specified Mean Pressure
  std::vector<double>mean_pressure_;
  //Pid Controller parameter
  std::vector<double> Pid_factor_;
  std::vector<double> P_;
  std::vector<double> D_;
  std::vector<double> delta;

  /*! @{ Errors computation */
  /* \brief Current error */
  std::vector<double> error_now_;
  /* \brief Error derivative */
  std::vector<double> error_derivative_;
  /*! @} */

  bool reset_control_;
  //ReferenceGenerator parameter
  double ref_init_;
  double ref_final_;
  double ref_traj_;
  double ref_slope_;
  double desired_position_;
  int loop_reference_traj_[7];

  
  std::vector<double> error_prev_;
  /** ! Initialize the shared memory. */
  void InitSharedMemory();

  /** ! Close the shared memory. */
  void CloseSharedMemory();

  /** ! Controller state */
  int CONTROLLER_STATE_;
  int CONTROLLER_TYPE_;
};
#endif /* _PNEUMATIC_ARM_7R_CONTROLLER_HH_ */
