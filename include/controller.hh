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

class Controller 
{
public:
  Controller();
  virtual ~Controller();

  /** !\brief Start the control real time thread */
  void StartingRealTimeThread();

  /** \brief Apply the control law. */
  void ApplyControlLaw();

  /** \brief Default control law.
      Set constant pressure on each muscle. */
  virtual void ComputeControlLaw();

  /** \brief Set the user control law,
      i.e. set the desired pressure */
  void SetUserControl(unsigned int idx, double control);

  /** \brief get the user control law,
      i.e. set the desired pressure */
  double GetUserControl(unsigned int idx);

  /** \brief Set if a given muscle may or not be activated. */
  void SetApplyControl(unsigned int idx, bool apply_control);

  /** \brief Get if a given muscle may is activated. */
  bool GetApplyControl(unsigned int idx);
  
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

  /** ! Initialize the shared memory. */
  void InitSharedMemory();

  /** ! Close the shared memory. */
  void CloseSharedMemory();

  /** ! Controller state */
  int CONTROLLER_STATE_;
};
#endif /* _PNEUMATIC_ARM_7R_CONTROLLER_HH_ */
