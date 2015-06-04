/*! 
 *  Class to display a text user interface to control
 *  the pneumatic arm.
 *  
 *  Author: O. Stasse
 *  Creation date: 05/05/2015
 *  CNRS, LAAS
 * 
 */

#ifndef _PNEUMATIC_ARM_NCURSES_UI_H_
#define _PNEUMATIC_ARM_NCURSES_UI_H_

#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include <ncurses.h>			/* ncurses.h includes stdio.h */
//#include <pneumatic_7arm_rt_thread.hh>

class NCursesUI
{
public:
  NCursesUI();
  ~NCursesUI();

  // Return false if finished
  bool DisplayInformation();
  void HandlingKeyboard();
  void Init();
  void CreateSharedMemory();
  void UpdateSharedMemory();
  int get_FINITE_STATE();
protected:
  //Pneumatic7ArmRtThread pneumaticArm_;
  WINDOW * main_win_;
  pthread_t handle_keyboard_;
  bool end_of_loop_;
  double * shmaddr_;
  double potentiometer_[7];
  double control_[14];
  int FINITE_STATE;
};
#endif /* _PNEUMATIC_ARM_NCURSES_UI_H_ */
