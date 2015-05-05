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

//#include <pneumatic_7arm_rt_thread.hh>

class NCursesUI
{
public:
  NCursesUI();
  ~NCursesUI();

  void DisplayInformation();
protected:
  //Pneumatic7ArmRtThread pneumaticArm_;
  
};
#endif /* _PNEUMATIC_ARM_NCURSES_UI_H_ */
