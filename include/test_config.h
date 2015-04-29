#ifndef TEST_CONFIG
#define TEST_CONFIG

// Setup the flags.
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include "string.h"
//#include <Eigen/Core>
#include <math.h>

//using namespace Eigen;
using namespace std;



class test
{
  public:
    bool DEFAULT_FLAG=0;
    bool CALIBERATION_FLAG=0;
    bool CONTROL_MODE_NOPRES_FLAG=0;
    bool CONTROL_MODE_PRES_FLAG=1;
    bool INFLATING_FLAG=0;
    bool PRES_INDIVIDUAL_FLAG=0;
    //Setup the control parameters
    int NUM_JOINTS = 1;     // Enter the value between 1 to 7
    bool MAN_PRES = 0;      // Manual Pressure ON = 1 and OFF = 0
    bool MAN_PRES_ALL = 0;  // Pressurize all Muscles withe same pressure
    // Joints
    bool JOINT_INDEX[7] = { 1,0,0,0,0,0,0 };
    bool JOINT_ALL = 0;
    bool JOINT_1 = 0;
    bool JOINT_2 = 0;
    bool JOINT_3 = 1;
    bool JOINT_4 = 0;
    bool JOINT_5 = 0;
    bool JOINT_6 = 0;
    bool JOINT_7 = 0;
    // Value of Manual pressure to each joints
    double MAN_PRES_VAL_ALL = 0.5;
    double MAN_PRES_VAL_1 = 0.5;
    double MAN_PRES_VAL_2 = 0.0;
    double MAN_PRES_VAL_3 = 0.0;
    double MAN_PRES_VAL_4 = 0.0;
    double MAN_PRES_VAL_5 = 0.0;
    double MAN_PRES_VAL_6 = 0.0;
    double MAN_PRES_VAL_7 = 0.0;
    //Controller type
    int CONTROLLER_TYPE = 1; // P = 0; PI = 1, PD = 2; PID = 3)

    double CTRL_FLAG[7];
    double pressure_command_array[7];

    test();
    void test_config();
    void controlinput();
    double get_CTRL_FLAG(int);
    double get_pressure_command_array(int);
    bool get_DEFAULT_FLAG();
    bool get_CALIBERATION_FLAG();
    bool get_CONTROL_MODE_NOPRES_FLAG();
    bool get_CONTROL_MODE_PRES_FLAG();
    bool get_INFLATING_FLAG();
    bool get_PRES_INDIVIDUAL_FLAG();

};

#endif
