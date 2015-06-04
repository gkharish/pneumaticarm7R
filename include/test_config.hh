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
    bool DEFAULT_FLAG;
    bool CALIBRATION_FLAG;
    bool CONTROL_MODE_NOPRES_FLAG;
    bool CONTROL_MODE_PRES_FLAG;
    bool INFLATING_FLAG;
    bool PRES_INDIVIDUAL_FLAG;
    int timeofsimulation;
    //Setup the control parameters
    int NUM_JOINTS;     // Enter the value between 1 to 7
    bool MAN_PRES;      // Manual Pressure ON = 1 and OFF = 0
    bool MAN_PRES_ALL;  // Pressurize all Muscles withe same pressure
    // Joints
    bool JOINT_ALL;
    bool JOINT_1;
    bool JOINT_2;
    bool JOINT_3;
    bool JOINT_4;
    bool JOINT_5;
    bool JOINT_6;
    bool JOINT_7;
    // Value of Manual pressure to each joints
    double MAN_PRES_VAL_ALL;
    double MAN_PRES_VAL_1;
    double MAN_PRES_VAL_2;
    double MAN_PRES_VAL_3;
    double MAN_PRES_VAL_4;
    double MAN_PRES_VAL_5;
    double MAN_PRES_VAL_6;
    double MAN_PRES_VAL_7;
    //Controller type
    int CONTROLLER_TYPE; // P = 0; PI = 1, PD = 2; PID = 3)

    double CTRL_FLAG[7];
    double pressure_command_array[7];

    test();
    void test_config();
    void controlinput();
    double get_CTRL_FLAG(int);
    double get_pressure_command_array(int);
    bool get_DEFAULT_FLAG();
    bool get_CALIBRATION_FLAG();
    bool get_CONTROL_MODE_NOPRES_FLAG();
    bool get_CONTROL_MODE_PRES_FLAG();
    bool get_INFLATING_FLAG();
    bool get_PRES_INDIVIDUAL_FLAG();
    int get_timeofsimulation();
};

#endif
