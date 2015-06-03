// Setup the flags.
#include "test_config.hh"

test::test():
  DEFAULT_FLAG(0),
  CALIBRATION_FLAG(0),
  CONTROL_MODE_NOPRES_FLAG(1),
  CONTROL_MODE_PRES_FLAG(0),
  INFLATING_FLAG(0),
  PRES_INDIVIDUAL_FLAG(0),
  NUM_JOINTS(1),
  MAN_PRES(0),
  MAN_PRES_ALL(0),
  JOINT_ALL(0),
  JOINT_1(0),
  JOINT_2(0),
  JOINT_3(1),
  JOINT_4(0),
  JOINT_5(0),
  JOINT_6(0),
  JOINT_7(0),
  MAN_PRES_VAL_ALL(0.5),
  MAN_PRES_VAL_1(0.5),
  MAN_PRES_VAL_2(0.0),
  MAN_PRES_VAL_3(0.0),
  MAN_PRES_VAL_4(0.0),
  MAN_PRES_VAL_5(0.0),
  MAN_PRES_VAL_6(0.0),
  MAN_PRES_VAL_7(0.0),
  CONTROLLER_TYPE(1)
{

}

void test::test_config()
{

  if(DEFAULT_FLAG == 1)
  {
    CALIBRATION_FLAG=1;

    cout << "CALIBRATION Mode is ON..." << endl;
  }
  if(CALIBRATION_FLAG == 1)
  {
    INFLATING_FLAG=0;
    CONTROL_MODE_PRES_FLAG=0;


    cout << "CALIBRATION Mode is ON..." << endl;
  }
  if(CONTROL_MODE_NOPRES_FLAG == 1)
  {
    INFLATING_FLAG=0;
    CONTROL_MODE_PRES_FLAG=0;
    this -> controlinput();
  }
  if(CONTROL_MODE_PRES_FLAG == 1)
  {
    INFLATING_FLAG=1;
    CONTROL_MODE_NOPRES_FLAG=0;
    this -> controlinput();
  }
  if(PRES_INDIVIDUAL_FLAG == 1)
  {
    DEFAULT_FLAG = 0;
    CALIBRATION_FLAG=0;
    INFLATING_FLAG=0;
    CONTROL_MODE_PRES_FLAG=0;
    CONTROL_MODE_NOPRES_FLAG=0;

  }

}

void test::controlinput()
{
  if(JOINT_ALL == 1)
  {
    JOINT_1 = 1;
    JOINT_2 = 1;
    JOINT_3 = 1;
    JOINT_4 = 1;
    JOINT_5 = 1;
    JOINT_6 = 1;
    JOINT_7 = 1;
  }

  if(MAN_PRES_ALL == 1)
  {
    MAN_PRES_VAL_1 = MAN_PRES_VAL_ALL;
    MAN_PRES_VAL_2 = MAN_PRES_VAL_ALL;
    MAN_PRES_VAL_3 = MAN_PRES_VAL_ALL;
    MAN_PRES_VAL_4 = MAN_PRES_VAL_ALL;
    MAN_PRES_VAL_5 = MAN_PRES_VAL_ALL;
    MAN_PRES_VAL_6 = MAN_PRES_VAL_ALL;
    MAN_PRES_VAL_7 = MAN_PRES_VAL_ALL;
  }

  if(JOINT_1 == 1)
  {
    CTRL_FLAG[0] = 1.0;
  }
  if(JOINT_2 == 1)
  {
    CTRL_FLAG[1] = 1;
  }
  if(JOINT_3 == 1)
  {
    CTRL_FLAG[2] = 1.0;
  }
  if(JOINT_4 == 1)
  {
    CTRL_FLAG[3] = 1.0;
  }
  if(JOINT_5 == 1)
  {
    CTRL_FLAG[4] = 1.0;
  }
  if(JOINT_6 == 1)
  {
    CTRL_FLAG[5] = 1.0;
  }
  if(JOINT_7 == 1)
  {
    CTRL_FLAG[6] = 1.0;
  }

  if(MAN_PRES == 1 && MAN_PRES_ALL == 1)
  {
    for (int lpz = 0; lpz <7; lpz++)
    {
      pressure_command_array[lpz] = MAN_PRES_VAL_ALL;
    }
  }
  if(MAN_PRES == 1 && MAN_PRES_ALL == 0)
  {
    pressure_command_array[0] = MAN_PRES_VAL_1;
    pressure_command_array[1] = MAN_PRES_VAL_2;
    pressure_command_array[2] = MAN_PRES_VAL_3;
    pressure_command_array[3] = MAN_PRES_VAL_4;
    pressure_command_array[4] = MAN_PRES_VAL_5;
    pressure_command_array[5] = MAN_PRES_VAL_6;
    pressure_command_array[6] = MAN_PRES_VAL_7;
  }
}

double test::get_pressure_command_array(int index)
{
  return(pressure_command_array[index]);
}

double test::get_CTRL_FLAG(int index)
{
  return(CTRL_FLAG[index]);
}
bool test::get_DEFAULT_FLAG()
{
  return(DEFAULT_FLAG);
}
bool test::get_CALIBRATION_FLAG()
{
  return(CALIBRATION_FLAG);
}
bool test::get_CONTROL_MODE_NOPRES_FLAG()
{
  return(CONTROL_MODE_NOPRES_FLAG);
}
bool test::get_CONTROL_MODE_PRES_FLAG()
{
  return(CONTROL_MODE_PRES_FLAG);
}
bool test::get_INFLATING_FLAG()
{
  return(INFLATING_FLAG);
}
bool test::get_PRES_INDIVIDUAL_FLAG()
{
  return(PRES_INDIVIDUAL_FLAG);
}
