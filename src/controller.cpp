#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/time.h>
// #include <random>

#include <tgmath.h>
#include <native/task.h>
#include <native/timer.h>

#include <debug.hh>
#include <controller.hh>
#include <shared_memory.hh>
#include <pneumaticarm_model.hh>
using namespace std;


//PneumaticarmModel model;
void principale_controller_function(void *arg)
{
  Controller * aController = static_cast<Controller *> (arg);
  if (aController!=0)
  {
    cout << " Controller object is created" << endl;
    aController->ApplyControlLaw();
  }
}

Controller::Controller()
{
  positions_.resize(7);
  simulated_positions_.resize(7);
  controls_.resize(16);
  simulated_controls_.resize(16);
  apply_controls_.resize(16);
  user_controls_.resize(16);
  initconfig_controls_.resize(16);
  simulated_initconfig_controls_.resize(16);
  JOINT_NUM_.resize(7);
  mean_pressure_.resize(7);
  delta.resize(7);
  error_now_.resize(7);
  error_derivative_.resize(7);
  error_prev_.resize(7);
  simulated_delta_.resize(7);
  simulated_error_now_.resize(7);
  simulated_error_derivative_.resize(7);
  simulated_error_prev_.resize(7);

  loop_reference_traj_.resize(7);
  ref_init_.resize(7);
  ref_final_.resize(7);
  ref_traj_.resize(7);
  ref_slope_.resize(7);
  ref_type_.resize(7);
  desired_position_.resize(7);
  reset_control_ = false;



  /** \ Pid controller parameter intialization  */
  Pid_factor_.resize(7);
  P_.resize(7);
  D_.resize(7);

  //Here parameters are inititalized to test jont num 4 the elbow joint 
  // Elbow
  P_[3] = 0.0005;
  D_[3] = 0;
  Pid_factor_[3] = 1;
  ref_init_[3] = 0;
  desired_position_[3] = 45;
  ref_slope_[3] = 1;
  ref_type_[3] = 1;
  ref_traj_[3] = 0;
// Arm rotation
  P_[2] = 0.0005;
  D_[2] = 0;
  Pid_factor_[2] = 1;
  ref_init_[2] = 0;
  desired_position_[2] = 30;
  ref_slope_[2] = 4;
  ref_type_[2] = 2;
  // Elbow rotation
  P_[4] = 0.001;
  D_[4] = 0;
  Pid_factor_[4] = 1;
  ref_init_[4] = 0;
  desired_position_[4] = 30;
  ref_slope_[4] = 1;
  ref_type_[4] = 1;

  // Wrist 
  P_[5] = 0.0005;
  D_[5] = 0;
  Pid_factor_[5] = 1;

  ref_init_[5] = 0;
  desired_position_[5] = 30;
  ref_slope_[5] = 1;
  ref_type_[5] = 2;

  for(unsigned int i=0;i<16;i++)
    {
      apply_controls_[i] = false;
      user_controls_[i] = 0.0;
      controls_[i]=0.0;
      simulated_controls_[i] =0.0;
    }

  InitSharedMemory();
}


Controller::~Controller()
{
}

void Controller::SetJointNum(unsigned int i)
{
  JOINT_NUM_[i] = true;
  ODEBUG("Setting Joint NUm :" << i);
}

void Controller::SetControllerType(int i)
{
  CONTROLLER_TYPE_ = i;
}

void Controller::ApplyControlLaw()
{
  RTIME   now, TASK_PERIOD = 10e6;//1000000; ..present,
  rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));
  int loop = 0;
  /*Plant Model object created*/
  PneumaticarmModel *modelp = new PneumaticarmModel();
  if(modelp!=0)
      ODEBUGL("Pneumatic model object is created",4);
  modelp -> setProblemDimension(1);
  modelp -> setParameters();
  //model -> server_start();
  double integrator_timestep = 0.001;
  vector<double> previous_state, newstate,  u;
             
        
  //int control_len = 0, control_size, state_len = 0, states_size;
  u.resize(2);
  previous_state.resize(2);
  newstate.resize(2);
  //states_size = previous_state.size();
 /* Variables used in Timer*/
  double t, present_time, previous_time ;
 /* time_t now , previous;
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  now  = spec.tv_sec;
  present_time = (spec.tv_nsec / 1.0e9);*/
  now = rt_timer_read();
  present_time = now/1.0e9;
  previous_time = present_time;
    
  for(unsigned int i=0; i <7; i++)
    {
      unsigned index1 = 16;
      positions_[i] = shmaddr_[index1++];
      ref_init_[i] = 0;   //positions_[i];
       
     
     loop_reference_traj_[i] = 0;
    }
    modelp -> Set_StateVector(positions_[3]*3.14/180, 0);

    simulated_positions_[3] = (modelp -> Get_StateVector(0))*180/3.14;
    previous_state[0] = simulated_positions_[3];
    previous_state[1] = 0;
    newstate[0] = previous_state[0];
    newstate[1] = 0;
    u[0] = 0;  //shmaddr_[6];  //modelp -> Get_ControlVector(0);  //shmaddr_[6];
    u[1] = 0;  //shmaddr_[7];  //modelp -> Get_ControlVector(1); //shmaddr_[7];
    simulated_initconfig_controls_[6] = 0.0; 
    simulated_initconfig_controls_[7] = 0.0; 
    for (unsigned int i =0; i<2; i++)
          modelp -> Set_ControlVector(u[i], i);
    for (unsigned int i =0; i<2; i++)
          modelp -> Set_StateVector(0, i);
  
    //for (unsigned int i =2; i<4; i++)
    //      modelp -> Set_StateVector(2.5, i);
  while(1)
    {
      // Waiting the next iteration
      rt_task_wait_period(NULL);      

      shm_sem_.Acquire();
      unsigned int index =16;
      for(unsigned int i=0;i<7;i++)
      {
        positions_[i] = shmaddr_[index++];
        ref_final_[i] = GetDesiredPosition(i);
      }
      shm_sem_.Release();
      ODEBUGL("DEbug Before referencegen", 4);
      //modelp -> Set_StateVector(positions_[3]*3.14/180, 0);

      
      //ReferenceGenerator(loop*TASK_PERIOD/1.0e9);
      // ODEBUGL("After Refgen", 1);
      ComputeControlLaw(TASK_PERIOD);

      u[0] = simulated_controls_[6];
      u[1] = simulated_controls_[7];
      for (unsigned int i =0; i<2; i++)
          modelp -> Set_ControlVector(u[i], i);
       ODEBUGL("DEbug after set_Controlvector", 4);

     /* clock_gettime(CLOCK_REALTIME, &spec);
      now  = spec.tv_sec;
      present_time = (spec.tv_nsec / 1.0e9); */
      now = rt_timer_read();
      present_time = now/1.0e9;
      t = present_time - previous_time;
      modelp -> integrateRK4(t, integrator_timestep);
      simulated_positions_[3] = (modelp -> Get_StateVector(0))*180/3.14;
      ODEBUGL("DEbug after integrator" << simulated_positions_[3] , 1);

      for (unsigned int i=0; i<2; i++)
          previous_state[i] = newstate[i];

      // ODEBUGL("After Control Law" , 1);
      shm_sem_.Acquire();
      for(unsigned int i=0;i<16;i++)
	shmaddr_[i] = controls_[i];
      shmaddr_[23] = (int)ref_traj_[3];
      shmaddr_[24] = ( modelp -> Get_StateVector(0)) *180/3.14;  //newstate[0]*180/3.14;
      shm_sem_.Release();
      loop++;
    }
}

void Controller::ComputeControlLaw(long double timestep)
{
   
  if (CONTROLLER_TYPE_== 3)
    {
      for (unsigned int i=0;i<16;i++)
	{
	  if (reset_control_ ==false)
          {
              controls_[i] = initconfig_controls_[i];
              simulated_controls_[i] = simulated_initconfig_controls_[i];
          }
          else 
          {
              controls_[i] = 0.0;
              simulated_controls_[i] = 0.0;
          }
	}
    }
  if(CONTROLLER_TYPE_== 1)
    {
      for (unsigned int i=0;i<16;i++)
	{
	  if (apply_controls_[i] ==true && reset_control_ ==false)
          {
              controls_[i] = user_controls_[i];
              simulated_controls_[i] = user_controls_[i];
          }
	  else 
          {
              controls_[i] = 0.0;//initconfig_controls_[i];
              simulated_controls_[i] = 0.0;
          }
	}
        controls_[6] = initconfig_controls_[6]+2;   // Step input open loop
        controls_[7] = initconfig_controls_[7]-2;
    }
  if (CONTROLLER_TYPE_ == 2)
    {
      //ODEBUGL("Inside Pid control: " << JOINT_NUM_[3], 1);
      // JOINT_NUM_[3] = tru= true && end_of_loop_ == false)
   
      for (unsigned int i =0; i<7; i++)
	{
	  if (JOINT_NUM_[i] == true && reset_control_==false)  
	    {
	      ReferenceGenerator(loop_reference_traj_[i]*timestep/1.0e9, i,  ref_type_[i]);
	      //ODEBUG("Inside Joint num:" << i );
             //ref_traj_ = ref_final_;
	      error_now_[i] = ref_traj_[i] - positions_[i];
              simulated_error_now_[i] = ref_traj_[i] - simulated_positions_[i];
	      error_derivative_[i] = error_now_[i] - error_prev_[i];
              simulated_error_derivative_[i] = simulated_error_now_[i] - simulated_error_prev_[i];
	      error_prev_[i] = error_now_[i];
              simulated_error_prev_[i]  = simulated_error_now_[i];
	      ODEBUGL("error_now: " << error_now_[i],3);
	      ODEBUGL("error_prev:" << error_prev_[i],3);
	      PidController(error_now_[i], error_derivative_[i],i);
              SimulatedPidController(simulated_error_now_[i], simulated_error_derivative_[i],i);
              loop_reference_traj_[i]++;
	     ODEBUGL(" loop_traj" << loop_reference_traj_[i] << "\n",0);
	    }
	  else
	    {
	      controls_[2*i] = initconfig_controls_[2*i];
	      controls_[2*i+1] = initconfig_controls_[2*i+1];
              simulated_controls_[2*i] = simulated_initconfig_controls_[2*i];
	      simulated_controls_[2*i+1] = simulated_initconfig_controls_[2*i+1];

	    }
	}
      double wn = 6;
      double delc = 0.2*( (sin((double)(loop_reference_traj_[3]*timestep*2*PI*wn/1.0e9)))+ 
                          (sin((double)(loop_reference_traj_[3]*timestep*2*PI*(wn*0.5)/1.0e9)))+ 
                          (sin((double)(loop_reference_traj_[3]*timestep*2*PI*(wn*0.25)/1.0e9)))+
                          (sin((double)(loop_reference_traj_[3]*timestep*2*PI*(wn*1.5)/1.0e9)))+
                          (sin((double)(loop_reference_traj_[3]*timestep*2*PI*(wn*1.75)/1.0e9))) );
      double tim = loop_reference_traj_[3]*timestep/1.0e9;
      double delc1;
      double step_time = 1.5;
      double step_amp = 0.1;
      bool exit = false;
      int lp  = (int)(tim/step_time);
      delc1 = step_amp*lp;
     // delc1 = 0.02*(rand()%100); 
      //delc1 = d(gen);
     if(delc1 > 2)
              delc1 = 2;
    double init_pres1 = initconfig_controls_[6];
    double init_pres2 = initconfig_controls_[7];
    controls_[6] = init_pres1 + 1.5 ;
    controls_[7] = init_pres2 + 1.5 ;
    }


}

void Controller::ResetControl(bool idx)
{
  reset_control_ = idx;
}
void Controller::PidController(double error, double error_derivative, int joint_num)
{
  double update_delta;
  //  double error_acceptable_ = 1;
 // if (error >= error_acceptable_ || error <= -error_acceptable_)
 if(true)
    {
      update_delta =  Pid_factor_[joint_num]*(P_[joint_num]*error + D_[joint_num]*error_derivative);
      delta[joint_num] = delta[joint_num]+update_delta;
    }
   
  double control_limit_agonistic =  initconfig_controls_[2*joint_num]+ delta[joint_num];
  double control_limit_antagonistic = initconfig_controls_[2*joint_num+1] - delta[joint_num];


  if (control_limit_agonistic  <=4.5 && control_limit_agonistic >=0)
    { controls_[2*joint_num]= control_limit_agonistic;}
  else
    {
      if (control_limit_agonistic < 0.0)  
        controls_[2*joint_num] = 0.0;
      else 
	if (control_limit_agonistic > 4.5) 
	  controls_[2*joint_num] = 4.5;
    }

  if (control_limit_antagonistic <= 4.5 && control_limit_antagonistic >=  0.0)
    controls_[2*joint_num + 1] = control_limit_antagonistic;
  else
    {
      if (control_limit_antagonistic <0.0) controls_[2*joint_num +1] = 0.0;
      else if (control_limit_antagonistic >4.5) controls_[2*joint_num+1] = 4.5;
    }
     

  // controls_[2*joint_num] = MeanPressure(joint_num)+ delta[joint_num];
  // controls_[2*joint_num+1] = MeanPressure(joint_num) - (delta[joint_num]);
   
  ODEBUGL("Update delta:     " <<update_delta, 4);
  ODEBUGL("Pid command : " << delta[joint_num],4);
}

void Controller::SimulatedPidController(double error, double error_derivative, int joint_num)
{
  double simulated_update_delta;
  //  double error_acceptable_ = 1;
 // if (error >= error_acceptable_ || error <= -error_acceptable_)
 if(true)
    {
      simulated_update_delta =  Pid_factor_[joint_num]*(P_[joint_num]*error + D_[joint_num]*error_derivative);
      simulated_delta_[joint_num] = simulated_delta_[joint_num] + simulated_update_delta;
    }
   
  double simulated_control_limit_agonistic =  simulated_initconfig_controls_[2*joint_num] + 2.5; //simulated_delta_[joint_num];
  double simulated_control_limit_antagonistic = simulated_initconfig_controls_[2*joint_num +1] - 2.0;//simulated_delta_[joint_num];


  /*if (control_limit_agonistic  <=4.5 && control_limit_agonistic >=0)
    { controls_[2*joint_num]= control_limit_agonistic;}
  else
    {
      if (control_limit_agonistic < 0.0)  
        controls_[2*joint_num] = 0.0;
      else 
	if (control_limit_agonistic > 4.5) 
	  controls_[2*joint_num] = 4.5;
    }

  if (control_limit_antagonistic <= 4.5 && control_limit_antagonistic >=  0.0)
    controls_[2*joint_num + 1] = control_limit_antagonistic;
  else
    {
      if (control_limit_antagonistic <0.0) controls_[2*joint_num +1] = 0.0;
      else if (control_limit_antagonistic >4.5) controls_[2*joint_num+1] = 4.5;
    }*/
     

  simulated_controls_[2*joint_num] = simulated_control_limit_agonistic;
  simulated_controls_[2*joint_num+1] = simulated_control_limit_antagonistic;
   
  ODEBUGL("Simulated Update delta:     " <<simulated_update_delta, 4);
  ODEBUGL("Simulated Pid command : " << delta[joint_num],4);
}



double Controller::MeanPressure(int i)
{
  //mean_pressure_[i] = initconfig_controls_[i];
  return(mean_pressure_[i]);
}

void Controller::ReferenceGenerator(long double timestep, unsigned int joint_num, unsigned int type)
{
  if (type == 0)
  {

      
    if(ref_init_[joint_num] <= ref_final_[joint_num])
    {
      if(ref_traj_[joint_num] >= ref_final_[joint_num])
	ref_traj_[joint_num] = ref_final_[joint_num];
      else
        ref_traj_[joint_num] = ref_init_[joint_num] + ref_slope_[joint_num]*(double)timestep;
    }
  if(ref_init_[joint_num] >  ref_final_[joint_num])
    {
      if(ref_traj_[joint_num] <= ref_final_[joint_num])
	ref_traj_[joint_num] = ref_final_[joint_num];
      else
        ref_traj_[joint_num] = ref_init_[joint_num] - ref_slope_[joint_num]*(double)timestep;
    }
  }
  
  if (type == 1)
  {
      //SetStepResponse();
      ref_traj_[joint_num] = desired_position_[joint_num];
  }

  if (type == 2)
  {
      if (joint_num == 3)
      {
        ref_traj_[joint_num] = 30 + 30*(sin((double)timestep*2*PI/10 ));
      }
      else 
        ref_traj_[joint_num] = 25* sin((double)timestep*2*PI/10);
  }


 ODEBUGL("Ref_traj_ : "<< ref_traj_[joint_num], 1); 
}

double Controller::GetDesiredPosition(unsigned int idx)
{
  return(desired_position_[idx]);
}

void Controller::SetUserControl(unsigned int idx, double control)
{
  user_controls_[idx] = control;
  initconfig_controls_[idx] = control;
}

double Controller::GetUserControl(unsigned int idx)
{
  return user_controls_[idx];
}

void Controller::SetApplyControl(unsigned int idx, bool apply_control)
{
  apply_controls_[idx] = apply_control;
}

bool Controller::GetApplyControl(unsigned int idx)
{
  return apply_controls_[idx];
}

void Controller::StartingRealTimeThread()
{
  unsigned n=0;
  n = rt_task_create(&principal_task_, "Principale Controller Function", 0, 99, 0);
  if (n!=0)
    {
      std::cerr << "Failed @ RT Create" << n << std::endl;
    }
  else 
    { ODEBUGL("END of RT Create",3); }

  n = rt_task_start(&principal_task_, principale_controller_function, this);
  if (n!=0)
    {
      std::cerr<< "Failed of RT STart" <<n<< std::endl;
    }
  else 
    { ODEBUGL("END of RT Start",3); }

}

void Controller::InitSharedMemory()
{
  // Update and/or create the file.
  std::ofstream aof;
  aof.open(SHM_LOG_FILENAME,
           std::ofstream::out | std::ofstream::app);
  struct timeval current_time;
  gettimeofday(&current_time,0);
    
  aof << "Starting control " 
      << current_time.tv_sec << "." 
      << current_time.tv_usec << std::endl;
  aof.close();
  
  // Attached the shared memory to a memory segment.
  shmaddr_ = CreateSharedMemoryForPneumaticArm(false);

}

void Controller::CloseSharedMemory()
{
  shmdt(shmaddr_);

  // Update the file.
  std::ofstream aof;
  aof.open(SHM_LOG_FILENAME,
           std::ofstream::out | std::ofstream::app);
  struct timeval current_time;
  gettimeofday(&current_time,0);
    
  aof << " Stop control at " 
      << current_time.tv_sec << "." 
      << current_time.tv_usec << std::endl;
  aof.close();

}


double Controller::GetErrorNow(unsigned int idx)
{
  return error_now_[idx];
}

double Controller::GetErrorDerivative(unsigned int idx)
{
  return error_derivative_[idx];
}

double Controller::GetUpdateDelta(unsigned int idx)
{
    return(P_[idx]*error_now_[idx]);
}

bool Controller::GetJointNum(unsigned int idx)
{
    return(JOINT_NUM_[idx]);
}

