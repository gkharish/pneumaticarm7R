#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/time.h>

#include <tgmath.h>
#include <native/task.h>
#include <native/timer.h>

#include <debug.hh>
#include <controller.hh>
#include <shared_memory.hh>

void principale_controller_function(void *arg)
{
  Controller * aController = static_cast<Controller *> (arg);
  if (aController!=0)
    aController->ApplyControlLaw();
}

Controller::Controller()
{
  positions_.resize(7);
  controls_.resize(16);
  apply_controls_.resize(16);
  user_controls_.resize(16);
  initconfig_controls_.resize(16);
  JOINT_NUM_.resize(7);
  mean_pressure_.resize(7);
  delta.resize(7);
  error_now_.resize(7);
  error_derivative_.resize(7);
  error_prev_.resize(7);
  loop_reference_traj_.resize(7);

  reset_control_ = false;

  /** \ Pid controller parameter intialization  */
  Pid_factor_.resize(7);
  P_.resize(7);
  D_.resize(7);

  //Here parameters are inititalized to test jont num 4 the elbow joint 
  // Elbow
  P_[3] = 0.001;
  D_[3] = 0;
  Pid_factor_[3] = -1;
// Elbow
  P_[2] = 0.0002;
  D_[2] = 0;
  Pid_factor_[2] = 1;

  // Elbow
  P_[4] = 0.0002;
  D_[4] = 0;
  Pid_factor_[4] = -1;

  // Wrist 
  P_[5] = 0.0002;
  D_[5] = 0;
  Pid_factor_[5] = -1;
  //mean_pressure_[3] = 1;
  // mean_pressure_[]
  
  /** \ Refernce generator paramter intialization  */
  desired_position_ = -90;   // Value is in degree
  ref_slope_ = 1;
  ref_traj_ = 0;
  /** \ Mean Pressure */

  for(unsigned int i=0;i<16;i++)
    {
      apply_controls_[i] = false;
      user_controls_[i] = 0.0;
      controls_[i]=0.0;
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
  RTIME   TASK_PERIOD = 10e6;//1000000; ..present,
  rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));
  int loop = 0;

  for(unsigned int i=0; i <7; i++)
    {
      unsigned index1 = 16;
      ref_init_ = 0;
      positions_[i] = shmaddr_[index1++];
      if(JOINT_NUM_[i] == true)
	ref_init_ = positions_[i];
      ref_final_ = GetDesiredPosition();
      loop_reference_traj_[i] = 0;
    }

  while(1)
    {
      // Waiting the next iteration
      rt_task_wait_period(NULL);      

      unsigned int index =16;
      for(unsigned int i=0;i<7;i++)
        positions_[i] = shmaddr_[index++];
      //ODEBUGL("DEbug Before referencegen", 1);
      
      // ReferenceGenerator(loop*TASK_PERIOD/1.0e9);
      // ODEBUGL("After Refgen", 1);
      ComputeControlLaw(TASK_PERIOD);
      // ODEBUGL("After Control Law" , 1);
      for(unsigned int i=0;i<16;i++)
	shmaddr_[i] = controls_[i];

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
	    controls_[i] = initconfig_controls_[i];
          else 
	    controls_[i] = 0.0;
	}
    }
  if(CONTROLLER_TYPE_== 1)
    {
      for (unsigned int i=0;i<16;i++)
	{
	  if (apply_controls_[i] ==true && reset_control_ ==false)
	    controls_[i] = user_controls_[i];
	  else 
	    controls_[i] = 0.0;//initconfig_controls_[i];
	}
    }
  if (CONTROLLER_TYPE_ == 2)
    {
      //ODEBUGL("Inside Pid control: " << JOINT_NUM_[3], 1);
      // JOINT_NUM_[3] = tru= true && end_of_loop_ == false)
   
      for (unsigned int i =0; i<7; i++)
	{
	  if (JOINT_NUM_[i] == true && reset_control_==false)  
	    {
	      ReferenceGenerator(loop_reference_traj_[i]*timestep/1.0e9);
	      //ODEBUG("Inside Joint num:" << i );
              ref_traj_ = ref_final_;
	      error_now_[i] = ref_traj_ - positions_[i];
	      error_derivative_[i] = error_now_[i] - error_prev_[i];
	      error_prev_[i] = error_now_[i];  
	      ODEBUGL("error_now: " << error_now_[i],3);
	      ODEBUGL("error_prev:" << error_prev_[i],3);
	      PidController(error_now_[i], error_derivative_[i],i);
	      loop_reference_traj_[i]++;
	     ODEBUGL(" loop_traj" << loop_reference_traj_[i] << "\n",0);
	    }
	  else
	    {
	      controls_[2*i] = initconfig_controls_[2*i];
	      controls_[2*i+1] = initconfig_controls_[2*i+1];
	    }
	}
    }


}

void Controller::ResetControl(bool idx)
{
  reset_control_ = idx;
}
void Controller::PidController(double error, double error_derivative, int joint_num)
{
  double update_delta;
  double error_acceptable_ = 1;
 // if (error >= error_acceptable_ || error <= -error_acceptable_)
 if(true)
    {
      update_delta =  Pid_factor_[joint_num]*(P_[joint_num]*error + D_[joint_num]*error_derivative);
   
      /* if (delta[joint_num] >=2)
	 delta[joint_num] = 2;
	 else if (delta[joint_num] <= -2)
	 delta[joint_num] = -2;
	 else*/
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
   
  //ODEBUGL("Update delta:     " <<update_delta, 1 );
                  
  //ODEBUGL("Pid command : " << delta[joint_num],1);


}
double Controller::MeanPressure(int i)
{
  //mean_pressure_[i] = initconfig_controls_[i];
  return(mean_pressure_[i]);
}

void Controller::ReferenceGenerator(long double timestep)
{
  if(ref_init_ <= ref_final_)
    {
      if(ref_traj_ >= ref_final_)
	ref_traj_ = ref_final_;
      else
        ref_traj_ = ref_init_ + ( (ref_final_ - ref_init_)/abs(ref_final_ - ref_init_) )*ref_slope_*timestep;
    }
  if(ref_init_ >  ref_final_)
    {
      if(ref_traj_ <= ref_final_)
	ref_traj_ = ref_final_;
      else
        ref_traj_ = ref_init_ + ( (ref_final_ - ref_init_)/abs(ref_final_ - ref_init_) )*ref_slope_*timestep;
    }
 ODEBUGL("Ref_traj_ : "<< ref_traj_, 1); 
}

double Controller::GetDesiredPosition()
{
  return(desired_position_);
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

