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
  JOINT_NUM_.resize(7);
  mean_pressure_.resize(7);
  delta.resize(7);

  /** \ Pid controller parameter intialization  */
  Pid_factor_.resize(7);
  P_.resize(7);
  D_.resize(7);
  //Here parameters are inititalized to test jont num 4 the elbow joint 
  P_[3] = 0.02;
  D_[3] = 0;
  Pid_factor_[3] = 1;
  mean_pressure_[3] = 1;
  
  /** \ Refernce generator paramter intialization  */
  desired_position_ = 45;   // Value is in degree
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
    cout << "Setting Joint NUm :" << i << endl;
}

void Controller::SetControllerType(int i)
{
    CONTROLLER_TYPE_ = i;
}

void Controller::ApplyControlLaw()
{
  RTIME   TASK_PERIOD = 1.0e8;//1000000; ..present,
  rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));
  int loop = 0;

  for(unsigned int i=0; i <7; i++)
    {
        unsigned index1 = 16;
        positions_[i] = shmaddr_[index1++];
         if(JOINT_NUM_[i] == true)
            ref_init_ = positions_[i];
        ref_final_ = GetDesiredPosition();
    }   
  
  while(1)
    {
      // Waiting the next iteration
      rt_task_wait_period(NULL);      

      unsigned int index =16;
      for(unsigned int i=0;i<7;i++)
        positions_[i] = shmaddr_[index++];
      //ODEBUGL("DEbug Before referencegen", 1);
      
      ReferenceGenerator(loop*TASK_PERIOD/1.0e9);
     // ODEBUGL("After Refgen", 1);
      ComputeControlLaw();
     // ODEBUGL("After Control Law" , 1);
      for(unsigned int i=0;i<16;i++)
	shmaddr_[i] = controls_[i];

      loop++;
    }
}

void Controller::ComputeControlLaw()
{
  if(CONTROLLER_TYPE_== 1)
    for(unsigned int i=0;i<16;i++)
    {
      if (apply_controls_[i] ==true)
        controls_[i] = user_controls_[i];
      else 
        controls_[i] = 0.0;
    }
  if(CONTROLLER_TYPE_ == 2)
  {
    //ODEBUGL("Inside Pid control: " << JOINT_NUM_[3], 1);
   // JOINT_NUM_[3] = true;
    for (unsigned int i =0; i <7; i++)
     {
       if(JOINT_NUM_[i] == true)
        {
          
            cout << "Inside Joint num:" << i << endl;
            double error_now = ref_traj_ - positions_[i];
           double error_derivative = error_now - error_prev_;
           error_prev_ = error_now;  
           ODEBUGL("error_now: " << error_now,1);
           std::cout << "error_prev:" << error_prev_ << endl;
           PidController(error_now, error_derivative,i);
        }
       else
        {
           controls_[2*(i+1)-1] =0.0;
           controls_[2*(i+1)] = 0.0;
        }
     }
  }


}

void Controller::PidController(double error, double error_derivative, int joint_num)
{
    
    double update_delta =  Pid_factor_[joint_num]*(P_[joint_num]*error + D_[joint_num]*error_derivative);
   // cout << "Delta :" << delta;
    controls_[2*(joint_num+1)-1] = MeanPressure(joint_num) + delta[joint_num];
    controls_[2*(joint_num+1)] = MeanPressure(joint_num) - (delta[joint_num]);
    if (delta[joint_num] <=2)
        delta[joint_num] = 2;
    else
         delta[joint_num] = delta[joint_num]+update_delta;

              
    ODEBUGL("Pid command : " << delta[joint_num],1);


}
double Controller::MeanPressure(int i)
{
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
   // ODEBUGL("Ref_traj_ : "<< ref_traj_, 1); 
}

double Controller::GetDesiredPosition()
{
    return(desired_position_);
}

void Controller::SetUserControl(unsigned int idx, double control)
{
  user_controls_[idx] = control;
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


