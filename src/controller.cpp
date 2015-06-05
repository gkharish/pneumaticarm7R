#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/time.h>

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

  for(unsigned int i=0;i<16;i++)
    {
      apply_controls_[i] = false;
      user_controls_[i] = 0.0;
    }

  InitSharedMemory();
}


Controller::~Controller()
{
}

void Controller::ApplyControlLaw()
{
  RTIME   TASK_PERIOD = 1.0e6;//1000000; ..present,
  rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));

  while(CONTROLLER_STATE_)
    {
      // Waiting the next iteration
      rt_task_wait_period(NULL);      

      unsigned int index =16;
      for(unsigned int i=0;i<7;i++)
        positions_[i] = shmaddr_[index++];
      
      ComputeControlLaw();
      
      for(unsigned int i=0;i<16;i++)
        shmaddr_[i] = controls_[i];
    }
}

void Controller::ComputeControlLaw()
{
  for(unsigned int i=0;i<16;i++)
    {
      if (apply_controls_[i] ==true)
        controls_[i] = user_controls_[i];
      else 
        controls_[i] = 0.0;
    }
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

  pause();

  ODEBUGL("END of Pause",3);

  n = rt_task_delete(&principal_task_);
  
  if (n!=0)
    std::cerr << "Failed of RT Task delete" << std::endl;
  else 
    { ODEBUG("END of RT taslk delete"); }
  
  CloseSharedMemory();
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


