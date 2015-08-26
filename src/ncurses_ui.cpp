/* System includes */
#include <string>
#include <iostream>
#include <sstream>

/* Ncurses includes */
#include <ncurses_ui.hh>

/* Local framework. */
#include <shared_memory.hh>
#include <debug.hh>

void * FunctionHandlingKeyboard(void *argc)
{
  NCursesUI * ancursesui = static_cast<NCursesUI *>(argc);
  ancursesui->HandlingKeyboard();
  return NULL;
}
 
NCursesUI::NCursesUI(Controller *aController):
  end_of_loop_(false)
{
  ODEBUGL(" Controler:" << aController,3);
  Controller_ = aController;
  for(unsigned int i=0;i<NB_CONTROLS;i++)
    control_[i] = 0.0;
 // Openinng file to log the data
 log_data_.open("logdata.txt");
  /* start the curses mode */
  initscr();				
  /* No waiting for the keyboard. */
  cbreak();

  // Initializing colors.
  if(has_colors() == FALSE)
    {	
      endwin();
      printf("Your terminal does not support color\n");
      exit(1);
    }
  //start_color();

  // Create a pair of color
  //init_pair(1, COLOR_RED, COLOR_BLACK);

  int row,col;				/* to store the number of rows and *
					 * the number of colums of the screen */

  getmaxyx(stdscr,row,col);		/* get the number of rows and columns */
  
  main_win_ = newwin(LINES,COLS, 0,0);

  // Initialize pressure for all motors.
  double PressureForMuscles[NB_CONTROLS] = {
    0.0, 0.0, 0.0, 0.0, 1.0,
    1.0, 0.2, 4.0, 0.1, 0.1,
    0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0};
  
  ODEBUGL(" Controler:" << Controller_,3);
  if (Controller_!=0)
    {
      for(unsigned int i=0;i<NB_CONTROLS;i++)
	{
	  ODEBUGL(" Pressure:" << PressureForMuscles[i] << " for muscle " << i ,0);
	  Controller_->SetUserControl(i,
				      PressureForMuscles[i]);
	}
    }

}


void NCursesUI::HandlingKeyboard()
{
  bool loop=true;
  cout << "Enter 'l' for close loop, 'm' for manual command mode and q for exit: " << endl;
  cout << "\n Please enter Joint number to be controlled: " << endl;
  int  controller_type_flag = 0;
  while(loop)
    {
      // Reading key.
      int c = getch();
     if (c=='r') 
        { 
           Controller_->ResetControl(true);
        }
     if (c=='i') 
        { 
           Controller_->SetControllerType(3);
           controller_type_flag = 0;
        } 
     if (c=='l') 
        { 
           Controller_->SetControllerType(2);
           controller_type_flag = 1;
        }
     if (c=='o') 
        { 
           Controller_->SetControllerType(1);
           //controller_type_flag = 1;
        }
       if (c=='m')
         {
             Controller_->SetControllerType(1);
             controller_type_flag = 0;
           
         }
      if (c=='s')
        {
         if (Controller_!=0)
              Controller_->StartingRealTimeThread();
         }
      if ((controller_type_flag==1) && (c>='1') && (c<='7'))
        {                
	  if  (Controller_!=0)
	    {
	     // unsigned int idx =2* c-1;
              unsigned int idx1 = c-'1';

              unsigned int idx = 2*idx1+1;
              Controller_->SetApplyControl(idx,true);
	      Controller_->SetApplyControl(idx-1,true);
              
              Controller_ -> SetJointNum(idx1);
              //Controller_ -> SetMeanPressure(idx1);
            }

        }
        
      if ((controller_type_flag==0) && (c>='a') && (c<='f'))
        {    
           if (Controller_!=0)
             {
                unsigned int idx = 9+c-'a';

                if (Controller_->GetApplyControl(idx))
                      Controller_->SetApplyControl(idx,false);
                else
                      Controller_->SetApplyControl(idx,true);
              }

        }
      if((controller_type_flag==0) && (c>='1') && (c<='9'))
         {
            if(Controller_!=0)
              {
                unsigned int idx = c-'1';

                if (Controller_->GetApplyControl(idx))
                      Controller_->SetApplyControl(idx,false);
                else
                      Controller_->SetApplyControl(idx,true);
               }
         }
        if (c=='q')
        {
          loop=false;
          end_of_loop_=true;
          FINITE_STATE = 5;
        
         }            
     
    }


}

void NCursesUI::CreateSharedMemory()
{
  // Attached the shared memory to a memory segment.
  shmaddr_ = CreateSharedMemoryForPneumaticArm(false);
  if (shmaddr_!=0)
    UpdateSharedMemory();
}

void NCursesUI::UpdateSharedMemory()
{
  unsigned int index =0;
  unsigned int indx = 0;
  for(unsigned int i=0;i<NB_CONTROLS;i++)
  {
      control_[i] = shmaddr_[index++];
      log_data_ << control_[i] << " ";
  }
  for ( unsigned int i = 0; i < 7; i++)
   {
     potentiometer_[i] = shmaddr_[i+16];
     log_data_ << potentiometer_[i] << " ";
   }


  for(unsigned int i=0;i<7;i++)
  {
    
      if (Controller_->GetJointNum(i) == true)
            indx = i;
  }
  log_data_ << "\t\t";
  log_data_ << Controller_ -> GetErrorNow(indx);
  log_data_ << "\t\t";
  log_data_ << Controller_-> GetUpdateDelta(indx);
  log_data_ << "\t\t";
   log_data_ << "\n" ;
  //shmaddr_[23] = this -> get_FINITE_STATE();
}

void NCursesUI::Init()
{
  CreateSharedMemory();
  pthread_create(&handle_keyboard_,
                 NULL,
                 FunctionHandlingKeyboard,(void *)this);
}

bool NCursesUI::DisplayInformation()
{
  static unsigned long int counter=0;
  
  int row,col;				/* to store the number of rows and *
					 * the number of colums of the screen */
  wrefresh(main_win_);
  
  getmaxyx(main_win_,row,col);

  {
    std::ostringstream oss;
    oss << "Time :" << (double)counter*0.01;
    std::string as = oss.str();
    mvwprintw(main_win_,1,col/2,"%s",as.c_str());
  }

  attron(COLOR_PAIR(1));
  mvwprintw(main_win_,2,2,"Potentiometer");
  mvwprintw(main_win_,2,col/2,"Control");
  attroff(COLOR_PAIR(1));
  for(unsigned int i=0;i<7;i++)
    {
      {
        std::ostringstream oss;
        oss << potentiometer_[i];
        std::string as = oss.str();
        mvwprintw(main_win_,3+i,2,"%s",as.c_str());
      }
      {
        std::ostringstream oss;
        oss << control_[i];
        std::string as = oss.str();
        mvwprintw(main_win_,3+i,col/2,"%f",control_[i]);
      }

      {
        std::ostringstream oss;
        oss << control_[i+7];
        std::string as = oss.str();
        mvwprintw(main_win_,3+i,col/2+10,"%f",control_[i+7]);
      }

      {
        mvwprintw(main_win_,3+i,col/2+20,"%f",Controller_->GetErrorNow(i));
        mvwprintw(main_win_,3+i,col/2+30,"%f",Controller_->GetErrorDerivative(i));
      }

    }

  counter++;
  return end_of_loop_;
}
int NCursesUI::get_FINITE_STATE()
{
    return(FINITE_STATE);
}

bool NCursesUI::GetEndofLoop()
{
    return(end_of_loop_);
}
NCursesUI::~NCursesUI()
{
    log_data_.close();
    endwin();
}
