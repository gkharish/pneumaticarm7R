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
  start_color();

  // Create a pair of color
  init_pair(1, COLOR_RED, COLOR_BLACK);

  int row,col;				/* to store the number of rows and *
					 * the number of colums of the screen */

  getmaxyx(stdscr,row,col);		/* get the number of rows and columns */
  
  main_win_ = newwin(LINES,COLS, 0,0);

  // Initialize pressure for all motors.
  double PressureForMuscles[NB_CONTROLS] = {
    1.0, 1.0, 1.0, 1.0, 1.0,
    1.1, 1.1, 1.1, 1.1, 1.1,
    2.0, 1.2, 1.2, 2.0, 
    1.0, 1.0};
  
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
  while(loop)
    {
      // Reading key.
      int c = getch();
      if (c=='q')
        {
          loop=false;
          end_of_loop_=true;
          FINITE_STATE = 5;
        }
      if (c=='s')
        {            
	  if (Controller_!=0)
	    Controller_->StartingRealTimeThread();
        }
      if ((c>='1') && (c<='7'))
        {                
	  if  (Controller_!=0)
	    {
	      unsigned int idx =2* c-1;
              unsigned int idx1 = c-1;
              Controller_->SetApplyControl(idx,true);
	      Controller_->SetApplyControl(idx-1,true);
              
              Controller_ -> SetJointNum(idx1);
              //Controller_ -> SetMeanPressure(idx1);
            }

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
  for(unsigned int i=0;i<NB_CONTROLS;i++)
    control_[i] = shmaddr_[index++];

  for(unsigned int i=0;i<7;i++)
    potentiometer_[i] = shmaddr_[index++];

  shmaddr_[23] = this -> get_FINITE_STATE();
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

    }

  counter++;
  return end_of_loop_;
}
int NCursesUI::get_FINITE_STATE()
{
    return(FINITE_STATE);
}
NCursesUI::~NCursesUI()
{
 endwin();
}
