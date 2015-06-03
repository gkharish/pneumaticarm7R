/* System includes */
#include <string>
#include <iostream>
#include <sstream>

/* Ncurses includes */
#include <ncurses_ui.hh>

/* Local framework. */
#include <shared_memory.hh>

void * FunctionHandlingKeyboard(void *argc)
{
  NCursesUI * ancursesui = static_cast<NCursesUI *>(argc);
  ancursesui->HandlingKeyboard();
  return NULL;
}
 
NCursesUI::NCursesUI():
  end_of_loop_(false)
{
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
  for(unsigned int i=0;i<16;i++)
    control_[i] = shmaddr_[index++];

  for(unsigned int i=0;i<7;i++)
    potentiometer_[i] = shmaddr_[index++];

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
        mvwprintw(main_win_,3+i,col/2,"%s",as.c_str());
      }
    }

  counter++;
  return end_of_loop_;
}

NCursesUI::~NCursesUI()
{
 endwin();
}
