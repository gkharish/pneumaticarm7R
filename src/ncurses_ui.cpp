#include <ncurses_ui.hh>

#include <string>
#include <iostream>
#include <sstream>

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
void NCursesUI::Init()
{
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
  
  double potentiometer[7]={ 1.23, 0.07, -2.0, 2.24, 3.04, -1.04, -2.06};
  double control[7]={ 100,200,300,400,500,600,700};
  getmaxyx(main_win_,row,col);

  {
    std::ostringstream oss;
    oss << "Time :" << counter*0.01;
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
        oss << potentiometer[i]+((double)rand()/(double)RAND_MAX)*0.1;
        std::string as = oss.str();
        mvwprintw(main_win_,3+i,2,"%s",as.c_str());
      }
      {
        std::ostringstream oss;
        oss << control[i]+((double)rand()/(double)RAND_MAX)*100.0;
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
