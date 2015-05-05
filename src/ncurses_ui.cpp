#include <ncurses_ui.hh>

#include <ncurses.h>			/* ncurses.h includes stdio.h */  
#include <string>
#include <iostream>
#include <sstream>
 
NCursesUI::NCursesUI()
{
 initscr();				/* start the curses mode */
}

void NCursesUI::DisplayInformation()
{
  static unsigned long int counter=0;

  int row,col;				/* to store the number of rows and *
					 * the number of colums of the screen */
  refresh();
  
  double potentiometer[7]={ 1.23, 0.07, -2.0, 2.24, 3.04, -1.04, -2.06};
  double control[7]={ 100,200,300,400,500,600,700};
  getmaxyx(stdscr,row,col);		/* get the number of rows and columns */

  {
    std::ostringstream oss;
    oss << "Time :" << counter*0.01;
    std::string as = oss.str();
    mvprintw(1,col/2,"%s",as.c_str());
  }

  mvprintw(2,2,"Potentiometer");
  mvprintw(2,col/2,"Control");
  for(unsigned int i=0;i<7;i++)
    {
      {
        std::ostringstream oss;
        oss << potentiometer[i]+((double)rand()/(double)RAND_MAX)*0.1;
        std::string as = oss.str();
        mvprintw(3+i,2,"%s",as.c_str());
      }
      {
        std::ostringstream oss;
        oss << control[i]+((double)rand()/(double)RAND_MAX)*100.0;
        std::string as = oss.str();
        mvprintw(3+i,col/2,"%s",as.c_str());
      }
    }
  
  counter++;
}

NCursesUI::~NCursesUI()
{
 endwin();
}
