#include <ncurses_ui.hh>
#include <unistd.h>

int main(void)
{
  NCursesUI ancursesui;

  while(1)
    {
      // Sleep 10ms.
      usleep(10000);

      // Display window
      ancursesui.DisplayInformation();
    }
  exit(0);
}
