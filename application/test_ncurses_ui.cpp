#include <ncurses_ui.hh>
#include <unistd.h>

int main(void)
{
  NCursesUI ancursesui;

  ancursesui.Init();
  bool loop=true;
  while(loop)
    {
      // Sleep 10ms.
      usleep(10000);

      // Display window
      if (ancursesui.DisplayInformation())
        loop=false;
    }
  exit(0);
}
