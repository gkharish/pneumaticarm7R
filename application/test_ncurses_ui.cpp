#include <ncurses_ui.hh>
#include <unistd.h>
#include <controller.hh>

int main(void)
{
  Controller aController;
  NCursesUI ancursesui(aController);
    
  ancursesui.Init();
  bool loop=true;
  while(loop)
    {
      // Sleep 10ms.
      usleep(10000);

      // Display window
      ancursesui.UpdateSharedMemory();
      if (ancursesui.DisplayInformation())
        loop=false;
    }
  exit(0);
}
