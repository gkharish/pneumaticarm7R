#include <pneumatic_7arm_rt_thread.hh>

int main(void)
{
  Pneumatic7ArmRtThread aPneumatic7ArmRtThread;

  aPneumatic7ArmRtThread.Initializing();
	aPneumatic7ArmRtThread.StartingRealTimeThread();
}
