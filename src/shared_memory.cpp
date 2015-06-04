#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <stdlib.h>

#include <shared_memory.hh>


double * CreateSharedMemoryForPneumaticArm(bool create)
{

  // Create the key related to the file.
  key_t key_to_shm = ftok(SHM_LOG_FILENAME,2061);
  int shmflg=SHM_R | SHM_W;
  if (create) 
    shmflg = IPC_CREAT | SHM_R | SHM_W;
  
  // Create the memory shared link with the key.
  int shmid;
  if ((shmid = shmget(key_to_shm, 16*sizeof(double)+7*sizeof(double)+sizeof(int), 
                       shmflg)) < 0)
    {
      shmflg = SHM_R | SHM_W;
      if ((shmid = shmget(key_to_shm, 16*sizeof(double)+7*sizeof(double)+sizeof(int), 
			  shmflg)) < 0)
	{
	  perror("Pb with the shared memory.");
	  exit(-1);
	}
    }
  
  // Attached the shared memory to a memory segment.
  double *shmaddr = (double *)shmat(shmid, 0,0);
  
  return shmaddr;
}
