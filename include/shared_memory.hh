#ifndef _PNEUMATIC_ARM_7R_SHARED_MEMORY_HH_
#define _PNEUMATIC_ARM_7R_SHARED_MEMORY_HH_

#define SHM_LOG_FILENAME "/var/log/pneumatic_arm.shm"
/** \brief This function creates a shared memory to control the INSA pneumatic ARM 
    It uses the file SHM_LOG_FILE_NAME with proj_id=2064.
    The map is 14 pressure values and 7 potentiometer values.
    If create is set to true then the system fails if the key already exists,
    otherwise it continues.
 */
double * CreateSharedMemoryForPneumaticArm(bool create);

#endif
