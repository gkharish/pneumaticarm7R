/*************************
 * Debugging  macros     *
 * CNRS, LAAS 2015       *
 * O. Stasse             *
 *************************/

#ifndef _DEBUG_PNEUMATIC_ARM_7R_H_
#define _DEBUG_PNEUMATIC_ARM_7R_H_

#define DBG_INFO  std::endl << __FILE__ << "\n" << __LINE__
#ifndef NDEBUG
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 3
#endif 
#define ODEBUG(x) std::cerr << x << std::endl
#define ODEBUGL(x,y) if (y<DEBUG_LEVEL) std::cerr << x << std::endl;
#else
#define ODEBUG(x)
#define ODEBUGL(x,y);
#endif

#endif
