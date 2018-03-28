/****************************************************************
 * System Trace Macrocell
 * Driver for STM logging
 *
 *
 ***************************************************************/

#ifndef STM_LOGGING_H
#define STM_LOGGING_H

#include "FileHandle.h"

//#ifdef __cplusplus
//extern "C" {
//extern void startRedirectToSTM(unsigned short channel, unsigned short guaranteed = 0);
//#else

extern void startRedirectToSTM(unsigned short channel, unsigned short guaranteed );
//#endif
extern void stopRedirectToSTM();
extern void stm_start();
//FILEHANDLE* mbed_target_override_console(int fd);


//#ifdef __cplusplus
//}
//#endif

#endif
