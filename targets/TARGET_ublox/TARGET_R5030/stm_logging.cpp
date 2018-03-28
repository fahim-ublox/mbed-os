#include "stm_logging.h"
#include <stm_stim.h>
#include <stm_cr.h>
#include "objects.h"

#define STM_TRACE_LOG_CHANNEL    0x0010  /* log channel for PTP or APP processor */
//#define NULL 0

static volatile struct stm_stim_s* stm__stim_reg = (struct stm_stim_s*)(0x50000000);

/*
 * No semi hosting when this file is included.
 */
#pragma import(__use_no_semihosting)

static volatile char* hwtubeChar  = NULL;
static volatile char* hwtubeNl    = NULL;
static volatile char* hwtubeFirst = NULL;
static int firstChar = 1;


void redirect_to_STM(unsigned short channel, unsigned short guaranteed)
{
    hwtubeFirst = (char*)&(stm__stim_reg->ports[channel].g_dts);

    if (guaranteed) {
        hwtubeChar = (char*)&(stm__stim_reg->ports[channel].g_d);
    } else {
        hwtubeChar = (char*)&(stm__stim_reg->ports[channel].i_d);
    }

    hwtubeNl = (char*)&(stm__stim_reg->ports[channel].g_dm);
    firstChar = 1;
}



int send_data_to_STM(int ch)
{
   if (hwtubeChar) {
      char tempch = ch; // temp char avoids endianness issue

      if (tempch == 0x7f) {
         // Do not allow to clear the tube.
         tempch = ' ';
      }

      if (tempch == '\n') {
         if (firstChar) {
            *hwtubeFirst = ' ';
         }
         *hwtubeNl    = ' ';
         firstChar    = 1;
      } else if (firstChar) {
         *hwtubeFirst = tempch;
         firstChar    = 0;
      } else {
         *hwtubeChar  = tempch;
      }
   }

   return ch;
}

void sys_open_new(void)
{
   static int started = 0;

   if (started == 0) {
      const char startText[] = "Starting...\n";
      int i;
      int channel = STM_TRACE_LOG_CHANNEL;

      redirect_to_STM(channel, 0);
      started = 1;

      for(i = 0; i < 12; i++) {
         send_data_to_STM(startText[i]);
      }
   }
}

void stm_start()
{
	const char startText[] = "STM Running...\n";
	int i = 0, a = -50;
	sys_open_new();
	while(a<0)
	{
	      for(i = 0; i < 15; i++) {
	         send_data_to_STM(startText[i]);
	      }
	      a++;
	}
}


int ferror()
{
  /* Your implementation of ferror */
  //return EOF;
}

void _ttywrch(int ch)
{
  //ITM_SendChar(ch);
}

void _sys_exit(int return_code)
{
//label:  goto label;  /* endless loop */
}

FILEHANDLE* mbed_target_override_console(int fd)
{
    if (fd == 1)
    {
    	return (FILEHANDLE*)hwtubeChar;
    }
}
