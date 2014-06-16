#include <stdio.h>

#include "stop_tractor.h"

#include "client.h"
/* In order to get prototypes for 'cl_send' and 'cl_cmdwait'. */

#include "functions.h"
/* In order to get the macro DFLTTOOLPOS */

/* This function probably needs some adjustments ... */

void stop_tractor(void)
{
  cl_cmdwait("flush\n"); /* Empty the command buffer. */
  cl_cmdwait("stop\n"); /* Stop the robot (??). */
  
  /* It should be considered whether the following commands should go here
   * (they are copied from "main.c". (I.e. find out what they do!) 
   * "main.c" also does file handling for the log. This is not (yet) implemented here.
   */
  
  {
    char str[128];
    sprintf(str,"set \"hakoliftinggearstateref\" %i\n",DFLTTOOLPOS);
    cl_send(str);
  }
  
  cl_send("set \"hakopowertakeoffstateref\" 0\n");
  cl_send("set \"hakoenginespeed\" 900\n");
  cl_send("set \"hakomanual\" 1\n");
  cl_cmdwait("control \"savelog\"\n");
  cl_send("control \"resetlog\"\n");
  cl_send("control \"stoplog\"\n");
  cl_send("control \"removelogvars\"\n");
}
