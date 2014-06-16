#include "stream2line.h"
#include <stdio.h>
#include <string.h>

int main(void){
stream2line_type buf;
stream2lineinit(&buf);

strcpy(buf.ib,"fwd 1\r\n");
buf.Nib=7;
stream2line(&buf);

printf("nlines %d \n",buf.nlines); 
printf("%s \n",buf.lb[buf.lineout]);
buf.nlines--;
buf.lineout++;
if( buf.lineout==2)
  buf.lineout=0;
strcpy(buf.ib,"turn 90\n\r");
buf.Nib=9;
stream2line(&buf);
printf("nlines %d \n",buf.nlines); 
printf("%s \n",buf.lb[buf.lineout]);

}
