#include "linesensor.h"
#include <stdio.h>
#include <math.h>

linesensor_partype param;

double white[MAXLINESENSORSIZE],black[MAXLINESENSORSIZE];
double ls_corrected[MAXLINESENSORSIZE];
int ls[MAXLINESENSORSIZE];
find_line_type linedata;



int main (void){
int i,linesensorsize;
    linesensorsize=16;
   for (i=0;i<linesensorsize;i++){
     white[i]=100;
     black[i]=0;
     ls[i]=100;  
     linedata.avg_s[i]=0;
   }
   calc_linesensor_param(&param,white,black,linesensorsize);
   linedata.k_filt=0.8;
   
   ls[15]=40;
   for(i=0;i<16;i++){
      linesensor_correct(ls_corrected,ls,&param,linesensorsize);
      find_line(&linedata,ls_corrected,linesensorsize);
   }
   printf("%lf \n",linedata.linepos_b);
   return 0;
   
}
