#include <math.h>
#include "linesensor.h"


int calc_linesensor_param(linesensor_partype param[],double white[],double black[],int N){
  int i;
  for (i=0;i<N;i++){
    param[i].off=black[i];
    param[i].gain=1.0/(white[i]-black[i]);
  }
  return(0);
}

void linesensor_correct(double output[],int input[], linesensor_partype param[],int N){
  int i;
  for( i=0; i<N; i++)
    output[i]=param[i].gain*(input[i]-param[i].off);
}

#define THRESH_B  0.43

int find_line(find_line_type *p,double line_sensor_val[], int N){
  int j;
  float min, max, cross; /* sensor intensity */
  float cmb, mb, wmb, cm2b, cm3b; /* black line vars */
  float cmw, mw, wmw, cm2w,maxposw; /* white line vars */
  float le, re; /* left edge, right edge of line */
  int sb, sw; /* sensor on black and white side of cross */
  /* reduce noice */
  /* printf("line sensor update\n"); */
  for (j=0; j<N; j++)
  {
     p->avg_s[j]=p->avg_s[j]*p->k_filt+line_sensor_val[j]*(1-p->k_filt);
  }
    /* find max (black) and min (white) */
    max=0;
    min=100;maxposw=0;
    for (j=0; j<N; j++)
    {
      if (p->avg_s[j]>max){
        max=p->avg_s[j];
	maxposw=j;
      }
      if (p->avg_s[j]<min)
        min=p->avg_s[j];
    }
    if (p->noline_b) {
      if (min < THRESH_B ){
         p->count++;
         if (p->count > 5){
           p->count=0;
           p->noline_b=0;
         }
      }
    }  
    else {
      if (min > THRESH_B ){
         p->count++;
         if (p->count > 5){
           p->count=0;
           p->noline_b=1;
         }
      }
    }

    /* center of gravity and total mass */
    mb=0; wmb=0;
    mw=0; wmw=0;
    for (j=0; j<N;j++)
    {
      mb=mb+(max-p->avg_s[j]);
      mw=mw+(p->avg_s[j]-min);
      wmb=wmb+(max-p->avg_s[j])*j;
      wmw=wmw+(p->avg_s[j]-min)*j;
    }
    cmb=wmb/mb;
    cmw=wmw/mw;
    /* 2nd moment of venter of gravity */
    cm2b=0;
    cm2w=0;
    cm3b=0;
    for (j=0; j<N;j++)
    {
      cm2b=cm2b+(max-p->avg_s[j])*pow(j-cmb,2);
      cm3b=cm3b+(max-p->avg_s[j])*pow(j-cmb,4);
      cm2w=cm2w+(p->avg_s[j]-min)*pow(j-cmw,2);
    }
    /* find left edge of black line */
    cross=(min+max)/2.0;
    j=1; sb=0;  sw=-1;
    while ((sw<0) && (j<N))
    {  /* find sensors sb black side, sw white side */
      if (p->avg_s[j]>cross)
        sb=j; /* not crossed */
      else
        sw=j; /* crossed */
      j++;
    }
    if ((sw>0) && (p->avg_s[sw]<p->avg_s[sb]))
    {  /* calc interpolation if possible */
      le=sb+(p->avg_s[sb]-cross)/(p->avg_s[sb]-p->avg_s[sw]);
      if (le<0) le=le/3;
      if (le<-1) le=-1;
    }
    else
      le=-1;
    /* find right edge of black line */
    j=N-2; sb=N-1; sw=-1;
    while ((sw<0) && (j>=0))
    { /* find two sensors */
      if (p->avg_s[j]>cross)
        sb=j; /* not crossed */
      else
        sw=j; /* crossed */
      j--;
    };
    if ((sw>=0) && (p->avg_s[sw]<p->avg_s[sb]))
    { /* calc interpolation if possible */
      re=sb-(p->avg_s[sb]-cross)/(p->avg_s[sb]-p->avg_s[sw]);
      if (re>(N-1)) re=N-1+(re-N+1)/3.0;
      if (re>N) re=N;
    }
    else
      re=N;
    /* store results globally */
    {
       double centre;
       centre=(N-1)/2.0;
       p->linepos_b = (cmb-centre) * 2; /* center is between sensor 3 and 4 */
       p->linepos_w = (maxposw-centre) * 2;
       p->right_edge_b = (le+1.1-centre)*2;  /* left edge approx 1 sensor off [result
      in cm] */
      p->left_edge_b = (re-1.1-centre)*2; /* right edge approx 1 sensor off [result in
      cm] */
    }
 //   mass_black = mb; /* no line if mb < ca. 70 */
 //   mass_white = mw; /* no line if mb < ca. ?? */
 //   mass_variance_black = cm2b; /* split if cm2b > ca. 400 and mass > ca. 150 */
 //   mass_variance3_black = cm3b; /* split if cm2b > ca. 400 and mass > ca. 150 */
 //   mass_variance_white = cm2w;
  
  return (0);
}

int find_line1(find_line_type *p,double line_sensor_val[], int N){
 int j;
  float min, max, cross; /* sensor intensity */
  float cmb, mb, wmb, cm2b, cm3b,minposb; /* black line vars */
  float cmw, mw, wmw, cm2w,maxposw; /* white line vars */
  float le, re; /* left edge, right edge of line */
  int sb, sw; /* sensor on black and white side of cross */
  /* reduce noice */
  /* printf("line sensor update\n"); */
  for (j=0; j<=N; j++)
  {
     p->avg_s[j]=p->avg_s[j]*p->k_filt+line_sensor_val[j]*(1-p->k_filt);
  }
    /* find max (black) and min (white) */
    max=0;
    min=100;maxposw=0;
    minposb=0;
    for (j=0; j<N; j++)
    {
      if (p->avg_s[j]>max){
        max=p->avg_s[j];
	maxposw=j;
      }
      if (p->avg_s[j]<min){
        min=p->avg_s[j];
        minposb=j;
      }   
    }
    /* center of gravity and total mass */
    mb=0; wmb=0;
    mw=0; wmw=0;
    for (j=0; j<N;j++)
    {
      mb=mb+(max-p->avg_s[j]);
      mw=mw+(p->avg_s[j]-min);
      wmb=wmb+(max-p->avg_s[j])*j;
      wmw=wmw+(p->avg_s[j]-min)*j;
    }
    cmb=wmb/mb;
    cmw=wmw/mw;
    /* 2nd moment of venter of gravity */
    cm2b=0;
    cm2w=0;
    cm3b=0;
    for (j=0; j<N;j++)
    {
      cm2b=cm2b+(max-p->avg_s[j])*pow(j-cmb,2);
      cm3b=cm3b+(max-p->avg_s[j])*pow(j-cmb,4);
      cm2w=cm2w+(p->avg_s[j]-min)*pow(j-cmw,2);
    }
    /* find left edge of black line */
    cross=(min+max)/2.0;
    j=1; sb=0;  sw=-1;
    while ((sw<0) && (j<N))
    {  /* find sensors sb black side, sw white side */
      if (p->avg_s[j]>cross)
        sb=j; /* not crossed */
      else
        sw=j; /* crossed */
      j++;
    }
    if ((sw>0) && (p->avg_s[sw]<p->avg_s[sb]))
    {  /* calc interpolation if possible */
      le=sb+(p->avg_s[sb]-cross)/(p->avg_s[sb]-p->avg_s[sw]);
      if (le<0) le=le/3;
      if (le<-1) le=-1;
    }
    else
      le=-1;
    /* find right edge of black line */
    j=N-2; sb=N-1; sw=-1;
    while ((sw<0) && (j>=0))
    { /* find two sensors */
      if (p->avg_s[j]>cross)
        sb=j; /* not crossed */
      else
        sw=j; /* crossed */
      j--;
    };
    if ((sw>=0) && (p->avg_s[sw]<p->avg_s[sb]))
    { /* calc interpolation if possible */
      re=sb-(p->avg_s[sb]-cross)/(p->avg_s[sb]-p->avg_s[sw]);
      if (re>(N-1)) re=N-1+(re-N+1)/3.0;
      if (re>N) re=N;
    }
    else
      re=N;
    /* store results globally */
    p->linepos_b = (minposb-3.5) * 2; /* center is between sensor 3 and 4 */
    p->linepos_w = (maxposw-3.5) * 2;
    p->right_edge_b = (le+1.1-3.5)*2;  /* left edge approx 1 sensor off [result
    in cm] */
    p->left_edge_b = (re-1.1-3.5)*2; /* right edge approx 1 sensor off [result in
    cm] */
 //   mass_black = mb; /* no line if mb < ca. 70 */
 //   mass_white = mw; /* no line if mb < ca. ?? */
 //   mass_variance_black = cm2b; /* split if cm2b > ca. 400 and mass > ca. 150 */
 //   mass_variance3_black = cm3b; /* split if cm2b > ca. 400 and mass > ca. 150 */
 //   mass_variance_white = cm2w;
  
  return (0);
}

