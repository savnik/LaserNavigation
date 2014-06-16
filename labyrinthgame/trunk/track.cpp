#include "lab.h"
#include <math.h>
#include <string.h>
#include "fwguppy.h"


/*******************************************************************/
/* Title    : Conv_track_save                                      */
/* Funktion : Converts the track so that it can be saved in the    */
/*            propper format.                                      */
/*                                                                 */
/* Dependencies :  --                                              */
/* In       : ref    : Arrray holding the track points             */
/*            aspect : The cameras Aspect Ratio                    */
/*            c, s   : Cos and Sin to the labyrinths slope         */
/* Out      :  --                                                  */
/* Remark   :  --                                                  */
/* Ref.     :  --                                                  */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 18/11 91                                             */
/*******************************************************************/
// void Conv_track_save(short int (*ref)[2], double *aspect, double *c, double *s)
// {
//    double x0,y0;
//    int i;
// 
//    i = 0;
//    while(ref[i][0] != -1)
//    {
//       x0 = (*c)*ref[i][0] - (*s)*ref[i][1]*(*aspect);
//       y0 = (*s)*ref[i][0] + (*c)*ref[i][1]*(*aspect);
//       ref[i][0] = x0;
//       ref[i][1] = y0;
//       i++;
//    }
// }

/*******************************************************************/
/* Title    : Conv_track_draw                                      */
/* Funktion : Converts the track so that it can be drawn on the    */
/*            screen.                                              */
/*                                                                 */
/* Dependencies :  --                                              */
/* In       : ref    : Arrray holding the track points             */
/*            aspect : The cameras Aspect Ratio                    */
/*            c, s   : Cos and Sin to the labyrinths slope         */
/* Out      :  --                                                  */
/* Remark   :  --                                                  */
/* Ref.     :  --                                                  */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 18/11 91                                             */
/*******************************************************************/
// void Conv_track_draw(short int (*ref)[2], double *aspect, double *c, double *s)
// {
//    double x0,y0;
//    int i;
// 
//    i = 0;
//    while(ref[i][0] != -1)
//    {
//       x0 = (*c)*ref[i][0] + (*s)*ref[i][1];
//       y0 = -(*s)*ref[i][0]/(*aspect) + (*c)*ref[i][1]/(*aspect);
//       ref[i][0] = x0;
//       ref[i][1] = y0;
//       i++;
//    }
// }



/*******************************************************************/
/* Title    :  FindTrackPart                                       */
/* Funktion :  Find a part of the labyrinth track, that is closest */
/*             to a given point.                                   */
/*                                                                 */
/* Dependencies :  ---                                             */
/* In       : x, y  : The point to match with the track.           */
/*            itot  : The number of points in the track.           */
/*            minpt : The first point before the the trackpart     */
/*                    closest to the point (x,y).                  */
/*            ref   : Array holding the track points.              */
/* Out      :  --                                                  */
/* Remark   :  --                                                  */
/* Ref.     :  --                                                  */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 18/11 91                                             */
/*******************************************************************/
void findTrackPart(float x2, float y2, float * linex, float * liney, int itot, int *minpt, int ref[][2])
{
   float min, dum, l, lx, ly;
   float x0, y0, x1, y1;

   min = 2 * COLS;
//   x2 = *x;  y2 = *y;
  /*  for(i=0;i<*itot;i++) */
  for(int i = 0; i < (itot-1); i++)  /* naa 7-1-97 */
  {
    x0 = x2 - ref[i][0];
    y0 = y2 - ref[i][1];
    x1 = ref[i+1][0] - ref[i][0];
    y1 = ref[i+1][1] - ref[i][1];
    dum = (x0*x1 + y0*y1) / (x1*x1 + y1*y1);
    lx = x0 - x1 * dum;
    ly = y0 - y1 * dum;
    l = sqrt(lx*lx + ly*ly);
    if ((dum <= 1.5) && (dum >= -0.5) && (l < min))
    {
      min = l;
      *minpt = (short int)i;
      *linex = x1*dum + (int)ref[i][0];
      *liney = y1*dum + (int)ref[i][1];
    }
  }
}


/*******************************************************************/
/* Title    :  FindTrackPoint                                      */
/* Funktion :  Find a point of the labyrinth track, that is        */
/*             closest to a given point.                           */
/*                                                                 */
/* Dependencies :  ---                                             */
/* In       : x, y  : The point to match with the track.           */
/*            itot  : The number of points in the track.           */
/*            minpt : The first point before the the trackpart     */
/*                    closest to the point (x,y).                  */
/*            ref   : Array holding the track points.              */
/* Out      :  --                                                  */
/* Remark   :  --                                                  */
/* Ref.     :  --                                                  */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 18/11 91                                             */
/*******************************************************************/
void FindTrackPoint(int *x, int *y, int *itot, short int *minpt, short int (*ref)[2])
{
   int x0,y0,min2,l2;
   short int i;

   min2 = 2*COLS*COLS;
/*   for(i=0;i<=(*itot);i++) */
   for(i=0;i<(*itot);i++)    /* naa 7-1-97 */
   {
      x0 = *x - ref[i][0];
      y0 = *y - ref[i][1];
      l2 = x0*x0 + y0*y0;
      if(l2 < min2)
      {
         min2 = l2;
         *minpt = i;
      }
   }
}








