/********************************************************************/
/* Navn       : JustNiv                                             */
/* Funktion   : Justerer graatoneniveauerne i billede 0 saaledes at */
/*              lokale kontraster bevares, mens forskelle i bille-  */
/*              det, der skyldes ujaevn belysning, glattes ud.      */
/* Kald       : -                                                   */
/* Ind        : -                                                   */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/* Dato       : 9/7 1990                                            */
/********************************************************************/
#include "lab.h"

#define NIVEAU  64
             /* oensket middel-niveau                               */
#define RES 5 /* kantlaengde af mindste justeringsenhed             */
              /* ULIGE TAL!!!!!!!!!!!!!!!                           */

JustNiv(void)
{
  UBYTE *pt0, *pt1;
  int sum,dum,kant,rowOffset;
  unsigned int a_vram,a_vram1;
  register int i,j,r,c;
  double norm, kvot;

  Cfillpl( PLANE1, BLACK );
  a_vram = (int)Fgetmp(PLANE0);
  a_vram1 = (int)Fgetmp(PLANE1);
  Fdebug("in justniv"); 
  kant = (RES-1)/2;
  norm = (2*D+1)*(2*D+1) * NIVEAU;
  for( r = D; r < ROWS-D; r+=RES)
  {
    pt0 = (UBYTE *) (a_vram + D + ROW_FACTOR*r);
/*    pt1 = pt0 + (O_PLANE1-O_PLANE0);
  */    pt1 =(UBYTE *)(a_vram1 + D + ROW_FACTOR*r);

 
    sum = 0;
    for( i=-D; i<=D; i++)
    {
      rowOffset = i*ROW_FACTOR;
      for( j=-D; j<=D; j++)
        sum += *(pt0 + j + rowOffset);
    }

    kvot = norm/sum;
    for( i=-kant; i<=kant; i++)
    {
      rowOffset = i*ROW_FACTOR;
      for( j=-kant; j<=kant; j++)
      {
        dum = *(pt0 + j + rowOffset) * kvot;
        if( dum > MAX_GREY ) dum = MAX_GREY;
        if( dum < MIN_GREY ) dum = MIN_GREY;
        *(pt1 + j + rowOffset) = dum;
      }
    }
    pt0 += RES;
    pt1 += RES;

    for( c = D+1; c < COLS-D; c+=RES)
    {
      for( i=-D; i<=D; i++)
      {
        rowOffset = i*ROW_FACTOR;
        for(j=0;j<RES;j++)
        {
           sum += *(pt0 + D + j + rowOffset) 
               - *(pt0 - D - j - 1 + rowOffset);
        }
/*        sum += *(pt0 + D     + rowOffset) - *(pt0 - D - 1 + rowOffset)
             + *(pt0 + D - 1 + rowOffset) - *(pt0 - D - 2 + rowOffset)
             + *(pt0 + D - 2 + rowOffset) - *(pt0 - D - 3 + rowOffset);
*/
      }

      kvot = norm/sum;
      for( i=-kant; i<=kant; i++)
      {
        rowOffset = i*ROW_FACTOR;
        for( j=-kant; j<=kant; j++)
        {
          dum = *(pt0 + j + rowOffset) * kvot;
          if( dum > MAX_GREY ) dum = MAX_GREY;
          if( dum < MIN_GREY ) dum = MIN_GREY;
          *(pt1 + j + rowOffset) = dum;
        }
      }
      pt0 += RES;
      pt1 += RES;
    }
  }
  Fdebug("After loop ");
  Cexgpl(PLANE0, PLANE1);
}
