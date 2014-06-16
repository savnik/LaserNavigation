
/********************************************************************/
/* Navn       : FindKugle                                           */
/* Funktion   : Finder kuglens position i billedet vha. en graa-    */
/*              tonejustering efterfulgt af et kuglematchfilter     */
/* Kald       : Justniv        MatchKugle     MaxInt                */
/* Ind        : *ramme - pointer til ramme som der skal soeges      */
/*                       indenfor                                   */
/*              *x     - pointer til x, hvori kuglens x-position    */
/*                       returneres                                 */
/*              *y     - pointer til y, hvori kuglens y-position    */
/*                       returneres                                 */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/* Dato       : 3/7 1990                                            */
/********************************************************************/
#include "lab.h"
#include <sbtotal.h>

int LinThres (short int (*edge)[2]);
int MatchKugle (void);
extern int MaxInt (RECT *area, double *xmax, double *ymax, short int (*edge)[2]);

FindKugle(RECT *area, double *x, double *y, short int (*edge)[2])
{
/*  JustNiv();
*/
  LinThres(edge);
 
  MatchKugle();
 
  MaxInt( area, x, y, edge);
 
}


/********************************************************************/
/* Navn       : MatchKugle                                          */
/* Funktion   : Fremhaever kugleformer i billedet vha. af et filter */
/*              med strukturen:                                     */
/*                 -------+++++++++++++++-------                    */
/*              som anvendes forst i vandret saa i lodret retning.  */
/*              Antallet af plusser skal vaere lig bredden af kug-  */
/*              len i pixels. Da der kun testes i to retninger      */
/*              matches ogsaa paa f.eks. lyse kvadrater i billedet. */
/*              Originalbilledet er plan 0, mens det filtrerede     */
/*              billede placeres i plan 1.                          */
/* Kald       :                                                     */
/* Ind        :                                                     */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/* Dato       : 9/7 1990                                            */
/********************************************************************/
#define DD 5     /* Antal plusser = 2*DD + 1                        */
#define NN 5     /* NN minusser i hver side af filteret             */
MatchKugle(void)
{
  UBYTE *pt0, *pt1;
  register int r,c,i;
  int dum,dumtmp,a_vram,a_vram1;

  Cfillpl( PLANE1, BLACK );
  
  a_vram = (int)Fgetmp(PLANE0);
  a_vram1 = (int)Fgetmp(PLANE1);
  /* Vandret filtrering *********************************************/
  pt0 = (UBYTE *) (a_vram+DD+NN + ROW_FACTOR*(DD+NN) );
/*  pt1 = pt0 + (O_PLANE1 - O_PLANE0); */
  pt1 = (UBYTE *) (a_vram1+DD+NN + ROW_FACTOR*(DD+NN) );
  for( r = DD+NN; r < ROWS-DD-NN; r++)
  {
    dum = 0;
    for( i=-DD-NN; i<=-DD-1; i++ ) dum -= (int) (*(pt0+i));
    for( i=-DD; i<=DD; i++ )       dum += (int) (*(pt0+i));
    for( i=DD+1; i<=DD+NN; i++ )   dum -= (int) (*(pt0+i));

    dumtmp = dum/16;
    if( dumtmp > MAX_GREY ) dumtmp = MAX_GREY;
    if( dumtmp < MIN_GREY ) dumtmp = MIN_GREY;
    *(pt1++) = dumtmp;

    for( c = DD+NN; c < COLS-DD-NN; c++)
    {
      dum +=       (int)(*(pt0-DD-NN+1))
             - 2 * (int)(*(pt0-DD+1))
             + 2 * (int)(*(pt0+DD+1))
             -     (int)(*(pt0+DD+NN+1));
      dumtmp = dum/16;
      if( dumtmp > MAX_GREY ) dumtmp = MAX_GREY;
      if( dumtmp < MIN_GREY ) dumtmp = MIN_GREY;
      *(pt1++) = dumtmp;
      pt0++;
    }
 /*   pt0 += 2*DD + 2*NN + (O_PLANE1 - O_PLANE0);
    pt1 = pt0 + (O_PLANE1 - O_PLANE0);*/
/*    pt0 += 2*DD + 2*NN + (ROW_FACTOR-COLS);
    pt1 += 2*DD + 2*NN + (ROW_FACTOR-COLS);
*/  pt0 = (UBYTE *) (a_vram+DD+NN + ROW_FACTOR*(r) );
    pt1 = (UBYTE *) (a_vram1+DD+NN + ROW_FACTOR*(r) );   
  }

  /* Lodret filtrering **********************************************/
  pt0 = (unsigned char *) (a_vram+DD+NN + ROW_FACTOR*(DD+NN) );
 /* pt1 = pt0 + (O_PLANE1 - O_PLANE0);
*/
  pt1 =(unsigned char *) (a_vram1+DD+NN + ROW_FACTOR*(DD+NN) );
  for( c = DD+NN; c < COLS-DD-NN; c++)
  {
    dum = 0;
    for( i=-DD-NN; i<=-DD-1; i++ ) dum -= (int) (*(pt0+i*ROW_FACTOR));
    for( i=-DD; i<=DD; i++ )       dum += (int) (*(pt0+i*ROW_FACTOR));
    for( i=DD+1; i<=DD+NN; i++ )   dum -= (int) (*(pt0+i*ROW_FACTOR));

    dumtmp = ((int)(*pt1) * dum)/256;
    if( dumtmp > MAX_GREY ) dumtmp = MAX_GREY;
    if( dumtmp < MIN_GREY ) dumtmp = MIN_GREY;
    *pt1 = dumtmp;
    pt1 += ROW_FACTOR;

    for( r = DD+NN; r < ROWS-DD-NN; r++)
    {
      dum +=       (int)(*(pt0-ROW_FACTOR*(DD+NN-1)))
             - 2 * (int)(*(pt0-ROW_FACTOR*(DD-1)))
             + 2 * (int)(*(pt0+ROW_FACTOR*(DD+1)))
             -     (int)(*(pt0+ROW_FACTOR*(DD+NN+1)));

      dumtmp = ((int)(*pt1) * dum)/256;
      if( dumtmp > MAX_GREY ) dumtmp = MAX_GREY;
      if( dumtmp < MIN_GREY ) dumtmp = MIN_GREY;
      *pt1 = dumtmp;
      pt1 += ROW_FACTOR;
      pt0 += ROW_FACTOR;
    }
    pt0 = (UBYTE *) (a_vram + c+1 + ROW_FACTOR*(DD+NN));
/*
    pt1 = pt0 + (O_PLANE1 - O_PLANE0);
*/
    pt1 = (UBYTE *) (a_vram1 + c+1 + ROW_FACTOR*(DD+NN));
    
  }
  Cexgpl(PLANE0, PLANE1);
}


/*******************************************************************/
/* Title    : LinThres                                             */
/* Funktion : Makes a linear threshold of the image in PLANE 0, so */
/*            that the pixelvalue is given by pix = pix*pix/255.   */
/*                                                                 */
/* Dependencies :  --                                              */
/* In       : edge   : Arrray holding the frame border.            */
/* Out      :  --                                                  */
/* Remark   :  --                                                  */
/* Ref.     :  --                                                  */
/* Made by  : Soeren Juhl Jensen                                   */
/* Date     : 18/11 91                                             */
/*******************************************************************/
LinThres(short int (*edge)[2])
{
        register int c,r;
        double temp;
        UBYTE *ptr0, *pt;

        ptr0 = (UBYTE *)Fgetmp(PLANE0);
        for(r=0;r<512;r++)
        {
                if(edge[r][0] != edge[r][1])
                        for(c=0;c<512;c++)
                        {
                                if((c>edge[r][0]) && (c<edge[r][1]))
                                {
                                        pt = ptr0 + c + r*ROW_FACTOR;
                                        temp = *pt;
                                        temp = temp*temp/255;
                                        *pt = (int)temp;
                                }
                        }
        }
}
