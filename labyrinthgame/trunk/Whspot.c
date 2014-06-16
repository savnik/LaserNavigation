/********************************************************************/
/* Titel   :Whspot.c                                                */
/* Funktion:Finder massemidtpunktet (intensitet = masse) af en lys  */
/*          figur paa en moerk baggrund. Interesseomraadet kan      */
/*          udgoere en vilkaarlig del af billedet.                  */
/* Kald    :-                                                       */
/* Ind     :*intArea- pointer til rect med koordinater paa          */
/*                    interesseomraadet.                            */
/*          thres- threshold vaerdi (skillevaerdi for lys/moerk).   */
/* Ud      :Returnerer det totale antal pixels med en intensitet    */
/*          hoejere end threshold vaerdien.                         */
/*          midtk- massemidtpunkt kolonne.                          */
/*          midtl- massemidtpunkt linie.                            */
/* Bemaerk :Programmet tager en pixel og er den over thres ses den  */
/*          som havende masse 1,under som havende massen 0.         */
/* Lavet af:Jakob Carlsen og Tommy Gade Jensen                      */
/* Tid     :26/ 9-89                                                */
/********************************************************************/
/* Aendret af Allan Theill Sorensen saa kun linier med lige numre   */
/* regnes med, og saa der benyttes dynamisk thresholding.           */
/********************************************************************/
#include "lab.h"

Whspot(RECT *intArea, int *thres, double *midtk, double *midtl, short int (*edge)[2])
{
  /* temp: indeholder pixelvaerdien for den pixel som pt peger paa. */
  /* tot: taeller antallet af pixelvaerdier over thres.             */
  /* mml: vandret moment.                                           */
  /* mmk: lodret moment.                                            */
  /* l,k: loekke variable.                                          */
  /* pt: pointer til pixel i billed.                                */
  int temp,tot,mml,mmk,l,k,x,y,a_vram;
  double ftot;
  UBYTE *pt;
  
  a_vram = (int)Fgetaddr(0);
  intArea->p1.y=2*(intArea->p1.y/2);  /*  kun linier med            */
  intArea->p2.y=2*(intArea->p2.y/2);  /*  lige numre                */
  /* udpeger den foerste pixel i interesseomraadet.                 */
  pt = (UBYTE *)
       (intArea->p1.y * ROW_FACTOR + intArea->p1.x + a_vram);
  mml = mmk = tot = 0;
  for(l = intArea->p1.y; l <= intArea->p2.y; l+=2 )
                                                   /* linie loekke. */
  {
      for(k = intArea->p1.x; k <= intArea->p2.x; k++)
                                                 /* kolonne loekke. */
      {
          if( (k > edge[l][0]) && (k < edge[l][1]) )
          {
                  temp = *pt;        /* aflaes pixel vaerdi.                */
                  if (temp > *thres) /* pixel er lys.                       */
                  {
                      mml += l;      /* summere momenterne.                 */
                      mmk += k;
                      tot++;
                  }
              }
          pt++; /* udpeg naeste pixel i linien.                     */
      }
      pt += ( 2*ROW_FACTOR - 1 - intArea->p2.x + intArea->p1.x );
                /* udpeg naeste linie                               */
  }
  if ( tot==0 ) return(tot);
  ftot = tot; /* provokerer double division i naeste linier.        */
  *midtl = (mml / ftot ) + 0.5; /* udregn massemidtpunkt.           */
  *midtk = (mmk / ftot ) + 0.5;

  x = 2*((int)(*midtk/2));
  y = 2*((int)(*midtl/2));
  *thres = *(UBYTE *)(y*ROW_FACTOR + x + a_vram)
           - THRES_OFFSET;
              /* Dynamisk thresholding:                               */
              /* 'thres' for naeste Whspot er 20 gratoner lavere    */
              /* end graaatonevaerdien for (x,y) fra denne Whspot.  */

  return(tot);
}
