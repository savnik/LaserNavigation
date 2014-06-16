#include "lab.h"
#include "math.h"
#include <stdlib.h>
#include <stdio.h>
#include "fwguppy.h"
#include "kalman.h"
#include "refgen.h"
#include "track.h"
#include "maxint.h" 
#include "labyrinthgame.h"

/********************************************************************/
/* Omskrevet udgave af Run.c til brug med nu indput og output       */
/*                                                                  */
/********************************************************************/
/********************************************************************/
/* Globale variable paa faste adresser til dataoverforsel og        */
/* synkronisering med MVME117-kortet.                               */
/********************************************************************/
// extern int      *gSema, *gXpos, *gYpos, *gStop;
// extern int      *gCmdSema, *gCmdID, *gReturnValue, *gDAReady;

/********************************************************************/
/* Navn       : RunLab                                              */
/* Funktion   : Regulator program til styring af kuglen             */
/* Kald       : drawstring     Fejl           FindKugle             */
/*              InitKalman     InitRefGen     KalmanOutput          */
/*              MaskPlane      MaxInt         OpdatKalman           */
/*              RefGen         VisInstruktion Whspot                */
/*              FindTrackPart                                       */
/* Ind        : *ramme   - pointer til 'rect' som markerer laby-    */
/*                         rintens ramme                            */
/*                         kun ramme->polypoint[0].x og y bruges    */
/*              *kalx    - pointer til parametre til tilstands-     */
/*                         regulator i x-retning                    */
/*              *kaly    - pointer til parametre til tilstands-     */
/*                         regulator i y-retning                    */
/*                                                                  */
/*              *rg      - pointer til parametre til reference-     */
/*                         generator                                */
/*              ref[][]  - pointer til array med punkterne i refe-  */
/*                         renceruten                               */
/*              itot     - antal punkter i referenceruten           */
/*              maxfejl  - maksimal fejl foer stop, afvielser x & y */
/*                                                                  */
/*              maxsort  - antal sorte felter foer stop             */
/*                         billeder  uden kule i træk  før auto stop*/
/*              bredde   - rammens bredde i pixels                  */
/*               bruges kun til at udregne et forhold  til output   */
/*              edge     - omraade som afgraenser rammen            */
/*                          et måske 510x2 arry brugt i kugle finder*/
/*              ASPECT   - det benyttede kameras Aspect Ratio       */
/*                             Y's forhold til X, f.eks. 1          */
/*              c, s     - cos og sin til rammens drejningsvinkel   */
/*                    værdierne ikke vinklen i grader, f.eks. 1 og 0*/
/*              sprog    - det benyttede sprog  DANSK or ENGLISH    */
/* Ud         : -                                                   */
/*                   *gXpos og *gYpos er DA signalet                */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/*              Modificeret af Soren Juhl Jensen, efteraar 1991     */
/*              Decimeret af Andeas Emborg, foraar 2013             */
/* Dato       : 2/7 1990                                            */
/********************************************************************/

// extern int InitKalman (kalman_parm *parm,double x0);
// extern int VisInstruktion (char *msg1, char *msg2);
// extern int FindKugle (RECT *area, double *x, double *y, short int (*edge)[2]);
// extern int FindTrackPart (int *x, int *y, int *itot, short int *minpt, short int (*ref)[2]);
// extern int InitRefGen (refgen_parm *rg, short int xstart, short int ystart);
// extern int MaxInt (RECT *area, double *xmax, double *ymax, short int (*edge)[2]);
// extern int drawstring (int x, int y, char *str, int a1, int a2, int a3);
// extern int MaskPlane (int mask, int plane);
// extern int RefGen (refgen_parm *rg, short int (*ref)[2], int *i, int itot, double *aspect);
// extern int Whspot (RECT *intArea, int *thres, double *midtk, double *midtl, short int (*edge)[2]);
// extern int OpdatKalman (kalman_parm *parm, double y,double r,double v);
// extern int Fejl (char *msg1, char *msg2);
// extern double rundata[RUNDATASIZE][18];


/**
 * write instruction in danish or english
 * \param msg1 danish
 * \param msg2 english
 * \returns 0 */
int VisInstruktion (const char *msg1, const char *msg2)
{
  if(TRUE)
    printf("VisInstruktion: %s\n", msg1);
  else
    printf("VisInstruktion: %s\n", msg2);
  return 0;
}


/**
 * Main control loop */
int RunLab(POLYGON *ramme,
       kalman_parm *kalx, kalman_parm *kaly,
       refgen_parm *rg,
       short int (*ref)[2],
       int itot,
       int maxfejl,
       int maxsort,
       int bredde,
       short int (*edge)[2],
       //double *ASPECT,
       double *c,
       double *s,
       int *sprog)
{
  int    a, t, i /*, thres*/, dum, count;
  int    maxfejl2, fRefFejl, fEndOfRoute;
  int    xi1,yi1,xi2,yi2 /*,logcount=0*/,datacount;
  short int minpt, tref[MAXPUNKT][2];

  double x, y;
  double x1, y1, x2, y2;
//  double x0, y0;
  double xlp, ylp;
  double ex, ey;
  double xpix2da; //, ypix2da;
  double xlpold, ylpold;
  double vxlp,vylp;
//  double xinit;
  double ux, uy; //, fdum;
  double KalmanOutput(kalman_parm *parm, double y,double r,double v);
//  FILE   *fdata;

  RECT   intArea;
  /******************************************************************/
  /* Initialiseringer                                               */
  /******************************************************************/
  maxfejl2    = maxfejl*maxfejl;
             /* kvadratet paa tilladt max-afvigelse                 */
  xpix2da     = LABWIDTH / bredde * DA_UNITS / DA_VOLTAGE;
             /* da-enheder per pixel, x-retning                     */
  //ypix2da     = xpix2da / (*ASPECT);
             /* da-enheder per pixel, y-retning                     */
  fRefFejl    = FALSE;
  fEndOfRoute = FALSE;
  x           = 0;
  y           = 0;
  t           = 0;
  count       = 0;
  for(dum=0;dum<MAXPUNKT;dum++)
  {
     tref[dum][0] = ref[dum][0];
     tref[dum][1] = ref[dum][1];
  }


  /******************************************************************/

  /* find kuglen 1. gang                                            */
  VisInstruktion("Finder kuglen","Finding the Ball");
//chr Cscanfrm();
  intArea.p1.x = 0;
  intArea.p1.y = 0;
  intArea.p2.x = COLS - 1;
  intArea.p2.y = ROWS - 1;
  findKugleFirst(&x, &y);
  xi1 = (int)x - ramme->polypoint[0].x;
  yi1 = (int)y - ramme->polypoint[0].y;
  xi2 = (*c)*xi1 - (*s)*yi1;//*(*ASPECT);
  yi2 = (*s)*xi1 + (*c)*yi1;//*(*ASPECT);
  xi1 = xi2;
  yi1 = yi2;
//  xinit=xi1;
// chr removed
//   {
//       char buf[256];
//       Ccharmp(PLANE4);
//       sprintf(buf,"%lf %lf ",x,y);
//       Ccharxy(20,500);
//       Fdrawstr(buf);
//   }
// chr removed end

    InitKalman(kalx,xi1);
    InitKalman(kaly,yi1);


  /* Find den del af ruten, hvor referencegenereringen skal startes */
  FindTrackPart(&xi2,&yi2,&itot,&minpt,tref);
  if(minpt > 0)
  {
     i = minpt - 1;
     tref[i][0] = xi1;
     tref[i][1] = yi1;
     tref[i+1][0] = xi2;
     tref[i+1][1] = yi2;
  }
  else
  {
        i = minpt;
        tref[i][0] = xi1;
        tref[i][1] = yi1;
  }
  xi1 = x;
  yi1 = y;
  InitRefGen(rg,tref[i][0],tref[i][1]);
  intArea.p1.x = (int)x - DX/2;
  intArea.p1.y = (int)y - DY/2;
  intArea.p2.x = intArea.p1.x + DX;
  intArea.p2.y = intArea.p1.y + DY;

  /* initialiser thres                                              */
//chr  Cscanfrm();
//  thres = MaxInt( &intArea, &x, &y, edge) - THRES_OFFSET;

//chr  Cfillpl( PLANE0, BLACK );

  /* vent paa MVME117                                               */
/*  while( *gDAReady == FALSE );
*/
  VisInstruktion("Slut k\34rsel med h\34jre musetast",
                 "End running with right mouse botton");
// chr
//   if(*sprog == DANSK)
//      drawstring(MARGIN, 490, "Tur nr. %3d", ++count,0,0 );
//         /* 2 x 0 inserted by ap */
//   else if(*sprog == ENGLISH)
//      drawstring(MARGIN, 490, "Run no. %3d", ++count,0,0 );
//         /* 2 x 0 inserted by ap */
// chr end

//chr  Cplotmp( MaskPlane( BLUE | RED , 4 ) );
//    if (not openFwCam())
//    { //
//      printf("failed to open camera - application will crash!\n");
//      fflush(stdout);
//    }
   datacount=0;
  /* reguleringsloekke **********************************************/
  do
  {
    if( !fRefFejl ) fEndOfRoute = RefGen(rg, tref, &i, itot); //, ASPECT);
    xlp = rg->xlp;
    ylp = rg->ylp;
    if (datacount==0){
       xlpold=xlp;
       ylpold=ylp;
    }
    vxlp=(xlp-xlpold)*25;
    vylp=(ylp-ylpold)*25;
    xlpold=xlp;
    ylpold=ylp;
    /* offset for naeste Whspot                                     */
    intArea.p1.x = (int)x - DX/2;
    intArea.p1.y = (int)y - DY/2;
    intArea.p2.x = intArea.p1.x + DX;
    intArea.p2.y = intArea.p1.y + DY;

    /* scan ny frame og find kuglen                                 */
/*    Cscanfld(1); */
    /** wait for new image */
//    grabwait();
    a = findKugleAgain(&x, &y);
    /** find ball, OK if a==1
        ball position in x,y */
//    a = Whspot( &intArea, &thres, &x, &y, edge);


    /* Vend kuglens koordinater til retvendte koordinater           */
    x2 = x - (double)ramme->polypoint[0].x;
    y2 = y - (double)ramme->polypoint[0].y;
    x1 = x2*(*c) - y2*(*s);//*(*ASPECT);
    y1 = x2*(*s)+ y2*(*c);//*(*ASPECT);

    /* beregn fejlsignal                                            */
    ex = x1-xlp;
    ey = y1-ylp;

    /* beregn styresignal                                           */
 /*   ux = KalmanOutput(kalx, ex);
    uy = KalmanOutput(kaly, ey);
*/
/*    disabledisp();
    printf(" dum %f d %f %f %f  x1 xlp vxlp  %f %f %f xinit %f\n",kalx->dum[0],kalx->d[0],kalx->d[1],kalx->d[2],x1,xlp,
                                                                       vxlp,xinit);
     while (!Fmouser());
     while (Fmouser());
    enabledisp();
*/
    ux = KalmanOutput(kalx, x1,xlp,vxlp);
    uy = KalmanOutput(kaly, y1,ylp,vylp);

    /* styresignaler til MVME117                                    */
    *gXpos = ux*xpix2da;
    *gYpos = uy*xpix2da;

    //chr os9evt( intNo );
// log af kalman states - fjernet
//     rundata[datacount][0]=xlp;
//     rundata[datacount][1]=vxlp;
//     rundata[datacount][2]=x1;
//     rundata[datacount][3]=*gXpos;
//     rundata[datacount][4]=ylp;
//     rundata[datacount][5]=vylp;
//     rundata[datacount][6]=y1;
//     rundata[datacount][7]=*gYpos;
//     KalmanStates(kalx,x1,&rundata[datacount][8]);
//     KalmanStates(kaly,y1,&rundata[datacount][13]);
//     datacount++;
//     if (datacount >= RUNDATASIZE )
//       datacount=0;

/*      while( *gSema==1 );
    *gSema=1;
*/
    /* opdater regulator                                            */
    OpdatKalman(kalx, x1,xlp,vxlp);
    OpdatKalman(kaly, y1,ylp,vylp);

    /* afsaet plet ved position og LP-filtreret reference           */
// chr removed
//     Ccolour( BLUE );
//     Cbox( (int)x, (int)x, ((int)y & 0xfffe) , ((int)y &0xfffe));
//     Ccolour( YELLOW );
//     x0 = xlp*(*c) + ylp*(*s);
//     y0 = -xlp*(*s)/(*ASPECT) + ylp*(*c)/(*ASPECT);
//     x2 = x0 + (double)ramme->polypoint[0].x;
//     y2 = y0 + (double)ramme->polypoint[0].y;
//     Cbox( (int)x2, (int)x2, ((int)y2 &0xfffe+1), ((int)y2 &0xfffe) +1 );
//
//     {
//        RECT intarea1;
//        intarea1.p1.x=intArea.p1.x-6;
//        intarea1.p1.y=intArea.p1.y-6;
//        intarea1.p2.x=intArea.p2.x+6;
//        intarea1.p2.y=intArea.p2.y+6;
//         piclog_fld(intarea1,&logcount);
//     }
//     Ccolour( WHITE );
//     Cbox( (int)x, (int)x, ((int)y & 0xfffe)+1 , ((int)y &0xfffe)+1);
// chr removed end
    /* test for billede uden kugle                                  */
    if( a==0 )
      t++;
    else t = 0;

    if((i==itot) || (i==0))
    {
       tref[minpt-1][0] = ref[minpt-1][0];
       tref[minpt-1][1] = ref[minpt-1][1];
       tref[minpt][0] = ref[minpt][0];
       tref[minpt][1] = ref[minpt][1];
    }

    /* test for afvigelse fra referencen                            */
    if( ex*ex + ey*ey > maxfejl2 )
      fRefFejl = TRUE;
    else
      fRefFejl = FALSE;
#ifdef insertthis
      fEndOfRoute = TRUE;

      rg->xlp = x1;
      rg->ylp = y1;
      if((abs(x-xi1) < 25) && (abs(y-yi1) < 25))
         VisInstruktion("Kunne ikke finde kuglen, slut regulering med h\34jre musetast",
                        "Could not find the ball, end regulation with right mousebotton");
      else
         VisInstruktion("Kuglen hang fast, slut regulering med h\34jre musetast",
                        "The ball stuck, end regulation with right mousebotton");
// chr       if(*sprog == DANSK)
//          drawstring(MARGIN, 490, "Tur nr. %3d", ++count,0,0 );
//                 /* 2 x 0 inserted by ap */
//       else if(*sprog == ENGLISH)
//          drawstring(MARGIN, 490, "Run no. %3d", ++count,0,0 );
//                 /* 2 x 0 inserted by ap */
// chr end      Cplotmp( MaskPlane( BLUE | RED , 4 ) );
#endif


    /* start igen                                                   */
    if( fEndOfRoute && !fRefFejl )
   {
        fEndOfRoute = FALSE;
            i = 0;
            rg->forward = TRUE;

        VisInstruktion("Slut k\34rsel med h\34jre musetast",
                       "End running with right mousebotton");
//         if(*sprog == DANSK)
//            drawstring(MARGIN, 490, "Tur nr. %3d", ++count,0,0 );
//                 /* 2 x 0 inserted by ap */
//         else if(*sprog == ENGLISH)
//            drawstring(MARGIN, 490, "Run no. %3d", ++count,0,0 );
                /* 2 x 0 inserted by ap */
//chr        Cplotmp( MaskPlane( BLUE | RED , 4 ) );
    }
  } while ( t<maxsort);
  //&& !Fmouser() );
//   /* reguleringsloekke slut *****************************************/
//

// chr ny metde
//   while ( Fmouser() );
//  stopgrab();
// chr end
  /* Nulstil D/A
                                      */
  *gXpos = 0;
  *gYpos = 0;
// chr gen kalman states - fjernet
//   os9evt( intNo );
//   fdata=fopen("rundata.dat","w");
//   {int i,j;
//      for (i=0;i<datacount;i++){
//        for (j=0;j<18;j++)
//          fprintf(fdata,"%7e ",rundata[i][j]);
//        fprintf(fdata,"\n");
//      }
//   }
//   fclose(fdata);
// chr end
//chr   Cexgpl(PLANE0,PLANE1);
//chr   while (!Fmouser());
//chr   while (Fmouser());
  if( t >= maxsort )
    printf("---------- %s (%s)\n","Kuglen forsvandt","The ball disappeared");
  return count;
}
