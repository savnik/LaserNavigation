/********************************************************************/
/* Titel      : lab.c                                               */
/* CPU-kort   : ScanBeam                                            */
/* Funktion   : Hovedprogram til styring og regulering af labyrint- */
/*              spil. Administrerer og behandler kommandoer fra     */
/*              hoved- og undermenuer.                              */
/* Kald       : drawstring      Fejl           Detect_Frame         */
/*              FrameConst      JusterLab      KlikKanter           */
/*              KlikRute        MaskPlane      DrawRect             */
/*              MVME117cmd      RunLab         KlikFrame            */
/*              TegnRute        KlikHoles      Help                 */
/*              VisInstruktion  ChangeTrack    NewTrackPoint        */
/*              Conv_track_save Conv_track_draw                     */
/* Ind        : -                                                   */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/*              Modificeret af Soren Juhl Jensen, efteraar 1991     */
/* Dato       : 21/3 1990                                           */
/********************************************************************/
#define NEWMENU

#include "lab.h"
#include "menu.h"
#include <stdio.h>

extern unsigned int initmenusystem();

typedef short int (*REF_ARRAY)[2];

/********************************************************************/
/* Globale variable paa faste adresser til dataoverforsel og        */
/* synkronisering med MVME117-kortet.                               */
/********************************************************************/
int     *gSema, *gXpos, *gYpos, *gStop;
int     *gCmdSema, *gCmdID, *gReturnValue, *gDAReady, *gSprog;
kalman_parm *gkalx,*gkaly;
REF_ARRAY gRef, gHul, gRam;
int intNo,sprog;
/********************************************************************/

#define SLUT     0
#define DA       1
#define LOADREF  2
#define SAVEREF  3
#define LOADDEMO 4
#define LOADHOLE 5
#define LOADDEMOHOLE 6
#define SAVEHOLE 7
#define LOADKALMAN 8

void Fdebug(char *);
int Fejl (char *msg1, char *msg2);
extern int drawstring (int x, int y, char *str, int a1, int a2, int a3);
extern int Detect_Frame (POLYGON *frame, short int (*hole_ref)[2], int maxhole, double *ASPECT);
extern int DrawRect (POLYGON *myRect, int maskPlane, int colour);
extern int FrameConst (POLYGON *frame, int *width, double *aspect, short int (*edge)[2], double *c, double *s);
extern int KlikFrame (POLYGON *frame, int maskplane, int colour);
int VisInstruktion (char *msg1, char *msg2);
extern int KlikRute (short int (*ref)[2], int x0, int y0, int bredde, int maskPlane, int colour);
extern int Conv_track_save (short int (*ref)[2], double *aspect, double *c, double *s);
int MVME117cmd (int cmd);
extern int Conv_track_draw (short int (*ref)[2], double *aspect, double *c, double *s);
extern int TegnRute (short int (*ref1)[2], short int (*ref2)[2], int x0, int y0, int nyBredde, int maskPlane, int colour);
extern int ChangeTrack (POLYGON *frame, int *itot, short int (*ref)[2], double *aspect, double *c, double *s);
extern int NewTrackPoint (POLYGON *frame, int *itot, short int (*ref)[2], double *aspect, double *c, double *s);
extern int KlikHoles (short int (*hole_ref)[2], int maskplane, int colour);
extern int JusterLab (void);
extern int RunLab (POLYGON *ramme, kalman_parm *kalx, kalman_parm *kaly, refgen_parm *rg, short int (*ref)[2], int itot, int maxfejl, int maxsort, int bredde, short int (*edge)[2], double *ASPECT, double *c, double *s, int *sprog);
extern int Help (int *language);
extern int Message (char *msg, int colour, int *texty, int length, int maskplane);
extern int MaskPlane (int mask, int plane);

#include "os9evt.c"
double rundata[RUNDATASIZE][18];
main(void)
{
  int x0, y0, itot, tauint, bredde, a_dram, scanchan;
  int fRamme, fRute, fHuller, finished, dum, maxsort, maxfejl;
  int count, kamera, maxhole, x, y, x1, y1, x2, y2, x3, y3;
  short int tref[MAXPUNKT][2], hole_ref[MAXHOLES][2], i, j;
  double aspect, Aspect[10], c, s, hastint,dumdob;
  char kammsg[10], tmpstring[80];
  int MaskPlane(int mask, int plane);

  POINT ramme_temp[4];
  POLYGON ramme;
  short int  edge[ROWS][2];

  kalman_parm kalx, kaly;
  refgen_parm rg;

  /******************************************************************/
  /* Tildeling af menuvariable                                      */
  /******************************************************************/
  MENU menusys[no_of_lans][no_of_menus];
  unsigned int error;
  MENU *hovedmenu, *rammemenu, *rutemenu, *ruteinputmenu, *hulmenu;
  MENU *hulinputmenu, *parametermenu, *nyparmmenu, *setupmenu;
  MENU *sprogmenu, *kameramenu, *startmenu;
  int hovedvalg, rammevalg, rutevalg, ruteinputvalg, hulvalg;
  int hulinputvalg, parametervalg, nyparmvalg, setupvalg;
  int sprogvalg, kameravalg, startvalg;

  /******************************************************************/
  /* Initialiseringer                                               */
  /******************************************************************/
  initmeteorsb();
  
  a_dram = (int)Fgetaddr(1);

  gSema   =         (int *)(a_dram + 0x17500);
  gXpos =           (int *)(a_dram + 0x17504);
  gYpos =           (int *)(a_dram + 0x17508);
  gStop =           (int *)(a_dram + 0x1750C);
  gRef  =           (REF_ARRAY)(a_dram + 0x17510);
  gHul  =           (REF_ARRAY)(a_dram + 0x17510 + 2*2*MAXPUNKT);
  gRam  =           (REF_ARRAY)(a_dram + 0x17510 + 2*2*(MAXPUNKT+MAXHOLES));
  gCmdSema =        (int *)(a_dram + 0x17510 + 2*2*(MAXPUNKT+MAXHOLES+4));
  gCmdID =          (int *)(a_dram + 0x17510 + 2*2*(MAXPUNKT+MAXHOLES+4) + 0x4);
  gReturnValue =    (int *)(a_dram + 0x17510 + 2*2*(MAXPUNKT+MAXHOLES+4) + 0x8);
  gDAReady =        (int *)(a_dram + 0x17510 + 2*2*(MAXPUNKT+MAXHOLES+4) + 0xC);
  gSprog =          (int *)(a_dram + 0x17510 + 2*2*(MAXPUNKT+MAXHOLES+4) + 0x10);
  gkalx =           (kalman_parm *) (a_dram + 0x17510 + 2*2*(MAXPUNKT+MAXHOLES+4) + 0x14);
  gkaly =           (kalman_parm *) (a_dram + 0x17510 + 2*2*(MAXPUNKT+MAXHOLES+4) + sizeof(kalman_parm)+0x14);
  *gCmdSema   = 0;
/*  intNo = initos9evt();*/
    intNo=1;
  scanchan    = SCANCHAN;
  Cscanvch(scanchan);
  Cscansch(scanchan);


  tauint      = TAUINT;
  hastint     = HASTINT;
  rg.tau      = -tauint/100.0;
  fRamme      = FALSE;
  fRute       = FALSE;
  fHuller     = FALSE;
  finished    = FALSE;
  maxfejl     = MAXFEJL;
  maxsort     = MAXBLACK;
  count       = 0;
  ramme.polysize = 4;
  ramme.polypoint = ramme_temp;
 
  Aspect[0] = 0;
  Aspect[1] = SL429C;
  Aspect[2] = SL429D;
  Aspect[3] = SL429E;
  Aspect[4] = SL447;
  kamera = 3;
  aspect = Aspect[3];
  strncpy(kammsg,"SL429E",10);
  disabledisp() ;
  MVME117cmd( LOADKALMAN);
  printf(" a= \n");
  {int i,j;
     for (i=0;i<gkalx->n;i++){
        for (j=0;j<gkalx->n;j++){ 
          printf("%f  ",gkalx->a[i][j]);
        }
        printf("\n");
     }
     printf("b = \n");
     for (i=0;i<gkalx->n;i++){
       for (j=0;j<gkalx->ni;j++)  
         printf("%f  ",gkalx->b[i][j]);
       printf("\n");
     }
     printf("c= \n");
     for (i=0;i<gkalx->no;i++){
       for(j=0;j<gkalx->n;j++)  
         printf("%f  ",gkalx->c[i][j]);
       printf("\n");
     }
     for (i=0;i<gkalx->ni;i++)
       printf("%f  ",gkalx->d[i]);

     for (i=0;i<gkalx->n;i++){
        for (j=0;j<gkalx->n;j++){ 
          printf("%f ",gkaly->a[i][j]);
        }
        printf("\n");
     }
     printf("b = \n");
     for (i=0;i<gkaly->n;i++)
        printf("%f  %f  ",gkaly->b[i][0],gkaly->b[i][1]);   
     printf("c= \n");
     for (i=0;i<gkaly->n;i++)
       printf("%f  ",gkaly->c[i]);
     printf("\n");
     printf("d= %f %f \n",gkaly->d[0],gkaly->d[1]);
     for (i=0;i<gkalx->n;i++){
       printf(" %f  ",gkalx->initvect[i]);
     }
     printf("\n");    
  }
printf(" press left mouse button to continue \n");  
while(!Fmousel());
enabledisp();
/*
  Ccharmp(PLANE4);
  Ccharcol(RED,WHITE);
  Ccharxy(200,200);
  Fdrawstr("initno %d",intNo); 
  Fdrawstr(" a_dram %x",a_dram);
  while(!Fmousel());
*/

  error = initmenusystem(menusys, menudata, 2, 12);
  if(error != 0)
  {
        Fejl("Menusystem kan ikke initieres",
             "Menusystem can not be initialized");
        finished = TRUE;
  }
  *gSprog       = DANSK;
  sprog         = *gSprog;
  hovedmenu     = &(menusys[0][0]);
  rammemenu     = &(menusys[0][1]);
  rutemenu      = &(menusys[0][2]);
  ruteinputmenu = &(menusys[0][3]);
  hulmenu       = &(menusys[0][4]);
  hulinputmenu  = &(menusys[0][5]);
  parametermenu = &(menusys[0][6]);
  nyparmmenu    = &(menusys[0][7]);
  setupmenu     = &(menusys[0][8]);
  sprogmenu     = &(menusys[0][9]);
  kameramenu    = &(menusys[0][10]);
  startmenu     = &(menusys[0][11]);


  /* Initialiser scanning til et 'field' ad gangen                  */
  Cscaneo( EVEN );

  /* Slet grafik- og billedplan                                     */
  Cfillpl( MaskPlane( WHITE, 4 ), BLACK );
  Cfillpl( MaskPlane( WHITE, 5 ), WHITE );
  Cfillpl( PLANE0, BLACK );
  Cscanfrm();
  /* reset DA-converters */
  *gXpos = 0;
  *gYpos = 0;
  MVME117cmd( DA );
 
  /******************************************************************/

  /******************************************************************/
  /* Hovedloekke                                                    */
  /******************************************************************/
  do
  {
     setclearlimit(hovedmenu,15);


     Cfillpl(PLANE5,WHITE);
     Ccharmp(PLANE5);
     Ccharcol( BLACK, BLUE );
     if(*gSprog == DANSK)
     {
        if( fRamme )                                 
           drawstring( MARGIN, ROWS-60, "Ramme  OK", 0,0,0 ); /* 3 x 0 inserted by ap */
        else                                     
           drawstring( MARGIN , ROWS-60, "Ramme  mangler", 0,0,0 ); /* 3 x 0 inserted by ap */
        if( fRute )
           drawstring( MARGIN , ROWS-40, "Rute   OK", 0,0,0 ); /* 3 x 0 inserted by ap */
        else
           drawstring( MARGIN , ROWS-40, "Rute   mangler", 0,0,0); /* 3 x 0 inserted by ap */
        if( fHuller )    
           drawstring( MARGIN , ROWS-20, "Huller OK", 0,0,0);  /* 3 x 0 inserted by ap */
        else                                       
           drawstring( MARGIN , ROWS-20, "Huller mangler", 0,0,0 );  /* 3 x 0 inserted by ap */
        drawstring( MARGIN, ROWS-80, "Sidste k\034rsel: %d ture", count, 0,0); /* 2 x 0 inserted by ap */
     }
     else if(*gSprog == ENGLISH)
     {
        if( fRamme )                                 
           drawstring( MARGIN , 450, "Frame OK" ,0,0,0 ); /* 3 x 0 inserted by ap */
        else                                     
           drawstring( MARGIN , 450, "Frame missing", 0,0,0 ); /* 3 x 0 inserted by ap */
        if( fRute )                                 
           drawstring( MARGIN , 470, "Track OK", 0,0,0); /* 3 x 0 inserted by ap */
        else
           drawstring( MARGIN , 470, "Track missing", 0,0,0 ); /* 3 x 0 inserted by ap */
        if( fHuller )    
           drawstring( MARGIN , 490, "Holes OK", 0,0,0 ); /* 3 x 0 inserted by ap */
        else                                       
           drawstring( MARGIN , 490, "Holes missing", 0,0,0 ); /* 3 x 0 inserted by ap */
        drawstring( MARGIN, 430, "Last driving: %d runs", count, 0,0 ); /* 2 x 0 inserted by ap */
     }

          
     hovedvalg = menuchoice(hovedmenu);
     switch(hovedvalg)
     {
        /**************************************************************/
        /* Ramme                                                      */
        /**************************************************************/
        case  1 :
           do
           {
              setclearlimit(rammemenu,10);
              rammevalg = menuchoice(rammemenu);
              switch(rammevalg)
              {
                 /* Find Ramme *****************************************/
                 case  1 :
                    if(fHuller)
                    {
                       Cscanfrm();
                       Fdebug("before ramme ");
                       for(dum=0;dum<ramme.polysize;dum++)
                       {
                          ramme.polypoint[dum].x = gRam[dum][0];
                          ramme.polypoint[dum].y = gRam[dum][1];
                       }
                        Fdebug("before hole_ref");
                       for(dum=0;dum<maxhole;dum++)
                       {
                          hole_ref[dum][0] = gHul[dum][0];
                          hole_ref[dum][1] = gHul[dum][1];
                       }
                       Cfillpl(PLANE4,BLACK);
                       Fdebug("Before Detect_frame");
                       fRamme = Detect_Frame( &ramme , hole_ref, maxhole,
                                              &aspect);
                       if( fRamme )
                       {
                          Cfillpl(PLANE4,BLACK);
                          Cscanfrm();
                          DrawRect(&ramme,MaskPlane( BLUE | GREEN , 4),BLUE );
                          FrameConst(&ramme,&bredde,&aspect,edge,&c,&s);
                       }
                       else
                          Fejl("Kan ikke finde rammen",
                               "Can not find the frame");
                       fRute = FALSE;
                    }
                    else
                       Fejl("Der er ingen huller i hukommelsen",
                            "There is no holes in memory");
                    break;

                 /* Klik ramme med musen ***************************************/
                 case  2 : 
                    Cscanfrm();
                    fRamme = KlikFrame( &ramme ,
                                        MaskPlane( BLUE | GREEN | RED, 4),
                                        BLUE );
                    if(fRamme)
                    {
                       VisInstruktion("\34jeblik !","Just a minute !");
                       FrameConst(&ramme,&bredde,&aspect,edge,&c,&s);
                       fRute = FALSE;
                    }
/*                    disabledisp();
                    {for(i=0;i<ROWS;i++){
                        printf(" %d %d \n",edge[i][0],edge[i][1]);
                        if(i %20 == 19)
                          getchar();
                    }
                    }    
                    enabledisp(); 
  */                  break;
              }
           }  while (rammevalg != 3);
           break;  

        /**************************************************************/
        /* Rute                                                       */
        /**************************************************************/
        case  2 :
           if ( fRamme)
           { 
              do  
              {
                 setclearlimit(rutemenu,10);
                 rutevalg = menuchoice(rutemenu);
                 switch(rutevalg)
                 {
                    /* Klik rute med musen ****************************************/
                    case  1 : 
                       Cscanfrm();
                       x0 = ramme.polypoint[0].x;
                       y0 = ramme.polypoint[0].y;
                       itot = KlikRute( gRef, x0, y0, bredde,
                                        MaskPlane( GREEN | RED | BLUE, 4),
                                        GREEN );
                       Conv_track_save(gRef,&aspect,&c,&s);
                       fRute = TRUE;
                       break;

                    /* Laes rute fra disk *****************************************/
                    case  2 : 
                       do
                       {
                          ruteinputvalg = menuchoice(ruteinputmenu);
                          switch(ruteinputvalg)
                          {
                             /* Standardrute *******************************/
                             case  1 : 
                                MVME117cmd( LOADDEMO );
                                if( *gReturnValue )
                                {
                                   Cscanfrm();
                                   x0 = ramme.polypoint[0].x;
                                   y0 = ramme.polypoint[0].y;
                                   for(i=0;i<MAXPUNKT;i++)
                                   {
                                      tref[i][0] = gRef[i][0];
                                      tref[i][1] = gRef[i][1];
                                   }
                                   Conv_track_draw(tref,&aspect,&c,&s);
                                   itot = TegnRute( gRef, tref, x0, y0,
                                                    bredde,
                                                    MaskPlane( GREEN |
                                                               RED |
                                                               BLUE,
                                                               4),
                                                    GREEN);
                                   fRute = TRUE;
                                }
                                else Fejl("Fejl ved diskindl\33sning fra OS-9",
                                          "Reading from disk at OS-9, failed");
                                break;

                             /* Anden rute *********************************/
                             case  2 : 
                                VisInstruktion("Indtast filnavn til OS-9",
                                               "Write filename to OS-9");
                                MVME117cmd( LOADREF );
                                if( *gReturnValue )
                                {
                                   Cscanfrm();
                                   x0 = ramme.polypoint[0].x;
                                   y0 = ramme.polypoint[0].y;
                                   for(i=0;i<MAXPUNKT;i++)
                                   {
                                      tref[i][0] = gRef[i][0];
                                      tref[i][1] = gRef[i][1];
                                   }
                                   Conv_track_draw(tref,&aspect,&c,&s);
                                   itot = TegnRute( gRef, tref, x0, y0,
                                                    bredde,
                                                    MaskPlane( GREEN |
                                                               RED |
                                                               BLUE,
                                                               4),
                                                    GREEN);
                                   fRute = TRUE;
                                }
                                else 
                                   Fejl("Fejl ved diskindl\33sning fra OS-9",
                                        "Reading from disk at OS-9, failed");
                                break;
                          }
                       }  while(ruteinputvalg != 3);
                          break;

                    /* Skriv rute paa disk **********************************/
                    case  3 :
                       if( fRute )
                       {
                          VisInstruktion("Indtast filnavn til OS-9",
                                         "Write filename to OS-9");
                          MVME117cmd( SAVEREF );
                          if( !(*gReturnValue) ) 
                             Fejl("Fejl ved diskskrivning til OS-9",
                                  "Writing to disk at OS-9, failed");
                       }
                       else Fejl("Der er ingen rute i hukommelsen",
                                 "There is no track in memory");
                       break;

                    /* Aendring af rutepunkt *******************************/
                    case  4 :
                       VisInstruktion("Klik p\35 punktet, som \34nskes \33ndret",
                                      "Mark the point to be changed");
                       ChangeTrack(&ramme,&itot,gRef,&aspect,&c,&s);
                       break;

                    /* Nyt rute punkt ************************************/
                    case  5 :
                       VisInstruktion("Klik p\35 et nyt punkt",
                                      "Mark the new point");
                       NewTrackPoint(&ramme,&itot,gRef,&aspect,&c,&s);
                       break;
                 }
              }  while (rutevalg != 6);
           }
           else Fejl("Kan ikke klikke ruten, rammen mangler",
                     "Can not click the track, the frame is missing");
           break;  

        /**************************************************************/
        /* Huller                                                     */
        /**************************************************************/
        case  3 :
           do
           {
              setclearlimit(hulmenu,10);
              hulvalg = menuchoice(hulmenu);
              switch(hulvalg)
              {
                 /* Klik huller med musen **********************************/
                 case  1 :
                    Cscanfrm();
                    dum = KlikFrame( &ramme ,
                                     MaskPlane( BLUE | GREEN | RED, 4),
                                     BLUE );
                    if(dum)
                    {
                       for(dum=0;dum<ramme.polysize;dum++)
                       {
                          gRam[dum][0] = (short int)ramme.polypoint[dum].x;
                          gRam[dum][1] = (short int)ramme.polypoint[dum].y;
                       }
                       maxhole = KlikHoles( gHul,MaskPlane( RED | GREEN | BLUE, 4),
                                            RED );
                       fRamme = FALSE;
                       fHuller = TRUE;
                    }
                    break;

                 /* Laes huller fra disk *****************************************/
                 case  2 : 
                    do
                    {
                       hulinputvalg = menuchoice(hulinputmenu);
                       switch(hulinputvalg)
                       {
                          /* Standardhuller *******************************/
                          case  1 : 
                             MVME117cmd( LOADDEMOHOLE );
                             if( !(*gReturnValue) )
                                Fejl("Fejl ved diskindl\33sning fra OS-9",
                                     "Reading from disk at OS-9, failed");
                             else 
                                fHuller = TRUE;  
                             dum = 0;
                             while(gHul[dum][0] != -1) dum++;
                             maxhole = dum;
                             for(dum=0;dum<ramme.polysize;dum++)
                             {
                                ramme.polypoint[dum].x = (int)gRam[dum][0];
                                ramme.polypoint[dum].y = (int)gRam[dum][1];
                             }
                             break;

                          /* Andre huller *********************************/
                          case  2 : 
                             VisInstruktion("Indtast filnavn til OS-9",
                                            "Write filename to OS-9");
                             MVME117cmd( LOADHOLE );
                             if( !(*gReturnValue) )
                                Fejl("Fejl ved diskindl\33sning fra OS-9",
                                     "Reading from disk at OS-9, failed");
                             else 
                                fHuller = TRUE;  
                             dum = 0;
                             while(gHul[dum][0] != -1) dum++;
                             maxhole = dum;
                             for(dum=0;dum<ramme.polysize;dum++)
                             {
                                ramme.polypoint[dum].x = (int)gRam[dum][0];
                                ramme.polypoint[dum].y = (int)gRam[dum][1];
                             }
                             break;
                       }
                    } while(hulinputvalg != 3);
                       break;

                 /* Skriv huller paa disk ************************************/
                 case  3 : 
                    if( fHuller )
                    {
                       VisInstruktion("Indtast filnavn til OS-9",
                                      "Write filename to OS-9");
                       MVME117cmd( SAVEHOLE );
                       if( !(*gReturnValue) )
                          Fejl("Fejl ved diskskrivning til OS-9",
                               "Writing to disk at OS-9, failed");
                    }
                    else Fejl("Der er ingen huller i hukommelsen",
                              "There is no holes in memory");
                    break;
              }
           }  while (hulvalg != 4);
           break;  

        /**************************************************************/
        /* Juster labyrint                                            */
        /**************************************************************/
        case  4 :
           Cfillpl(PLANE4,BLACK);
           JusterLab();
           fRamme = FALSE;
           fRute = FALSE;
           break;

        /**************************************************************/
        /* Start labyrint                                             */
        /**************************************************************/
        case  5 : 
           do
           {
              startvalg = menuchoice(startmenu);
              switch(startvalg)
              {
                 /*  Standard plade  */
                 case 1 :
                    Cfillpl(PLANE4,BLACK);
                    /* Laes Standard huller  */
                    if(!fHuller && !fRamme)
                    {
                       MVME117cmd( LOADDEMOHOLE );
                       if( !(*gReturnValue) ) 
                          Fejl("Fejl ved diskindl\33sning fra OS-9",
                               "Reading from disk at OS-9, failed");
                       else 
                          fHuller = TRUE;  
                       dum = 0;
                       while(gHul[dum][0] != -1) dum++;
                       maxhole = dum;
                       for(dum=0;dum<ramme.polysize;dum++)
                       {
                          ramme.polypoint[dum].x = (int)gRam[dum][0];
                          ramme.polypoint[dum].y = (int)gRam[dum][1];
                       }
                    }
                    /*  Find ramme */
                    if(fHuller && !fRamme)
                    {
                       Cscanfrm();
                       for(dum=0;dum<ramme.polysize;dum++)
                       {
                          ramme.polypoint[dum].x = gRam[dum][0];
                          ramme.polypoint[dum].y = gRam[dum][1];
                       }
                       for(dum=0;dum<maxhole;dum++)
                       {
                          hole_ref[dum][0] = gHul[dum][0];
                          hole_ref[dum][1] = gHul[dum][1];
                       }
                       fRamme = Detect_Frame( &ramme , hole_ref, maxhole,
                                              &aspect);
                       if( fRamme )
                       {
                          Cfillpl(PLANE4,BLACK);
                          Cscanfrm();
                          DrawRect(&ramme,MaskPlane( BLUE | GREEN , 4),BLUE );
                          FrameConst(&ramme,&bredde,&aspect,edge,&c,&s);
                          VisInstruktion("Check om rammen er ok!, hvis ja venstre musetast, hvis nej h\34jre",
                                         "Check if the frame is ok!, if yes : left mousebot., if no : right");
                          while(!Fmousel() && !Fmouser());
                          if(Fmousel())
                          {
                             fRamme = TRUE;
                             while(Fmousel());
                          }
                          if(Fmouser())
                          {
                             fRamme = FALSE;
                             while(Fmouser());
                          }
                       }
                       else
                          Fejl("Kan ikke finde rammen",
                               "Can not find the frame");
                       fRute = FALSE;
                    }
                    /*  Laes standard rute */
                    if(fRamme && !fRute)
                    {
                       MVME117cmd( LOADDEMO );
                       if( *gReturnValue )
                       {
                          Cscanfrm();
                          x0 = ramme.polypoint[0].x;
                          y0 = ramme.polypoint[0].y;
                          for(i=0;i<MAXPUNKT;i++)
                          {
                             tref[i][0] = gRef[i][0];
                             tref[i][1] = gRef[i][1];
                          }
                          Conv_track_draw(tref,&aspect,&c,&s);
                          itot = TegnRute( gRef, tref, x0, y0,
                                           bredde,
                                           MaskPlane( GREEN |
                                                      RED |
                                                      BLUE,
                                                      4),
                                           GREEN);
                          fRute = TRUE;
                       }
                       else Fejl("Fejl ved diskindl\33sning fra OS-9",
                                 "Reading from disk at OS-9, failed");
                    }   

                 /*  Anden plade  */
                 case 2 :
                    if(startvalg == 2)
                    {
                       Cfillpl(PLANE4,BLACK);
                       /*  Laes andre huller  */
                       fHuller = FALSE;
                       VisInstruktion("Indtast filnavn til OS-9",
                                      "Write filename to OS-9");
                       MVME117cmd( LOADHOLE );
                       if( !(*gReturnValue) )
                          Fejl("Fejl ved diskindl\33sning fra OS-9",
                               "Reading from disk at OS-9, failed");
                       else 
                          fHuller = TRUE;  
                       dum = 0;
                       while(gHul[dum][0] != -1) dum++;
                       maxhole = dum;
                       for(dum=0;dum<ramme.polysize;dum++)
                       {
                          ramme.polypoint[dum].x = (int)gRam[dum][0];
                          ramme.polypoint[dum].y = (int)gRam[dum][1];
                       }
                       /*  Find ramme  */
                       if(fHuller)
                       {
                          Cscanfrm();
                          for(dum=0;dum<ramme.polysize;dum++)
                          {
                             ramme.polypoint[dum].x = gRam[dum][0];
                             ramme.polypoint[dum].y = gRam[dum][1];
                          }
                          for(dum=0;dum<maxhole;dum++)
                          {
                             hole_ref[dum][0] = gHul[dum][0];
                             hole_ref[dum][1] = gHul[dum][1];
                          }
                          fRamme = Detect_Frame( &ramme , hole_ref, maxhole,
                                                 &aspect);
                          if( fRamme )
                          {
                             Cscanfrm();
                             DrawRect(&ramme,MaskPlane( BLUE | GREEN , 4),BLUE );
                             FrameConst(&ramme,&bredde,&aspect,edge,&c,&s);
                             VisInstruktion("Check om rammen er ok!, hvis ja venstremusetast, hvis nej h\34jre",
                                            "Check if the frame is ok!, if yes : left mousebot., if no : right");
                             while(!Fmousel() && !Fmouser());
                             if(Fmousel())
                             {
                                fRamme = TRUE;
                                while(Fmousel());
                             }
                             if(Fmouser())
                             {
                                fRamme = FALSE;
                                while(Fmouser());
                             }
                          }
                          else
                             Fejl("Kan ikke finde rammen",
                                  "Can not find the frame");
                          fRute = FALSE;
                       }
                       /*  Laes anden rute  */
                       if( fRamme && !fRute )
                       {
                          VisInstruktion("Indtast filnavn til OS-9",
                                         "Write filename to OS-9");
                          MVME117cmd( LOADREF );
                          if( *gReturnValue )
                          {
                             Cscanfrm();
                             x0 = ramme.polypoint[0].x;
                             y0 = ramme.polypoint[0].y;
                             for(i=0;i<MAXPUNKT;i++)
                             {
                                tref[i][0] = gRef[i][0];
                                tref[i][1] = gRef[i][1];
                             }
                             Conv_track_draw(tref,&aspect,&c,&s);
                             itot = TegnRute( gRef, tref, x0, y0,
                                              bredde,
                                              MaskPlane( GREEN |
                                                         RED |
                                                         BLUE,
                                                         4),
                                              GREEN);
                             fRute = TRUE;
                          }
                          else 
                             Fejl("Fejl ved diskindl\33sning fra OS-9",
                                  "Reading from disk at OS-9, failed");
                       }
                    }
                 /*  Start labyrint  */
                 case 3 :
                    if( fRute && fRamme )
                    {
                       Cfillpl( MaskPlane( RED | BLUE, 4) , BLACK );
                       Cplotmp( MaskPlane( RED | BLUE, 4) );
                       Ccolour( RED );
                       rg.hast = hastint*bredde/(1000*LABWIDTH);
                       *gStop = 0;
                       *gSema = 0;
                       *gDAReady = 0;
                       *gYpos=0;
                       *gXpos=0;
                       MVME117cmd( DA );
                       count = RunLab( &ramme, gkalx, gkaly, &rg, gRef, itot,
                          maxfejl, maxsort, bredde, edge, &aspect, &c, &s, gSprog);
          /*             *gStop = 1;
                       os9evt( intNo );
                       while( *gSema == 1 );
                       *gSema = 1;
            */        }
                    else
                    {
                       if( !fRamme ) 
                          Fejl("Kan ikke starte labyrinten, rammen mangler",
                               "Can not start the labyrinth, the frame is missing");
                       else if( !fRute ) 
                          Fejl("Kan ikke starte labyrinten, ruten mangler",
                               "Can not start the labyrinth, the track is missing");
                    }
                    break;
              }
           } while (startvalg != 4);            
           break;
        /**************************************************************/
        /* Parametre                                                  */
        /**************************************************************/
        case  6 :
           do
           {
              setclearlimit(parametermenu,10);
              parametervalg = menuchoice(parametermenu);
              switch(parametervalg)
              {
                 /* Nye parametre **********************************************/
                 case  1 : 
                    do
                    {
                       setclearlimit(nyparmmenu,10);
                       Cfillpl(PLANE5,WHITE);
                       if(*gSprog == DANSK)
                       {
                          Fsprintf( tmpstring,"Hastighed     = %-4.2f",hastint);
                          drawstring( MARGIN ,390 ,tmpstring,0,0,0);
/*                          drawstring( MARGIN ,390,"Hastighed     = %-4.2f", hastint,0,0 );
  */                            /* 2 x 0 inserted by ap */
                          drawstring( MARGIN ,390 + LINEHEIGHT,
                                      "LP-tidskonst. = %4d", tauint,0,0 );
                              /* 2 x - inserted by ap */
                          drawstring( MARGIN ,390 + 2*LINEHEIGHT,
                                      "Max afvigelse = %4d", maxfejl,0,0 );
                              /* 2 x 0 inserted by ap */
                          drawstring( MARGIN ,390 + 3*LINEHEIGHT,
                                      "Fejlbilleder  = %4d", maxsort,0,0 );
                              /* 2 x 0 inserted by ap */
                       }
                       else if(*gSprog == ENGLISH)
                       {
                          Fsprintf( tmpstring,"Velocity      = %-4.2f",hastint);
                          drawstring( MARGIN ,390 ,tmpstring,0,0,0);

/*                          drawstring( MARGIN ,390,"Velocity      = %-4.2f", hastint,0,0 );
  */                            /* 2 x 0 inserted by ap */
                          drawstring( MARGIN ,390 + LINEHEIGHT,
                                      "LP-timeconst. = %4d", tauint,0,0 );
                              /* 2 x 0 inserted by ap */
                          drawstring( MARGIN ,390 + 2*LINEHEIGHT,
                                      "Max deviation = %4d", maxfejl,0,0 );
                              /* 2 x 0 inserted by ap */
                          drawstring( MARGIN ,390 + 3*LINEHEIGHT,
                                      "Fail-images   = %4d", maxsort,0,0 );
                              /* 2 x 0 inserted by ap */
                       }
                       nyparmvalg = menuchoice(nyparmmenu);
                        switch(nyparmvalg)
                       {
                          /* Hastighed ************************************/
                          case  1 : 
                             VisInstruktion("Indtast hastighed st\34rre end 0",
                                            "Write velocity greater than 0");
                             if(*gSprog == DANSK)
                                {
                                  Fsprintf( tmpstring,"Hastighed     = %-4.2f",hastint);
                                  drawstring( MARGIN ,250 ,tmpstring,0,0,0);
                                }
/*                                drawstring(MARGIN,250,"Hastighed : %-4.2f",hastint,0,0);
  */                                    /* 2 x 0 inserted by ap */
                             else if(*gSprog == ENGLISH)
                                {
                                  Fsprintf( tmpstring,"Velocity      = %-4.2f",hastint);
                                  drawstring( MARGIN ,390 ,tmpstring,0,0,0);
                                }
/*                                drawstring(MARGIN,250,"Velocity : %-4.2f",hastint,0,0);
  */                                    /* 2 x 0 inserted by ap */
                             dumdob = keypad();
                             if( dumdob==0 )
                                Fejl("Hastighed 0 ikke tilladt",
                                     "Velocity 0 not allowed");
                             else
                             if( dumdob<=0 )
                                Fejl("Negativ hastighed ikke tilladt",
                                     "Negativ velocity not allowed");
                             else
                                hastint = dumdob;
                             break;

                          /* LP-filter tidskonst. *************************/
                          case  2 :
                             VisInstruktion("Indtast LP-filter konstant mellem 0 og 100",
                                            "Write LP-filter constant between 0 and 100");
                             if(*gSprog == DANSK)
                                drawstring(MARGIN,250,"LP-filter tidskonst.:",0,0,0);
                                        /* 3 x 0 inserted by ap */
                             else if(*gSprog == ENGLISH)
                                drawstring(MARGIN,250,"LP-filter timeconst.:",0,0,0);
                                        /* 3 x 0 inserted by ap */
                             dum = (int)keypad();
                             if( dum > 100 || dum < 0 )
                                Fejl("LP-filter konstant skal ligge mellem 0 og 100",
                                     "LP-filter constant scal be between 0 and 100");
                             else
                             {
                                tauint = dum;
                                rg.tau = -tauint/100.0;
                             }
                             break;

                          /* Maksimal afvigelse ***************************/
                          case  3 :
                             VisInstruktion("Indtast maksimal afvigelse fra reference til kugle",
                                            "Write maximaum deviation from reference to ball");
                             if(*gSprog == DANSK)
                                drawstring(MARGIN,250,"Maksimal afvigelse",0,0,0);
                                        /* 3 x 0 inserted by ap */
                             else if(*gSprog == ENGLISH)
                                drawstring(MARGIN,250,"Maximum deviation",0,0,0);
                                        /* 3 x 0 inserted by ap */
                             dum = (int)keypad();
                             if( dum <= 0 )
                                Fejl("Maksimal afvigelse mindre end 1 ikke tilladt",
                                     "Maximum deviation less than 1 not allowed");
                             else
                                maxfejl = dum;
                             break;

                          /* Antal fejl-billeder **************************/
                          case  4 :
                             VisInstruktion("Indtast det tilladte antal billeder uden kugle, f\34r regulatoren stoppes",
                                            "Write allowable amount of images without ball, before regulator is stopped");
                             if(*gSprog == DANSK)
                                drawstring(MARGIN,250,"Antal fejl-billeder",0,0,0);
                                        /* 3 x 0 inserted by ap */
                             else if(*gSprog == ENGLISH)
                                drawstring(MARGIN,250,"Amount of fail-images",0,0,0);
                                        /* 3 x 0 inserted by ap */
                             dum = (int)keypad();
                             if( dum <= 0 )
                                Fejl("Antal fejl-billeder mindre end 1 ikke tilladt",
                                     "Amount of fail-images less than 1 not allowed");
                             else
                                maxsort = dum;
                             break;

                          /* Standardparametre ****************************/
                          case  5 :
                             tauint      = TAUINT;
                             hastint     = HASTINT;
                             rg.tau      = -tauint/100.0;
                             maxfejl     = MAXFEJL;
                             maxsort     = MAXBLACK;
                             break;
                       }
                    }  while(nyparmvalg != 6);
                    break;

                 /* Setup *********************************************************/
                 case  2 :
                    do
                    {
                       setclearlimit(setupmenu,10);
                       Cfillpl(PLANE5,WHITE);
                       if(*gSprog == DANSK)
                       {
                          drawstring( MARGIN ,390,"Scan/Sync kanal = %1d", scanchan,0,0);
                                /* 2 x 0 inserted by ap */
                          drawstring( MARGIN ,390 + LINEHEIGHT,  "Kamera : ",0,0,0);
                                /* 3 x 0 inserted by ap */
                          Fsprintf( tmpstring,"    %s : %5.4f",kammsg,aspect );
                          drawstring( MARGIN ,390 + 2*LINEHEIGHT,
                                      tmpstring,0,0,0);
                                      /* 3 x 0 inserted by ap */
                       }
                       else if(*gSprog == ENGLISH)
                       {
                          drawstring( MARGIN ,390,"Scan/Sync chanal = %1d", scanchan,0,0);
                                /* 2 x 0 inserted by ap */
                          drawstring( MARGIN ,390 + LINEHEIGHT,  "Camera : ",0,0,0);
                                /* 3 x 0 inserted by ap */
                          Fsprintf( tmpstring,"    %s : %f5.4f",kammsg,aspect );
                          drawstring( MARGIN ,390 + 2*LINEHEIGHT,
                                      tmpstring,0,0,0);
                                      /* 3 x 0 inserted by ap */
                       }
                       setupvalg = menuchoice(setupmenu);
                       switch(setupvalg)
                       {
                          /* Scan/Sync kanal ************************************/
                          case  1 :
                             VisInstruktion("Indtast Scan/Sync kanal",
                                            "Write Scan/Sync chanal");
                             if(*gSprog == DANSK)
                                drawstring(MARGIN,250,"Scan/Sync kanal : ",0,0,0);
                                        /* 3 x 0 inserted by ap */
                             else if(*gSprog == ENGLISH)
                                drawstring(MARGIN,250,"Scan/Sync chanal : ",0,0,0);
                                        /* 3 x 0 inserted by ap */
                             dum = (int)keypad();
                             if( (dum < 1) || (dum > 4) )
                                Fejl("Scan kanal skal ligge mellem 1 og 4",
                                     "Scan chanal shal be between 1 and 4");
                             else
                                scanchan = dum;
                             break;
                          /* Kamera setup *********************************/
                          case  2 :
                             do
                             {
                                dum = kamera;
                                VisInstruktion("V\33lg kamera",
                                               "Choose camera");
                                Cfillpl(PLANE5,WHITE);
                                if(*gSprog == DANSK)
                                {
                                   drawstring( MARGIN ,390 + LINEHEIGHT,  "Kamera : ",0,0,0);
                                        /* 3 x 0 inserted by ap */
                                   Fsprintf( tmpstring,"    %s : %5.4f",kammsg,aspect );
                                   drawstring( MARGIN ,390 + 2*LINEHEIGHT,
                                               tmpstring,0,0,0);
                                               /* 3 x 0 inserted by ap */
                                }
                                else if(*gSprog == ENGLISH)
                                {
                                   drawstring( MARGIN ,390 + LINEHEIGHT,  "Camera : ",0,0,0);
                                        /* 3 x 0 inserted by ap */
                                   Fsprintf( tmpstring,"    %s : %5.4f",kammsg,aspect );
                                   drawstring( MARGIN ,390 + 2*LINEHEIGHT,
                                               tmpstring,0,0,0);
                                               /* 3 x 0 inserted by ap */
                                }
                                setclearlimit(kameramenu,12);
                                kameravalg = menuchoice(kameramenu);
                                switch(kameravalg)
                                {
                                   case 1 :
                                      strncpy(kammsg,"SL429C",10);
                                      kamera = 1;
                                      break;
                                   case 2 :
                                      strncpy(kammsg,"SL429D",10);
                                      kamera = 2;
                                      break;
                                   case 3 :
                                      strncpy(kammsg,"SL429E",10);
                                      kamera = 3;
                                      break;
                                   case 4 :
                                      strncpy(kammsg,"SL447",10);
                                      kamera = 4;
                                      break;
                                   case 5 :
                                      strncpy(kammsg,"Andet",10);
                                      kamera = 0;
                                      VisInstruktion("Indtast kameraets Aspect Ratio",
                                                     "Write the cameras Aspect Ratio");
                                      Aspect[0] = keypad();
                                      break;
                                }
                                aspect = Aspect[kamera];
                                if(kamera != dum)
                                {
                                   fRamme = FALSE;
                                   fRute  = FALSE;
                                }
                             }  while(kameravalg != 6);
                             break;

                          /* Standardsetup *********************************/
                          case  3 :
                             scanchan = SCANCHAN;
                             aspect = Aspect[1];
                             strncpy(kammsg,"SL429C",10);
                             break;
                       }
                    }  while(setupvalg != 4);
                    Cscanvch(scanchan);
                    Cscansch(scanchan);
                    aspect = Aspect[kamera];
                    break;

                 /* Sprog ***************************************************/
                 case 3 :
                    do
                    {
                       sprogvalg = menuchoice(sprogmenu);
                       switch(sprogvalg)
                       {
                          case 1:
                             *gSprog       = ENGLISH;
                             sprog         = *gSprog;
                             hovedmenu     = &(menusys[1][0]);
                             rammemenu     = &(menusys[1][1]);
                             rutemenu      = &(menusys[1][2]);
                             ruteinputmenu = &(menusys[1][3]);
                             hulmenu       = &(menusys[1][4]);
                             hulinputmenu  = &(menusys[1][5]);
                             parametermenu = &(menusys[1][6]);
                             nyparmmenu    = &(menusys[1][7]);
                             setupmenu     = &(menusys[1][8]);
                             sprogmenu     = &(menusys[1][9]);
                             kameramenu    = &(menusys[1][10]);
                             startmenu     = &(menusys[1][11]);
                             break;
                          case 2:
                             *gSprog       = DANSK;
                             sprog         = *gSprog;
                             hovedmenu     = &(menusys[0][0]);
                             rammemenu     = &(menusys[0][1]);
                             rutemenu      = &(menusys[0][2]);
                             ruteinputmenu = &(menusys[0][3]);
                             hulmenu       = &(menusys[0][4]);
                             hulinputmenu  = &(menusys[0][5]);
                             parametermenu = &(menusys[0][6]);
                             nyparmmenu    = &(menusys[0][7]);
                             setupmenu     = &(menusys[0][8]);
                             sprogmenu     = &(menusys[0][9]);
                             kameramenu    = &(menusys[0][10]);
                             startmenu     = &(menusys[0][11]);
                             break;

                          case 3:
                             *gSprog       = 2;
                             sprog         = *gSprog;
                             hovedmenu     = &(menusys[2][0]);
                             rammemenu     = &(menusys[2][1]);
                             rutemenu      = &(menusys[2][2]);
                             ruteinputmenu = &(menusys[2][3]);
                             hulmenu       = &(menusys[2][4]);
                             hulinputmenu  = &(menusys[2][5]);
                             parametermenu = &(menusys[2][6]);
                             nyparmmenu    = &(menusys[2][7]);
                             setupmenu     = &(menusys[2][8]);
                             sprogmenu     = &(menusys[2][9]);
                             kameramenu    = &(menusys[2][10]);
                             startmenu     = &(menusys[2][11]);
                             break;




                       }
                    }  while(sprogvalg != 4);
                    break;
              }
           }  while(parametervalg != 4); 
           break;                                       

        /**************************************************************/
        /* Hjaelp                                                     */
        /**************************************************************/
        case  7 :
           Help(gSprog);

           break;

        /**************************************************************/
        /* Slut                                                       */
        /**************************************************************/
        case  8 : 
           MVME117cmd( SLUT );
           finished = TRUE;
           break;
     }
  }
  while( !finished );
  /******************************************************************/
  /* Hovedloekke slut                                               */
  /******************************************************************/

  Cwreset();
}



/********************************************************************/
/* Navn       : Fejl                                                */
/* Funktion   : Udskriver fejlmeddelelse                            */
/* Kald       : Message                                             */
/* Ind        : msg1 : Meddelelse som skal udskrives. (Dansk)       */
/*              msg2 : Meddelelse som skal udskrives. (Engelsk)     */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/*              Modifiseret af Soeren Juhl Jensen                   */
/* Dato       :                                                     */
/********************************************************************/
Fejl(char *msg1, char *msg2)
{
  int texty;

  texty = MESSAGE_Y;
  Cfillpl( PLANE5, WHITE );
  if(sprog == DANSK)
     Message(msg1,RED,&texty,LINELENGTH,MaskPlane( WHITE, 5 ));
  else /*if(sprog == ENGLISH) */
     Message(msg2,RED,&texty,LINELENGTH,MaskPlane( WHITE, 5 ));
  Ccharcol( BLACK, BLUE );
  if(sprog == DANSK)
  {
     drawstring( MARGIN, texty += 3*LINEHEIGHT, "Tryk h\34jre musetast",0,0,0);
        /* 3 x 0 inserted by ap */
     drawstring( MARGIN, texty += LINEHEIGHT,   "for at forts\33tte",0,0,0);
        /* 3 x 0 inserted by ap */
  }
  else /* if(sprog == ENGLISH) */
  {
     drawstring( MARGIN, texty += 3*LINEHEIGHT, "Press right mouse-",0,0,0);
        /* 3 x 0 inserted by ap */
     drawstring( MARGIN, texty += LINEHEIGHT,   "botton to continue",0,0,0);
        /* 3 x 0 inserted by ap */
  }
  while( !Fmouser() );
}



/********************************************************************/
/* Navn       : VisInstruktion                                      */
/* Funktion   : Udskriver instruktionsmeddelelse                    */
/* Kald       : Message                                             */
/* Ind        : msg1 : Meddelelse som skal udskrives. (Dansk)       */
/*              msg2 : Meddelelse som skal udskrives. (Engelsk)     */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/*              Modifiseret af Soeren Juhl Jensen                   */
/* Dato       :                                                     */
/********************************************************************/
VisInstruktion(char *msg1, char *msg2)
{
  int texty;

  texty = MESSAGE_Y;
  Cfillpl( PLANE5 , WHITE);
  if(sprog == DANSK)
     Message(msg1,BLUE,&texty,LINELENGTH,MaskPlane( WHITE, 5 ));
  else if(sprog == ENGLISH)
     Message(msg2,BLUE,&texty,LINELENGTH,MaskPlane( WHITE, 5 ));
}



/********************************************************************/
/* Navn       : MVME117cmd                                          */
/* Funktion   : Sender kommando til MVME117 kortet vha. faelles-    */
/*              variable i hukommelsen. Venter paa OK fra MVME117   */
/* Kald       : -                                                   */
/* Ind        : cmd - kommandonummer                                */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/* Dato       : 3/7 1990                                            */
/********************************************************************/
MVME117cmd(int cmd)
{
  *gCmdID = cmd;
  *gCmdSema = 1;            /* Signaler til MVME117                  */
  os9evt( intNo );
  while( *gCmdSema == 1 );  /* Vent paa OK fra MVME117               */
}

void Fdebug(char * debugtext)
{
  Ccharmp(PLANE4);
  Ccharcol(RED,WHITE);
  Ccharxy(200,200);
  Fdrawstr(debugtext); 
  while(!Fmousel());
  while( Fmousel());
}
