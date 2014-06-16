/********************************************************************/
/* Titel      : labyrint.c                                          */
/* CPU-kort   : MVME117                                             */
/* Funktion   : Laeser ScanBeam-program ind fra disk, og modtager   */
/*              derefter kommandoer fra ScanBeam-kortet via faelles */
/*              variable.                                           */
/*              Varetager ud- og indlaesning fra disk samt start af */
/*              D/A-omsaetter programmet DA.                        */
/* Kald       : LoadLab        LoadRef        LoadHole    SaveHole  */
/*              SaveRef        StartScanBeam  DaLoop                */
/* Ind        : -                                                   */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/*              Modifiseret af Soeren Juhl Jensen  22/11 91         */
/* Dato       : 11/7 1990                                           */
/********************************************************************/
#include <errno.h>
#include <iau_pio.h>
#include <process.h>
#include <types.h>
#include "lab.h"
#include <stdio.h>
#include <module.h>
#include <memory.h>
#include <string.h>
#include <stdlib.h>

#define PERMS (MP_OWNER_READ+MP_OWNER_WRITE+MP_GROUP_READ+\
			MP_GROUP_WRITE+MP_WORLD_READ+MP_WORLD_WRITE)

#define LAB      0x605000

#define mswLAB   LAB >> 16
#define lswLAB   LAB & 0xFFFF

#define GO       1

#define FORGRUND 1

#define SLUT     0
#define DA       1
#define LOADREF  2
#define SAVEREF  3
#define LOADDEMO 4
#define LOADHOLE 5
#define LOADDEMOHOLE 6
#define SAVEHOLE 7

#define PROMT    0
#define DEFAULT  1

typedef short int (*REF_ARRAY)[2];

int	*gSema, *gXpos, *gYpos, *gStop;
int	*gCmdSema, *gCmdID, *gReturnValue, *gDAReady, *gSprog;
REF_ARRAY gRef, gHul, gRam;
/* Variable til synkronisering med ScanBeam-kortet            */

int more, intNo;
FILE *errmsg;
extern int setpr(int pid,short prior);
extern char	**_environ;
char		*argblk[] =
			{
				"download",
				"-d=5000",
				"-q",
				"labsbm",
				0,
			};


path_id DA_chan0, DA_chan2;


extern int getpid (void);
int LoadLab (char *);
int StartScanBeam (char *);
int DaLoop (void);
int LoadRef (int cmd, int *sprog);
int SaveRef (int *sprog);
int LoadHole (int cmd, int *sprog);
int SaveHole (int *sprog);
extern int wait (int *);
int getline (char *line, int n);

int main(void)
{
  int kommando, finished, a_dram;
	process_id id;
	u_int16 priority, group, user; 
  
  errmsg = fopen("/dd/SYS/errmsg.short","r");

  Femptyfifo( FALSE ); /* vent ikke paa ScanBeam-kommandoer */
  a_dram = (int)Fgetaddr(1); 
	_os9_id( &id, &priority, &group, &user );
	_os_permit( (void*)(a_dram),0x20000,PERMS,id );
  /*  Tildeling af variable i dynamisk ramomraade   */
  gSema	=           (int *)(a_dram + 0x17500);
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
  gSprog   =        (int *)(a_dram + 0x17510 + 2*2*(MAXPUNKT+MAXHOLES+4) + 0x10);

/*  intNo =  initsbevt(); */
  intNo = _ev_link("sbevt_0");
  *gCmdSema = 0; 
  finished = FALSE;

  setpr(getpid(),20000);
  printf("Henter ScanBeam-program fra disk ...");
  fflush( stdout );

  if( LoadLab((void *)a_dram) )
  {
    printf("\nKunne ikke hente ScanBeam-program.\n");
    exit(1);
  }
  printf(" OK.\n");
  pio_open_da( "/ipda0/da0",&DA_chan0 );
  pio_open_da( "/ipda0/da1",&DA_chan2 );


  StartScanBeam((char *)a_dram);

  do
  {
    _ev_wait( intNo, 0, 100);
    kommando = *gCmdID;

    switch ( kommando )
    {
      case     SLUT : *gCmdSema = 0;
                      finished = TRUE;
                      break;

      case       DA : *gCmdSema = 0;
                      DaLoop();
                      break;

      case  LOADREF : *gReturnValue = LoadRef( PROMT,gSprog );
                      *gCmdSema = 0;
                      break;

      case  SAVEREF : *gReturnValue = SaveRef(gSprog);
                      *gCmdSema = 0;
                      break;

      case LOADDEMO : *gReturnValue = LoadRef( DEFAULT,gSprog );
                      *gCmdSema = 0;
                      break;

      case LOADHOLE : *gReturnValue = LoadHole( PROMT,gSprog );
                      *gCmdSema = 0;
                      break;

      case LOADDEMOHOLE : *gReturnValue = LoadHole( DEFAULT,gSprog );
                      *gCmdSema = 0;
                      break;

      case  SAVEHOLE : *gReturnValue = SaveHole(gSprog);
                      *gCmdSema = 0;
                      break;
    }
  }
  while( !finished );
	_os_protect( (void*)(a_dram),0x20000,PERMS,id );
  _ev_unlink( intNo );
  fclose( errmsg );
  pio_close( DA_chan0 );
  pio_close( DA_chan2 );

  exit(0);
}

/********************************************************************/
/* Navn       : StartScanBeam                                       */
/* Funktion   : Starter labyrint-programmet paa ScanBeam-kortet     */
/* Kald       : -                                                   */
/* Ind        : -                                                   */
/* Ud         : -                                                   */
/* Reference  : ScanBeam-manual: Firmware side 5.                   */
/* Skrevet af : Allan Theill Sorensen                               */
/* Dato       : 11/7 1990                                           */
/********************************************************************/
StartScanBeam(char * a_dram)
{
  unsigned short * wp;
        wp = (unsigned short *)(a_dram+0x4800);
        *wp++ = 46;
        *wp++ = 0x0060;
        *wp++ = 0x5000;
        *((unsigned char *)(a_dram+0x1FE00)) = 1;

/*	Cjsruser( mswLAB, lswLAB);
*/
}


/********************************************************************/
/* Navn       : LoadLab                                             */
/* Funktion   : Bringer programmet Lab.c som koerer paa ScanBeam    */
/*              til at koere parallelt med dette (labyrint.c), som  */
/*              koerer paa OS-9 systemet.
/* Kald       : prerr                                               */
/* Ind        : -                                                   */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/* Dato       :                                                     */
/********************************************************************/
int LoadLab(char * agvdram)
{
  u_int16 type_lang;
  mh_com *sb_program;
  void *sb_exec;
  u_int16 mod_type;
  u_int16 attr;



  /*** Preparing for start of scanbeam program ***/
        mod_type = mktypelang(MT_PROGRAM,ML_OBJECT);
        if(_os_load( "cmds/labsbm",&sb_program,&sb_exec,MP_OWNER_READ,
                     &mod_type,&attr,MEM_ANY))
        {
                printf("%s: linking to sb-module failed\n",_prgname());
                return(errno);
        }

/*** Load program from os/9 to scanbeam ***/
        memcpy( (void*)(agvdram+0x5000),(void*)sb_program,sb_program->_msize );

/***** unload scanbeam module from OS9 memory  ***/
        _os_unload("labsbm", 0);

  return(0);

/*
	if (os9exec( os9forkc, argblk[0], argblk, _environ, 0, 0, 3) > 0 )
	{
		wait(0);
		return TRUE;
	}
	else
	{
		prerr( "Can't fork" );
		return FALSE;
	}
*/
}



/********************************************************************/
/* Navn       : LoadRef                                             */
/* Funktion   : Indlaeser referencerute fra disk og placerer den    */
/*              et forudbestemt sted i ScanBeam-kortets hukommelse. */
/* Kald       : -                                                   */
/* Ind        : cmd - hvis DEFAULT indlaeses DEMO.REF, hvis         */
/*                    PROMT promtes der paa terminalen for et fil-  */
/*                    navn                                          */
/*              sprog - det anvendte sprog                          */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/*              Modifiseret af Soeren Juhl Jensen  22/11 91         */
/* Dato       : 11/7 1990                                           */
/********************************************************************/
LoadRef( int cmd, int *sprog )
{
  FILE *fp;
  int  i;
  char filnavn[16];

  if( cmd==DEFAULT )
    fp = fopen("cmds/DEMO.REF", "r");
  else
  {
    if(*sprog == DANSK)
       printf("Hent rute fra fil : ");
    else if(*sprog == ENGLISH)
       printf("Get Track from file : ");
    getline(filnavn,16);
    fp=fopen(filnavn,"r");
  }
  if (fp==NULL)
  {
    prerr( "Labyrint " );
    return FALSE;
  }
  i=0;
  more = TRUE;
  while(more)
  {
    fscanf(fp,"%hd %hd",&(gRef[i][0]),&(gRef[i][1]));
    if(gRef[i][0] == -1) more = FALSE;
    i++;
  }
  fclose(fp);
  if(gRef[i-1][1] == 0) return FALSE;
  else return TRUE;
}

/********************************************************************/
/* Navn       : LoadHole                                            */
/* Funktion   : Indlaeser referencehuller fra disk og placerer dem  */
/*              et forudbestemt sted i ScanBeam-kortets hukommelse. */
/* Kald       : -                                                   */
/* Ind        : cmd - hvis DEFAULT indlaeses DEMO.HOL, hvis         */
/*                    PROMT promtes der paa terminalen for et fil-  */
/*                    navn                                          */
/*              sprog - det anvendte sprog                          */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Soeren Juhl Jensen                                  */
/* Dato       : 22/11 1990                                          */
/********************************************************************/
LoadHole(int cmd, int *sprog)
{
  FILE *fp;
  int  i;
  char filnavn[16];

  if( cmd==DEFAULT )
    fp = fopen("cmds/DEMO.HOL", "r");
  else
  {
    if(*sprog == DANSK)
       printf("Hent huller fra fil : ");
    else if(*sprog == ENGLISH)
       printf("Get Holes from file : ");
    getline(filnavn,16);
    fp=fopen(filnavn,"r");
  }
  if (fp==NULL)
  {
    printf( "Labyrint  file open error\n");
    return FALSE;
  }

  for(i=0;i<4;i++)
    fscanf(fp,"%hd %hd",&(gRam[i][0]),&(gRam[i][1]));
  i=0;
  more = TRUE;
  while(more)
  {
    fscanf(fp,"%hd %hd",&(gHul[i][0]),&(gHul[i][1]));
    if(gHul[i][0] == -1) more = FALSE;
    i++;
  }
  fclose(fp);
  if(gHul[i-1][1] != 0) return FALSE;
  else return TRUE;
}


/********************************************************************/
/* Navn       : SaveRef                                             */
/* Funktion   : Gemmer rute fra hukommelsen paa disk. Promter bru-  */
/*              geren for filnavn.                                  */
/* Kald       : -                                                   */
/* Ind        : sprog - det anvente sprog                           */
/* Ud         : FALSE hvis der var fejl fra SL/OS, ellers TRUE      */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theill Sorensen                               */
/*              Modifiseret af Soeren Juhl Jensen  22/11 91         */
/* Dato       : 11/7 1990                                           */
/********************************************************************/
SaveRef(int *sprog)
{
  FILE *fp;
  int  i;
  char filnavn[16];

  if(*sprog == DANSK)
     printf("Gem rute i fil : ");
  else if(*sprog == ENGLISH)
     printf("Save track in file : ");
  getline(filnavn,16);
  fp=fopen(filnavn,"w");
  if (fp==NULL)
  {
    printf( "Labyrint file open error\n");
    return FALSE;
  }
  i = 0;
  more = TRUE;
  while(more)
  {
  	fprintf(fp,"%hd  %hd\n",gRef[i][0],gRef[i][1]);
  	if(gRef[i][0] == -1) more = FALSE;
  	i++;
  }
  fclose(fp);
  return TRUE;
}

/********************************************************************/
/* Navn       : SaveHole                                            */
/* Funktion   : Gemmer huller fra hukommelsen paa disk. Promter bru-*/
/*              geren for filnavn.                                  */
/* Kald       : -                                                   */
/* Ind        : sprog - det anvendte sprog                          */
/* Ud         : FALSE hvis der var fejl fra SL/OS, ellers TRUE      */
/* Reference  : -                                                   */
/* Skrevet af : Soeren Juhl Jensen                                  */
/* Dato       : 22/11 1990                                          */
/********************************************************************/
SaveHole(int *sprog)
{
  FILE *fp;
  int  i;
  char filnavn[16];

  if(*sprog == DANSK)
     printf("Gem huller i fil : ");
  else if(*sprog == ENGLISH)
     printf("Save Holes in file : ");
  getline(filnavn,16);
  fp=fopen(filnavn,"w");
  if (fp==NULL)
  {
    printf( "Labyrint, file open error \n " );
    return FALSE;
  }
  for(i=0;i<4;i++)
     fprintf(fp,"%hd  %hd\n",gRam[i][0],gRam[i][1]);
  i = 0;
  more = TRUE;
  while(more)
  {
  	fprintf(fp,"%hd  %hd\n",gHul[i][0],gHul[i][1]);
  	if(gHul[i][0] == -1) more = FALSE;
  	i++;
  }
  fclose(fp);
  return TRUE;
}


/********************************************************************/
/* Navn       : getline                                             */
/* Funktion   : Indlaeser streng fra terminalen. Indlaesning af-    */
/*              sluttes med <CR>.                                   */
/* Kald       : -                                                   */
/* Ind        : *line - pointer til streng der skal indeholde den   */
/*                      laeste linie.                               */
/*              n     - laegden af line[]                           */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Steven Laursen                                      */
/* Dato       :                                                     */
/********************************************************************/
getline(char *line, int n)
{  char c;  int i=0;
   --n;
   while((c = getchar()) != '\n' && c != 13)
   {  if(c>31 && c<127)
      {  if(n>i) line[i++] = c; else
         { putchar(8); putchar(' ');
           putchar(7); putchar(8);    } } else
      if(c==8)
      {  if(i>0)
         { putchar(' ');putchar(c); --i;}
      }
   }
   putchar('\n');
   line[i]=0;
}


/********************************************************************/
/* Navn       : DaLoop                                              */
/* Funktion   : Varetager kommunikationen med de tilsluttede DA-    */
/*              omsaettere.                                         */
/* Kald       : -                                                   */
/* Ind        : -                                                   */
/* Ud         : -                                                   */
/* Reference  : -                                                   */
/* Skrevet af : Allan Theil Sorensen                                */
/* Dato       :                                                     */
/********************************************************************/
DaLoop(void)
{
	int u1x, u1y, sema, s1, s2, u, xmax, xmin, ymax, ymin, stop;
 
	stop = FALSE;
	sema = 0;
	xmax = 800;
	ymax = 1000;
	xmin = -800;
	ymin = -1000;
	
	*gDAReady = 1;
	do
	{
		_ev_wait( intNo, 0, 100 );
		
		u1x = *gXpos;
		u1y = *gYpos;
		
		*gSema = 0;
		
		if( u1x > xmax ) u = xmax;
		else if( u1x < xmin ) u = xmin;
		else u = u1x;
		pio_write_da( DA_chan0,u ); 
		if( u1y > ymax ) u = ymax;
		else if( u1y < ymin ) u = ymin;
		else u = u1y;
		pio_write_da( DA_chan2,u );
	}
	while( *gStop == 0 );
	
	/* Nulstil de to DA-omsaettere                      */
	pio_write_da( DA_chan0,0 );
	pio_write_da( DA_chan2,0 );
}
