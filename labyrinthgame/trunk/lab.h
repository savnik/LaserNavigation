/********************************************************************
 * Copyright (C) 2013 by DTU                                        *
 *                                                                  *
 *  Header-fil lab.h til labyrint-rutiner                           *
 *                                                                  *
 *  Allan Theill Sorensen F90                                       *
 *                                                                  *
 *  tilfoejelser :                                                  *
 *  Soeren Juhl Jensen E91                                          *
 *   jca@elektro.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef LAB_H
#define LAB_H

//#include <sbtotal.h>

//#include <oldevents.h>
void  prerr(char *);
char * _prgname(void);



#define MAXPUNKT        200
             /* Det maksimale antal punkter i ruten                 */
#define MAXFEJL         20   /* 40 */
             /* Maksimale tilladte fejl fra banereferencen          */
#define MAXBLACK         2
             /* Maksimale antal sorte 'fields' inden stop           */
#define D                               30

#define DX              25
             /* Bredde af interesseomraade                          */
#define DY              25
             /* Hoejde af interesseomraade                          */
#define TAUINT          91
#define TAUINT20                95
#define HASTINT         0.85
#define HASTINT20               0.5
#define MARGIN          2
#define LINEHEIGHT      20
#define LINELENGTH              20
#define MESSAGE_Y       40
             /* Y-vaerdi for starten af fejlmeddelser og            */
             /* instruktioner                                       */
#define LABWIDTH        0.275
             /* Bredde af labyrinten maalt mellem indre kanter      */
#define LABHEIGHT       0.230
             /* Hoejde af labyrinten maalt mellem indre kanter      */
#define O_LABWIDTH      0.293
#define O_LABHEIGHT     0.248
             /* Bredde og hoejde maalt mellem ydre kanter af indre  */
             /* ramme                                               */
#define FRAME_WIDTH     0.010
             /* Bredde af indre ramme                               */
#define DA_UNITS        4096
             /* Oploesning i DA-omsaetter                           */
#define DA_VOLTAGE      20.0
             /* Spaendingsomraade for DA-omsaetter                  */
#define THRES_OFFSET    20
             /* Afstand fra thres til max-vaerdi i pixels           */

#define SYNCADDR        0x00202600
#define VIAADDR_A       0xFFFF4001

#define MAINTITLE       "LABYRINTSPIL v2.0"

#define DANSK                   1
#define ENGLISH                 2

#define N_near           4
#define EPS_ANG          0.125
#define EPS_K            0.020
#define MAXHOLES         100
                /*  Tolerencer til brug ved detektion af rammen */

#define MAX_PIX          400
#define MAX_POINT        2000
                /*  Variable som bruges ved region growing        */

#define F1_MIN         10.0
#define F1_MAX         45.0
#define F2_MIN         0.10
#define F2_MAX         130.0
#define THRES_TIME     7
#define MOM_AREA       11
#define THRES_DIF      50
#define EPS_HOLE       10
#define THRES_ADD      15
#define MOM_EDGE       2
                /*  variable som bruges ved soegning efter huller */

#define SCANCHAN                1


/* length of ypixel / length of xpixel */
#define SL429C                  1.164
#define SL447                   1.039
#define SL429D                  1.091
#define SL429E                  0.823

/* #include <fp_control.h> */
#define RUNDATASIZE     6000



// typedef struct
// {
//   double a[6][6];
//   double b[6][6];     /* 3x1 matrix                              */
//   double c[6][6];     /* 1x3 matrix                              */
//   double d[6];        /* 1x1 matrix                              */
//   double x[6];     /* 3x1 matrix                              */
//   double dum[2];      /* 1x1 matrix                              */
//   double initvect[6];
//   double K[6];          /* kalman filter gains */
//   int n,ni,no;            /*order of system */
// } kalman_parm;

// typedef struct
// {
//   double tstep;
//   int delx;
//   int dely;
//   double xstep;
//   double ystep;
//   double xref;
//   double yref;
//   double xlp;
//   double ylp;
//   double hast;
//   double tau;
//   int forward;
// } refgen_parm;

// typedef struct
// {
//   int x;
//   int y;
// } POINT;
// 
// typedef struct
// {
//   POINT p1;
//   POINT p2;
// } RECT;
// 
// #define FALSE 0
// #define TRUE 1
// 
// typedef struct
// {
//         int     polysize;
//         RECT    polybox;
//         POINT   *polypoint;
// } POLYGON;

extern int intNo;

void mark_point(int maskPlane);

#endif
