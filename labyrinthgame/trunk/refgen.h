/***************************************************************************
 *   Copyright (C) 2013 by DTU (Christian Andersen, Andreas Emborg, m.fl.) *
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
#ifndef REFGEN_H
#define REFGEN_H
/** ****************************************************************
  Dato       : 2/7 1990                                            *
 *******************************************************************/


typedef struct
{
  double tstep; // number of steps left in this segment of the path
  int delx;
  int dely;
  double xstep; /// calculated step size in x direction (by RefGen)
  double ystep; /// calculated step size in y direction (by RefGen)
  double xref;
  double yref;
  double xlp; /// target x position after RefGen call (low pass filtered)
  double ylp; /// target y position after RefGen call (low pass filtered)
  double hast;
  double tau; /// low pass filter value for xlp,ylp
  int forward;
} refgen_parm;

/// segment index of last point
extern int lastPosIdx;
/// position on segment
extern double lastPosM;


typedef struct
{
  int x;
  int y;
} POINT;

typedef struct
{
  POINT p1;
  POINT p2;
} RECT;

#define FALSE 0
#define TRUE 1

typedef struct
{
        int     polysize;
        RECT    polybox;
        POINT   *polypoint;
} POLYGON;

/** ****************************************************************
 * Navn       : InitRefGen
 * Funktion   : Initialiserer variablene i referencegenerator-
 *              struct
 * Kald       : -
 * Ind        : \param *rg    - pointer til struct med referencegenerator-
 *                       variable
 *              \param xstart - x-koordinat paa foerste punkt i ruten
 *              \param ystart - y-koordinat paa foerste punkt i ruten
 * Ud         : -
 * Reference  : -
 * Skrevet af : Allan Theill Sorensen
 * Dato       : 9/7 1990
 ********************************************************************/
void InitRefGen (refgen_parm *rg, float xstart, float ystart, float hast);

/** *****************************************************************
 * Navn       : RefGen
 * Funktion   : Genererer referencevaerdier udfra array af knaek-
 *              punkter i referencen og udfra struct med parametre
 *              for referencegenereringen
 * Kald       : -
 * Ind        : \param *rg   - pointer til struct med parametre til refe-
 *                      rencegenereringen
 *              \param ref[] - array med knaekpunkter for referencen
 *              \param i     - indeks til oejeblikkelige placering i ref[]
 *                      saaledes at referencen er paa vej fra
 *                      punkt i-1 til punkt i
 *              \param itot  - antal punkter i alt
 * \returns     true hvis referencen har naaet sidste punkt i ref[]
 *              ellers false
 * Reference  : -
 * Skrevet af : Allan Theill Sorensen
 * Dato       : 9/7 1990
 ********************************************************************/
int RefGen (refgen_parm *rg, int ref[][2], int *i, int itot);

/**
 * go forward along route at constant speed (advance distance)
 * \param rg is struct with result (refX, refY)
 * \param ref is track
 * \param idx is current segment number (starting with 1 (point 0 to 1))
 * \param refCnt is number of points in track
 * \param lastX is current ball position in frame coordinates (pixels)
 * \param lastY is current ball position in frame coordinates (pixels)
 * \param advanceDist is distance to ref-position on track - in pixels.
 * \returns true if no more points (game finished successfully)
 * \returns reference position in rg structure and current track index in idx */
int RefGen2(refgen_parm *rg, int ref[][2], int *idx, int refCnt, double lastX, double lastY, double advanceDist);

/**
 * Find closest point on route 
 * \param rg is where the closest point is returned (xref, yref)
 * \param ref is track
 * \param idx is segment index with closest point.
 * \param refCnt is number of elements in track.
 * \param ballX is ball X position in pixels.
 * \param ballY is ball Y posiition.
 * \returns distance to closest point. */
double findOnRoutePoint(refgen_parm *rg, int ref[][2], int *idx, int refCnt, double ballX, double ballY);


#endif
