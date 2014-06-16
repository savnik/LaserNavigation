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
#ifndef LABYRINTHGAME_H
#define LABYRINTHGAME_H



/** ****************************************************************
  Dato       : 2/7 1990                                            *
 *******************************************************************/

typedef struct
{
  double a[6][6];
  double b[6][6];     /* 3x1 matrix                              */
  double c[6][6];     /* 1x3 matrix                              */
  double d[6];        /* 1x1 matrix                              */
  double x[6];        /* 3x1 matrix                              */
  double dum[2];      /* 1x1 matrix                              */
  double initvect[6];
  double K[6];        /* kalman filter gains */
  int n,ni,no;        /* order of system */
} kalman_parm;

/// kalman parameters for x-direction
extern kalman_parm * gkalx;
/// kalman parameters for y direction
extern kalman_parm * gkaly;

/// initialize kalman position (in one direction)
void InitKalman (kalman_parm *parm,double x0);

/** update kalman filter
 * \param param kalman axis parameters
 * \param y is the ???
 * \param r is ???
 * \param v is ??? */
void OpdatKalman (kalman_parm *parm, double y,double r,double v);

/**
 * Load parameters for kalman filter - from file kalman.ini */
bool LoadKalman();

/** ******************************************************************
 * Navn       : KalmanOutput                                        *
 * Funktion   : Beregner styresignalet fra en tilstandsbaseret      *
 *              regulator                                           *
 * Kald       : -                                                   *
 * Ind        : *parm - pointer til struct med regulatorparametre   *
 *              y     - fejlsignal                                  *
 * Ud         : styresignalet fra regulatoren                       *
 * Reference  : -                                                   *
 * Skrevet af : Allan Theill Sorensen                               *
 * Dato       : 9/7 1990                                            *
 ********************************************************************/
double KalmanOutput(kalman_parm *parm, double y,double r,double v);


#endif
