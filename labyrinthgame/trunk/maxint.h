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
#ifndef MAXINT_H
#define MAXINT_H
/** ****************************************************************
  Dato       : 2/7 1990                                            *
 *******************************************************************/

int MaxInt (RECT *area, double *xmax, double *ymax, short int (*edge)[2]);
// extern int drawstring (int x, int y, char *str, int a1, int a2, int a3);
// extern int MaskPlane (int mask, int plane);
// extern int RefGen (refgen_parm *rg, short int (*ref)[2], int *i, int itot, double *aspect);
// extern int Whspot (RECT *intArea, int *thres, double *midtk, double *midtl, short int (*edge)[2]);

#endif
