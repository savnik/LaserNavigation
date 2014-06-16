/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
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

#ifndef UCAMPAREST_H
#define UCAMPAREST_H

//#include "ucamcommon.h"
#include <ugen4/ucampar.h>
///////////////////////////////////////////////////////

class UCamParEst : public UCamPar
{
public:
  /**
  Constructor */
  UCamParEst();
  /**
  Destructor */
  ~UCamParEst();
  /**
  Print to log - or console */
  inline void toLog(const char * info, int importanceLevel, UTime * toTime = NULL)
  { // no logging yet
    printf("%s\n", info);
  }
  /**
  Initialize for parameter estimation */
  void initParEst();
  /**
  Init matrices for 35 unknowns */
  void initParEst35();
  /**
  Save relevant dataset in structure. */
  bool setInitialValues35(
             unsigned int m,      // dataset element
             float pixX,          // pix position
             float pixY,          //
             UPosition posChart,  // position on chart (z = 0)
             UPosition posB,      // translation of barcode
             URotation rotB,      // rotation of barcode
             unsigned int set);   // position set
  /**
  Set two rows in jacobian matrix (mA) and result vector (vK)
  with the parameters provided. */
  bool setEstMatrix(
               int m,              // equation element
               float pixX,         // pix position
               float pixY,         //
               UPosition posChart, // position on chart (z = 0)
               UPosition posB,     // translation of barcode
               URotation rotB);    // rotation of barcode
  /**
  Set two rows in jacobian matrix (mA) and result vector (vK)
  with the parameters provided including position and rotation of
  the gmk number provided in 'set'. */
  bool setEstMatrix35(
               int m,   // equation element
               float pixX,  // pix position
               float pixY,  //
               UPosition posChart, // position on chart (z = 0)
               UPosition posB, // translation of barcode
               URotation rotB, // rotation of barcode
               int set);
  /**
  Solve equation and adjust parameters */
  bool adjustParameters(bool debug);
  /**
  Solve equation and adjust parameters */
  bool adjustParameters35(bool debug);
protected:
  /**
  Solve a set of equations by least square method.
  mA * R = vK, where R is the unknown. <br>
  Returns true if solved */
  bool solveLeastSquare(UMatrix4 * R); // result vector
private:
  /**
  Matrix where to put jacopian */
  UMatrixBig * mA;
  /**
  vector where to put point value */
  UMatrixBig * vK;
  /**
  Pixel position */
  float pixPosX[20];
  float pixPosY[20];
  /**
  Position on guidemark chart */
  UPosition posOnGmk[20];
  /**
  Position of guidemark relative to camera */
  UPosition posGmk[5];
  URotation rotGmk[5];
};

#endif

