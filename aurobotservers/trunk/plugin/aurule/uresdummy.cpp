/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
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

#include <stdio.h>
#include <math.h>

#include <urob4/uvarcalc.h>
#include <ugen4/usmltagin.h>
#include <urob4/ucmdexe.h>

#include "uresdummy.h"

///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////

UResDummy::~UResDummy()
{ // stop sequencer loop and wait for it to finish
  stop(true);
}

///////////////////////////////////////////

void UResDummy::createBaseVar()
{
  varDist = addVar("dist",  100.25, "d", "(r/w) distance to end of row ");  
  varTRel = addVar("trel",  M_PI/2.0, "d", "(r/w) heading in odometry coordinates ");  
  varRow = addVar("row",   0.0, "d", "(r) current row number ");  
  varHead = addVar("head",  0.0, "d", "(r/w) current headland line ");  
  addMethod("setTarget", "dd", "param 1 is row number, param 2 is headland line");
  addMethod("setTarget", "ddd", "param 1 is row number, param 2 is headland line, "
      "param 3 is iteration number");
  addMethod("correct", "", "Do localization");
  addMethod("correct", "d", "Do localization as control statement");
}

//////////////////////////////////////////////

bool UResDummy::methodCall(const char * name, const char * paramOrder,
                       char ** strings, const double * pars,
                       double * value,
                       UDataBase ** returnStruct,
                       int * returnStructCnt)
{ // implement method call from math
  bool result = true;
  double d;
  // evaluate standard functions
  if ((strcasecmp(name, "setTarget") == 0) and (strncmp(paramOrder, "ddd", 2) == 0))
  { // set target simulator
    setLocalVar(varDist, 10.0);
    setLocalVar(varRow, pars[0]);
    setLocalVar(varHead, pars[1]);
    if (strlen(paramOrder) == 3)
    {
      if (pars[0] > 1)
        *value = 2.0;
      else
        *value = 1.0;
    }
    else 
      *value = 2.0;
  }
  else if ((strcasecmp(name, "correct") == 0) and (strlen(paramOrder) <= 1))
  { // send also to client
    d = getLocalValue(varDist);
    if (d > 0)
    { // decrease distance
      d -= 0.47;
      setLocalVar(varDist, d);
    }
    *value = 2.0;
  }
  else
    result = false;
  if (returnStructCnt != NULL)
    *returnStructCnt = 0;
  return result;
}

///////////////////////////////////////////////
