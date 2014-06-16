/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                   *
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

#include "uresseq.h"

// UResSeq::UResSeq()
// { // these first two lines is needed
//   // to save the ID and version number
//   setResID(getResID());
//   resVersion = getResVersion();
//   // other local initializations
//   // let sequencer calculator work on local 'seq' structure
//   //
//   createVarSpace(5, 0, 0, "Plan sequencer settings and status");
//   createBaseVar();
//   // let calculator work on this local var-pool
//   calc->setVarPool(getVarPool());
//   calc->addSysVars();
// }

///////////////////////////////////////////

void UResSeq::createBaseVar()
{
  UVarPool * vp;
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    vp->addVar("version", getResVersion() / 100.0, "d", "Resource version");
    varLineParsed = vp->addVar("lineParsed", lineParsed, "d", "Current line in sequencer");
    varSampleTime = vp->addVar("sampleTime", sampleTime, "d", "Time between sequencer activations (in seconds)");
  }
}

///////////////////////////////////////////

UResSeq::~UResSeq()
{
}

///////////////////////////////////////////

const char * UResSeq::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  snprintf(buff, buffCnt, "%s Sequenser is OK?\n", preString);
  return buff;
}

///////////////////////////////////////////


bool UResSeq::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to core for event handling
  bool result = true;
  int n = 0;
  char * p1 = missingThese;
  //
/*  if (resVarPool == NULL)
  {
    if (p1 != NULL)
    {
      strncpy(p1, UResVarPool::getResID(), missingTheseCnt);
      n = strlen(p1);
      p1 = &missingThese[n];
      result = false;
    }
  }*/
  result = calc->gotAllResources(p1, missingTheseCnt - n);
  n = strlen(p1);
  p1 = &missingThese[n];
  result &= UResBase::gotAllResources(p1, missingTheseCnt - n);
  return result;
}

//////////////////////////////////////////////////////////////////

bool UResSeq::setResource(UResBase * resource, bool remove)
{
  bool result = false;
//  bool isUsed = false;
  //
/*  if (resource->isA(UResVarPool::getResID()))
  { // this ressource may hold variables that can be accessed by this module
    result = calc->setResource(resource, remove);
    if (remove)
    {
      resVarPool = NULL;
      // this may trigger unexpected events - segmentation fault - and is not recommended
      // while a script is running
      //calc->setVarPool(NULL);
      result = true;
    }
    else if (resVarPool != ((UResVarPool * )resource))
    { // not the same, so change.
      resVarPool = (UResVarPool *) resource;
      //calc->setVarPool(resVarPool->getVarPool());
      // test if system variables is available
      if (calc->getVarPool()->getLocalVarIndex("simulated") < 0);
        // not found, so add the system variables needed by the sequencer function
        calc->addSysVars();
    }
  }*/
  result  = calc->setResource(resource, remove);
  result |= UResVarPool::setResource(resource, remove);
  //
  return result;
}

////////////////////////////////////////

void UResSeq::sequencerUpdate()
{
  // set line status
  varLineParsed->setValued(lineParsed, 0);
  // implement (potential) new sample time
  sampleTime = varSampleTime->getValued(); // sequenser sample time
}


