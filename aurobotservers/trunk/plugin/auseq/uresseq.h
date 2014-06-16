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

#ifndef URESSEQ_H
#define URESSEQ_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/usmrcl.h>
#include <urob4/uresbase.h>

#include "useqloop.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResSeq : public USeqLoop, public UResVarPool //, public USmrCl
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResSeq) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResSeq()
  { // set name and version number
    setResID(getResClassID(), 196);
    // other local initializations
    // let sequencer calculator work on local 'seq' structure
    //
    createVarSpace(5, 0, 0, "Plan sequencer settings and status", false);
    createBaseVar();
    // let calculator work on this local var-pool
    calc->setVarPool(getVarPool());
    calc->addSysVars();
  }
  ;
  /**
  Destructor */
  virtual ~UResSeq();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "seq"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 196; };*/
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Is this resource missing any base ressources */
  bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Set ressource as needed (probably not used by this resource) */
  bool setResource(UResBase * resource, bool remove);

// the above methods are used by the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  /**
  Is called when sequencer has active stuff to do */
  virtual void sequencerUpdate();

protected:
  // Locally used methods
  /**
  Create the smrif related variables */
  void createBaseVar();

protected:
  /**
  local variables used - or provided - by this resource. */
  //UResVarPool * resVarPool;
  /** index to parser line */
  UVariable * varLineParsed;
  /** index to sample time */
  UVariable * varSampleTime;
};

#endif

