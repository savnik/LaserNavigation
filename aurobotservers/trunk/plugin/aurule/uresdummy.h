/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                        *
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

#ifndef URESMISDUMMY_H
#define URESMISDUMMY_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/usmrcl.h>
#include <urob4/uresbase.h>
#include <urob4/uvarcalc.h>
#include <urob4/ulogfile.h>


/**
 * This dummy class is used for testing only - can satisfy some function calls
 * as if it was a rowFinder.
 *
 * This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResDummy : public UResVarPool
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResDummySeq) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResDummy()
  { // these first two lines is needed
    // to save the ID, version number and description
    setResID("rowfinder", 204);
    // set description for global variables owned by this resource (optional)
    setDescription("Dummy version of row-finder");
    // create global variables for this resource
    createBaseVar();
  };
  /**
  Destructor */
  virtual ~UResDummy();

public:
  /**
   * A varPool method with this class as implementor is called.
   * \param name is the name of the called function
   * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
   * \param strings is an array of string pointers for the string type parameters (may be NULL if not used)
   * \param doubles is an array with double typed parameters (may be NULL if not used)
   * \param value is the (direct) result of the class, either a double value, or 0.0 for false 1.0 for true
   *              (2.0 for implicit stop if a controll call from mission sequencer).
   * \param returnStruct is an array of class object pointers that can be used as parameters or return objects (may be NULL)
   * \param returnStructCnt is the number of objects in the returnStruct buffer
   * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
  virtual bool methodCall(const char * name, const char * paramOrder,
                          char ** strings, const double * doubles,
                          double * value,
                          UDataBase ** returnStruct = NULL,
                          int * returnStructCnt = NULL);

protected:
  // Locally used methods
  /**
  Create the smrif related variables */
  void createBaseVar();

protected:
  /** index to local variables */
  int varDist, varTRel, varRow, varHead;
};

#endif

