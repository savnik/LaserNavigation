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

#ifndef URESCAMIF_H
#define URESCAMIF_H

#include <urob4/uresvarpool.h>
#include <urob4/uclienthandler.h>
#include <urob4/ucmdexe.h>

/**
* Camera interface class.
* This class acts as a client to the camera server.
* It is owned by the UFunctionCamIf class that also handles the command
  functions of the interface.
* The client handled standard variable as default. These may be
  updated by a timed or event driven push command to the connected camera
  server to this client.
* Two (at least) additional resources may be added to the interface
  to handle specific structures as a road-polygon or guidemarks.
  These data-type resources may then be accessed by other plugins for further processing.
*/
class UResCamIf : public UClientHandler, public UResVarPool, public UServerPush
{
public:
  /**
  Constructor */
  UResCamIf()
  { // set name and version
    setResID(getResClassID(), 200);
    //
    verboseMessages = true;
    createVarSpace(10, 0, 0, "Camera server interface variables", false);
    createBaseVar();
    // connect
    tryHoldConnection = true;
  };
  /**
  Destructor */
  virtual ~UResCamIf();
  /**
  Create basic polygon values */
  void createBaseVar();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "camif"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 182; };*/
  /**
  Print status for this resource */
  virtual const char * print(const char * preString, char * buff, int buffCnt)
  { return UClientHandler::snprint(preString, buff, buffCnt); };
  /**
  Print status for this resource */
  inline virtual const char * snprint(const char * preString, char * buff, int buffCnt)
  { return print(preString, buff, buffCnt); };
  /**
  Is this resource missing any base ressources */
  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Set ressource as needed (probably not used by this resource) */
  bool setResource(UResBase * resource, bool remove);

public:
  /**
    The varPool has methods, and a call to one of these is needed.
    Do what is needed and return (a double sized) result in 'value' and
    return true if the method call is allowed.
    The 'paramOrder' indicates the valid parameters d, s or c for double, string or class that is available as input values for the call.
   * If the returnStruct and returnStructCnt is not NULL, then
    a number (no more than initial value of returnStructCnt) of
    structures based on UDataBase may be returned into returnStruct array
    of pointers. The returnStructCnt should be modified to the actual
    number of used pointers (if needed). */
    virtual bool methodCall(const char * name, const char * paramOrder,
                            char ** strings, const double * pars,
                            double * value,
                            UDataBase ** returnStruct,
                            int * returnStructCnt);


protected:
  /**
  Called by client handler, when connected or disconnected */
  virtual void connectionChange(bool connected);

};



#endif

