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
#ifndef UFUNCTIONCAMDATA_H
#define UFUNCTIONCAMDATA_H

#include <urob4/ufuncplugbase.h>
#include <urob4/usmltag.h>
#include <urob4/uresclientifvar.h>

#include "uclienthandlercamif.h"
#include "uclientcamifgmk.h"
#include "uclientcamifpath.h"

class UResCamIfImg;
class UResCamIfCam;

#define __CAMIF_VERSION__ "2.1038"

/**
Direct interface to camera server

@author Christian Andersen
*/
class UFunctionCamData : public UFuncPlugBase
{
public:
  /**
  Constructor and module definition */
  UFunctionCamData();
  /**
  Destructor */
  ~UFunctionCamData();
  /**
  This function has a ressource that may be provided to others. */
  //virtual const char * resourceList();
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  Set resource pointer */
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  Test if all resources are loaded as intended */
  //virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Get pointer to shared resource */
  //virtual UResBase * getResource(const char * ressID);

protected:
  /**
  Handle the get command */
  bool handleGet(UServerInMsg * msg, void * extra);
  /**
  Add a ressource to ressource list and
  tell server core that there is new ressources */
  //bool addResource(const char * resName);

protected:
  /**
  Reference to camera interface connection */
  UResCamIfGmk * camifGmk;
  /**
  Is camera interface ressource created locally by this function */
  bool camifGmkLocal;
  /**
  Reference to camera interface connection */
  UResCamIfPath * camifPath;
  /**
  Is camera interface ressource created locally by this function */
  bool camifPathLocal;
  /**
  Reference to camera interface connection */
  UResCamIfImg * camifImg;
  /**
  Is camera interface ressource created locally by this function */
  bool camifImgLocal;
  /**
  Reference to camera interface connection */
  UResCamIfCam * camifCam;
  /**
  Is camera interface ressource created locally by this function */
  bool camifCamLocal;

private:
  //static const int MAX_RESOURCE_LIST_SIZE = 200;
  /**
  Ressource list provided */
  //char resList[MAX_RESOURCE_LIST_SIZE];
};


#endif
