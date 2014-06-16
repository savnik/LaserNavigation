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
#ifndef UFUNCTIONLASER_H
#define UFUNCTIONLASER_H

//#include <unocv4/ufunctionbase.h>

#include "ulaserdevice.h"
#include "ulaserpool.h"
#include "uresv360.h"
#include "ufunclaserbase.h"

//#define MAX_MEASUREMENTS_NOT_USED 361

/**
Laser control functions

@author Christian Andersen
*/
class UFunctionLaser : public UFuncLaserBase
{
public:
  /**
  Constructor */
  UFunctionLaser();
  /**
  Destructor */
  ~UFunctionLaser();
  /**
  Create any resources that this modules needs and add these to the resource pool
  to be integrated in the global variable pool and method sharing. */
  virtual void createResources();
  /**
  Set a resource pointer as appropriate for this class. */
  virtual bool setResource(UResBase * ressource, bool remove);
  /**
  Test if all ressources are loaded as intended */
//  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Names of ressource names provided by this function.
  Used by server core to get name and count of provided ressources. */
//  const char * resourceList();
  /**
  get pointer to ressource with this name.
  Returns pointer if ressource is available, otherwise NULL.
  Used by the server core to get link to ressource. */
//  UResBase * getResource(const char * ressID);
  /**
  Handle command from client (or console).
  Return true if command is handles and alle needed
  actions are taken.
  Return false if command is not known (due
  to syntax error or that it belongs to another
  function) */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

protected:
  enum Dataformat { BIN, HEX, TAG };
  /**
  Handle get scandata command */
  bool handleScanGetCommand(UServerInMsg * msg, void * extra);
  /**
  Handle get scandata command */
  bool handleScanPushCommand(UServerInMsg * msg);
  /**
  Set scanner parameters */
  bool handleSetCommand(UServerInMsg * msg);
  /**
  Handle get scanner info commands */
  //bool handleGetCommand(UServerInMsg * msg);
  /**
  Send this scan to client */
  bool sendScan(UServerInMsg * msg, ULaserData * laserData,
                int dataInterval,
                Dataformat dataCodex,
                ULaserDevice * sick,
               bool andPose);

private:
  /**
  Pointer to the sick laser communication object */
  ULaserPool * lasPool;
  /**
  Is the laser pool locally owned */
  bool lasPoolLocal;
  /**
  Pointer to the virtual 360 deg laser scanner object */
  UResV360 * v360;
  /**
  Is the laser pool locally owned */
  //bool v360Local;
  /**
  Last serial send to client */
  unsigned long lastSerial[MAX_LASER_DEVS + 1];
  /**
  DEfault device */
  //int lasDef;
  /**
  POinter to pose history */
  UResPoseHist * poseHist;
};

#endif
