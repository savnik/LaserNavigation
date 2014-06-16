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
#ifndef UFUNCTIONCAMGMK_H
#define UFUNCTIONCAMGMK_H

#include <ucam4/ufunctioncambase.h>
#include <urob4/upush.h>

#include "ubarcode.h"
#include "ucalibrate.h"


/**
Guidemark finder class
Version tag $Id: ufunctioncamgmk.h 1827 2012-02-12 09:01:02Z jca $
@author Christian Andersen
*/
class UFunctionCamGmk : public UFunctionCamBase
{
public:
  /**
  Constructor */
  UFunctionCamGmk();
  //UFunctionCamGmk(UCamPool * cams, UImagePool * images);
  /**
  Destructor */
  virtual ~UFunctionCamGmk();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs. */
  virtual void createResources();
  /**
  Name of function
  The returned name is intended as informative to clients
  and should include a version number */
//  virtual const char * name();
  /**
  Returns list of handled functions */
//  virtual const char * commandList();
  /**
  Handle camera and image commands */
  virtual bool handleCommand(UServerInMsg * msg, void * pushObj);

protected:
  /**
  Function to do the guidemark analysis */
  bool handleGmkGetCommand(UServerInMsg * msg, void * pushObj);
  /**
  Sent a guidemark to a client */
  bool sendGmk(UServerInMsg * msg, UBarcode * gmk,
                                const char * devName, int dev);

protected:
  /**
  Calibration structure */
  UCalibrate * calib;
  /**
  Default image pool image to use as source */
  UVariable * varDefImg;
  /**
  Guidemark count in latest search */
  UVariable * varCount;
  /**
  Image serial used for last search */
  UVariable * varImgSerial;
  /**
  IDs of found guidemarks */
  UVariable * varIDs;
  /**
  Position of first guidemark in robot coordinates */
  UVariable * varGmkPose;
  /**
  Position of robot in first guidemark coordinates */
  UVariable * varRobPose;
  /**
  Position of robot in first guidemark coordinates */
  UVariable * varCamPose;
  /**
  Use filter for vertical  guidemarks */
  UVariable * varUseVert;
  /**
  Use filter for diagonal guidemarks */
  UVariable * varUseDiag;
};


#endif
