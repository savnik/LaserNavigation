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
#ifndef UFUNCTIONCAM_H
#define UFUNCTIONCAM_H

#include "ufunctioncambase.h"
#include "ucomcam.h"

/**
Functions that control the cameras

@author Christian Andersen
*/
class UFunctionCam : public UFunctionCamBase
{
public:
  /**
  Constructor */
  UFunctionCam();
  /**
  Constructor */
  //UFunctionCam(UCamPool * cams, UImagePool * images);
  /**
  Destructor */
  virtual ~UFunctionCam();
  /**
  Name of function
  The returned name is intended as informative to clients
  and should include a version number */
//  virtual const char * name();
  /**
  Returns list of handled functions.
  The following functions are handled:
  <h2\>camGet:</h2\>\n
  Return specified parameter values. e.g.\n
  \<camGet camera=1 name size fps/>\n
  The allowed attributes are described below\n
  <h3>Attributes</h3>
  name: Return the name of the camera\n
  posName: Return the position name of the camera (e.g. cameraLeft)\n
  pos, posX, posY, posZ: Return the position of the camera relative to the robot.\n
  rot, rotOmega, rotPhi, rotKappa: Returns the orientation of the camera relative to the robot\n
  <h3>This is too complicated - I will find another way of documenting attributes</h3>
         */
//  virtual const char * commandList();
  /**
  Handle camera and image commands */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

private:
  /**
  Handle commands, that change settings of the camera */
  bool handleCamSetCommand(UServerInMsg * msg);
  /**
  Handle commands that returns actual settings */
  bool handleCamGetCommand(UServerInMsg * msg);
  /**
  Handle commands that returns actual settings */
  bool handleCamPushCommand(UServerInMsg * msg);
  /**
  Get list of video devices */
  bool handleCamsGetCommand(UServerInMsg * msg);
  /**
  Send status reply to the message client
  for each of the requested areas in the 'cmaValues'
  from the 'cam' camera.
  Returns true if send. */
  bool sendStatusReply(UServerInMsg * msg,
                       UComCamSml * camValues,
                       UCamMounted * cam);

};

#endif
