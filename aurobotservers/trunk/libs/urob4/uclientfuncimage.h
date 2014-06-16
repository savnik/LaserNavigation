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
#ifndef UCLIENTFUNCIMAGE_H
#define UCLIENTFUNCIMAGE_H

#include <ugen4/uimage2.h>
//#include <ugen4/urawimage.h>
//#include <ugen4/ucammount.h>

#include "uresifbase.h"

/**
Cllient function to reconstruct images from camera server component

@author Christian Andersen
*/
class UClientFuncImage : public UResIfBase // UClientFuncBase
{
public:
  /**
  Constructor */
  UClientFuncImage();
  /**
  Destructor */
  virtual ~UClientFuncImage();
  /**
  Name of function
  The returned name is intended as informative to clients
  and should include a version number */
  virtual const char * name();
  /**
  Function, that shall return a string with all handled commands,
  i.e. should return "gmk gmk2d guidemark", if commands
  starting with any of these three keywords are handled
  by this function */
  virtual const char * commandList();
  /**
  Got fresh data destined to this function. */
  virtual void handleNewData(USmlTag * tag);
  /**
  The server has set (or changed) the namespace */
  virtual void changedNamespace(const char * newNamespace);

protected:
  /**
  Called when a new image in RGB format
  (openCv format) is available. */
  virtual void gotNewImage(UImage * img, int poolNum, USmlTag * tag);
  /**
  Get a image buffer area for the image about to be received - of at least this size */
  virtual UImage * getImageBuffer(int poolNumber, int height, int width, int channels, int depth);
  /**
  Receiced camera details for this device */
  virtual void gotNewCamInfo(int device, UPosRot pose, double focalLength, double k1, double k2, const char * name);

private:
  /**
  Decode images and imageGet replies.
  Returns true if unused data is available in buffer */
  void handleImages(USmlTag * tag);
  /**
  Decode received image list */
  void handleImageList(USmlTag * tag);

protected:
  /**
  Name of camera position */
  char posName[MAX_MOUNT_NAME_SIZE];

private:
  /**
    Image buffer for most recently received data.
    The image may contain incomplete data if
    accesses outsode the go */
    UImage * imgBuffer;
};

#endif
