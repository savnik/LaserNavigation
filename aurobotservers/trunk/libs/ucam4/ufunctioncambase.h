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
#ifndef UFUNCTIONCAMBASE_H
#define UFUNCTIONCAMBASE_H

#include <urob4/ufunctionbase.h>
#include <urob4/ufunctionimgbase.h>

#include "ucampool.h"

/**
Base function with knowledge of camera and image pool structures.

@author Christian Andersen
*/
class UFunctionCamBase : public UFunctionImgBase
{
public:
  /**
  Constructor */
  UFunctionCamBase();
  /**
  Destructor */
  virtual ~UFunctionCamBase();
  /**
  Create any resources that this modules needs and add these to the resource pool
  to be integrated in the global variable pool and method sharing. */
  virtual void createResources();
  /**
  Space separated list of resources provided by this function */
  //virtual const char * resourceList();
  /**
  Get a shared resource */
  //virtual UResBase * getResource(const char * resID);
  /**
  Set shared resource */
  virtual bool setResource(UResBase * ressource, bool remove);
  /**
  Function tests if all shared resources are loaded
  for full functionality */
  //virtual bool gotAllResources(char * missingThese, int missingTheseCnt);

protected:
  /**
  Get a camera and an image reference from this set of
  parameters.
  The camera is returned in 'cam'. \n
  The image (raw only) is returned in 'img', if
  img is NULL, then imagepool img 0 is allocated. <br>
  The image device (often corresponding to /dev/videoX number) should be
  set in 'imgDevice' if unknown, then set to -1 on call. <br>
  'imgDevice' is set on exit if -1 before call. <br>
  'imgRemRad' should be set to as appropiate, but is valid if
  image is captured from camera directly, otherwise the
  value is set from image on return. \n
  the 'imgBase' pointer may point to a (pushed) image or
  may be NULL. If null, then an image is taken from camera. \n
  The camera may be selected from a position name 'posName' as
  an alternative to a device number.
  \param rectfiedImg is a number in image-pool where the rectified image are placed
  if the number is not a valid image pool number (i.e. -1) then no rectified image is produced.
    It further requires that the rectification parameters are set.
  \Returns true if an image is available. NB! a camera may not
  be available, if the device from image-descriptor is invalid,
  and in such cases the 'cam' will be returned as NULL. */
  bool getCamAndRawImage(
            UCamPush ** cam,      // result camera
            UImage ** imgRaw,  // result image
            int * imgDevice,     // camera device number
            //bool * imgRemRad,    // should/is radial error remoced
            void * imgBase,      // pushed image (YUV)
            const char * posName,      // camera position name
            int rectfiedImg  // destination imagepool number for rectified image
            );
  /**
  Get camera pointer form either a device number or a
  device position name.\n
  If 'imgDevice' is >=0 then this value is used to get a
  device. if not, \n
  then if 'posName' has a length > 0 the camera is found from
  this name,\n
  if imgDevice < 0 and posName is empty, then
  device 0 is returned (and may be invalid (NULL).\n
  Returns the requested camera or NULL. */
  UCamPush * getCam(int imgDevice,     /* camera device number */
                    const char * posName);      /* camera position name */
  /**
  Get the device number of the default (first) camera device
  \returns number of the default device */
  int getDefaultCamDevice();
  /**
  Get camera device pointer.
  \param camDevNum is the camera device number
  \returns NULL if device do'nt exist or a pointer to the requested camera device */
  UCamPush * getCam(int camDevNum);

protected:
  /**
  Structure holding available camera objects */
  UCamPool * camPool;
private:
  /**
  Locally created cam pool (i.e. not created by others) */
  bool camPoolLocal;
};

#endif
