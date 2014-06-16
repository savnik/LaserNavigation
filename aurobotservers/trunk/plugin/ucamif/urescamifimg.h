/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
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

#ifndef URES_CAM_IF_IMG_H
#define URES_CAM_IF_IMG_H

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/uclientfuncimage.h>
#include <urob4/uimagepool.h>

/**
* A guidemark data structure holder for (especially) the camera client plugin.
* The data is extracted to a var-pool structure.
*/
class UResCamIfImg : public UClientFuncImage
{
public:
  /**
  Constructor */
  UResCamIfImg()
  { // set name and version
    setResID(getResClassID(), 200);
    verboseMessages = true;
    // pointer to external resources
    imgPool = NULL;
    // create space for 10 variables, 5 structures and 10 functions
    createVarSpace(10, 0, 0, "Camera server interface image copy handling", false);
    createBaseVar();
    tick = 0;
  };
  /**
  Destructor */
  virtual ~UResCamIfImg();
  /**
  Create base variables in varPool for this interface type */
  void createBaseVar();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "camImg"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 182; };*/
  /**
  Print status for this resource */
  virtual const char * snprint(const char * preString, char * buff, int buffCnt);
  /**
  Print status for this resource */
  inline virtual const char * print(const char * preString, char * buff, int buffCnt)
  { return snprint(preString, buff, buffCnt); };
  /**
  Is this resource missing any base ressources */
  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Set ressource as needed (probably not used by this resource) */
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  debug */
  void doTimeTick();

protected:
  /**
  Called when a new image 'img'
  (in openCv format) is available.
  The tag reference is include to be able to request more data from the same connection */
  virtual void gotNewImage(UImage * img, int poolNum, USmlTag * tag);
  /**
  Get image buffer from image pool.
  Get a image buffer area for the image about to be received - of at least this size */
  virtual UImage * getImageBuffer(int poolNumber, int height, int width, int channels, int depth);

protected:
  /**
  Pointer to image pool resource */
  UImagePool * imgPool;
  /**
  Index to variable with last image time */
  UVariable * varImgTime;
  /**
  Index to variable with destination for last image */
  UVariable * varImgPoolNum;
  /**
  Index to variable with number of images received */
  UVariable * varImgCnt;

private:
  int tick;

};



#endif

