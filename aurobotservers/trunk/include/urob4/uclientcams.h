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
#ifndef UCLIENTCAMS_H
#define UCLIENTCAMS_H

#include <ugen4/ulock.h>
#include <ugen4/ucampar.h>

#include "usmltag.h"

/**
Number of camera structures that can be
saved in the client data area */
#define MAX_CLIENT_CAMS 20

/**
Class to store requested camera data
received from a camera server. */
class UClientCamData : public UCamPar, public ULock
{
  public:
  /**
  Constructor */
  UClientCamData();
  /**
  Destructor */
  ~UClientCamData();
  /**
  Decode received data */
  bool handleCamGet(USmlTag * tag);
  /**
  Print status to string buffer */
  char * snprint(const char * preString, char * buff, const int buffCnt);

  public:
  /**
  Camera device number */
  int device;
  /**
  Camra position */
  UPosition pos;
  /**
  Update (client real time) */
  UTime updTime;
  /**
  Camera rotation */
  URotation rot;
  /**
  Image size from camera */
  int  width;
  /**
  Image width from camera */
  int height;
  /**
  Focal length of camera */
  //double focalLength;
  /**
  Frames per second */
  double fps;
  /**
  Max size of camera name */
  static const int MAX_NAME_LENGTH = 33;
  /**
  Copy of the camera name */
  char name[MAX_NAME_LENGTH];
};


/**
Holds client information on camera status - like position and image size etc.

@author Christian Andersen
*/
class UClientCams
{
public:
  /**
  Constructor */
  UClientCams();
  /**
  Destructor */
  ~UClientCams();
  /**
  Get data structure with camera data.
  Returns NULL if device is not found (and not 'mayCreate) or
  if no more space for new devices. */
  UClientCamData * getCamData(int device, bool mayCreate);
  /**
  Currently available camera device count */
  inline int getCamsCnt()
  { return camsCnt; };
  /**
  Get cam mera device info with this index.
  NB! index range is not tested */
  inline UClientCamData * getCam(int idx)
  { return cams[idx]; };
  /**
  Get maximum number of camera devices */
  inline int getCamsCntMax()
  { return MAX_CLIENT_CAMS; };

protected:
  /**
  Camera data */
  UClientCamData * cams[MAX_CLIENT_CAMS];
  /**
  Number of camera structures used */
  int camsCnt;
};

#endif
