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
#ifndef UCOMIMANA_H
#define UCOMIMANA_H


#include <ugen4/ucommon.h> // file name size etc
//#include "urobcamdef.h"    // message number

/**
Communication of image analysis request and state of present analysis

@author Christian Andersen
*/
class UComImAna{
public:
    UComImAna();

    ~UComImAna();

public:
  /**
  Request analysis valid.
  The request is for mode in 'aReqMode' */
  bool aReqValid;
  /**
  Request analysis state valid.
  The state is for the analysis mode defined in
  'aReqMode' */
  bool aReqState;
  /**
  Analysis mode or type */
  unsigned int aReqMode;
  /**
  Parameters for the requested mode.
  Count of valid parameters. */
  unsigned int aParamCnt;
  /**
  Maximum number of params */
  static const int MAX_PARAMS = 4;
  /**
  Parameter values - each is 16 bit signed values */
  int aParam[MAX_PARAMS];
  /**
  Image source valid */
  bool aImgSrcValid;
  /**
  Image source type, one of
  0 : new camera image(s).
  1 : Load from file
  2 : Load from file-list - filenames in listfile */
  unsigned int aImgSrc;
  /**
  Number of devices to be used */
  unsigned int aDevCnt;
  /**
  Maximum number of devices */
  static const int MAX_DEVICES = 5;
  /**
  Device number list */
  unsigned int aDevList[MAX_DEVICES];
  /**
  Number of filenames */
  unsigned int aFileNameCount;
  /**
  Maximum number of filenames. */
  static const int MAX_FILENAMES = 10;
  /**
  Filenames for images or image liste.
  This may be one for each device.
  Filenames are relative to default image path and
  includes filetype (e.g. ".png") */
  char aFileNames[MAX_FILENAMES][MAX_IMG_NAME_SIZE];
  ////////////////
  ////////////////
  // Status values
  //
  // not implemented
};

#endif
