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
#ifndef UFUNCTIONIMAGE_H
#define UFUNCTIONIMAGE_H

#include "ufunctioncambase.h"

/**
Function that can handle images and part of images

@author Christian Andersen
*/
class UFunctionImage : public UFunctionCamBase
{
public:
  /**
  Constructor */ 
  UFunctionImage();
  /**
  Constructor */
  //UFunctionImage(UCamPool * cams, UImagePool * images);
  /**
  destructor */
  ~UFunctionImage();
  /**
  Name and version number of function */
//  const char * name();
  /**
  Commands handled by this function */
//  const char * commandList();
  /**
  Handle command */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

protected:
  /**
  Deal with image get functions */
  bool handleImageGetCommand(UServerInMsg * msg, void * imgBase);
  /**
  List available images */
  //bool handleImageListCommand(UServerInMsg * msg);
};

#endif
