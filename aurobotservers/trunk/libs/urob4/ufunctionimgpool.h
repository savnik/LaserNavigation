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
#ifndef UFUNCTIONIMGPOOL_H
#define UFUNCTIONIMGPOOL_H

#include "ufunctionimgbase.h"

/**
Function to retreive image pool images to a client.
The actual owner of the image pool is UFunctionImg, and thus
this function should just receive a pointer from a 'UFunctionImageBase'
that creates the image pool.
@author Christian Andersen
*/
class UFunctionImgPool : public UFunctionImgBase
{
public:
  /**
  Constructor */
  UFunctionImgPool()
  {
    setCommand("poolGet poolList poolSet poolPush", "fimgpool", "imagePool (" __DATE__ " jca@elektro.dtu.dk)");
  };
  /**
  Destructor */
  virtual ~UFunctionImgPool();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs. */
  virtual void createResources();
  /**
  Handle command */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

protected:
  /**
  Deal with image get functions */
  bool handleImageGetCommand(UServerInMsg * msg, void * imgBase);
  /**
  Deal wit poolset commands */
  bool handleImageSetCommand(UServerInMsg * msg, void * imgBase);
  /**
  List available images */
  bool handleImageListCommand(UServerInMsg * msg);
  /**
  Handle image pool push command */
  bool handlePoolPushCommand(UServerInMsg * msg);

};

#endif
