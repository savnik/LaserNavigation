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
#ifndef UFUNCTIONIMGBASE_H
#define UFUNCTIONIMGBASE_H

#include "uimagepool.h"

#include "ufunctionbase.h"

/**
Image and image pool base functions
This class is a base class only, i.e. do not handle commands on its own.

@author Christian Andersen
*/
class UFunctionImgBase : public UFunctionBase
{
public:
  /**
  Constructor */
  UFunctionImgBase();
  /**
  Constructor */
  //UFunctionImgBase(UImagePool * images);
  /**
  Destrctor */
  virtual ~UFunctionImgBase();
  /**
  Set command handler (for sending messages to clients) */
  inline virtual void setCmdHandler(UCmdExe * cmdExe)
  { 
    cmdHandler = cmdExe;
    if (imgPool != NULL)
      imgPool->setCmdExe(cmdExe);
  };
  /**
  This function has a ressource that may be provided to others. */
//  const char * resourceList();
  /**
  Set resource pointer */
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  Test if all resources are loaded as intended */
//  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Get pointer to shared resource */
//  virtual UResBase * getResource(const char * ressID);
  /**
   * Show images as default - intended for use by client only */
  void showImagesAsDefault(bool show)
  {
     imgPool->showImagesAsDefault(show);
  }

protected:
  /**
  Send the image to the client in this message.
  Returns false if nothing send.
  \param silent if true and no imagedata is to be send, then no reply.
  Sends the image meta, as specified from the
  flags, and the binary  data if anything to send. */
  bool sendImage(UServerInMsg * msg, //int client,    // client info
               const char * tagName, // tagname to use in image pack <tagName x= ...
               /*URawImage * imgRaw,*/
               UImage * imgRgb,  // image and format
               int imgLeft = 0,     // left column to send
               int imgTop  = 0,     // top row to send
               int imgWidth = 640,  // max width
               int imgHeight = 480, // max height
               int imgSource = -1,          // image source number (imgPool)
               bool isCodecBin = true,      // binatry or hex-coded
               bool imgIsBW = false,         // send as BW-only
               bool imgGetNum = true,       // send image number
               bool imgGetTod = true,       // send time-of-day format
               bool imgGetTime = false,     // send hh:mm:ss.ccc time format
               const char * posName = NULL, // send this camera position name
               bool imgGetName = true,    // send image name (RGB format only)
               bool silent = false
               );

protected:
  /**
  Structure holding a pool of shared and reusable images */
  UImagePool * imgPool;

private:
  /**
  Locally created resource */
  //bool imgPoolLocal;
};

#endif
