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
#ifndef UIMAGEPOOL_H
#define UIMAGEPOOL_H

#include "ucmdexe.h"
#include "uimgpush.h"
#include "uresbase.h"

#define MAX_IMAGES_IN_POOL 100

class URawImage;
/**
 * Stores up to 100 images (UImage type) for general purpose.
 * Each image kan trigger events (push events), when the image user calls the update function for the image.
 * Each image is identified on the number, and thus the image number should be selected in a series that do not generate conflicts with other image producing plugins.
 * Reserved images are up to 30, that is used for camera device number (0-9 for USB camera and 10..19 for firewire cameras.
 * The class creates image space on demand (on heap) and releases on exit.

@author Christian Andersen
*/
class UImagePool : public UResVarPool
{
public:
  /**
  constructor */
  UImagePool()
  { // set name and version number
    setResID(getResClassID(), 590);
    UImagePoolInit();
  };
  /**
  Destructor */
  virtual ~UImagePool();
  /**
   * Initialization of image pool class */
  void UImagePoolInit();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "imgPool"; };
  /**
  Get image and create if needed.
  \param source is image number in the image pool
  \param mayCreate if this is false, and the image is not created yet, then a NULL pointer is returned.
  \param height (optional) number of rows in the image (only set if mayCreate is true)
  \param width (optional) number of columns in the image (only set if mayCreate is true)
  \param channels (optional) may to either 1 (BW), 3 (e.g. RGB) or 4 (eg RGBA) (only set if mayCreate is true)
  \param depth (optional - default is 8) is number of bytes for each channel in each pixel (may be 8,16,24 or 32) (set only if mayCreate is true)
  \returns a pointer to the image in the image pool (or NULL if the image do not exist and mayCreate is false) */
  virtual UImage * getImage(unsigned int source, bool mayCreate,
                   const unsigned int height = 0, const unsigned int width = 0,
                   const int channels = 3, const int depth = 8);
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs. */
  virtual void createResources();
  /**
  Number of created images */
  int createdImages();
  /**
  Free images to release heap space */
  void release();
  /**
  Get number of used handles */
  inline int getImageCnt()
    { return imagesCnt; } ;
  /**
  Set command executor for use when a new image is created
  then this image will be assigned this command executor. */
  void setCmdExe(UCmdExe * server)
  { cmdexe = server; }; 
  /**
  Set resource (core pointer needed by push functionality) */
  bool setResource(UResBase * resource, bool remove);
  /**
  Get maximum imagecount in image pool */
  inline int getMaxImageCnt()
  { return MAX_IMAGES_IN_POOL;};
  /**
  Print short list of available images to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Test if image pool has all needed resources. */
  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /** does this image exist
   * \param idx is index to image pool, must be less than MAX_IMAGES_IN_POOL.
   * \returns true if image is created. */
  inline bool isCreated(unsigned int idx)
  {
    if (idx < MAX_IMAGES_IN_POOL)
      return images[idx] != NULL;
    else
      return false;
  }
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();
  /**
   * Show images as default - intended for use by client only */
  void showImagesAsDefault(bool show)
  {
    if (varShowDefault != NULL)
      varShowDefault->setBool(show);
  }

protected:
  /**
  Pool  of image pointers */
  UImgPush * images[MAX_IMAGES_IN_POOL];
  /**
  max number of image created with highest number until now */
  int imagesCnt;
  /**
  Command executor for poll push */
  UCmdExe * cmdexe;
private:
  /// variable in image pool
  UVariable * varImgCnt;
  /// list of images available
  UVariable * varImgList;
  /// should images in image-pool be shown as default.
  UVariable * varShowDefault;
};

#endif
