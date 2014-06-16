/***************************************************************************
 *   Copyright (C) 2004 by Christian Andersen                              *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef UMAPGMK_H
#define UMAPGMK_H

#include <ugen4/u3d.h>
#include "umapcommon.h"


/**
Class that holds additional information for a map object
regarding guidemark information. size, number etc. */
class UMapGmk
{
public:
  /**
  Constructor */
  UMapGmk();
  UMapGmk(const unsigned long ID, const float size, const int frame);
  /**
  Destructor */
  ~UMapGmk();
  /**
  Set guidemark info as a copy of the source data. */
  inline void copy(UMapGmk * source) { *this = *source;};
  /**
  Get code */
  inline unsigned long getID() {return code;};
  /**
  Get number of blocks each side of GMK frame. */
  inline int getSize() {return frameSize;};
  /**
  Get size of each block in GMK frame. */
  inline float getBlockSize() { return blockSize;};
  /**
  Set (or change) ID */
  inline void setID(const unsigned long ID){ code = ID;};
  /**
  Set block size (nformation only). */
  inline void setBlock(const float size) { blockSize = size;};
  /**
  Set GMK framesize */
  inline void setFrame(const int size) { frameSize = size;};
  /**
  Print guidemark information to console */
  void print();
  /**
  Save guidemark infor in html-like format.
  Returns true if saved. */
  bool save(Uxml3D * fxmap, const char * name = NULL);
  /**
  Load guidemark info - ex position - from a xml-class object.
  Returns true if loaded.
  Errors are reported to UxmlFile-class. */
  bool load(Uxml3D * fxmap, char * name = NULL);

private:
  /**
  Guidemark code (in short form (32 but integer)) */
  unsigned long code;
  /**
  Size of each frame block - im meter.
  e.g. 0.02 for 2 cm blocks */
  float blockSize;
  /**
  Number of blocks each side ofGMK */
  int frameSize;
};



/**
Guidemark measurement iformation */
class UMapZGmk
{
public:
  /**
  Constructor */
  UMapZGmk();
  /**
  Constructor with specific code plock size */
  UMapZGmk(float block, int frame);
  /**
  Destructor */
  ~UMapZGmk();
  /**
  Clear measurement and including ID and size */
  void clear();
  /**
  Set Guidemark code.
  Code 0 is invalid */
  void setID(const unsigned long ID);
  /**
  Get ID */
  inline unsigned long getID() { return code;};
  /**
  Set measurement position ad SD. */
  void setPosition(const UPosition * z, const UPosition * sd, const UTime t);
  /**
  Set measured rotation and rotation SD.
  If rotation is not set, then it is assumed unvalid */
  void setOrientation(const URotation * r, const URotation * sd);
  /**
  Get measured position relative to robot (x,y,z)R */
  inline UPosition getPos() {return zPos;};
  /**
  get measuret orientation of guidemark (relative to robot) (O,P,K)R */
  inline URotation getOri() {return zOri;};
  /**
  Get position measurement SD (in robot coordinates) (x,y,z)R */
  inline UPosition getPosSD() {return zPosSD;};
  /**
  Get orientation measurement SD (in robot coordinates) (O,P,K)R */
  inline URotation getOriSD() {return zOriSD;};
  /**
  Get guidemark code frame size (in blocks).*/
  inline int getFrame() { return frameSize;};
  /**
  Get guidemark block size (in meter). */
  inline float getBlock() { return blockSize;};
private:
  /**
  Guidemar (unique) ID */
  unsigned long code;
  /**
  Guidemark block size in meter */
  float blockSize;
  /**
  Guidemark frame size (in blocks) */
  int frameSize;
  /**
  Measurement position (3D) */
  UPosition zPos;
  /**
  Measurement rotation (3D) */
  URotation zOri;
  /**
  Is rotation measurement valid? */
  bool zOriValid;
  /**
  Position measurement SD error (i.e. v_k) */
  UPosition zPosSD;
  /**
  Rotation measurement SD error (i.e. v_k) */
  URotation zOriSD;
  /**
  Measurement time */
  UTime zT;
};


#endif
