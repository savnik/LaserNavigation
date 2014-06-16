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

#ifndef UBARCODE_H
#define UBARCODE_H

#include <ugen4/u3d.h>
//#include "urob4/urobcamdef.h"
#include <ugen4/ugmk.h>

/**Data related to detection of a barcode image.
  *@author Christian Andersen
  */

class UBarcode : public UGmk
{
public:
  /**
  Constructor */
  UBarcode();
  /**
  Destructor */
  ~UBarcode();
  /**
  Clear (invalidate the data) and set code number. */
  void clear(const int n);
  /**
  Set most data related to barcode */
  int setData(
          int codeHex[], // code to send
          int codeHexCnt, // code length - hex numbers
          float approxDist, // approximate distance
          int codeCount,    // serial number for codes in this image
          bool validPosRot, // pos and rot is valid
          UPosition pos, // distance from robot
          URotation rot, // orientation of barcode
          //unsigned long intCode, // first 8 half bytes
          UTime positionTime  // image time
          );

public:
  /**
  Validity of code */
  //bool valid;
  /**
  Approximate distance to barcode
  estimated from frame height only.
  This estimate is valid regardless of position and
  orientation is estimated.
  Could save the estimation time if this distance is
  sufficient. */
  float appDist;
  /**
  Code number in image. There may be up to 32 (parameter)
  codes detected in one image. */
  int codeNumber;
  /**
  Number of codes found in this image. */
  int codeNumberCount;
  /**
  Barcode found in image. One character is used to
  one hexidecimal number. <br>
  I.e. if code is 326b30 then <br>
  code[0] == 3, code[1] = 2, code[2] = 6, code[3] = 11 (b hex) ... etc. */
  //int codeLong[MAX_BARCODE_LENGTH];
  /**
  Number of numbers in code array. */
  //int codeLongCnt;
  /**
  Integer (long (32 bit)) version of code. Should be shown in hex format to
  match the code printed on the barchart */
  //unsigned long codeInt; // first 8 codes packed to a 4 byte int
  /**
  Estimated position */
  //UPosition relPos;
  /**
  Estimated rotation of barcode */
  //URotation relRot;
  /**
  Time (imagetime) for barcode position */
  //UTime relPosTime;
  /**
  Validity of position information. true (1) if valid. */
  //bool relPosValid;
};

#endif

