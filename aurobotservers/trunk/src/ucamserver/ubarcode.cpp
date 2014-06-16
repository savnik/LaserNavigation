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

#include "ubarcode.h"
#include <ugen4/crc.h>

UBarcode::UBarcode(){
}
UBarcode::~UBarcode(){
}

void UBarcode::clear(const int n)
{
  codeNumberCount = n;
  UGmk::clear();
}

/////////////////////////////////

int UBarcode::setData(
          int codeHex[], // code to send
          int codeHexCnt, // code length
          float approxDist, // approximate distance
          int codeCount,     // number found in this image
          bool validPosRot, // pos and rot is valid
          UPosition gmkPos, // distance from robot
          URotation gmkRot, // orientation of barcode
          //unsigned long intCode, // first 8 half bytes
          UTime positionTime  // image time
          )
{
  char codeStr[MAX_CODE_LENGTH + 1];
  const char hex[17] = "0123456789abcdef";
  int i;
  //
  for (i = 0; i < mini(MAX_CODE_LENGTH, codeHexCnt); i++)
  {
    codeStr[i] = hex[codeHex[i]];
  }
  codeStr[i] = '\0';
  set(codeStr);
  appDist = approxDist;
  codeNumber = codeCount;
  setTime(positionTime);
  set(gmkPos, gmkRot);
  validPos = validPosRot;
  return codeNumberCount;
}

