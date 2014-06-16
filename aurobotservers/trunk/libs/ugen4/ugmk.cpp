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
#include "ugmk.h"
#include <ugen4/crc.h>

UGmk::UGmk()
        : UPosRot()
{
  clear();
  code[0] = '\0';
  codeInt = 0;
}

/////////////////////////////////////////////////////

UGmk::~UGmk()
{}

/////////////////////////////////////////////////////

void UGmk::clear()
{
  valid = false;
  validPos = false;
  crcOK = false;
}

/////////////////////////////////////////////////////

void UGmk::set(const char * newCode)
{
  strncpy(code, newCode, MAX_CODE_LENGTH);
  codeInt = getCodeInt(code, &crcOK);
  valid = true;
}

/////////////////////////////////////////////////////

// void UGmk::setTime(UTime atTime)
// {
//   time = atTime;
// }

///////////////////////////////////////////////////////

unsigned long UGmk::getCodeInt(const char * newCode, bool * okCrc)
{
  int i;
  unsigned long val = 0;
  const int MBC = 16;
  unsigned char block8[MBC];
  int n = mini(32, strlen(newCode));
  int cw; // code field width
  int blockCnt; // number of code blocks (4-bit) including index
  int block8cnt; // number of 8-bit codes
  // find code width 2,3,4 or 5 4-bit blocks
  for (cw = 2; cw < 4; cw++)
    if (cw * cw >= n)
      break;
  blockCnt = cw * cw;
  block8cnt = mini((blockCnt + 1) / 2, MBC);
  //
  for (i = 0; i < block8cnt; i++)
    block8[i] = 0;
  i = block8cnt - 1;
  for (int j = n - 1; j >= 0; j--)
  {
    unsigned int v = hex2int('0', newCode[j]);
    if (j % 2 == 0)
      block8[i] = v;
    else
      block8[i--] += v << 4;
    val += v << ((n - j - 1) * 4);
  }
  n = getCRC(block8, block8cnt);
  if (okCrc != NULL)
    *okCrc = (n == 0);
  if (n == 0)
    val = val >> 8;
  return val;
}

/////////////////////////////////////////////////////

void UGmk::set(UPosition pos, URotation rot)
{
  setPos(pos);
  setRot(rot);
  validPos = true;
}


