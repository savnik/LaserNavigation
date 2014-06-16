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

#include <string>

#include <umap4/upose.h>
#include <urob4/uvarpool.h>

#include "umission.h"

UMisBase::UMisBase()
{
  calc = NULL;
}

//////////////////////////////////

UMisBase::~UMisBase()
{
}

//////////////////////////////////

void UMisBase::setCalc(UCalc * newCalculator)
{
  calc = newCalculator;
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisLine::UMisLine()
{
  type = EL_UNKNOWN;
}

//////////////////////////////////

UMisLine::~UMisLine()
{
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisSegment::UMisSegment()
{
  linesCnt = 0;
  name[0] = '\0';
}

//////////////////////////////////

UMisSegment::~UMisSegment()
{
}

//////////////////////////////////
  
bool UMisSegment::addLine(UMisLine * newLine)
{
  bool result;
  //
  result = linesCnt < MAX_LINE_CNT_IN_SEGMENT;
  if (result)
  {
    if (lines[linesCnt] != NULL)
      delete lines[linesCnt];
    newLine->setCalc(this);
    lines[linesCnt++] = newLine;
  }
  return result;
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////


UMission::UMission()
{
}

//////////////////////////////////

UMission::~UMission()
{
}

//////////////////////////////////
