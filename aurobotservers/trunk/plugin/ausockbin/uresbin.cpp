/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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

#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>

#include <urob4/uvarcalc.h>
#include <ugen4/ucommon.h>
#include <urob4/usmltag.h>

#include "uresbin.h"

///////////////////////////////////////////


void UResBin::UResBinInit()
{
  setLogName("bintest");
  openLog();
  // create status variables
  createBaseVar();
  verbose = false;
}

///////////////////////////////////////////

UResBin::~UResBin()
{
}

///////////////////////////////////////////

void UResBin::createBaseVar()
{
  getVarPool()->setDescription("binary data handler", false);
  varBinCnt = addVar("binCnt",  0.0, "d", "(r) number of established poly items");
  varUpdTime = addVar("updateTime", 0.0, "d", "(r) time of last polygon update");
}

//////////////////////////////////////////////////////////////

void UResBin::handleNewData(USmlTag * tag)
{ // handle poly message
  const char * buf = tag->getTagStart();
  int bufCnt = tag->getTagCnt();
  //
  varUpdTime->setTimeNow();
  varBinCnt->add(1);
  printf("Got %d chars, first is %c%c%c\n", bufCnt, buf[0], buf[1], buf[2]);
  // log to file (bintest.log)
  toLog("bintest", bufCnt, buf);
  fprintf(getF(), "Got %d chars, first is %c%c%c\n", bufCnt, buf[0], buf[1], buf[2]);
}


