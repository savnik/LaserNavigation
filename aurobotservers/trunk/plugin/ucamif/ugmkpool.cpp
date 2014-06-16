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


#include "ugmkpool.h"


UGmkPool::UGmkPool()
{
  int i;
  //
  for (i = 0; i < MAX_GUIDEMARK_COUNT; i++)
    gmks[i] = NULL;
  gmksCnt = 0;
}


UGmkPool::~UGmkPool()
{
  int i;
  for (i = 0; i < gmksCnt; i++)
    if (gmks[i] != NULL)
      delete gmks[i];
}

////////////////////////////////////////////////

UGmk * UGmkPool::getGmk(unsigned long shortCode, bool mayCreate)
{
  UGmk ** result = gmks;
  int i;
  //
  for (i = 0; i < (gmksCnt); i++)
  {
    if (result != NULL)
      if ((*result)->getCodeInt() == shortCode)
        break;
    result++;
  }
  if ((*result == NULL) and mayCreate and (i >= (gmksCnt)))
  {
    *result = new UGmk();
    gmksCnt++;
  }
  return *result;
}

//////////////////////////////////////////////////

int UGmkPool::getGmkNewCnt(UTime sinceTime)
{
  UGmk ** gmk = gmks;
  int i;
  int result = 0;
  //
  // count number of guidemarks with a newer time
  for (i = 0; i < gmksCnt; i++)
  {
    if (((*gmk)->getTime() - sinceTime) > 0.0)
      result++;
    gmk++;
  }
  //
  return result;
}

//////////////////////////////////////////////////

int UGmkPool::getGmkCnt()
{
  return gmksCnt;
}

//////////////////////////////////////////////////

UGmk * UGmkPool::getGmkNew(int serial, UTime sinceTime)
{
  UGmk ** gmk = gmks;
  int i;
  int cnt = 0;
  //
  // count number of guidemarks with a newer time
  for (i = 0; i < gmksCnt; i++)
  {
    if (((*gmk)->getTime() - sinceTime) > 0.0)
      cnt++;
    if (cnt == serial)
      break;
    gmk++;
  }
  if (cnt == serial)
    return *gmk;
  else
    return NULL;
}

////////////////////////////////////////////////////

UGmk * UGmkPool::getGmkNum(unsigned int serial)
{
  UGmk * gmk = NULL;
  //
  if (int(serial) < gmksCnt)
    gmk = gmks[serial];
  //
  return gmk;
}

////////////////////////////////////////////////////
  
UGmk * UGmkPool::getGmkId(unsigned long ID)
{
  UGmk ** pgmk = gmks;
  UGmk * gmk = NULL;
  int i;
  //
  for (i = 0; i < gmksCnt; i++)
  {
    if ((*pgmk)->getCodeInt() == ID)
    {
      gmk = *pgmk;
      break;
    }
    pgmk++;
  }
  return gmk;
}

