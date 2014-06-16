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

#include <stdio.h>
#include <string.h>

#include "urespool.h"
#include "ucmdexe.h"

UResPool::UResPool()
{
  resCnt = 0;
}

///////////////////////////////////////

UResPool::~UResPool()
{
}

///////////////////////////////////////

bool UResPool::addResource(UResBase * resource)
{
  int i = 0;
  bool result = (resource != NULL);
  //
  if (result)
  {
    for (i = 0; i < resCnt; i++)
    {
      if (res[i] == NULL)
        break;
    }
    if (i < MAX_RESOURCE_POINTERS)
    {
      res[i] = resource;
      if (i >= resCnt)
        resCnt = i + 1;
    }
    else
      result = false;
  }
  return result;
}

///////////////////////////////////////

void UResPool::saveSettings()
{
  UResBase * r;
  int i;
  //
  for (i = 0; i < resCnt; i++)
  {
    r = res[i];
    if (r != NULL)
      r->saveSettings();
  }
}

///////////////////////////////////////

void UResPool::stop(bool andWait)
{
  UResBase * r;
  int i;
  //
  for (i = 0; i < resCnt; i++)
  {
    r = res[i];
    if (r != NULL)
    {
      r->stop(andWait);
      Wait(0.1);
    }
  }
}

///////////////////////////////////////

UResBase * UResPool::removeResourceFunc(int funcIndex)
{
  UResBase * result = NULL;
  UResBase * r;
  int i;
  //
  for (i = 0; i < resCnt; i++)
  {
    r = res[i];
    if (r != NULL)
    {
      if (r->getResFuncIdx() == funcIndex)
      { // get pointer to removed resource
        result = res[i];
        res[i] = NULL;
        break;
      }
    }
  }
  return result;
}

///////////////////////////////////////

UResBase * UResPool::removeResourceIndex(int resIndex)
{
  UResBase * result = NULL;
  //
  if ((resIndex >= 0) and (resIndex < resCnt))
  {
    if (res[resIndex] != NULL)
    {
      result = res[resIndex];
      res[resIndex] = NULL;
    }
  }
  return result;
}

///////////////////////////////////////

UResBase * UResPool::getResource(int resIndex)
{
  UResBase * result = NULL;
  //
  if ((resIndex >= 0) and (resIndex < resCnt))
  {
    result = res[resIndex];
  }
  return result;
}

///////////////////////////////////////

UResBase * UResPool::getResource(const char * resID)
{
  UResBase * result = NULL;
  UResBase * r;
  int i;
  //
  for (i = resCnt - 1; i >= 0; i--)
  {
    r = res[i];
    if (r != NULL)
      if (r->isA(resID))
      { // this is the requested resource
        result = r;
        break;
      } 
  }
  //
  return result;
}

///////////////////////////////////////

int UResPool::getResFunc(int resIndex)
{
  int result = -1;
  UResBase * r;
  //
  if ((resIndex >= 0) and (resIndex < resCnt))
  {
    r = res[resIndex];
    if (r != NULL)
      result = r->getResFuncIdx();
  }
  return result;
}

///////////////////////////////////////

void UResPool::print(const char * preString)
{
  const int MSL = 4000;
  char s[MSL];
  //
  print(preString, s, MSL);
  printf("%s", s);
}

///////////////////////////////////////////////////

void UResPool::print(const char * preString,
                      char * buff, int buffCnt)
{
  int i, m;
  UResBase * r;
  const int MSL = 40;
  char s[MSL];
  char * p1;
  //
/*  for (i = 0; i < resCnt; i++)
  {
    if (res[i] != NULL)
      n++;
  }*/
  if (buff != NULL)
  {
//    if (n > 1)
    {
      m = 0;
      p1 = buff;
      p1[0] = '\0';
/*      if (strlen(p1) > 0)
      {
        if (n > 1)
          snprintf(p1, buffCnt - m, "%s - %d resources loaded\n", preString, n);
        else
          snprintf(p1, buffCnt - m, "%s - %d resource  loaded\n", preString, n);
      }*/
      for (i = 0; i < resCnt; i++)
      {
        m += strlen(p1);
        p1 = &buff[m];
        if (m >= buffCnt)
          break;
        r = res[i];
        if (r != NULL)
        {
          snprintf(p1, buffCnt - m, "Resource %d is %s version %d\n",
                   i, r->getResID(), r->getResVersion());
          m += strlen(p1);
          p1 = &buff[m];
          if (r->isA(UCmdExe::getResClassID()))
            snprintf(p1, buffCnt - m, " - server core resource\n");
          else
          { // let the resource print for itself
            r->print(" - ", p1, buffCnt - m);
            if (*p1 == '\0')
            { // ressource has no print functions - use name
              snprintf(p1, buffCnt - m, "%s (no status)\n", s);
            }
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////

bool UResPool::handleReplay()
{
  bool result = false;
  UResBase * ra, *rs;
  int i;
  //
  for (i = 0; i < resCnt; i++)
  {
    ra = res[i];
    if (ra != NULL)
    {
      result = (ra->replayTimeAdvancePending);
      if (result)
        break;
    }
  }
  if (result)
  { //update other resources to this replay time to
    for (i = 0; i < resCnt; i++)
    { //
      rs = res[i];
      if (rs != NULL and ra != rs)
      { // resource exist and is not the source replay master
        if (rs->isReplay())
          // update
          rs->replayToTime(ra->replayTimeNow);
      }
    }
    ra->replayTimeAdvancePending = false;
  }
  //
  return result;
}

