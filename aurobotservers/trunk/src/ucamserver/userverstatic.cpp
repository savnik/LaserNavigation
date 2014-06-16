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
#include "userverstatic.h"
 
#include <ucam4/ufunctioncam.h>
#include <ucam4/ufunctionimage.h>
#include <urob4/ufunctionimgpool.h>
#include <urob4/ufunctionvarpool.h>
#include <urob4/ufunctionif.h>

#include "ufunctioncamgmk.h"
#include "ufunctioncampath.h" 

//////////////////////////////////////////////

UServerStatic::UServerStatic()
 : UCmdExe()
{
  printf("\n");
}

//////////////////////////////////////////////

UServerStatic::~UServerStatic()
{
}


//////////////////////////////////////////////

bool UServerStatic::getStaticHelpList(char * list, const int listCnt)
{ // Get help text for available static modules
  //bool result; 
  char * p1 = list;
  int n = 0;
  //
  UCmdExe::getStaticHelpList(list, listCnt);
  n += strlen(p1);
  p1 = &list[n];
  //
  snprintf(p1, listCnt - n, "load='camControl'  Load camera pool functions\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='imagePool'   Load image pool functions\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='image'       Load image snapshot functions\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='path'        Load vision based road finder functions\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='gmk'         Load guidemark functions\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='var'         Load variable pool interface\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='if' alias='aa'  Load an interface named 'aa' to another server\n");
  //
  return true;
}

//////////////////////////////////////////////

bool UServerStatic::loadStaticModule(const char * moduleName, const char * aliasName, char * why, const int whyCnt)
{ // load available static module of this name
  bool result = true;
  // "static" function pointers
  UFunctionCamGmk * fgmk;
  UFunctionCamPath * fpath;
  UFunctionCam * fcam;
  UFunctionImgPool * fpool;
  UFunctionImage * fimage;
  UFunctionVarPool * fvarpool;
  UFunctionIf * fif;
  //
  if (strcasecmp(moduleName, "camControl") == 0)
  {
    fcam = new UFunctionCam();
    if (fcam != NULL)
      addFunction(fcam);
  }
  else if (strcasecmp(moduleName, "imagePool") == 0)
  {
    fpool = new UFunctionImgPool();
    if (fpool != NULL)
      addFunction(fpool);
  }
  else if (strcasecmp(moduleName, "image") == 0)
  {
    fimage = new UFunctionImage();
    if (fimage != NULL)
      addFunction(fimage);
  }
  else if (strcasecmp(moduleName, "path") == 0)
  {
    fpath = new UFunctionCamPath();
    if (fpath != NULL)
      addFunction(fpath);
  }
  else if (strcasecmp(moduleName, "gmk") == 0)
  {
    fgmk = new UFunctionCamGmk();
    if (fgmk != NULL)
      addFunction(fgmk);
  }
  else if (strcasecmp(moduleName, "var") == 0)
  {
    fvarpool = new UFunctionVarPool();
    if (fvarpool != NULL)
      addFunction(fvarpool);
  }
  else if (strcasecmp(moduleName, "if") == 0)
  {
    fif = new UFunctionIf();
    if (fif != NULL)
    {
      if (strlen(aliasName) > 0)
        fif->setAliasName(aliasName);
      addFunction(fif);
    }
  }
  else
    // ask the base module if it knows the requested module
    result = UCmdExe::loadStaticModule(moduleName, aliasName, why, whyCnt);
  //
  return result;
}
