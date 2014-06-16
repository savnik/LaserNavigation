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

//#include <ucam4/ufunctioncam.h>
#include <ucam4/ufunctionimage.h>
#include <ucam4/ufunctioncambase.h>
#include <urob4/ufunctionimgpool.h>
#include <urob4/ufunctionvarpool.h>
#include <ulms4/ufunctionlaser.h>
#include <urob4/ufunctionif.h>

#include "ufunctioncamif.h"
//#include "ufunctionsmrif.h"
//#include "ufunctionseq.h"
#include "ufunctionlaserif.h"
//#include "ufunctionavoid.h"
//#include "ufunctionroaddrive.h"
//#include "ufunctionexres.h"
//#include "ufunctionexuse.h"
#include "ufunctiondisp.h"
//#include <urob4/ufunctionif.h>
#include "ufunctionlaserifscan.h"
#include "ufuncpoly.h"
//#include "ufuncballfinder.h"

#include "userverstatic.h"

//////////////////////////////////////////////

UServerStatic::UServerStatic()
 : UCmdExe()
{
}

//////////////////////////////////////////////

UServerStatic::~UServerStatic()
{
}


//////////////////////////////////////////////

bool UServerStatic::getStaticHelpList(char * list, const int listCnt)
{ // Get help text for available static modules
  bool result;
  char * p1 = list;
  int n = 0;
  //
  result = UCmdExe::getStaticHelpList(list, listCnt);
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='display'     Load display module\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='camControl'  Load camera pool functions\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='imagePool'   Load image pool functions\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='image'       Load image snapshot functions\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='camData'     Load camera interface data handler (img, cam, gmk and path)\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='var'         Load variable pool interface\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='if' alias='yyy'  Load interface to a server with the interface name 'yyy'\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='laserData'   Load laser data handler (road, obst and scan--features)\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='laserscan'     Load laser scan data interface module\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='poly'          Load poly-line module\n");
  n += strlen(p1);
  p1 = &list[n];
  //
  return result;
}

//////////////////////////////////////////////

bool UServerStatic::loadStaticModule(const char * moduleName, const char * aliasName, char * why, const int whyCnt)
{ // load available static module of this name
  bool result = true;
  // "static" function pointers
  //UFunctionCam * fcam;
  UFunctionImgPool * fpool;
  //UFunctionImage * fimage;
  UFunctionCamData * fcamifd;
  UFunctionVarPool * fvarpool;
  UFunctionLaserIfData * flifd;
  UFunctionLaserIfScan * flifscan;
  UFunctionDisp * fdisp;
//  UFunctionNavIf * fnif;
  UFunctionIf * fif;
  UFunctionBase * func;
  //
/*  if (strcasecmp(moduleName, "ballfinder") == 0)
  {
    func = new UFuncBallFinder();
    if (func != NULL)
      result = addFunction(func);
  }
  else*/
  if (strcasecmp(moduleName, "imagePool") == 0)
  {
    fpool = new UFunctionImgPool();
    if (fpool != NULL)
      result = addFunction(fpool);
  }
/*  else if (strcasecmp(moduleName, "image") == 0)
  {
    fimage = new UFunctionImage();
    if (fimage != NULL)
      result = addFunction(fimage);
  }*/
  else if (strcasecmp(moduleName, "camData") == 0)
  {
    fcamifd = new UFunctionCamData();
    if (fcamifd != NULL)
      result = addFunction(fcamifd);
  }
  else if (strcasecmp(moduleName, "var") == 0)
  {
    fvarpool = new UFunctionVarPool();
    if (fvarpool != NULL)
      result = addFunction(fvarpool);
  }
/*  else if (strcasecmp(moduleName, "smr") == 0)
  {
    fsmr = new UFunctionSmrIf();
    if (fsmr != NULL)
      addFunction(fsmr);
  }*/
  else if (strcasecmp(moduleName, "laserData") == 0)
  {
    flifd = new UFunctionLaserIfData();
    if (flifd != NULL)
      result = addFunction(flifd);
  }
  else if (strcasecmp(moduleName, "laserscan") == 0)
  {
    flifscan = new UFunctionLaserIfScan();
    if (flifscan != NULL)
      result = addFunction(flifscan);
  }
  else if (strcasecmp(moduleName, "display") == 0)
  {
    fdisp = new UFunctionDisp();
    if (fdisp != NULL)
      result = addFunction(fdisp);
  }
  else if (strcasecmp(moduleName, "if") == 0)
  {
    fif = new UFunctionIf();
    if (fif != NULL)
    {
      if (aliasName != NULL)
        if (strlen(aliasName) > 0)
          fif->setAliasName(aliasName);
      result = addFunction(fif); 
    }
    if (not result and why != NULL)
      snprintf(why, whyCnt, "Interface module with alias name '%s' can not be loaded\n", aliasName);
  }
/*  else if (strcasecmp(moduleName, "navif") == 0)
  {
    fnif = new UFunctionNavIf();
    if (fnif != NULL)
    {
      if (aliasName != NULL)
        if (strlen(aliasName) > 0)
          fnif->setAliasName(aliasName);
      addFunction(fnif);
    }
  }*/
  else if (strcasecmp(moduleName, "poly") == 0) 
  {
    func = new UFuncPoly();
    if (func != NULL)
      addFunction(func);
  }
  else
    // ask the base module if it knows the requested module
    result = UCmdExe::loadStaticModule(moduleName, aliasName, why, whyCnt);
  //
  return result;
}
