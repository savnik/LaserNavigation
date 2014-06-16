/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 ***************************************************************************/
#include "userverstatic.h"

#include <urob4/ufunctionimgpool.h>
#include <urob4/ufunctionvarpool.h>
#include <ulms4/ufunctionlaser.h>
#include <urob4/ufunctionif.h>
#include <urob4/ulogfile.h>


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
  char * p1 = list;
  int n = 0;
  //
  UCmdExe::getStaticHelpList(list, listCnt);
  n += strlen(p1);
  p1 = &list[n];
  //
  snprintf(p1, listCnt - n, "load='imagePool'   Load image pool functions\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='var'         Load variable pool interface\n");
  n += strlen(p1);
  p1 = &list[n];
  snprintf(p1, listCnt - n, "load='laserPool'   Load laser scanner device pool\n");
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
  UFunctionImgPool * fpool;
  UFunctionVarPool * fvarpool;
  UFunctionLaser * flaser;
  UFunctionIf * fif;
  //
 if (strcasecmp(moduleName, "imagePool") == 0)
  {
    fpool = new UFunctionImgPool();
    if (fpool != NULL)
      addFunction(fpool);
  }
  else if (strcasecmp(moduleName, "var") == 0)
  {
    fvarpool = new UFunctionVarPool();
    if (fvarpool != NULL)
      addFunction(fvarpool);
  }
  else if (strcasecmp(moduleName, "laserPool") == 0)
  {
    flaser = new UFunctionLaser();
    if (flaser != NULL)
      addFunction(flaser);
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
