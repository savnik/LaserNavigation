/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@oersted.dtu.dk
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <stdio.h>

#include <ugen4/ucommon.h>

#include "uvarcalc.h"
#include "uresvarpool.h"
#include "ucmdexe.h"

////////////////////////////////////

// UResVarPool::UResVarPool()
// {
//   setResID("resVar", 200);
//   varPool = NULL;
//   logVar = NULL;
//   snprintf(logVarName, MAX_FILENAME_LENGTH, "%s/%s.var.log", dataPath, appName);
//   createVarSpace(3,0,0,"Variables and methods for this resource");
//   createBaseVar();
// }

////////////////////////////////////

UResVarPool::~UResVarPool()
{
  if (varPool != NULL)
  {
    delete varPool;
    varPool = NULL;
  }
}

////////////////////////////////////

bool UResVarPool::createVarSpace(const int varCnt, const int structCnt,
                                 const int methCnt, const char * note, bool copy)
{
  bool result;
  // create space for (double type) variables.
  if (varPool == NULL)
    varPool = new UVarCalc();
  result = (varPool != NULL);
  if (result)
    varPool->createVarSpace(varCnt, structCnt, methCnt, note, copy);
  return result;
}

/////////////////////////////////////////////////////////////

void UResVarPool::createBaseVar()
{
  ; //addVar("version", getResVersion(), "d", "(r) Version of this resource");
}

/////////////////////////////////////////////////////

UVariable * UResVarPool::addVar(const char * name, const double initialValue,
             const char * type,
             const char * comment)
{
  if (varPool != NULL)
    return varPool->addVar(name, initialValue, type, comment);
  else
    return NULL;
}

/////////////////////////////////////////////////////

UVariable * UResVarPool::addVar(const char * name, const char * initialValue,
                                const char * type,
                                const char * comment)
{
  if (varPool != NULL)
    return varPool->addVarA(name, initialValue, type, comment);
  else
    return NULL;
}

/////////////////////////////////////////////////////

UVarPool * UResVarPool::addStruct(const char * name,
                     const char * comment, bool copy)
{
  if (varPool != NULL)
    return varPool->addStructLocal(name, comment, copy);
  else
    return NULL;
}

/////////////////////////////////////////////////////

int UResVarPool::addMethod( const char * formalName,
                            const char * paramTypes,
                            const char * comment)
{
  if (varPool != NULL)
    return varPool->addMethod(this, formalName, paramTypes, comment);
  else
    return -1;
}

/////////////////////////////////////////////////////

int UResVarPool::addMethodV( const char * formalName,
                            const char * paramTypes,
                            const char * comment)
{
  if (varPool != NULL)
    return varPool->addMethodV(this, formalName, paramTypes, comment);
  else
    return -1;
}

/////////////////////////////////////////////////////

bool UResVarPool::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to server core for event handling
  bool result = true;
  int n = 0;
  char * p1 = missingThese;
  //
  if (varPool != NULL)
  {
    if (not varPool->gotCmdExe())
    {
      strncpy(p1, UCmdExe::getResClassID(), missingTheseCnt);
      n = strlen(p1);
      p1 = &missingThese[n];
      result = false;
    }
  }
  result &= UResBase::gotAllResources(p1, missingTheseCnt - n);
  return result;
}

//////////////////////////////////////////////////////////////////

bool UResVarPool::setResource(UResBase * resource, bool remove)
{
  bool isUsed = false;
  //
/*  if ((varPool != NULL) and resource->isA(UCmdExe::getResClassID()))
  { // this ressource may hold variables that can be accessed by this module
    if (remove)
      varPool->setCmdExe(NULL);
    else
    { // resource is also a var-pool resource, so access as so.
      varPool->setCmdExe((UCmdExe *) resource);
    }
    result = true;
  }*/
  if (varPool != NULL)
    isUsed = varPool->UServerPush::setResource(resource, remove);
  //
  if (isUsed)
    setLocalVar("version", getResVersion(), false);
  //
  isUsed |= UResBase::setResource(resource, remove);
  return isUsed;
}

///////////////////////////////////////////


const char * UResVarPool::print(const char * preString, char * buff, int buffCnt)
{
  int m = 0, i;
  char * p1;
  UVarPool *vp;
  //
  if (buff != NULL)
  {
    p1 = buff;
    snprintf(p1, buffCnt, "%sVariable pool available (%s)\n", preString, bool2str(varPool != NULL));
    m = strlen(p1);
    p1 = &buff[m];
    //
    if (varPool != NULL)
    {
      snprintf(p1, buffCnt - m, " - variables (simple) %d/%d\n", varPool->getVarsCnt(), varPool->getVarMax());
      m += strlen(p1);
      p1 = &buff[m];
      //
      snprintf(p1, buffCnt - m, " - variables (struct) %d/%d\n", varPool->getStructCnt(), varPool->getStructMax());
      m += strlen(p1);
      p1 = &buff[m];
      if (varPool->isVarPoolVerbose())
      {
        for (i = 0; i < varPool->getStructCnt(); i++)
        {
          vp = varPool->getStruct(i);
          if (vp != NULL)
          {
            snprintf(p1, buffCnt - m, "      %s (%d/%d, %d/%d, %d/%d)\n", vp->getPreName(),
                    vp->getVarsCnt(), vp->getVarMax(),
                    vp->getStructCnt(), vp->getStructMax(),
                    vp->getMethodCnt(), vp->getMethodMax());
            m += strlen(p1);
            p1 = &buff[m];
          }
          vp++;
        }
      }
      //
      snprintf(p1, buffCnt - m, " - methods           %d/%d\n", varPool->getMethodCnt(), varPool->getMethodMax());
      m += strlen(p1);
      p1 = &buff[m];
    }
  }
  return buff;
}

/////////////////////////////////////////////////////

bool UResVarPool::getGlobalValue(const char * name, double * value)
{
  bool result = false;
  if (varPool != NULL)
    result = varPool->getGlobalValue(name, value);
  return result;
}

/////////////////////////////////////////////////////

bool UResVarPool::getGlobalString(const char * name, const char ** value)
{
  bool result = false;
  if (varPool != NULL)
    result = varPool->getGlobalString(name, value);
  return result;
}

/////////////////////////////////////////////////////

bool UResVarPool::getLocalValue(const char * name, double * value)
{
  bool result = false;
  if (varPool != NULL)
    result = varPool->getLocalValue(name, value);
  return result;
}

/////////////////////////////////////////////////////

bool UResVarPool::getLocalValue(const char * name, bool * value)
{
  bool result = false;
  if (varPool != NULL)
    result = varPool->getLocalValue(name, value);
  return result;
}

/////////////////////////////////////////////////////

bool UResVarPool::getLocalValue(const char * name, UTime * value)
{
  bool result = false;
  if (varPool != NULL)
    result = varPool->getLocalValue(name, value);
  return result;
}

/////////////////////////////////////////////////////

bool UResVarPool::getGlobalValue(const char * name, bool * value)
{
  bool result = false;
  if (varPool != NULL)
    result = varPool->getGlobalValue(name, value);
  return result;
}

/////////////////////////////////////////////////////

bool UResVarPool::getGlobalValue(const char * name, UTime * value)
{
  bool result = false;
  if (varPool != NULL)
    result = varPool->getGlobalValue(name, value);
  return result;
}

/////////////////////////////////////////////////////

bool UResVarPool::getLocalValueBool(int idx)
{
  bool result;
  if (varPool != NULL)
    result = varPool->getLocalValueBool(idx);
  else
    result = false;
  return result;
}

/////////////////////////////////////////////////////

UTime UResVarPool::getLocalValueTime(int idx)
{
  UTime result;
  if (varPool != NULL)
    result = varPool->getLocalValueTime(idx);
  return result;
}

///////////////////////////////////////////////////

UPosRot UResVarPool::getLocalValue6D(int idx)
{
  UPosRot result;
  if (varPool != NULL)
    result = varPool->getLocalValue6D(idx);
  return result;
}

///////////////////////////////////////////////////

bool UResVarPool::setLocalVar6D(int idx, UPosRot * value)
{
  bool result = false;
  if (varPool != NULL)
    result = varPool->setLocalVar6D(idx, value);
  return result;
}

///////////////////////////////////////////////////

UPosition UResVarPool::getLocalValue3D(int idx)
{
  UPosition result;
  if (varPool != NULL)
    result = varPool->getLocalValue3D(idx);
  return result;
}

///////////////////////////////////////////////////

bool UResVarPool::setLocalVar3D(int idx, UPosition * value)
{
  bool result = false;
  if (varPool != NULL)
    result = varPool->setLocalVar3D(idx, value);
  return result;
}

///////////////////////////////////////////////////

const char * UResVarPool::getVarDescription(int idx)
{
  const char * result;
  if (varPool != NULL)
    result = varPool->getVarDescription(idx);
  else
    result = "";
  return result;
}

/////////////////////////////////////////////////////

double UResVarPool::getLocalValue(int idx)
{
  double result;
  if (varPool != NULL)
    result = varPool->getLocalValue(idx);
  else
    result = 0.0;
  return result;
}

/////////////////////////////////////////////////////

bool UResVarPool::methodCall(const char * name, const char * paramOrder,
                          char ** strings, const double * doubles,
                          double * value,
                          UDataBase ** returnStruct,
                          int * returnStructCnt)
{
  // debug
  if (varPool != NULL)
    printf("UResVarPool::methodCall: There is noone to implement '%s' "
        "with parameter '%s' in resource '%s'\n",
        name, paramOrder, varPool->getPreName());
  else
    printf("No var-pool (UVarPool) in this resource - hmmm!\n");
  // debug end
  return false;
}

/////////////////////////////////////////////////////

bool UResVarPool::callGlobal(const char * name, const char * paramOrder,
                      char ** strings, const double * doubles,
                      double * value,
                      UDataBase ** returnStruct,
                      int * returnStructCnt)
{
  bool result;
  if (varPool != NULL)
  { // var pool is present, do the call
    result = varPool->callGlobal(name, paramOrder,
           strings, doubles, value, returnStruct, returnStructCnt);
  }
  else
    result = false;
  return result;
}

/////////////////////////////////////////////////////

bool UResVarPool::callGlobalV(const char * name, const char * paramOrder,
                           UVariable ** params,
                           UDataBase ** returnStruct,
                           int * returnStructCnt)
{
  bool result;
  if (varPool != NULL)
  { // var pool is present, do the call
    result = varPool->callScopeV(name, paramOrder, params,
                        returnStruct, returnStructCnt);
  }
  else
    result = false;
  return result;
}


//////////////////////////////////////////////////////

bool UResVarPool::callVS(const char * function, const char * stringParam)
{
  int n;
  UVariable * par[3];
  UVariable vs;
  UVariable vr;
  UDataBase *dbr;
  //
  vs.setValues(stringParam, 0, true);
  par[0] = &vs;
  dbr = &vr;
  return callGlobalV(function, "s", par, &dbr, &n);
}


///////////////////////////////////////////

int UResVarPool::callVSCD(const char * function, const char * strPar, UDataBase * data, int cooSys)
{
  int n;
  UVariable vs;
  UVariable vCoo;
  UVariable * par[3] = {&vs, (UVariable *) data, &vCoo} ;
  UVariable vr; // return value from function
  UDataBase *dbr; 
  bool isOK;
  // put values into parameter list
  vs.setValues(strPar, 0, true);
  vCoo.setInt(cooSys);
  vr.setDouble(-1.0, 0, true);
  dbr = &vr;
  isOK = callGlobalV(function, "scd", par, &dbr, &n);
  if (not isOK)
    return -1;
  else
    return vr.getInt();
  //  printf("UFuncPlan::callVSCD: failed to call '%s(%s, data, %d)'\n", function, strPar, cooSys);
  //printf("UFuncPlan::callVSCD: debug1\n");
}


/////////////////////////////////////////////////////

bool UResVarPool::callLocal(const char * name, const char * paramOrder,
                       char ** strings, const double * doubles,
                       double * value,
                       UDataBase ** returnStruct,
                       int * returnStructCnt)
{
  bool result;
  //
  if (varPool != NULL)
  { // get pointer to method - name holds both structure and method name, e.g. 'lasrtifRoad.leftLine'
    result = varPool->callLocal(name, paramOrder,
                           strings, doubles, value, returnStruct, returnStructCnt);
  }
  else
    result = false;
  return result;
}

////////////////////////////////////////////////////

bool UResVarPool::setLocalVar(int idx, double value, const int element)
{
  if (varPool != NULL)
    return varPool->setLocalVar(idx, value, element);
  else
    return false;
}

////////////////////////////////////////////////////

bool UResVarPool::setLocalVarAdd(int idx, double value, const int element)
{
  if (varPool != NULL)
    return varPool->setLocalVarAdd(idx, value, element);
  else
    return false;
}

////////////////////////////////////////////////////

bool UResVarPool::setLocalVar(const char * name, const double value, bool mayAdd)
{
  if (varPool != NULL)
    return varPool->setLocalVar(name, value, mayAdd, UVariable::d);
  else
    return false;
}

////////////////////////////////////////////////////

bool UResVarPool::setGlobalVar(const char * name, const double value, bool mayAdd)
{
  if (varPool != NULL)
    return varPool->setGlobalVar(name, value, mayAdd);
  else
    return false;
}

////////////////////////////////////////////////////

bool UResVarPool::logFileOpen()
{
  logVarLock.lock();
  logVar.openLog();
  logVarLock.unlock();
  return logVar.isOpen();
}

////////////////////////////////////////////////////

void UResVarPool::logFileClose()
{
  logVarLock.lock();
  logVar.closeLog();
  logVarLock.unlock();
}

////////////////////////////////////////////////////

void UResVarPool::logFileStart(const char * structName, bool open)
{
  UVarPool * vp;
  //
  if (varPool != NULL)
  {
    if (strlen(structName) == 0)
      vp = varPool;
    else
      vp = varPool->getStructDeep(structName, NULL, 0);
    if (vp != NULL)
    {
      if (open)
        vp->setLogfile(&logVar, &logVarLock);
      else
        vp->setLogfile(NULL, &logVarLock);
    }
  }
}

////////////////////////////////////////////////////

bool UResVarPool::isLogFileOpen(const char * structName)
{
  UVarPool * vp;
  bool result = false;
  //
  if (varPool != NULL)
  {
    if (strlen(structName) == 0)
      vp = varPool;
    else
      vp = varPool->getStructDeep(structName, NULL, 0);
    if (vp != NULL)
      result = vp->isLogfileOpen();
  }
  return result;
}

//////////////////////////////////////////////////////

void UResVarPool::setDescription(const char * note, bool copy)
{
  if (varPool != NULL)
    varPool->setDescription(note, copy);
}

//////////////////////////////////////////////////////

bool UResVarPool::isVarPoolUpdated(int lastCnt, int * newCnt)
{
  if (varPool != NULL)
    return varPool->isUpdated(lastCnt, newCnt);
  else
    return false;
}

