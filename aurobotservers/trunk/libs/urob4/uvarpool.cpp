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
 *
 * $Date: 2012-07-25 10:40:55 +0200 (Wed, 25 Jul 2012) $
 * $Id: uvarpool.cpp 59 2012-10-21 06:25:02Z jcan $
 *
 ***************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "uvarpool.h"
#include "uresvarpool.h"


////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////


UVarPool::UVarPool()
{
  vars = NULL;
  varsCnt = 0;
  varsMaxCnt = 0;
  varPools = NULL;
  varPoolsCnt = 0;
  varPoolsMaxCnt = 0;
  meths = NULL;
  methsCnt = 0;
  methsMaxCnt = 0;
  preName[0] = '\0';
  preNameFull[0] = '\0';
  parentVarPool = NULL;
  varPoolsLocal = NULL;
  logVar = NULL;
  parentVarPoolScope = NULL;
  descriptionCopy = NULL;
}

////////////////////////////////////////////////////

UVarPool::~UVarPool()
{
  deleteAll();
  if (varPools != NULL)
    free(varPools);
  if (vars != NULL)
    free(vars);
  if (meths != NULL)
    free(meths);
  if (varPoolsLocal != NULL)
    free(varPoolsLocal);
  if (descriptionCopy != NULL)
    free(descriptionCopy);
}

////////////////////////////////////////////////////

bool UVarPool::createVarSpace(const int varCnt, const int structCnt,
                              const int methCnt, const char * note, bool copy)
{
  bool result;
  //
  result = createVarStack(varCnt);
  if (result)
    // create space for structures (other UVarPool structures
    result = createVarPoolStack(structCnt);
  if (result)
    // create space for dynamic function call descriptors.
    result = createFunctionStack(methCnt);
  setDescription(note, copy);
  //
  return result;
}

////////////////////////////////////////////////////

bool UVarPool::createVarStack(const int maxCount)
{
  bool result;
  //
  if (vars != NULL)
  {
    result = (maxCount <= varsMaxCnt);
    if (not result)
    { // change size if increased only
      // allocate new space
      lock();
      vars = (UVariable **)realloc(vars, sizeof(UVariable*) * maxCount);
      result = (vars != NULL);
      if (result)
        varsMaxCnt = maxCount;
      else
        varsMaxCnt = 0;
      unlock();
    }
  }
  else
  { // allocate new space
    vars = (UVariable **)malloc(sizeof(UVariable*) * maxCount);
    result = (vars != NULL);
    if (result)
    {
      varsMaxCnt = maxCount;
      varsCnt = 0;
    }
  }
  return result;
}

////////////////////////////////////////////////////

bool UVarPool::createVarPoolStack(const int maxCount)
{
  bool result;
  UVarPool ** ns;
  UVarPool ** nsd;
  bool * nsl, *nsld;
  //
  if (varPools != NULL)
  {
    result = (maxCount <= varPoolsCnt);
    if (not result)
    { // change size if increased only
        // allocate new space
      ns = (UVarPool **)malloc(sizeof(UVarPool*) * maxCount);
      nsl = (bool *)calloc(sizeof(bool), maxCount);
      result =  (ns != NULL) and (nsl != NULL);
      if (result)
      { // lock during move operation
        lock();
        // save pointer for delete
        nsd = varPools;
        nsld = varPoolsLocal;
        // copy old variables to new space
        memcpy(ns, varPools, varPoolsCnt * sizeof(UVarPool *));
        memcpy(nsl, varPoolsLocal, varPoolsCnt * sizeof(bool));
        // move vars pointer to new space
        varPools = ns;
        varPoolsLocal = nsl;
          // set new (increased) max size
        varPoolsMaxCnt = maxCount;
        // ok to use
        unlock();
        // remove old space
        free(nsd);
        free(nsld);
      }
    }
  }
  else
  { // allocate space
    varPools = (UVarPool **)malloc(sizeof(UVarPool*) * (maxCount + 1));
    varPoolsLocal = (bool *)malloc(sizeof(bool) * (maxCount + 1));
    result = (varPools != NULL);
    if (result)
    {
      varPoolsMaxCnt = maxCount;
      varPoolsCnt = 0;
    }
  }
  return result;
}

////////////////////////////////////////////////////

bool UVarPool::createFunctionStack(const int maxCount)
{
  bool result;
  UVarMethod * ns;
  UVarMethod * nsd;
  //
  if (meths != NULL)
  { // exists - request for size change
    result = (maxCount <= methsCnt);
    if (not result)
    { // change size if increased only
      // allocate new space
      ns = (UVarMethod *)calloc(sizeof(UVarMethod), maxCount);
      result =  (ns != NULL);
      if (result)
      { // lock during move operation
        lock();
        // save pointer for delete
        nsd = meths;
        // copy old variables to new space
        memcpy(ns, meths, methsCnt * sizeof(UVarMethod));
        // move vars pointer to new space
        meths = ns;
        // set new (increased) max size
        methsMaxCnt = maxCount;
        // ok to use
        unlock();
        // remove old space
        free(nsd);
      }
    }
  }
  else
  { // allocate space
    meths = (UVarMethod *)malloc(sizeof(UVarMethod) * (maxCount + 1));
    result = (meths != NULL);
    if (result)
    {
      methsMaxCnt = maxCount;
      methsCnt = 0;
    }
  }
  return result;
}

////////////////////////////////////////////////////

UVariable * UVarPool::addVar(const char * name, const double initialValue,
                     const char * type,
                     const char * comment)
{
  UVariable::varType tp;
  tp = UVariable::typeFromChar(type);
  if (tp == UVariable::none)
    tp = UVariable::d;
  return addVar(name, initialValue, tp, comment);
}

////////////////////////////////////////////////////

UVariable * UVarPool::addVarA(const char * name, const char * attValue,
                     const char * type,
                     const char * comment)
{
  UVariable::varType tp;
  char * p1;
  const char *p2;
  double d = 0.0;
  UVariable * var;
  //
  tp = UVariable::typeFromChar(type);
  if (tp == UVariable::none)
  {
    tp = UVariable::d;
    if (strlen(attValue) > 0)
    {
      if (attValue[0] == ' ')
        tp = UVariable::s;
      else
      { // test for double value
        d = strtod(attValue, &p1);
        if (p1 == attValue)
        {
          tp = UVariable::b;
          if (strcasecmp(attValue, "true") == 0)
            d = 1.0;
          else if (strcasecmp(attValue, "false") == 0)
            d = 0.0;
          else
            // is a string
            tp = UVariable::s;
        }
      }
    }
  }
  p2 = attValue;
  if ((tp == UVariable::s) and (*p2 == ' '))
    p2++;
  var = addVar(name, d, tp, comment, p2);
  //var->setValued(attValue, 0, true);
  return var;
}

//////////////////////////////////////////////////////

UVariable * UVarPool::addVar(const char * name, const double initialValue,
                     UVariable::varType type,
                     const char * comment, const char * attValue)
{
  UVariable * var;
  int elem = 0;
  //
  var = addVar(name, NULL, comment);
  if (var != NULL)
  { // set variable type
    var->setType(type);
    // update value
    if (attValue == NULL)
      // no string
      var->setValued(initialValue, elem, true);
    else if (var->isDouble())
      var->setValued(attValue, elem, true);
    else if (var->isString())
      var->setValues(attValue, elem, true);
    // notify for push event (is implicit above)
    // setUpdated(name);
//    var->saveToLog(logVar, preNameFull, logVarLock);
    //    updatedLocalVar(result);
  }
  return var;
}

//////////////////////////////////////////////////////

// UVariable * UVarPool::addVar(const char * name,
//                      const UVariable * source,
//                      const char * comment)
// {
//   UVariable * var;
//   //
//   var = addVar(name, source, comment);
//   if (var != NULL)
//   { // just update
//     var->setValue(source, 0);
//     // notify for push event
//     setUpdated();
//     var->saveToLog(logVar, preNameFull, logVarLock);
//   }
//   return var;
// }

///////////////////////////////////////////////////

UVariable * UVarPool::addVar(const char * name,
                           const UVariable * source,
                           const char * comment)
{
  UVariable * var;
  UVarPool * vp;
  const int INCREASE_VAR_CNT = 50;
  int elem;
  const char * p1;
  const int MVL = MAX_VAR_NAME_SIZE;
  char vn[MVL];
  //
  p1 = strchr(name, '.');
  if (p1 != NULL)
  { // struct, so get varPool for struct
    vp = getStructDeep(name, true, vn, MVL);
    if (vp != NULL)
      var = vp->addVar(vn, source, comment);
    else
      var = NULL;
  }
  else
  {
    if (varsCnt >= varsMaxCnt)
      // make space for more variables
      createVarStack(varsMaxCnt + INCREASE_VAR_CNT);
    // test if there already
    var = getLocalVariable(name, &elem);
    if (var == NULL)
    { // not found, so add
      if (varsCnt >= varsMaxCnt)
        // make space for more variables
        createVarStack(varsMaxCnt + INCREASE_VAR_CNT);
      if (varsCnt < varsMaxCnt)
      { // add
        var = new UVariable();
        var->setParent(this);
        // insert into structure
        vars[varsCnt] = var;
        varsCnt++;
        if (comment != NULL)
          // assuming that description is a literal constant string
          var->description = comment;
      }
    }
    if (var != NULL)
    { // just update
      // set variable info
      var->setName(name, &elem);
      if (source != NULL)
        var->setValue(source, elem);
    }
    //setUpdated(name);
/*    if (var != NULL and var->getValueBuffer() != NULL)
      var->saveToLog(logVar, preNameFull, logVarLock);*/
  }
  return var;
}

////////////////////////////////////////////////////////

int UVarPool::addStruct(const char * name, UVarPool * varStruct)
{
  int result = -1;
  UVarPool * varp;
  const int INCREASE_STRUCT_CNT = 10;
  //
  varp = getStruct(name);
  if (varp == NULL)
  {
    if (varPoolsCnt >= varPoolsMaxCnt)
    { // recreate structure space with space to more data
      createVarPoolStack(varPoolsMaxCnt + INCREASE_STRUCT_CNT);
    }
    if ((varPoolsCnt < varPoolsMaxCnt) and
         (varStruct != NULL))
    {
      for (result = 0; result < varPoolsCnt; result++)
      { // first unused structure slot is used
        if (varPools[result] == NULL)
          break;
      }
      // set at this free entry
      varPools[result] = varStruct;
      // do not delete when this var pool is deleted
      varPoolsLocal[result] = false;
      // set new struct count
      varPoolsCnt = maxi(varPoolsCnt, result + 1);
      // set pre-name if not set already
      if (strlen(varStruct->getPreName()) == 0)
        varStruct->setPreName(name);
      // set the parent link too
      varStruct->setParentVarPool(this);
      // notify for push event
      setUpdated(name);
    }
  }
  return result;
}

////////////////////////////////////////////////////////

int UVarPool::addStructLink(const char * name, UVarPool * toVarStruct)
{
  int result = -1;
  UVarPool * varp;
  const int INCREASE_STRUCT_CNT = 10;
  //
  varp = getStruct(name);
  if (varp == NULL)
  {
    if (varPoolsCnt >= varPoolsMaxCnt)
    { // recreate structure space with space to more data
      createVarPoolStack(varPoolsMaxCnt + INCREASE_STRUCT_CNT);
    }
    if ((varPoolsCnt < varPoolsMaxCnt) and
        (toVarStruct != NULL))
    { // find empty slot for structure
      for (result = 0; result < varPoolsCnt; result++)
      { // first unused structure slot is used
        if (varPools[result] == NULL)
          break;
      }
      // set at this place
      varPools[result] = toVarStruct;
      // do not delete when this var pool is deleted
      varPoolsLocal[result] = false;
      // set new struct count
      varPoolsCnt = maxi(varPoolsCnt, result + 1);
      // notify for push event
      setUpdated(name);
    }
  }
  return result;
}

////////////////////////////////////////////////////////

UVarPool * UVarPool::addStructLocal(const char * name, const char * note, bool copy)
//                         int vCnt, int sCnt, int mCnt)
{
  UVarPool ** varpp, * varp;
  const int INCREASE_STRUCT_CNT = 50;
  int v;
  //
  varp = getStruct(name);
  if (varp == NULL)
  {
    if (varPoolsCnt >= varPoolsMaxCnt)
    { // recreate structure space with space to more data
      createVarPoolStack(varPoolsMaxCnt + INCREASE_STRUCT_CNT);
    }
    if (varPoolsCnt < varPoolsMaxCnt)
    { // find an empty slot for the new structure
      varpp = varPools;
      for (v = 0; v < varPoolsCnt; v++)
      { // look for an empty slot
        if ((*varpp) == NULL)
          break;
        varpp++;
      }
      varp = new UVarPool();
      if (varp->createVarSpace(0, 0, 0, note, copy))
      {
        varp->setPreName(name);
        varp->setParentVarPool(this);
        varPoolsLocal[v] = true;
        *varpp = varp;
        varPoolsCnt = maxi(varPoolsCnt, v + 1);
        // notify for push event
        setUpdated(name);
      }
      else
      { // no space
        delete varp;
        varp = NULL;
      }
    }
  }
  return varp;
}

////////////////////////////////////////////////////////

void UVarPool::deleteStruct(const char * name)
{
  int i;
  bool found = false;
  UVarPool ** vp;
  //
  //lock();
  vp = varPools;
  for (i = 0; i < varPoolsCnt; i++)
  {
    if ((*vp) != NULL)
    { // struct is not deleted
      if (strcasecmp((*vp)->getPreName(), name) == 0)
      {
        found = true;
        break;
      }
    }
    vp++;
  }
  if (found)
  {
    if (varPoolsLocal[i])
    { // this a locally owned structure, so
      // remove all substructures too
      (*vp)->deleteAll();
      // delete the full var-pool structure from heap
      // if created locally (otherwise it is probably a resource
      // that is deleted elsewhere
      delete *vp;
    }
    // remove pointer to structure
    *vp = NULL;
    for (i = varPoolsCnt - 1; i >= 0; i--)
    { // decrease count of structures until
      // there is a valid structure
      if (varPools[i] != NULL)
        break;
      varPoolsCnt--;
    }
  }
}

////////////////////////////////////////////////////////

void UVarPool::deleteAll()
{
  UVarPool * vp;
  int i;
  //
  for (i = 0; i < varPoolsCnt; i++)
  { // do the same in any subordinate structures
    vp = varPools[i];
    if (vp != NULL)
    {
      varPools[i] = NULL;
      // delete the full var-pool structure from heap
      // if created locally (otherwise it is probably a resource
      // that is deleted elsewhere
      if (varPoolsLocal[i])
      { // delete all sub-elements too
        delete vp;
      }
    }
  }
  methsCnt = 0;
  for (i = varsCnt - 1; i >= 0; i--)
  {
    varsCnt = i;
    if (vars[i] != NULL)
    {
      delete vars[i];
      vars[i] = NULL;
    }
  }
  varsCnt = 0;
  varPoolsCnt = 0;
}

////////////////////////////////////////////////////////


void UVarPool::deleteMethod(const char * name, const char * paramStr)
{
  int i;
  //
  i = getLocalMethodIdx(name, paramStr);
  if (i >= 0)
  {
    meths[i].deleteMethod();
    while (i == (methsCnt - 1))
    {
      methsCnt--;
      i--;
      if (meths[i].valid() > 0)
        break;
    }
  }
}

/////////////////////////////////////////////////////////////

void UVarPool::deleteMethods(UVarMethodImplement * owner)
{
  int i;
  UVarMethod * vm = meths;
  UVarPool ** vp;
  UVarMethodImplement * ob;
  bool delCnt = 0;
  //
  for (i = 0; i < methsCnt; i++)
  {
    ob = vm->implementedBy;
    if (owner == ob)
    {
      vm->deleteMethod();
      delCnt++;
    }
    vm++;
  }
  if (delCnt > 0)
  { // reduce method cnt if possible
    for (i = methsCnt - 1; i >= 0; i--)
    {
      if (meths[i].valid() > 0)
        break;
      methsCnt--;
    }
  }
  vp = varPools;
  for (i = 0; i < varPoolsCnt; i++)
  { // do the same in any subordinate structures
    if ((*vp) != NULL)
      (*vp)->deleteMethods(owner);
    vp++;
  }
}

////////////////////////////////////////////////////////

bool UVarPool::setPreName(const char * name)
{
  const int MPL = MAX_VAR_NAME_SIZE;
  //
  strncpy(preName, name, MPL);
  //
  return (int(strlen(name)) > MPL);
}

////////////////////////////////////////////////////////

bool UVarPool::setLocalVar(const int index, const double value, const int element)
{
  bool result;
  UVariable * var;
  bool changed;
  //
  result = (index >= 0) and (index < varsCnt);
  if (result)
  {
    //lock();
    var = vars[index];
    if (element < var->getElementCnt())
      changed = (value != var->getValued(element));
    else
      changed = true;
    var->setValued(value, element, false);
    // notify for push event
    setUpdated(var->name);
/*    if (changed)
      var->saveToLog(logVar, preNameFull, logVarLock);*/
    //unlock();
    result = changed;
  }
  else
    printf("UVarPool::setLocalVar index error\n");
  return result;
}

////////////////////////////////////////////////////////

bool UVarPool::setLocalVar(const int index, UVariable * value)
{
  bool result;
  UVariable * var;
  //
  result = (index >= 0) and (index < varsCnt);
  if (result)
  { // get variable
    var = vars[index];
    // copy content
    var->copy(value, false);
    // notify for push events
    setUpdated(var->name);
    // log change (if logVar is open)
    //var->saveToLog(logVar, preNameFull, logVarLock);
  }
  else
    printf("UVarPool::setLocalVar index error\n");
  return result;
}

////////////////////////////////////////////////////////

bool UVarPool::setLocalVarAdd(const int index, const double value, const int element)
{
  bool result;
  UVariable * var;
  //
  result = (index >= 0) and (index < varsCnt);
  if (result)
  {
    //lock();
    var = vars[index];
    var->add(value, element);
    // notify for push event
    setUpdated(var->name);
    //if (value != 0.0)
    //  var->saveToLog(logVar, preNameFull, logVarLock);
    //unlock();
  }
  else
    printf("UVarPool::setLocalVarAdd index error\n");
  return result;
}

////////////////////////////////////////////////////

bool UVarPool::setLocalVar(const char * name, const double value,
                           bool mayAdd, UVariable::varType vartyp,
                           const char * attValue)
{
  int idx;
  bool result;
  UVarPool * vp;
  const char * p1;
  const int MVL = MAX_VAR_NAME_SIZE;
  char vn[MVL];
  const char * varName;
  int elem;
  UVariable * var;
  // is it a struct?
  p1 = strchr(name, '.');
  if (p1 == NULL)
  { // local var
    vp = this;
    varName = name;
  }
  else
  { // struct, so get varPool for struct
    vp = getStructDeep(name, mayAdd, vn, MVL);
    varName = vn;
  }
  // should now be local to vp*
  if (vp != NULL)
  {
    idx = vp->getLocalVarIndex(varName, &elem);
    result = (idx >= 0);
    if (elem < 0)
      elem = 0;
    if (result)
      result = vp->setLocalVar(idx, value, elem);
    else if (mayAdd)
    { // add as new variable
      var = vp->addVar(varName, value, vartyp, "new", attValue);
      result = (var != NULL);
    }
  }
  else
    result = false;
  return result;
}

////////////////////////////////////////////////////////////////

bool UVarPool::setLocalVar(const char * name, const char * value,
                           bool mayAdd, const char * vartyp,
                           UTime * updTime)
{
  bool result;
  UVarPool * vp;
  const char * p1;
  const int MVL = MAX_VAR_NAME_SIZE;
  char vn[MVL];
  const char * varName;
  int elem;
  UVariable * vv;
  // is it a struct?
  p1 = strchr(name, '.');
  if (p1 == NULL)
  { // local var
    vp = this;
    varName = name;
  }
  else
  { // struct, so get varPool for struct
    vp = getStructDeep(name, mayAdd, vn, MVL);
    varName = vn;
  }
  // should now be local to vp*
  if (vp != NULL)
  {
    vv = vp->getLocalVariable(varName, &elem);
    if (vv == NULL and mayAdd)
    { // add as new variable
      vv = vp->addVarA(varName, value, vartyp, "added");
    }
    else if (vv != NULL)
    { // set new values
      if (vv->isString())
        result = vv->setValues(value, elem , true, updTime);
      else
      { //
        if (updTime != NULL and updTime->valid and not vv->hasHist())
          vv->makeTimeSeries(10, 11);
        result = vv->setValued(value, elem , true, updTime);
      }
    }
    result = (vv != NULL);
  }
  else
    result = false;
  return result;
}

////////////////////////////////////////////////////

int UVarPool::getLocalVarIndex(const char * name, int * element)
{
  int result = -1;
  int i, n = 0, col = -1;
  UVariable ** var = vars;
  const int MVL = MAX_VAR_NAME_SIZE + 1; // allow for index
  char vn[MVL];
  const char * varName = name;
  const char * SIZE = ".sizeof";
  const char * p1;
  //
  p1 = strstr(name, SIZE);
  if (p1 != NULL and strlen(p1) == strlen(SIZE))
  { // remove size-of parameter
    n = mini(p1 - name, MVL - 1);
    strncpy(vn, name, n);
    vn[n] = '\0';
    varName = vn;
    if (element != NULL)
      *element = -1;
  }
  else if (strchr(varName, '[') != NULL)
  { // strip off name from array index
    UVariable::splitNameIndex(name, element, &col, &n);
    // make a copy
    strncpy(vn, name, n);
    vn[n] = '\0';
    varName = vn;
  }
  else if (element != NULL)
    *element = -1;
  for (i = 0; i < varsCnt; i++)
  {
    if (strcasecmp((*var)->name, varName) == 0)
    {
      result = i;
      if (element != NULL)
        if (((*var)->rows() > 1 or *element == 0) and col >= 0)
        {
          *element = *element * (*var)->cols() + col;
        };
      break;
    }
    var++;
  }
  return result;
}

/////////////////////////////////////////////////////

UVarPool *  UVarPool::getStruct(const char * name)
{
  int i;
  UVarPool * result = NULL;
  UVarPool ** vp;
  bool getRoot = false;
  //
  //lock();
  vp = varPools;
  // may be request for root structure
  getRoot = (parentVarPool == NULL) and (strcasecmp(name, "root") == 0);
  // all other cases
  for (i = 0; i < varPoolsCnt; i++)
  {
    if ((*vp) != NULL)
    { // struct is not deleted
      if (strcasecmp((*vp)->getPreName(), name) == 0)
      {
        if (getRoot)
          result = this;
        else
          result = *vp;
        break;
      }
    }
    vp++;
  }
  //unlock();
  return result;
}

/////////////////////////////////////////////////////

UVarPool *  UVarPool::getStruct(const int idx)
{
  UVarPool * result = NULL;
  //
  if ((idx >= 0) and (idx < varPoolsCnt))
  {
    result = varPools[idx];
  }
  return result;
}

/////////////////////////////////////////////////////

UVarPool *  UVarPool::getStructDeep(const char * fullName,
                        const bool mayAdd,
                        char * varName, const int varNameCnt)
{
  UVarPool * vp, * vp1, * vp2;
  const char * p1;
  int n;
  const int MVL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  char structName[MVL] = "";
  char vName[MVL];
  bool isAStruct = false;
  //
  // make local copy
  strncpy(vName, fullName, MVL);
  // set initial search pointer
  vp = this;
  // loop until structure is found
  do
  { // look for struct separator
    vp1 = vp;
    p1 = strchr(vName, '.');
    if (p1 == NULL)
    { // not a structure or exactly a structure
      n = vp1->getLocalVarIndex(vName, NULL);
      if (n >= 0)
        // a local variable with this name is found
        break;
      // look for a structure with exactly this name
      vp2 = vp1->getStruct(vName);
      isAStruct = (vp2 != NULL);
      if (isAStruct)
        // is a structure, so that is it
        vp = vp2;
      break;
    }
    // get structure name length ...
    n = p1 - vName;
    if (n == 0)
    { // started with a dot - that is OK
      isAStruct = strlen(p1) == 1;
      break;
    }
    // and copy name
    strncpy(structName, vName, MVL);
    // and zero terminate
    structName[n] = '\0';
    // get name in this new structure
    p1++;
    memmove(vName, p1, strlen(p1) + 1);
    // get the structure pointer
    vp = vp1->getStruct(structName);
    // vp2=NULL if structure is not found
  } while (vp != NULL);
  // add?
  if (mayAdd and (vp == NULL) and (strlen(structName) > 0))
  { // try to add
    // vp1 holds the structure to add to and vName the name
    do
    { // create structure not found
      vp1 = vp1->addStructLocal(structName, "new", false);
      // should more levels be created?
      p1 = strchr(vName, '.');
      if (p1 == NULL)
        break;
      n = p1 - vName;
      // and copy name
      strncpy(structName, vName, MVL);
      // and zero terminate
      structName[n] = '\0';
      // get name in this new structure
      p1++;
      memmove(vName, p1, strlen(p1) + 1);
    } while (vp1 != NULL);
    vp = vp1;
  }
  // return result
  if (varName != NULL)
  { // return variable name in the found vp structure
    if (isAStruct)
      varName[0] = '\0';
    else
      strncpy(varName, vName, varNameCnt);
  }
  return vp;
}

/////////////////////////////////////////////////////

UVarPool *  UVarPool::getStructDeep(const char * fullName,
                                    char * varName, const int varNameCnt)
{
  UVarPool * vp;
  vp = getStructDeep(fullName, false, varName, varNameCnt);
  return vp;
}


/////////////////////////////////////////////////////

int  UVarPool::getStructIdx(const char * name)
{
  int i;
  int result = -1;
  UVarPool ** vp;
  //
  //lock();
  vp = varPools;
  for (i = 0; i < varPoolsCnt; i++)
  {
    if (strcasecmp((*vp)->getPreName(), name) == 0)
    {
      result = i;
      break;
    }
    vp++;
  }
  //unlock();
  return result;
}

/////////////////////////////////////////////////////

UVarMethod *  UVarPool::getLocalMethod(const char * name, const char * paramList)
{
  int i;
  UVarMethod * result = NULL;
  UVarMethod * vm;
  //
  vm = meths;
  for (i = 0; i < methsCnt; i++)
  {
    if (strcasecmp(vm->name, name) == 0)
    {
      if (paramList == NULL)
      {
        result = vm;
        break;
      }
      else if (strcmp(vm->paramOrder, paramList) == 0)
      {
        result = vm;
        break;
      }
    }
    vm++;
  }
  return result;
}

/////////////////////////////////////////////////////

int  UVarPool::getLocalMethodIdx(const char * name, const char * paramList)
{
  int i;
  int result = -1;
  UVarMethod * vm;
  //
  vm = meths;
  for (i = 0; i < methsCnt; i++)
  {
    if (strcasecmp(vm->name, name) == 0)
    {
      if (paramList == NULL)
      {
        result = i;
        break;
      }
      else if (strcmp(vm->paramOrder, paramList) == 0)
      {
        result = i;
        break;
      }
    }
    vm++;
  }
  return result;
}

/////////////////////////////////////////////////////

UVarPool *  UVarPool::getLocalMethod(const char * name, const char * paramList, UVarMethod ** method)
{
  UVarPool * vp;
  const char * p1;
  const int MNL = MAX_VAR_NAME_SIZE;
  char mn[MNL];
  //
  p1 = strchr(name, '.');
  if (p1 != NULL)
  { // dot notation is used - resolve structure
    vp = getStructDeep(name, mn, MNL);
    p1 = mn;
  }
  else
  { // no dot, so must be local (or missing)
    vp = this;
    p1 = name;
  }
  if (method != NULL)
  {
    if (vp != NULL)
      *method = vp->getLocalMethod(p1, paramList);
    else
      *method = NULL;
  }
  return vp;
}

/////////////////////////////////////////////////////

UVarMethod *  UVarPool::getLocalMethod(const int index)
{
  UVarMethod * result = NULL;
  //lock();
  if ((index >= 0) and (index < methsCnt))
    result = &meths[index];
  //unlock();
  return result;
}

/////////////////////////////////////////////////////

bool UVarPool::getLocalValue(const char * name, double * value)
{
  bool result;
  UVariable * var;
  int elem = 0;
  // get pointer to variable object
  var = getLocalVariable(name, &elem);
  // test if variabvle is found
  result = (var != NULL);
  if (result and value != NULL)
  { // extract value - if needed
    if (elem >= 0)
      *value = var->getValued(elem);
    else if (elem == -1)
      // requested sizeof
      *value = var->getElementCnt();
  }
  //
  return result;
}

/////////////////////////////////////////////////////

bool UVarPool::getGlobalValue(const char * name, double * value)
{
  bool result;
  UVariable * var;
  int elem = 0;
  // get pointer to variable object
  var = getGlobalVariable(name, &elem);
  // test if variabvle is found
  result = (var != NULL);
  if (result and value != NULL)
  { // default is element number 0
    if (elem < 0)
      elem = 0;
    // extract value - if needed
    *value = var->getValued(elem);
  }
  //
  return result;
}

/////////////////////////////////////////////////////

bool UVarPool::getGlobalString(const char * name, const char ** value)
{
  bool result;
  UVariable * var;
  int elem = 0;
  // get pointer to variable object
  var = getGlobalVariable(name, &elem);
  // test if variable is found
  result = (var != NULL);
  if (result)
  { // default is element number 0
    result = var->isString();
    if (result and value != NULL)
    {
      if (elem < 0)
        elem = 0;
      // extract value - if needed
      *value = var->getString(elem);
    }
  }
  //
  return result;
}

/////////////////////////////////////////////////////

UVariable * UVarPool::getLocalVariable(const char * name, int * element)
{
  UVariable * result = NULL;
  int i, n;
  UVarPool * vp;
  const int MVL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  char vn[MVL];
  const char  * varName = vn;
  const char * SIZE = ".sizeof";
  const char * p1;
  const char *p2 = NULL;
  //
  p1 = strchr(name, '.');
  if (p1 != NULL)
  {
    p2 = strstr(name, SIZE);
    if (p2 != NULL)
    { // reduce name to base variable name
      n = mini(MVL - 1, p2 - name);
      strncpy(vn, name, n);
      vn[n] = '\0';
    }
  }
  else
    varName = name;
  if (element != NULL)
    *element = 0;
  if (p1 == NULL or (p1 == p2 and p1 != NULL and strlen(p2) == strlen(SIZE)))
  {
    varName = name;
    i = getLocalVarIndex(varName, element);
    if (p2 != NULL)
      *element = -1;
    if (i >= 0)
      result = vars[i];
  }
  else
  { // a structure is specified
    vp = getStructDeep(name, false, vn, MVL);
    if (vp != NULL)
    { // strip name and potential index
      result = vp->getLocalVariable(varName, element);
    }
  }
  return result;
}

/////////////////////////////////////////////////////

UVariable * UVarPool::getGlobalVariable(const char * name, int * element)
{
  UVariable * result = NULL;
  int i;
//  UVariable ** var;
  UVarPool * vp, *vproot;
  const int MVL = MAX_VAR_NAME_SIZE;
  char vn[MVL];
  const char  * varName;
  const char * p1;
  // assume a root name is specified
  p1 = strchr(name, '.');
  vproot = getRootVarPool();
  if (p1 == NULL)
  { // var pool and name is known already
    varName = name;
    vp = vproot;
  }
  else
  { // get the specified structure - and the remaining name
    vp = vproot->getStructDeep(name, false, vn, MVL);
    varName = vn;
  }
  //lock();
  if (vp != NULL)
  {
    i = vp->getLocalVarIndex(varName, element);
    if (i >= 0)
      result = vp->getLocalVar(i);
  }
  else if (element != NULL)
    // ensure element has a value
    *element = 0;
  //unlock();
  return result;
}

/////////////////////////////////////////////////////

bool UVarPool::getGlobalValue(const char * name, bool * value)
{
  bool result;
  double v;
  // get result as double
  result = getGlobalValue(name, &v);
  if (result)
    // convert to boolean
    *value = (v > 0.5);
  return result;
}

/////////////////////////////////////////////////////

bool UVarPool::getGlobalValue(const char * name, UTime * value)
{
  bool result;
  double v;
  // get result as double
  result = getGlobalValue(name, &v);
  if (result)
    // convert to boolean
    value->setTime(v);
  return result;
}

/////////////////////////////////////////////////////

bool UVarPool::getLocalValue(const char * name, bool * value)
{
  bool result;
  double v = 0.0;
  // get result as double
  result = getLocalValue(name, &v);
  if (result)
    // convert to boolean
    *value = (v > 0.5);
  return result;
}

/////////////////////////////////////////////////////

bool UVarPool::getLocalValue(const char * name, UTime * value)
{
  bool result;
  double v = 0.0;
  // get result as double
  result = getLocalValue(name, &v);
  if (result)
    // convert to boolean
    value->setTime(v);
  return result;
}

/////////////////////////////////////////////////////

double UVarPool::getLocalValue(int idx, int element)
{
  double result;
  //
  //lock();
  if ((idx >= 0) and (idx < varsCnt))
    result = vars[idx]->getValued();
  else
    result = 0.0;
  //unlock();
  //
  return result;
}

/////////////////////////////////////////////////////

const char * UVarPool::getVarDescription(int idx)
{
  const char * result;
  //
  if ((idx >= 0) and (idx < varsCnt))
    result = vars[idx]->description;
  else
    result = "";
  //
  return result;
}

/////////////////////////////////////////////////////

bool UVarPool::getLocalValueBool(int idx)
{
  double result;
  //
  //lock();
  if ((idx >= 0) and (idx < varsCnt))
    result = vars[idx]->getValued();
  else
    result = 0.0;
  //unlock();
  //
  return (result > 0.5);
}

/////////////////////////////////////////////////////

int UVarPool::getLocalValueInt(int idx)
{
  int result;
  //
  //lock();
  if ((idx >= 0) and (idx < varsCnt))
    result = roundi(vars[idx]->getValued());
  else
    result = -1;
  //unlock();
  //
  return result;
}

/////////////////////////////////////////////////////

UTime UVarPool::getLocalValueTime(int idx)
{
  UTime result;
  //
  if ((idx >= 0) and (idx < varsCnt))
    result.setTime(vars[idx]->getValued());
  else
    result.clear();
  //
  return result;
}

/////////////////////////////////////////////////////

UPosition UVarPool::getLocalValue3D(int idx)
{
  UPosition result;
  UVariable * var;
  //
  if (idx >= 0)
  { // get variable
    var = getLocalVar(idx);
    result = var->get3D();
  }
  //
  return result;
}

/////////////////////////////////////////////////////

URotation UVarPool::getLocalValueRot(int idx)
{
  URotation result;
  UVariable * var;
  //
  if (idx >= 0)
  { // get variable
    var = getLocalVar(idx);
    result = var->getRot();
  }
  //
  return result;
}

/////////////////////////////////////////////////////

bool UVarPool::setLocalVar3D(int idx, UPosition * value)
{
  bool result = true;
  //
  UVariable * var;
  //
  if (idx >= 0)
  { // get variable
    var = getLocalVar(idx);
    var->set3D(value);
    setUpdated(var->name);
    //var->saveToLog(logVar, preNameFull, logVarLock);
  }
  else
    result = false;
  //
  return result;
}

/////////////////////////////////////////////////////

bool UVarPool::setLocalVarRot(int idx, URotation * value)
{
  bool result = true;
  //
  UVariable * var;
  //
  if (idx >= 0)
  { // get variable
    var = getLocalVar(idx);
    var->setRot(value);
    setUpdated(var->name);
    //var->saveToLog(logVar, preNameFull, logVarLock);
  }
  else
    result = false;
  //
  return result;
}

/////////////////////////////////////////////////////

UPosRot UVarPool::getLocalValue6D(int idx)
{
  UPosRot result;
  UVariable * var;
  //
  if (idx >= 0)
  { // get variable
    var = getLocalVar(idx);
    result = var->get6D();
  }
  return result;
}

/////////////////////////////////////////////////////

UPose UVarPool::getLocalValuePose(int idx)
{
  UPose result;
  UVariable * var;
  //
  if (idx >= 0)
  { // get variable
    var = getLocalVar(idx);
    result = var->getPose();
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVarPool::setLocalVar6D(int idx, UPosRot * value)
{
  bool result = true;
  UVariable * var;
  //
  if (idx >= 0)
  { // get variable
    var = getLocalVar(idx);
    var->set6D(value);
    setUpdated(var->name);
    //var->saveToLog(logVar, preNameFull, logVarLock);
  }
  else
    result = false;
  //
  return result;
}

/////////////////////////////////////////////////////

char * UVarPool::getVarName(int index)
{
  char * result = NULL;
  UVariable * var;
  //
  if ((index >= 0) and (index < varsCnt))
  {
    //lock();
    var = vars[index];
    result = var->name;
    //unlock();
  }
  return result;
}

//////////////////////////////////////////////

bool UVarPool::evaluateSystemFunction(const char * name,
                                   const double pars[], int parsCnt,
                                   double * value, bool syntaxCheck)
{ // no system specific functions here
  // --- should be overwritten
  return false;
}

///////////////////////////////////////////////////////////////

int UVarPool::addMethod(UVarMethodImplement * implementor,
                        const char * formalName,
                        const char * paramTypes,
                       const char * comment)
{
  int result = -1;
  UVarMethod * mt;
  const int INCREASE_VAR_CNT = 50;
  UVarPool * vp;
  const char * p1;
  const int MVL = MAX_VAR_NAME_SIZE;
  char vn[MVL];
  //
  p1 = strchr(formalName, '.');
  if (p1 != NULL)
  { // get the specified structure - and the remaining name
    vp = getStructDeep(formalName, true, vn, MVL);
    if (vp != NULL)
      result = vp->addMethod(implementor, vn, paramTypes, comment);
  }
  else
  { // implement here
    if (methsCnt >= methsMaxCnt)
      // make space for more variables
      createFunctionStack(methsMaxCnt + INCREASE_VAR_CNT);
    // is it there already?
    mt = getLocalMethod(formalName, paramTypes);
    if (mt == NULL)
    { // it is not found, so OK to add
      if (methsCnt < methsMaxCnt)
      { // add
        result = methsCnt++;
        mt = &meths[result];
        mt->setMethod(implementor, formalName, paramTypes, comment);
        // notify for push event
        // setUpdated();
      }
    }
  }
  return result;
}

/////////////////////////////////////////////////////////////

int UVarPool::addMethodV(UVarMethodImplement * implementor,
                        const char * formalName,
                        const char * paramTypes,
                        const char * comment)
{
  int result = -1;
  UVarMethod * mt;
  const int INCREASE_VAR_CNT = 50;
  UVarPool * vp;
  const char * p1;
  const int MVL = MAX_VAR_NAME_SIZE;
  char vn[MVL];
  //
  p1 = strchr(formalName, '.');
  if (p1 != NULL)
  { // get the specified structure - and the remaining name
    vp = getStructDeep(formalName, true, vn, MVL);
    if (vp != NULL)
      result = vp->addMethodV(implementor, vn, paramTypes, comment);
  }
  else
  { // implement here
    if (methsCnt >= methsMaxCnt)
      // make space for more variables
      createFunctionStack(methsMaxCnt + INCREASE_VAR_CNT);
    // is it there already?
    mt = getLocalMethod(formalName, paramTypes);
    if (mt == NULL)
    { // it is not found, so OK to add
      if (methsCnt < methsMaxCnt)
      { // add
        result = methsCnt++;
        mt = &meths[result];
        mt->setMethodV(implementor, formalName, paramTypes, comment);
        // notify for push event
        // setUpdated();
      }
    }
  }
  return result;
}

/////////////////////////////////////////////////////////////

UVarPool * UVarPool::getRootVarPool()
{
  UVarPool * result = this;
  //
  while (result->getParentVarPool() != NULL)
    result = result->getParentVarPool();
  //
  return result;
}


/////////////////////////////////////////////////////

bool UVarPool::callGlobal(const char * name, const char * paramOrder,
                             char ** strings, const double * doubles,
                             double * value,
                             UDataBase ** returnStruct,
                             int * returnStructCnt)
{
  return getRootVarPool()->callScope(name, paramOrder, strings, doubles,
              value, returnStruct, returnStructCnt);
}

/////////////////////////////////////////////////////

bool UVarPool::callGlobalV(const char * name, const char * paramOrder,
                          UVariable ** params,
                          UDataBase ** returnStruct,
                          int * returnStructCnt)
{
  return getRootVarPool()->callScopeV(name, paramOrder, params,
                        returnStruct, returnStructCnt);
}

/////////////////////////////////////////////////////

bool UVarPool::callScope(const char * name, const char * paramOrder,
                          char ** strings, const double * doubles,
                          double * value,
                          UDataBase ** returnStruct,
                          int * returnStructCnt)
{
  UVarPool * vp = this, *vpp;
  bool found = false;
  while (not found)
  {
    found = vp->callLocal(name, paramOrder, strings, doubles,
                        value, returnStruct, returnStructCnt);
    vpp = vp->getParentVarPoolScope();
    if (vpp != NULL)
      vp = vpp;
    else
      break;
  }
  if (not found)
  {
    vp = vp->getStruct("math");
    if (vp != NULL)
    { // there is a math structure, try this too.
      found = vp->callLocal(name, paramOrder, strings, doubles,
                            value, returnStruct, returnStructCnt);
    }
  }
  return found;
}

/////////////////////////////////////////////////////

bool UVarPool::callScopeV(const char * name, const char * paramOrder,
                         UVariable ** params,
                         UDataBase ** returnStruct,
                         int * returnStructCnt)
{
  UVarPool * vp = this, *vpp;
  bool found = false;
  while (not found)
  {
    found = vp->callLocalV(name, paramOrder, params, returnStruct, returnStructCnt);
    vpp = vp->getParentVarPoolScope();
    if (vpp != NULL)
      vp = vpp;
    else
      break;
  }
  if (not found)
  {
    vp = vp->getStruct("math");
    if (vp != NULL)
    { // there is a math structure, try this too.
      found = vp->callLocalV(name, paramOrder, params, returnStruct, returnStructCnt);
    }
  }
  return found;
}

/////////////////////////////////////////////////////

bool UVarPool::callLocal(const char * name, const char * paramOrder,
                    char ** strings, const double * doubles,
                    double * value,
                    UDataBase ** returnStruct,
                    int * returnStructCnt)
{
  UVarMethod * vm;
  bool result;
  //
  // get pointer to method - name holds both structure and method name, e.g. 'lasrtifRoad.leftLine'
  getLocalMethod(name, paramOrder, &vm);
  result = (vm != NULL);
  if (result)
  {
    result = vm->methodCall(doubles, strings, value, returnStruct, returnStructCnt);
  }
  return result;
}

////////////////////////////////////////////////////

bool UVarPool::callLocalV(const char * name, const char * paramOrder,
                         UVariable * params[],
                         UDataBase ** returnStruct,
                         int * returnStructCnt)
{
  UVarMethod * vm;
  bool result;
  //
  // get pointer to method - name holds both structure and method name, e.g. 'lasrtifRoad.leftLine'
  getLocalMethod(name, paramOrder, &vm);
  result = (vm != NULL);
  if (result)
  {
    result = vm->methodCallV(params, returnStruct, returnStructCnt);
  }
  return result;
}

////////////////////////////////////////////////////

const char * UVarPool::getFullPreName(char * name, const int MNL)
{
  const int MSL = 500;
  char s[MSL];
  //
  if (parentVarPool != NULL)
  { // get parents prename (or an empty string if parent is root)
    parentVarPool->getFullPreName(s, MSL);
    snprintf(name, MNL, "%s%s.", s, preName);
  }
  else
    name[0] = '\0';
  //
  return name;
}

////////////////////////////////////////////////////

void UVarPool::setParentVarPool(UVarPool * parent)
{
  if (parent != this)
    parentVarPool = parent;
  // make also a full string of the name of this struct relative to parent root
  getFullPreName(preNameFull, MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES);
}

////////////////////////////////////////////////////

void UVarPool::setParentVarPoolScope(UVarPool * parent)
{
  parentVarPoolScope = parent;
  // make also a full string of the name of this struct relative to parent root
  //getFullPreName(preNameFull, MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES);
}

////////////////////////////////////////////////////

void UVarPool::setLogfile(ULogFile * fileHandle, ULock * fileLock)
{
  UVariable * v;
  int i;
  //
  if (logVar != NULL)
  { // stop logging
    logVarLock->lock();
    logVar = NULL;
    logVarLock->unlock();
  }
  // set file as requested
  logVarLock = fileLock;
  logVar = fileHandle;
  if (logVar != NULL)
  { // This is a start of logging, and the initial value of all variables are needed
    if (logVar->isOpen())
      for (i = 0; i < varsCnt; i++)
      {
        v = vars[i];
        v->saveToLog(logVar, preNameFull, logVarLock);
      }
  }
}

///////////////////////////////////////////////////////////

bool UVarPool::setScopeVar(const char * name, const double value, bool mayAdd)
{
  UVarPool *vp = this, *vpp;
  bool result = false;
  bool isGlobal;
  // is a global variable specified
  isGlobal = strncmp(name, "global.", 7) == 0;
  while (vp != NULL)
  { // try parent var pool
    vpp = vp->getParentVarPoolScope();
    if (not isGlobal or vpp == NULL)
      // try in this var pool
      result = vp->setLocalVar(name, value, false, UVariable::d);
    if (result or vpp == NULL)
      break;
    vp = vpp;
  }
  if (not result and mayAdd)
  {
    if (isGlobal)
      result = vp->setLocalVar(name, value, true, UVariable::d);
    else
      // not found, so add locally
      result = setLocalVar(name, value, true, UVariable::d);
  }
  //
  return result;
}

///////////////////////////////////////////////////////////

bool UVarPool::setScopeVar(const char * name,
                           const UVariable * source, bool mayAdd)
{
  UVarPool *vp = this, *vpp;
  bool result = false;
  bool isGlobal;
  UVariable * var = NULL;
  int elem = 0;
  // is a global variable specified
  // this is the only way to know that it is not a local substructure variable
  isGlobal = strncmp(name, "global.", 7) == 0;
  while (vp != NULL)
  { // try parent var pool
    vpp = vp->getParentVarPoolScope();
    if (not isGlobal or vpp == NULL)
    { // try in this var pool
      var = vp->getLocalVariable(name, &elem);
      result = (var != NULL);
    }
    if (result or vpp == NULL)
      break;
    vp = vpp;
  }
  if (not result and mayAdd)
  { // not found, so add locally or in specific global structure
    if (not isGlobal)
      vp = this;
    var = vp->addVar(name, source, "new (by rule)");
    result = (var != NULL);
  }
  else if (result)
  { // variable found - so set value
    result = var->setValue(source, elem);
  }
  //
  return result;
}
///////////////////////////////////////////////////////////

bool UVarPool::getScopeValue(const char * name, double * value)
{
  UVarPool * vp = this;
  bool result = true;
  //
  while (vp != NULL)
  { // try in this var pool
    result = vp->getLocalValue(name, value);
    if (result)
      break;
    vp = vp->getParentVarPoolScope();
  }
  if (not result)
  { // try the math struct
    vp = getRootVarPool()->getStruct("math");
    if (vp != NULL)
      result = vp->getLocalValue(name, value);
  }
  return result;
}

//////////////////////////////////////////////

void UVarPool::listVars(const char * preStr, char * buff, const int buffCnt, bool andStructs)
{
  UVariable ** var;
  int i, j;
  UTime t;
  double dt, dtr, v;
  char * p1 = buff, *p2;
  int n = 0;
  const char MTL = 30;
  char pt[MTL];
  UVarPool * vp;
  //
  var = getVars();
  t.now();
  dtr = t.getDecSec();
  *p1 = '\0';
  for (i = 0; i < getVarsCnt(); i++)
  {
    n += strlen(p1);
    p1 = &buff[n];
    snprintf(p1, buffCnt - n, "%s%s =", preStr, (*var)->name);
    if ((*var)->isString())
    {
      n += strlen(p1);
      p1 = &buff[n];
      *p1++ = ' ';
      n++;
      (*var)->getValuesAsStringQuoted(p1, buffCnt - n, 0);
    }
    else
    {
      for (j = 0; j < (*var)->getElementCnt(); j++)
      { // advance buffer pointer
        n += strlen(p1);
        p1 = &buff[n];
        // get value
        v = (*var)->getValued(j);
        // test if value is a time
        dt = dtr - v;
        if ((dt < (3600.0 * 24.0 * 365.0)) and (dt > (-3600.0 * 24.0)) and (buffCnt - n > 40))
        { // is time
          p2 = pt;
          t.setTime(v);
          t.getDateString(pt, true);
          strcat(pt, " ");
          p2 = &pt[strlen(pt)];
          t.getTimeAsString(p2, true);
          snprintf(p1, buffCnt - n, " %g(%s)", v, pt);
        }
        else
        { // normal sized variable
          snprintf(p1, buffCnt - n, " %g", v);
        }
      }
    }
    n += strlen(p1);
    p1 = &buff[n];
    if (n < buffCnt);
      strcat(p1, "\n");
    var++;
  }
  if (andStructs)
  {
    snprintf(pt, MTL, "%s%s.", preStr, getPreName());
    for (i = 0; i < getStructCnt(); i++)
    {
      n += strlen(p1);
      p1 = &buff[n];
      vp = getStruct(i);
      vp->listVars(pt, p1, buffCnt - n, true);
    }
  }
}

//////////////////////////////////////////

void UVarPool::setDescription(const char * note, bool copy)
{
  if (copy)
  {
    descriptionCopy = (char *)realloc(descriptionCopy, strlen(note) + 1);
    strcpy(descriptionCopy, note);
    description = descriptionCopy;
  }
  else
    description = note;
}
