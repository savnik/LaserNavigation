/***************************************************************************
 *   Copyright (C) 2008 by DTU Christian Andersen   *
 *   chrand@mail.dk   *
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

#include "uvariable.h"
#include "uvarmethod.h"


////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////

bool UVarMethodImplement::methodCall(const char * name, const char * paramOrder,
                                     char ** strings, const double * doubles,
                                     double * value,
                                     UDataBase ** returnStruct,
                                     int * returnStructCnt)
{
  // debug
  printf("UVarMethodImplement::methodCall: There is noone to implement '%s' "
      "with parameter '%s' (interface only)\n",
      name, paramOrder);
  // debug end
  return false;
}

////////////////////////////////////////////////////

bool UVarMethodImplement::methodCallV(const char * name, const char * paramOrder,
                                     UVariable * params[],
                                     UDataBase ** returnStruct,
                                     int * returnStructCnt)
{
  // debug
//   printf("UVarMethodImplement::methodCallV: There is noone to implement '%s' "
//       "with parameter '%s' (interface only)\n",
//       name, paramOrder);
  // debug end
  return false;
}

////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////


UVarMethod::UVarMethod()
{
  implementedBy = NULL;
  description = "";
  name[0] = '\0';
  paramOrder[0] = '\0';
  //useVParams = false;
}

////////////////////////////////////////////////////

void UVarMethod::setMethod(UVarMethodImplement * implementor,
                           const char * formalName,
                           const char * paramsList,
                           const char * comment)
{
  strncpy(name, formalName, MAX_VAR_NAME_SIZE);
  strncpy(paramOrder, paramsList, MAX_PARAM_COUNT);
  implementedBy = implementor;
  description = comment;
  //useVParams = false;
}

///////////////////////////////////////////////////

void UVarMethod::setMethodV(UVarMethodImplement * implementor,
                           const char * formalName,
                           const char * paramsList,
                           const char * comment)
{
  strncpy(name, formalName, MAX_VAR_NAME_SIZE);
  strncpy(paramOrder, paramsList, MAX_PARAM_COUNT);
  implementedBy = implementor;
  description = comment;
  //useVParams = true;
}

////////////////////////////////////////////////////

bool UVarMethod::methodCall(const double * paramDouble,
                            char ** paramStr,
                            double * returnValue,
                            UDataBase ** returnStruct,
                            int * returnStructCnt)
{
  return implementedBy->methodCall(name, paramOrder,
                                   paramStr, paramDouble,
                                   returnValue,
                                   returnStruct, returnStructCnt);
}

////////////////////////////////////////////////////

bool UVarMethod::methodCallV(UVariable * params[],
                             UDataBase ** returnStruct,
                             int * returnStructCnt)
{
  bool result;
  //
  result = implementedBy->methodCallV(name, paramOrder,
                                     params, returnStruct, returnStructCnt);
  if (not result)
  { // may be implemented by the old method
    char * paramStr[MAX_PARAM_COUNT];
    double paramDouble[MAX_PARAM_COUNT];
    int s, d, rCnt;
    UVariable ** var;
    UVariable * v;
    const char * p1;
    double returnValue;
    s = 0;
    d = 0;
    p1 = paramOrder;
    var = params;
    if (returnStructCnt != NULL)
      rCnt = *returnStructCnt;
    else
      rCnt = 0;
    while (*p1 > ' ')
    {
      switch(*p1)
      {
        case 's':
          paramStr[s++] = (char*)(*var)->getValueBuffer();
          break;
        case 'd':
          paramDouble[d++] = (*var)->getValued(0);
          break;
        default:
          // class parameters are in place already?
          break;
      }
      p1++;
      var++;
    }
    result = implementedBy->methodCall(name, paramOrder,
                                     paramStr, paramDouble,
                                     &returnValue,
                                     returnStruct, returnStructCnt);
    if (result and rCnt > 0)
    {
      if (returnStruct[0]->isAlsoA("var"))
        v = (UVariable*)returnStruct[0];
      else
        v = NULL;
      if (v != NULL)
      {
        v->setValued(returnValue, 0, true);
        if (*returnStructCnt == 0)
          *returnStructCnt = 1;
      }
    }
  }
  if (not result)
    // print error
    printf("UVarMethodImplement::methodCallV: There is noone to implement '%s' "
      "with parameter '%s' (neither new nor old type of call)\n",
      name, paramOrder);
  //
  return result;
}

////////////////////////////////////////////////////

void UVarMethod::deleteMethod()
{
  name[0] = '\0';
  description = "(deleted)";
}


////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////


