/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                        *
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

#include <string>

#include <umap4/upose.h>
#include <urob4/uvarcalc.h>
#include <urob4/usmltag.h>

#include "urule.h"
#include "uresrulestate.h"

// UMisBase::UMisBase()
// {
// }
//
// //////////////////////////////////
//
// UMisBase::~UMisBase()
// {
// }


//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisItem::UMisItem()
{
  next = NULL;
  lineNumber = -1;
}

//////////////////////////////////

UMisItem::~UMisItem()
{
  if (next != NULL)
    delete next;
  next = NULL;
}

//////////////////////////////////

UMisItem::ResultValue UMisItem::execute(UVarCalc * calc, int state)
{
  printf("UMisItem::execute: a '%s' - should be overwritten?\n", getDataType());
  return RV_EMPTY;
}

/////////////////////////////////

int UMisItem::symbolLength(const char * exp)
{ // characters allowed in a symbol
  const char * syA = "1234567890_.abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
  // characters allowed as first character in a symbol (no number no underscore)
  const char * sy1 = strchr(syA, '.');
  int n;
  const char *p1;
  //
  n = strspn(exp, sy1);
  p1 = &exp[n];
  n += strspn(p1, syA);
  p1 = &exp[n];
  return n;
}

/////////////////////////////////////

// bool UMisItem::checkSyntax(USmlSource * cnn)
// {
//   if (cnn != NULL)
//     ;
//   return true;
// }

/////////////////////////////////

const char * UMisItem::print(const char * preStr, char * buf, const int bufCnt)
{
  // just an interface function
  return buf;
}

//////////////////////////////////

const char * UMisItem::findStopChar(const char * source, char stop)
{
  const int MBL = 100;
  char brk[MBL];
  int brkIdx = 0;
//  const int MTL = 100000;
//  char ts[MTL]; // copy of source
  const char * result = NULL;
  const char * p1;
  int inStr = 0;
  //
  brk[0] = stop;
  p1 = source;
  while (*p1 != '\0')
  {
    if (*p1 == brk[brkIdx])
    {
      if (brkIdx == 0)
      {
        result = p1;
        break;
      }
      if (*p1 == '\'' or *p1 == '"')
        // decrease string nesting level
        inStr--;
      brkIdx--;
    }
    else if (strchr("[(\"'", *p1) != NULL)
    {
      if (inStr > 0 and (*p1 == '(' or *p1 == '['))
      { // brackets are allowed in strings in any flavor
        ;
      }
      else
      { // satrt of a substring
        brkIdx++;
        if (brkIdx >= MBL)
          // too many levels
          break;
        if (*p1 == '(')
          brk[brkIdx] = ')';
        else if (*p1 == '[')
          brk[brkIdx] = ']';
        else
        { // increase string nesting level
          brk[brkIdx] = *p1;
          inStr++;
        }
      }
    }
    else if (brkIdx == 0)
    { // stop if in a remark
      if (isRem(p1))
        break;
    }
    p1++;
  }
  return result;
}

//////////////////////////////////

bool UMisItem::isA(const char * matchType)
{
  return (strcmp(getDataType(), matchType) == 0);
}

//////////////////////////////////

bool UMisItem::isRem(const char * r)
{ // comment is either ';', '#' or '//'
  bool result;
  if (*r == ';' or *r == '#')
    result = true;
  else if (*r == '/')
    result = (r[1] == '/');
  else
    result = false;
  return result;
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisLineItem::UMisLineItem()
{
  line = NULL;
}

//////////////////////////////////

UMisLineItem::~UMisLineItem()
{
  if (line != NULL)
    free(line);
}

//////////////////////////////////

bool UMisLineItem::setLine(const int lineNum, const char * value, UVarCalc * calc)
{
  int n = strlen(value) + 1;
//  int n = strlen(value) + 2; // debug - write passed allocated value?
  //
  if (line != NULL)
    free(line);
  line = (char *)malloc(n);
  if (line != NULL)
  { // space allocated - copy string
    strncpy(line, value, n);
    xml2str(line, n, line, n);
    setLineNumber(lineNum);
  }
  return line != NULL;
}

/////////////////////////////////

const char * UMisLineItem::print(const char * preStr, char * buf, const int bufCnt)
{
  const int MLL = 2000;
  char d[MLL];
  //
  str2xmlMin(d, MLL, line);
  snprintf(buf, bufCnt, "%s%s\n", preStr, d);
  return buf;
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisAssign::UMisAssign()
{
  name = NULL;
  nameCnt = 0;
  assignment = NULL;
  indexExpr = NULL;
}

//////////////////////////////////

UMisAssign::~UMisAssign()
{
}

//////////////////////////////////

bool UMisAssign::setLine(const int lineNum, const char * value, UVarCalc * calc)
{
  int n = strlen(value) + 1;
  const char * p1, * p2 = NULL;
  bool result;
  const int MVL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  // set line itself
  UMisLineItem::setLine(lineNum, value, calc);
  // find name and assignment string
  p1 = line;
  while (isspace(*p1))
    p1++;
  result = calc->getIdentifier(p1, MVL, "_.", &p2);
  // get identifier length
  n = p2 - p1;
  result = n > 0;
  if (result)
  { // skip space before '='
    name = p1;
    nameCnt = n;
    // find assignment
    while (isspace(*p2))
      p2++;
    if (*p2 == '[')
    {
      indexExpr = p2;
      p2++;
      p2 = findStopChar(p2, ']');
      result = (*p2 == ']');
      if (not result)
      {
        snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
                 "***eval:: expected ']' but found '%c' at %d in '%s'\n",
                 *p2, p2 - line, line);
      }
      if (result)
      {
        p2++;
        while (isspace(*p2))
          p2++;
      }
    }
    if (result)
    {
      result = (*p2 == '=');
      if (not result)
      {
        snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
                "***eval:: no '=' or length > %d in '%s'\n",
                MVL, line);
      }
    }
  }
  if (result)
  { // skip the '='
    p2++;
    // skip white space
    while (isspace(*p2))
      p2++;
    assignment = p2;
    // now p2 should point to a number or an identifier or a sign or an open bracket
    result = isalnum(*p2) or ((strchr("-+('\"", *p2) != NULL) and *p2 != '\0');
    if (not result)
    {
      snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
               "***eval:: expected an expression, but found '%s'\n", p2);
    }
  }
  return result;
}

//////////////////////////////////

UMisItem::ResultValue  UMisAssign::execute(UVarCalc * calc, int state)
{
  const int MVL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  bool result;
  char id[MVL + 1];
  int n;
  const char * p2, *p1;
  UVariable * val;
  // format 'speedMax= 77.88'
  // make copy of identifier
  n = mini(MVL, nameCnt);
  strncpy(id, name, n);
  id[n] = '\0';
  val = new UVariable();
  if (indexExpr != NULL and n < (MVL - 10))
  { // evaluate index value
    p1 = indexExpr + 1;
    result = calc->evaluateV(line, p1, &p2, val, false);
    // p2 should be a ']'
    if (*p2 != ']')
      printf("assignment index error\n");
    else
      snprintf(&id[n], MVL - n, "[%d]", roundi(val->getValued()));
  }
  // evaluate value
  result = calc->evaluateV(line, (char *)assignment, NULL, val, false);
  if (not result)
  { // could not evaluate
    ; // errorTxt is OK as is??
  }
  else
  { // set the result, name, value and last character used
    result = calc->setScopeVar(id, val, state == UMisRuleState::init);
    if (not result)
      snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
               "Variable %s is not defined", id);
  }
  delete val;
  if (result)
    return RV_OK;
  else
    return RV_SYNTAX_ERROR;
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisCall::UMisCall()
{
  name = NULL;
  nameCnt = 0;
  parameters = NULL;
}

//////////////////////////////////

UMisCall::~UMisCall()
{
}

/////////////////////////////////////////////////////

bool UMisCall::setLine(const int lineNum, const char * value, UVarCalc * calc)
{
  int n = strlen(value) + 1;
  const char * p1, * p2 = NULL;
  bool result;
  const int MVL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  // set line itself
  UMisLineItem::setLine(lineNum, value, calc);
  // find name and assignment string
  p1 = line;
  while (isspace(*p1))
    p1++;
  result = calc->getIdentifier(p1, MVL, "_.", &p2);
  // get identifier length
  n = p2 - p1;
  result = n > 0;
  if (result)
  { // skip space before '='
    name = p1;
    nameCnt = n;
    // find assignment
    while (isspace(*p2))
      p2++;
    result = (*p2 == '(');
    parameters = p2;
    if (not result)
    {
      snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
               "***Call:: expected '(', but found '%s'\n", p2);
    }
  }
  return result;
}

//////////////////////////////////////

bool UMisCall::isA(const char * thisName)
{
  bool result = false;
  int n = strlen(thisName);
  if (n == nameCnt)
  {
    result = strncasecmp(thisName, name, n) == 0;
  }
  return result;
}

////////////////////////////////////////

const char * UMisCall::getNameCopy(char * dest, const int destCnt)
{
  int n;
  n = mini(destCnt-1, nameCnt);
  if (n >= 0)
  {
    strncpy(dest, name, n);
    dest[n] = '\0';
  }
  return dest;
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisIf::UMisIf()
{
  condition = NULL;
}

//////////////////////////////////

UMisIf::~UMisIf()
{
}

///////////////////////////////////////////////////

bool UMisIf::setLine(const int lineNum, const char * value, UVarCalc * calc)
{
  const char *p1;
  bool result;
  //
  result = UMisLineItem::setLine(lineNum, value, calc);
  //
  if (result)
  { // find start of condition
    p1 = line;
    while (isspace(*p1))
      p1++;
    // jump the 'if'
    p1 += 2;
    // and any extra space
    while (isspace(*p1))
      p1++;
    // this should be start of condition
    result = (*p1 == '(');
    if (result)
    {
      condition = p1;
    }
    else
    {
      snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
               "***if:: expected condition, found '%c' at %d in %s\n",
               *p1, p1 - line, line);
    }
  }
  return result;
}

//////////////////////////////////

UMisItem::ResultValue UMisIf::execute(UVarCalc * calc, int state)
{
  bool result;
  UVariable val;
  // evaluate value
  result = calc->evaluateV(line, (char *)condition, NULL, &val, false);
  if (result)
  {
    if (fabs(val.getValued()) > 0.5)
      return RV_IF_TRUE;
    else
      return RV_IF_FALSE;
  }
  else
    return RV_SYNTAX_ERROR;
}


//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisBreak::UMisBreak()
{
}

//////////////////////////////////

UMisBreak::~UMisBreak()
{
}

//////////////////////////////////

bool UMisBreak::setLine(const int lineNum, const char * value, UVarCalc * calc)
{
  const char * p1;
  const char * p2 = NULL;
  bool result;
  const int MVL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  // set line itself
  UMisLineItem::setLine(lineNum, value, calc);
  // find name and assignment string
  p1 = line;
  while (isspace(*p1))
    p1++;
  // remove break
  result = calc->getIdentifier(p1, MVL, "_.", &p2);
  // now p2 points at first character after 'break'
  // this may be nothing (end of line), a remark or an identifier
  while (isspace(*p2))
    p2++;
  if (*p2 == '\0' or isRem(p2))
  {
    param = NULL;
    paramCnt = 0;
  }
  else
  { // identifier should not be in quotes or apostrophs, but allow
    if (*p2 == '"' or *p2 == '\'')
      p2++;
    // save start of parameter
    param = p2;
    // find the end
    p1 = p2;
    // a number is allowed - should possibly be removed, but there for now
    strtol(p1, (char**)&p2, 10);
    if (p2 == p1)
    { // then maybe an identidier
      result = calc->getIdentifier(p1, MVL, "_", &p2);
    }
    paramCnt = p2 - p1;
    if (not result)
    {
      snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
               "***break:: expected an identifier but found '%s'\n", param);
    }
  }
  return result;
}

///////////////////////////////////

const char * UMisBreak::getParamCopy(char * dest, const int destCnt)
{
  int n;
  if (param == NULL and destCnt > 0)
    dest[0] = '\0';
  else
  {
    n = mini(destCnt - 1, paramCnt);
    if (n >= 0)
    {
      strncpy(dest, param, n);
      dest[n] = '\0';
    }
  }
  return dest;
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisEnable::UMisEnable()
{
}

//////////////////////////////////

UMisEnable::~UMisEnable()
{
}

//////////////////////////////////

bool UMisEnable::setLine(const int lineNum, const char * value, UVarCalc * calc)
{
  const char * p1;
  bool result;
  int n;
  // set line itself
  result = UMisLineItem::setLine(lineNum, value, calc);
  // find name and assignment string
  anEnable = strncmp(line, "enable", 6) == 0;
  if (anEnable)
    p1 = &line[6];
  else
    p1 = &line[7];
  // strip space after identifier
  while (isspace(*p1))
    p1++;
  // parameter may be in brackets - that is OK but ignored
  if (*p1 == '(')
    p1++;
  // may be in quotes - that is then evaluated as string
  if (*p1 == '"' or *p1 == '\'')
    // use full length, to allow an evaluated string
    n = strlen(p1);
  else
    // assume an identifier driectly
    n = symbolLength(p1);
  if (n == 0)
  {
    param = NULL;
    paramCnt = 0;
  }
  else
  { // save start of parameter
    param = p1;
    paramCnt = n;
  }
  return result;
}

///////////////////////////////////

const char * UMisEnable::getParamCopy(char * dest, const int destCnt)
{
  int n;
  if (param == NULL and destCnt > 0)
    dest[0] = '\0';
  else
  {
    n = mini(destCnt - 1, paramCnt);
    if (n >= 0)
    {
      strncpy(dest, param, n);
      dest[n] = '\0';
    }
  }
  return dest;
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisLoop::UMisLoop()
{
  initAssign = NULL;
  loopAssign = NULL;
  loopLine = NULL;
  condition = NULL;
}

//////////////////////////////////

UMisLoop::~UMisLoop()
{
  if (initAssign != NULL)
    delete initAssign;
  if (loopAssign != NULL)
    delete loopAssign;
}

///////////////////////////////////////////////////

bool UMisLoop::setLine(const int lineNum, const char * value, UVarCalc * calc)
{
  const char *p1, *p2;
  bool result;
  bool isWhile;
  bool isFor;
  int n;
  //
  result = UMisLineItem::setLine(lineNum, value, calc);
  //
  if (result)
  { // find start of condition
    n = symbolLength(line);
    isWhile = strncmp(line, "while", n) == 0;
    isFor = strncmp(line, "for", n) == 0;
    p1 = &line[n];
    while (isspace(*p1))
      p1++;
    result = (*p1 == '(');
    if (result)
    { // now get parameters in the bracket
      if (isFor)
      { // get first assignment
        p1++;
        p2 = findStopChar(p1, ';');
        result = (p2 != NULL);
        if (result and p2 > p1)
        {
          initAssign = new UMisAssign();
          result = initAssign->setLine(lineNumber, p1, calc);
          p1 = p2 + 1;
        }
        if (not result)
        {
          snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
                   "***for:: expected start assignment, found '%c' at %d in %s\n",
                   *p1, p1 - line, line);
        }
      }
      // get the conditition part
      condition = p1;
      // and the loop assignment
      if (isFor)
      { // skip the condition
        p2 = findStopChar(p1, ';');
        result = (p2 != NULL);
        if (result)
        {
          p1 = p2 + 1;
          p2 = findStopChar(p1, ')');
          result = (p2 != NULL);
        }
        if (result and p2 > p1)
        {
          loopAssign = new UMisAssign();
          result = loopAssign->setLine(lineNumber, p1, calc);
        }
        if (not result)
        {
          snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
                   "***for:: expected loop assignment, found '%c' at %d in %s\n",
                   *p1, p1 - line, line);
        }
      }
    }
    else
    {
      if (isWhile)
        snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
               "***while:: expected condition, found '%c' at %d in %s\n",
               *p1, p1 - line, line);
      if (isFor)
        snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
                 "***for:: expected parameters, found '%c' at %d in %s\n",
                 *p1, p1 - line, line);
    }
  }
  return result;
}

//////////////////////////////////

UMisItem::ResultValue UMisLoop::runInitAssignment(UVarCalc * calc)
{
  ResultValue result;
  if (initAssign != NULL)
    result = initAssign->execute(calc, UMisRuleState::init);
  else
    result = RV_OK;
  return result;
}

//////////////////////////////////

UMisItem::ResultValue UMisLoop::runLoopAssignment(UVarCalc * calc)
{
  ResultValue result;
  if (loopAssign != NULL)
    result = loopAssign->execute(calc, UMisRuleState::main);
  else
    result = RV_OK;
  return result;
}

//////////////////////////////////

UMisItem::ResultValue UMisLoop::evalCondition(UVarCalc * calc, bool * val)
{
  ResultValue result;
  bool res;
  UVariable valv;
  if (condition != NULL)
  {
    res = calc->evaluateV(line, (char *)condition, NULL, &valv, false);
    if (res)
      result = RV_OK;
    else
      result = RV_SYNTAX_ERROR;
    *val = (fabs(valv.getValued()) > 0.5);
  }
  else
  {
    result = RV_OK;
    *val = true;
  }
  return result;
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisRemark::UMisRemark()
{
}

//////////////////////////////////

UMisRemark::~UMisRemark()
{
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisString::UMisString()
{
}

//////////////////////////////////

UMisString::~UMisString()
{
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisControl::UMisControl()
{
}

//////////////////////////////////

UMisControl::~UMisControl()
{
}

//////////////////////////////////

bool UMisControl::setLine(const int lineNum, const char * value, UVarCalc * calc)
{
  const char *p1;
  bool result;
  //
  result = UMisCall::setLine(lineNum, value, calc);
  //
  if (result)
  {
    p1 = findStopChar(parameters, ':'); //
    // separate the condition from the call
    if (p1 != NULL)
    {
      p1++;
      while (isblank(*p1))
        p1++;
      strCond = p1;
    }
  }
  return result;
}

////////////////////////////////////////////////////

UMisItem::ResultValue UMisControl::execute(UVarCalc * calc, int repeat)
{
  UMisItem::ResultValue rv = UMisItem::RV_SYNTAX_ERROR;
  UMisItem::ResultValue rvCon;
  bool result;
  const char * p2;
//  UMisCall * call;
  const int MPL = 30;
  int n;
  char parOrder[MPL + 1];
  UDataBase * res;
  UVariable * repv, *resv;
  int ires;
  const int MNL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  char nm[MNL];
  UVariable * params[MPL];
  int paramsCnt = 0;
  //
  // get start (and end) of parameters
  p2 = getParameters();
  result = calc->evaluateParametersV(line, // source line (for error reporting)
                              &p2,      // position of parameter start i.e. an '(' - returns at end ')'
                              parOrder, // order - e.g. "ds" for (double, string)
                              params,     // array of double parameters
                              &paramsCnt,      // count of double sized parameters found
                              MPL);      // max number of parameters
  //
  if (result)
  { // the next non space character must be a ':'
    p2++;
    while (isspace(*p2))
      p2++;
    if (*p2 != ':')
    { // some other garbage found
      rv = UMisItem::RV_SYNTAX_ERROR;
      snprintf(calc->getErrorTxt(), MAX_ERROR_SIZE, "Expected ':' found %s\n", p2);
      result = false;
    }
  }
  if (result)
  { // add the repeat count to the parameter list
    strcat(parOrder, "d");
    repv = new UVariable();
    repv->setValued(double(repeat));
    params[paramsCnt++] = repv;
    // get the name into a buffer
    getNameCopy(nm, MNL);
    // try one of the language functions first
    // try to spot a locally defined method first
    n = 1;
    resv = new UVariable();
    res = resv;
    result = calc->callLocalV(nm, parOrder, params, &res, &n);
    //result = calc->callLocal(nm, parOrder, sppar, dpar, &res, NULL, 0);
    if (not result)
    { // metod is not found - try from root
      result = calc->callScopeV(nm, parOrder, params, &res, &n);
    }
    if (result)
    { // call is found - analyze result
      ires = roundi(resv->getValued());
      switch (ires)
      {
        case 0: rv = UMisItem::RV_FAILED; break;
        case 1: rv = UMisItem::RV_OK_AGAIN; break;
        case 2: rv = UMisItem::RV_OK; break;
        default:
          rv = UMisItem::RV_SYNTAX_ERROR;
          printf("Unknown result value (%g) of a control call\n", resv->getValued());
          break;
      }
      if (rv == UMisItem::RV_OK_AGAIN)
      { // test if explicit expression is true, i.e. completes the call
        rvCon = evaluateCondition(calc);
        // if result of condition is false, then keep RV_OK_AGAIN,
        // else either terminate now or syntax error
        if (rvCon != RV_OK_FALSE)
          rv = RV_OK;
      }
    }
    else
    {
      rv = UMisItem::RV_SYNTAX_ERROR;
      printf("line %d, method '%s(%s)' is not found (in scope)\n",
             lineNumber, nm, parOrder);
    }
    delete res;
  }
  for (n = 0; n < paramsCnt; n++)
  {
    if (params[n] != NULL)
      delete params[n];
  }
  return rv;
}


//////////////////////////////////

UMisItem::ResultValue UMisControl::evaluateCondition(UVarCalc * calc)
{
  UMisItem::ResultValue rv = UMisItem::RV_SYNTAX_ERROR;
  bool result;
  const char * p2;
  UVariable val;
  //
  result = calc->evaluateV(line, (char *)strCond, &p2, &val, false);
  if (not result)
  { // error in evaluation of external expression
    rv = UMisItem::RV_SYNTAX_ERROR;
    printf("Error line %d when evaluation explicit expression: %s\n",
           lineNumber, calc->getErrorTxt());
  }
  else
  { // there should be nothing after the expression
    while (isspace(*p2))
      p2++;
    if ((*p2 != '\0') and not isRem(p2))
    { // som garbage found after expression
      printf("line %d, expected remark or end of line, ignored from: %s\n", lineNumber, p2);
            // result = false;
            // rv = UMisItem::RV_SYNTAX_ERROR;
    }
  }
  if (result)
  {
    result = (roundi(val.getValued()) != 0);
    if (result)
      rv = RV_OK;
    else
      rv = RV_OK_FALSE;
  }
  //
  return rv;
}

////////////////////////////////// mlockall
//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

UMisRule::UMisRule()
{
  iniLines = NULL;
  mainLines = NULL;
  postLines = NULL;
  name[0] = '\0';
  condition = NULL;
  parameters = NULL;
  description = "";
  destination = NULL;
  isFull = false;
  isTop = false;
  isRun = false;
  isLoop = false;
  isRule = false;
  isSwitch = false;
  parametersCnt = 0;
  calc = NULL;
  fileName[0] = '\0';
  order = 50;
  inEditAppend = false;
  dependPlugin = "";
}

//////////////////////////////////

UMisRule::~UMisRule()
{
  if (iniLines != NULL)
    delete iniLines;
  if (mainLines != NULL)
    delete mainLines;
  if (postLines != NULL)
    delete postLines;
  if (parameters != NULL)
    free(parameters);
  if (condition != NULL)
    free(condition);
  if (destination != NULL)
    free(destination);
}

//////////////////////////////////

void UMisRule::addLine(UMisItem * lineList, UMisItem * newLine)
{ // add line to end of list
  UMisItem * ml = lineList;
  //
  if (ml == NULL)
    lineList = ml;
  else
  {
    while (ml->next != NULL)
      ml = ml->next;
    ml->next = newLine;
  }
  newLine->next = NULL;
}

//////////////////////////////////

int UMisRule::getLinesCnt(UMisItem * fromLine)
{
  int n = 0;
  UMisItem * ml = fromLine;
  //
  while (ml != NULL)
  {
    n++;
    ml = ml->next;
  }
  return n;
}

//////////////////////////////////

bool UMisRule::unpackRule(USmlSource * cnn, USmlTag * planTag)
{
  USmlTag tagb, *tagi = NULL; // tags for block lines
  USmlTag editTag, *tag;      // tag for rule attributes
  bool result = true;
  const int MDL = 100000;
  char d[MDL];  // buffer - plan lines or description
  UMisItem ** item = &mainLines;
  int m, n;
  int lastTagLine = -2;
  bool gotFirstNonRuleTag = false;
  //
  // rule block name, options, code and description
  if (planTag != NULL)
  {
    tag = planTag;
  }
  else
  { // no rule tag, but the first tag may be the rule tag
    // so get it now
    lastTagLine = cnn->getLineNumber();
    tag = &editTag;
    m = MDL - 1;
    cnn->getNextTag(tag, 100, NULL, d, &m);
    if (not tag->isTagA("rule"))
    { // first tag is not the plan attributes, so just append
      //d[m] = '\0';
      //m = trimWhiteSpace(d);
      gotFirstNonRuleTag = true;
      tagi = tag;
    }
  }
  // an edit plan has no name - it is set already
  if (inEditAppend)
  { // this is a plan and also a top level plan
    isFull = true;
  }
  else
  { // we need a name - if a callable plan or a rule
    tag->getAttValue("name", name, MAX_VAR_NAME_SIZE);
    result = strlen(name) > 0;
    isFull = tag->isTagA("rule");
    if (not isFull)
      // other blocks (command or block) do not need a name
      result = true;
  }
  if (not inEditAppend or tag->isTagA("rule"))
  { // regular plan tag
    if (isFull)
    { // is plan to run as default
      tag->getAttValueBool("run", &isRun, true);
      strncpy(fileName, cnn->getSourceName(), MAX_FILENAME_LENGTH);
    }
    //
    // set line number for this plan definition line
    setLineNumber(cnn->getLineNumber());
    //
    // other optional attributes
    if (tag->isTagA("commands"))
    { // is a command list, and may have a destination
      if (tag->getAttValue("to", d, MDL))
      {
        n = strlen(d) + 1;
        destination = (char *)malloc(n);
        strncpy(destination, d, n);
      }
    }
    if (tag->getAttValue("if", d, MDL))
    {
      n = strlen(d) + 1;
      result = (n > 1);
      if (not result)
      { // no condition specified, this is a syntax error
        cnn->syntaxError("Block 'if' has no condition - missing = ?");
      }
      else
      {
        condition = (char *)malloc(n);
        xml2str(condition, n, d, n);
        isRule = true;
      }
    }
    // get optional priority order
    tag->getAttValueInt("order", &order);
    //
    // check for valid syntax in header
    result &= checkSyntax(cnn);
    if (not result)
    {
      cnn->syntaxError("Block header has errors - skipping block");
      cnn->skipToEndTag(tag);
    }
  }
  // finished with plan attributes, now to the plan block
  // ensure to append any (new) lines
  while (*item != NULL)
    item = &(*item)->next;
  // continue until no more lines or tags
  while (result)
  {
    if (not gotFirstNonRuleTag)
    { // get next tag and the lines before the tag
      tagi = &tagb;
      lastTagLine = cnn->getLineNumber();
      m = MDL - 1;
      result = cnn->getNextTag(tagi, 100, NULL, d, &m);
      if (not result)
        // if end of file (or string ended)
        // then result is OK
        result = not cnn->isSourceAvailable();
    }
    else
      gotFirstNonRuleTag = false;
    d[m] = '\0';
    m = trimWhiteSpace(d);
    if (not result)
    { // no more data this is not as expected
      cnn->syntaxError("expected end of rule");
      logm->toLog("*** load error:", cnn->getErrorBuffer());
      cnn->skipToEndTag(tag);
      break;
    }
    else if (cnn->isErrorText())
    { // a warning text is available
      printf("%s\n", cnn->getErrorBuffer());
      logm->toLog("** warning:", cnn->getErrorBuffer());
    }
    if (tagi->isTagA("description"))
    {
      if (tagi->isAStartTag())
      {
        result = cnn->getToEndTag(tagi, d, MDL);
        if (result)
        { // copy description to description string
          m = trimWhiteSpace(d);
          xml2str(d, MDL, d, m);
          description = d;
        }
      }
      else
        description = "";
    }
    else if (tagi->isTagA("depend"))
    {
      if (tagi->getAttValue("plugin", d, MDL))
        // copy plug-in dependency to string
        dependPlugin = d;
    }
    else if (tagi->isTagA("parameters") and tagi->isAFullTag())
    {
      tagi->reset();
      // get length of attribute string - excluding the final tag end, the '/>'
      n = tagi->getToEnd() - tagi->getNext() - 1;
      if (n > 3)
      { // save all attributes in the parameter string
        parameters = (char*)malloc(n + 1);
        // copy all attributes
        strncpy(parameters, tagi->getNext(), n);
        // zero terminate the string
        parameters[n] = '\0';
        // count parameters
        parametersCnt = 0;
        while (tagi->getNextAttribute(d, NULL, 0))
          parametersCnt++;
      }
    }
    else if (tagi->isTagA("init") and (iniLines == NULL or inEditAppend) and isFull)
    { // next is an ini-block
      if (m > 0 and inEditAppend)
      { // unpack sequence of main lines before init block (legal in append only)
        unpackSequence(cnn, xml2str(d, MDL, d, m), item, lastTagLine + 1);
        while ((*item) != NULL)
          item = &(*item)->next;
      }
      result = unpackInitPostBlock(cnn, tagi, &iniLines, MP_INIT);
    }
    else if (tagi->isTagA("post") and (postLines == NULL or inEditAppend) and isFull)
    { // next is an post-block
      if (m > 0)
      { // unpack sequence of main lines before post statement
        unpackSequence(cnn, xml2str(d, MDL, d, m), item, lastTagLine + 1);
        while ((*item) != NULL)
          item = &(*item)->next;
      }
      // unpack post block
      result = unpackInitPostBlock(cnn, tagi, &postLines, MP_POST);
    }
    else if (tagi->isTagA("commands"))
    { // next is a command block
      if (m > 0)
      { // unpack lines before block command block
        unpackSequence(cnn, xml2str(d, MDL, d, m), item, lastTagLine + 1);
        while ((*item) != NULL)
          item = &(*item)->next;
      }
      // unpack commands
      result = unpackBlock(cnn, tagi, item);
    }
    else if (tagi->isTagA("block"))
    { // there is an block of commands (e.g. an if block)
      if (m > 0)
      { // unpack expression lines prior to new block
        unpackSequence(cnn, xml2str(d, MDL, d, m), item, lastTagLine + 1);
        while ((*item) != NULL)
          item = &(*item)->next;
      }
      // unpack block
      result = unpackBlock(cnn, tagi, item);
    }
    else if (tagi->isTagAnEnd(tag->getTagName()))
    { // are we finished
      if (m > 0)
        // unpack the expression lines prior to end tag
        unpackSequence(cnn, xml2str(d, MDL, d, m), item, lastTagLine + 1);
      // finished adding all items
      break;
    }
    else
    { // unsupported tag (or syntax error)
      if (cnn->isSourceAvailable())
        cnn->syntaxError("Ignored unknown tag");
      if (m > 0)
        // load the (text) commands
        unpackSequence(cnn, xml2str(d, MDL, d, m), item, lastTagLine + 1);
      if (tagi->isAStartTag())
        cnn->skipToEndTag(tagi);
    }
    if (not cnn->isSourceAvailable())
    { // no more data is available and all is handled
      break;
    }
    if (not result)
    { // skip the rest, we have failed
      cnn->syntaxError("Block failed - skipping");
      cnn->skipToEndTag(tag);
    }
    while ((*item) != NULL)
    { // advance item pointer to last element in statement list.
      item = &(*item)->next;
    }
  }
  return result;
}

//////////////////////////////////

bool UMisRule::unpackBlock(USmlSource * cnn, USmlTag * tag, UMisItem ** item)
{
  USmlTag tagi;
  UMisRule * block;
  bool result;
  // Add the new block
  block = new UMisRule();
  *item = block;
  // set syntax checker calculator
  block->setCalc(calc);
  block->setLogFile(logm);
  // unpack the block
  result = block->unpackRule(cnn, tag);
  //
  return result;
}

//////////////////////////////////

bool UMisRule::unpackInitPostBlock(USmlSource * cnn, USmlTag * tag, UMisItem ** itemEnd, MisPart iniBlock)
{ // this
  USmlTag tagi;
  bool result = true;
  const int MDL = 100000;
  char d[MDL];  // buffer - plan lines or description
  int m;
  UMisItem ** item = itemEnd;
  int line;
  // new lines are to be appended
  while (*item != NULL)
    item = &(*item)->next;
  //
  while (result)
  { // get line number of 'init' tag line
    line = cnn->getLineNumber();
    m = MDL - 1;
    result = cnn->getNextTag(&tagi, 100, NULL, d, &m);
    d[m] = '\0';
    m = trimWhiteSpace(d);
    if (m > 0)
    { // add the lines before the next tag.
      // sequence errors are not fatal, so do not use result
      unpackSequence(cnn, xml2str(d, MDL, d, m), item, line + 1, iniBlock);
      while (*item != NULL)
        item = &(*item)->next;
    }
    //
    if (not result)
    { // no more data this is not as expected
      cnn->syntaxError("Expected tag end");
      if (inEditAppend)
      { // recover from a missing end tag
        result = true;
        cnn->syntaxError("--- Placed implicit tag end and continued");
      }
      break;
    }
    else if (tagi.isTagA("commands") or
             tagi.isTagA("block") or
             (tagi.isTagA("rule") and (iniBlock == MP_INIT)))
    { // there is a block structure
      result = unpackBlock(cnn, &tagi, item);
    }
    else if (tagi.isTagAnEnd(tag->getTagName()))
    { // are we finished
      break;
    }
    else
    { // som other unsupported tag type, so ignore
      snprintf(d, MDL, "Unknown tag '%s' (ignored)", tagi.getTagName());
      cnn->syntaxError(d);
      if (tagi.isAStartTag())
        cnn->skipToEndTag(&tagi);
    }
    //
//    if (not result)
//    { // skip the rest, we have failed
//      cnn->syntaxError("Expected end of block or bad block (load failed)");
//      cnn->skipToEndTag(tag);
//    }
    if (*item != NULL)
    {
      while (*item != NULL)
        item = &(*item)->next;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UMisRule::unpackSequence(USmlSource * cnn, char * lines, UMisItem ** itemNext, int lineNum,
                             MisPart iniBlock)
{
  bool result = true;
  char * p1, *p2;
  UMisItem ** item = itemNext;
  int m;
  int line = lineNum;
  // debug
  // printf("inserting lines here:\n'%s'\n", lines);
  // debug end
  p2 = lines;
  while (p2 != NULL)
  { // separate into lines at newline
    p1 = strsep(&p2, "\n");
    m = trimWhiteSpace(p1);
    if (m > 0)
    { // p1 has a non-empty line - unpack it
      unpackLine(cnn, p1, item, line++, iniBlock);
      // new line should be appended - find new end
      while (*item != NULL)
        item = &(*item)->next;
    }
  }
  return result;
}

////////////////////////////////////////////////////

bool UMisRule::unpackLine(USmlSource * cnn, char * line, UMisItem ** itemNext, int lineNum, MisPart iniBlock)
{
  const char *p1, *pCtl;
  enum lineType {syntaxErr, aRemark, aControl, anAss, aCall,
                         anIf, anIfElse, aBreak, anEnable, aString,
                         aLoop, aSwitch, aCase, aDefault};
  lineType type = syntaxErr;
  int n;
  UMisItem * ml = NULL;
  bool result = false;
  //
  n = symbolLength(line);
  if (line[0] == ';' or line[0] == '#' or
      (line[0] == '/' and line[1] == '/'))
    type = aRemark;
  else if (destination != NULL)
    type = aString;
  else if (strncmp(line, "if", 2) == 0)
    type = anIf;
  else if (strncmp(line, "else", 4) == 0)
    type = anIfElse;
  else if (strncmp(line, "break", 5) == 0)
    type = aBreak;
  else if (strncmp(line, "enable", 6) == 0)
    type = anEnable;
  else if (strncmp(line, "disable", 7) == 0)
    type = anEnable;
  else if (strncmp(line, "continue", 8) == 0)
    type = aBreak;
  else if (strncmp(line, "for", 3) == 0)
    type = aLoop;
  else if (strncmp(line, "while", 5) == 0)
    type = aLoop;
  else if (strncmp(line, "switch", 6) == 0)
    type = aSwitch;
  else if (strncmp(line, "case", 4) == 0)
    type = aCase;
  else if (strncmp(line, "default", 7) == 0)
    type = aDefault;
  else
  {
    if (n > 0)
    {
      p1 = &line[n];
      while (isspace(*p1))
        p1++;
      if (*p1 == '=' or *p1 == '[')
        type = anAss;
      else if (*p1 == '(')
      {
        pCtl = findStopChar(line, ':');
        if (pCtl == NULL)
          type = aCall;
        else
          type = aControl;
      }
    }
  }
  switch (type)
  {
    case syntaxErr: break;
    case aRemark:
      ml = new UMisRemark();
      break;
    case anIf:
      ml = new UMisIf();
      break;
    case anIfElse:
      ml = new UMisIfElse();
      break;
    case aCall:
      ml = new UMisCall();
      break;
    case anAss:
      ml = new UMisAssign();
      break;
    case aControl:
      if (iniBlock == MP_MAIN)
        // control is not allowed in post blocks
        ml = new UMisControl();
      break;
    case aBreak:
      ml = new UMisBreak();
      break;
    case anEnable:
      ml = new UMisEnable();
      break;
    case aString:
      ml = new UMisString();
      break;
    case aLoop:
      ml = new UMisLoop();
      break;
    case aSwitch:
      ml = new UMisCaseSwitch();
      break;
    case aCase:
      ml = new UMisCase();
      break;
    case aDefault:
      ml = new UMisCaseDefault();
      break;
    default:
      break;
  }
  if (ml != NULL)
  {
    result = ml->setLine(lineNum, line, calc);
    if (result)
    { // syntax is - upto now - OK
      *itemNext = ml;
      //printf("UMisRule::unpackLine: Added %s\n", line);
    }
    else
    {
      type = syntaxErr;
      if (calc != NULL)
        cnn->syntaxError(calc->getErrorTxt());
      delete ml;
    }
  }
  else
    printf("Line %d ignored: %s\n", lineNum, line);
  return result;
}

///////////////////////////////////////////////////////

int UMisRule::trimWhiteSpace(char * s)
{
  char * p1 = s;
  char * p2;
  int n;
  while (isspace(*p1))
    p1++;
  n = strlen(p1);
  if (n > 0)
  {
    p2 = &p1[n] - 1;
    while (isspace(*p2) and (p2 > p1))
      p2--;
    n = p2 - p1 + 1;
  }
  if (n > 0 and p1 != s)
    memmove(s, p1, n);
  s[n] = '\0';
  return n;
}

////////////////////////////////////////////////////////

bool UMisRule::checkSyntax(USmlSource * cnn)
{
  int n;
  const  int MSL = 250;
  char s[MSL];
  bool isOK;
  //
  n = symbolLength(name);
  isOK = (n == (int)strlen(name));
  if (not isOK)
  {
    snprintf(s, MSL, "Block name is not legal - truncating name from %s to ", name);
    name[n] = '\0';
    strcat (s, name);
    cnn->syntaxError(s);
  }
  return isOK;
}

//////////////////////////////////////

const char * UMisRule::print(const char * preStr, char * buf, const int bufCnt)
{
  const char * tn;
  char * p1 = buf;
  int i, j, k, n = 0;
  UMisItem * mi;
  const int MDL = 10000;
  char d[MDL];
  const int MSL = 100;
  char s[MSL];
  char s2[MSL];
  const char * sPre;
  const int MIL = 10;
  bool ifElse[MIL];
  int ifCnt;
  int notIfCnt;
  const int indent = 4;
  //
  if (isFull)
    tn = "rule";
  else if (destination == NULL)
    tn = "block";
  else
    tn = "commands";
  //
  snprintf(buf, bufCnt - n, "%s<%s", preStr, tn);
  n += strlen(p1);
  p1 = &buf[n];
  if (strlen(name) > 0)
  { // add only name if it is defined
    snprintf(p1, bufCnt - n, " name=\"%s\"", name);
    n += strlen(p1);
    p1 = &buf[n];
  }
  if (isRun)
  {
    snprintf(p1, bufCnt - n, " run=\"true\"");
    n += strlen(p1);
    p1 = &buf[n];
  }
  if (condition != NULL)
  {
    str2xml(d, MDL, condition);
    snprintf(p1, bufCnt - n, " if=\"%s\"", d);
    n += strlen(p1);
    p1 = &buf[n];
  }
  if (destination != NULL)
  {
    snprintf(p1, bufCnt - n, " to=\"%s\"", destination);
    n += strlen(p1);
    p1 = &buf[n];
  }
  if (isEdit())
  {
    snprintf(p1, bufCnt - n, " edit=\"true\"");
    n += strlen(p1);
    p1 = &buf[n];
  }
  snprintf(p1, bufCnt - n, ">\n");
  n += strlen(p1);
  p1 = &buf[n];
  // then the description
  if (description.size() > 0)
  {
    strncpy(d, description.c_str(), MDL);
    str2xmlMin(d, MDL, d);
    snprintf(p1, bufCnt - n,
        "%s    <description>\n"
        "%s        %s\n"
        "%s    </description>\n", preStr, preStr, d, preStr);
    n += strlen(p1);
    p1 = &buf[n];
  }
  // then the parameters
  if (parameters != NULL)
  {
    snprintf(p1, bufCnt - n,
             "%s    <parameters %s/>\n", preStr,
              parameters);
    n += strlen(p1);
    p1 = &buf[n];
  }
  // now the items in the block
  // increase the indentation
  snprintf(s, MSL, "%s    ", preStr);
  snprintf(s2, MSL, "%s    ", s);
  for (i = 0; i < 3; i++)
  {
    switch (i)
    {
      case 0: mi = iniLines; break;
      case 1: mi = mainLines; break;
      case 2: mi = postLines; break;
      default:mi = NULL;
    }
    if (i == 0 and iniLines != NULL)
    {
      snprintf(p1, bufCnt - n, "%s<init>\n", s);
      n += strlen(p1);
      p1 = &buf[n];
      sPre = s2;
    }
    else if (i == 2 and postLines != NULL)
    {
      snprintf(p1, bufCnt - n, "%s<post>\n", s);
      n += strlen(p1);
      p1 = &buf[n];
      sPre = s2;
    }
    else
      sPre = s;
    // print lines in body
    ifCnt = 0;
    notIfCnt = 0;
    while (mi != NULL)
    { // print this line too
      if (ifCnt > 0)
      { // if line extra indentation - if not a block if
        if (mi->isA("misRule"))
          j = 1;
        else
          j = 0;
        if (notIfCnt >= 1 and not ((mi->isA("misIfElse") or mi->isA("misIf"))))
            ifCnt = 0;
        if (n < (bufCnt - indent * (ifCnt - j) - 1))
        { // add indentation
          for (k = 0; k < (ifCnt - j) * indent; k++)
            *p1++ = ' ';
          n += indent * (ifCnt - j);
          buf[n] = '\0';
        }
        if (not (mi->isA("misIfElse") or mi->isA("misIf")))
        { // were last line not an if either, ehen we are out
          notIfCnt++;
          while (notIfCnt == 1 and ifCnt > 0 and ifElse[ifCnt-1])
            ifCnt--;
          ifCnt--;
        }
      }
      mi->print(sPre, p1, bufCnt - n);
      if (ifCnt < MIL)
      {
        if (mi->isA("misIfElse") or mi->isA("misIf"))
        {
          ifElse[ifCnt] = mi->isA("misIfElse");
          ifCnt++;
          notIfCnt = 0;
        }
      }
      n += strlen(p1);
      p1 = &buf[n];
      mi = mi->next;
    }
    if (i == 0 and iniLines != NULL)
    {
      snprintf(p1, bufCnt - n, "%s</init>\n", s);
      n += strlen(p1);
      p1 = &buf[n];
    }
    else if (i == 2 and postLines != NULL)
    {
      snprintf(p1, bufCnt - n, "%s</post>\n", s);
      n += strlen(p1);
      p1 = &buf[n];
    }
  }
  if (isEdit())
  {
    snprintf(p1, bufCnt - n, "%sUncompleted added lines:\n"
        "-------\n%s------\n", preStr, getEditLines());
    n += strlen(p1);
    p1 = &buf[n];
  }
  // then just the end tag
  snprintf(p1, bufCnt - n, "%s</%s>\n", preStr, tn);
  return buf;
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

UMisCaseSwitch::UMisCaseSwitch()
{
  switchExpression = NULL;
}

/////////////////////////////////////////////////

UMisCaseSwitch::~UMisCaseSwitch()
{
}

/////////////////////////////////////////////////

bool UMisCaseSwitch::setLine(const int lineNum, const char * value, UVarCalc * calc)
{
  bool result;
  const char * p1;
  //
  result = UMisLineItem::setLine(lineNum, value, calc);
  //
  if (result)
  { // find the casevariable
    p1 = line;
    while (isspace(*p1))
      p1++;
    // jump the 'case'
    p1 += 6;
    // and any extra space
    while (isspace(*p1))
      p1++;
    // this should be start of condition
    result = (*p1 == '(');
    if (result)
    { // expression to be evaluated
      switchExpression = p1;
    }
    else
    {
      snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
               "***switch:: expected expression, found '%c' at %d in %s\n",
               *p1, p1 - line, line);
    }
  }
  return result;
}

////////////////////////////////////////////////

int UMisCaseSwitch::evaluateExpr(UVarCalc * calc)
{
  UVariable val;
  // evaluate value
  calc->evaluateV(line, (char *)switchExpression, NULL, &val, false);
  // return result as integer
  return roundi(val.getValued());
}

/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

UMisCase::UMisCase()
{
  caseListCnt = 0;
  caseList = NULL;
}

/////////////////////////////////////////////////

UMisCase::~UMisCase()
{
  if (caseList != NULL)
    free(caseList);
  caseListCnt = 0;
}

/////////////////////////////////////////////////

bool UMisCase::setLine(const int lineNum, const char * value, UVarCalc * calc)
{
  bool result;
  const char *p1, *p3;
  char * p2;
  int pars = 0;
  //
  result = UMisLineItem::setLine(lineNum, value, calc);
  caseListCnt = 0;
  //
  if (result)
  { // find the casevariable
    p1 = line;
    while (isspace(*p1))
      p1++;
    // jump the 'case'
    p1 += 4;
    // and any extra space
    while (isspace(*p1))
      p1++;
    // count parameters
    p3 = p1;
    if (isdigit(*p3) or *p3 == '-' or *p3 == '+')
    { // first character is a digit (0..9), as it should
      pars = 1;
      while (*p3 != '\0' and not isRem(p3))
      {
        if (*p3 == ',')
          pars++;
        p3++;
      }
      caseList = (int*)malloc(pars * sizeof(int));
      while (true)
      {
        caseList[caseListCnt] = strtol(p1, &p2, 0);
        caseListCnt++;
        if (caseListCnt == pars)
          break;
        while (*p2 != ',' and *p2 != '\0' and not isRem(p2))
          p2++;
        if (*p2 != ',')
          break;
        p1 = p2 + 1;
      }
    }
    if (caseListCnt != pars)
    {
      snprintf(calc->getErrorTxt(), calc->getErrorTxtMaxCnt(),
               "***case:: expected %d value(s), found %d, faied at %d in %s\n",
               pars, caseListCnt, p2 - line, line);
      result = false;
    }
  }
  return result;
}

/////////////////////////////////////////////////

bool UMisCase::isInList(int value)
{
  int i;
  bool result = false;
  for (i = 0; i < caseListCnt; i++)
  {
    result = (value == caseList[i]);
    if (result)
      break;
  }
  return result;
}

/////////////////////////////////////////////////
