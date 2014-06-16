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

#include <string>

#include <umap4/upose.h>
#include <urob4/uvarcalc.h>

#include "ucalc.h" 

UCalcOper::UCalcOper() 
{
  evaluated = false;
  left = NULL;
  right = NULL;
  valueType = 3;
  priority = 0;
  operLine = NULL;
  operPos = 0;
  oper = -1;
}

/////////////////////////////////////////////////////

UCalcOper::~UCalcOper()
{ // nothing to delete
}

////////////////////////////////////////////////////

void UCalcOper::evaluateMinus()
{
  UCalcOper * op;
  //
  op = right;
  while (true)
  {
    if (not op->evaluated and op->priority > priority)
      // need to be avaluated before conversion
      op->evaluate();
    if (op->evaluated)
    { // convert to positive operator
      if (oper == 11) // a '-'
        op->value = - op->value;
      else // if (oper == 12) // a '/'
        op->value = 1.0 / op->value;
      // that was it
      break;
    }
    // we are not finished
    if (op->oper == oper)
      // also a '-' or '/' so start a separate conversion from here
      op->evaluateMinus();
    // continue to first evaluated left operand
    op = op->left;
  }
  // make '-' to '+' or '/' to '*'
  // i.e. 11->9 and 12->10
  oper -= 2;
}

/////////////////////////////////////////////////////////////

bool UCalcOper::evaluate()
{
  bool result;
  //
  result = evaluated;
  if (not evaluated)
  {
    if (oper == -1)
    { //is the root, it has a right value only
      result = right != NULL;
      if (result)
        result = right->evaluate();
      evaluated = result;
      if (evaluated)
        // evaluated without error
        value = right->value;
      else
        value = 0.0;
    }
    else
    { // need to do an evaluation
      result = (left != NULL) and (right != NULL);
      if (result)
      { // operands both exist - evaluate if needed
        if (oper == 11 or oper == 12)
        { // convert - to + and / tp * before evaluation
          evaluateMinus();
        }
        if (not left->evaluated)
          result = left->evaluate();
        if (result and not right->evaluated)
          result = right->evaluate();
        if (result)
        { // evaluate this expression
          switch (oper)
          {
            case 1: value = left->value > right->value; break;
            case 2: value = left->value >= right->value; break;
            case 3: value = left->value < right->value; break;
            case 4: value = left->value <= right->value; break;
            case 6: value = left->value != right->value; break;
            case 8: value = left->value == right->value; break;
            case 9: value = left->value + right->value; break;
            case 10: value = left->value * right->value; break;
            case 11: value = left->value - right->value; break;
            case 12: value = left->value / right->value; break;
            case 13: value = double(long(left->value) & long(right->value)); break; // binary and
            case 14: value = left->value and right->value; break; // logical and
            case 15: value = double(long(left->value) | long(right->value)); break; // binary or
            case 16: value = left->value or right->value; break; // logical or
            case 17: value = double(long(left->value) >> long(right->value)); break; // bit shift right
            case 18: value = double(long(left->value) ^ long(right->value)); break; // xor
            case 19: value = double(long(left->value) << long(right->value)); break; // bit shift left
            case 20: value = long(left->value) % long(right->value); break; // modulo
            case 21: value = pow(left->value, right->value); break; // power (not an 'xor' that can be obtained by characters only)
            default:
              fprintf(stderr,
                      "UCalcOper::evaluate unknown operator '%d' at %d in '%s'",
                      oper, operPos, operLine);
              result = false;
              break;
          }
          evaluated = result;
        }
      }
    }
  }
  return result;
}

////////////////////////////////////////////////////

const char * UCalcOper::getOperStr()
{
  const int MOC = 22;
  const char * opStr[MOC] ={"=", ">", ">=", "<", "<=", "!" ,"!=", "=", "==",
                            "+", "*", "-", "/", "&", "and", "|", "or", ">>", "xor", "<<", "%", "^"};
  if (oper >= 0 and oper < MOC)
    return opStr[oper];
  else if (oper == -1)
    return "root";
  else
    return NULL;
}

////////////////////////////////////////////////////

bool UCalcOper::evaluateOperator(const char * line, const char * start,
                             const char ** end,
                             char * errorTxt, const int errorTextCnt,
                             bool syntaxCheck)
{  
  const char *p1, *p2;
  const char *p3; 
  bool isOK = true;
  //bool finished = false;
  //int n;
  const int MOC = 22; 
            //    0 1 2  3 4  5  6  7   8  9  10  11  12 13 14 15 16 17 18  19 20  21
            //    = > >= < <= !  != =   == +   *   -   /  & &&  | || >> xor <<  %   ^
  int pri[MOC] = {0,8,8, 8,8, -1,7, -1, 7, 10, 11, 10,11, 6, 3, 4, 2, 9, 5,  9,11, 12};
  //
  snprintf(errorTxt, errorTextCnt, "No error");
  p2 = start;
  operLine = line;
  // evaluate the back part of expression
  // find operator - >=, <= != < >
  oper = 0;
  while (isspace(*p2))
    p2++;
  // save start of operator
  p1 = p2;
  if (isalpha(*p2))
  { // may be 'and' or 'or'
    if ((strncmp(p2, "and", 3) == 0) and not isalnum(p2[3]))
    {
      oper = 14;
      p2 += 3;
    }
    else if ((strncmp(p2, "or", 2) == 0) and not isalnum(p2[2]))
    {
      oper = 16;
      p2 += 2;
    }
    else if ((strncmp(p2, "xor", 2) == 0) and not isalnum(p2[2]))
    {
      oper = 18;
      p2 += 3;
    }
    else
    {
      snprintf(errorTxt, errorTextCnt,
               "UCalc::evaluate unknown operator at %d in '%s'",
               p2 - line, line);
      isOK = false;
    }
  }
  p3 = p2;
  if (oper == 0)
  { // operator not found
    switch (*p2)
    { // other operators
      case '>': oper = 1; break;
      case '<': oper = 3; break;
      case '!': oper = 5; break; // boolean not
      case '=': oper = 7; break;
      case '+': oper = 9; break;
      case '*': oper = 10; break;
      case '-': oper = 11; break;
      case '/': oper = 12; break;
      case '&': oper = 13; break; // bit and
      case '|': oper = 15; break; // bit or
      case '%': oper = 20; break; // modulus
      case '^': oper = 21; break; // power
      default:
        // no operator, then value must be result
        isOK = false;
        snprintf(errorTxt, errorTextCnt,
                 "UCalc::unknown operator '%c' at %d in '%s'",
                 *p2, p1 - line, line);
        break;
    }
  }
  if (isOK)
  { // let p3 look at the following character
    p3++;
    // they may be equal
    if (*p3 == *p2)
    {
      if (((oper > 12) and (oper < 16)) or (oper == 7))
      { // boolean operation
        oper++;
        p3++;
      }
      else if (oper < 5)
      { // bit shift operation >> (17) or << (19)
        oper += 16;
        p3++;
      }
    }
    else if ((oper <= 7) and (*p3 == '='))
    { // one of >= <= != ==
      oper++;
      p3++;
    }
    else if (oper == 5)
    { // an '!' (not or faculty) is not implemented as operator
      p3--;
      isOK = false;
      snprintf(errorTxt, errorTextCnt,
               "UCalc::evaluate not valid placed '!' at %d in '%s'",
               p1 - line, line);
    }
  }
  if (isOK)
  {
    priority = pri[oper];
/*    n = p3 - p1;
    if (n > 0 and n < 5)
    {
      strncpy(operStr, p1, n);
      operStr[n] = '\0';
    }
    else
      strncpy(operStr, "err", 5);*/
  }
  if (end != NULL)
  {
    while (isspace(*p3))
      p3++;
    *end = p3;
  }
  return isOK;
}

////////////////////////////////////////////////////

void UCalcOper::setValue(double val, const char * line, const char * start)
{
  value = val;
  evaluated = true;
  priority = 14;
  operLine = line;
  operPos = start-line;
}

////////////////////////////////////////////////////

void UCalcOper::insertOperator(UCalcOper * oper)
{
  if (oper->priority >= priority)
  { // insert as right leg
    oper->left = right;
    right->parent = oper;
    right = oper;
    oper->parent = this;
  }
  else if (oper->priority >= parent->priority)
  { // insert between this and parent
    oper->left = this;
    oper->parent = parent;
    parent->right = oper;
    parent = this;
  }
  else
    // higher up
    parent->insertOperator(oper);
}

////////////////////////////////////////////////////

void UCalcOper::print(const char * preStr)
{
  const int MSL = 100;
  char s[MSL];
  //
  snprintf(s, MSL, "%s  ", preStr);
  if (oper < 0 and right == NULL)
    // just a value
    printf("%s %g\n", preStr, value);
  else
  { // print left and right side of operator
    if (evaluated)
      printf("%s %s: (%g)\n", preStr, getOperStr(), value);
    else
      printf("%s %s:\n", preStr, getOperStr());
    if (left != NULL)
      left->print(s);
    else
      printf("%s (left is empty)\n", s);
    if (right != NULL)
      right->print(s);
    else
      printf("%s (right is empty)\n", s);
  }
}

////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////


UCalc::UCalc()
{
  errorTxt[0] = '\0';
}

////////////////////////////////////////////////////

UCalc::~UCalc()
{
  UVarPool * vp;
  // remove methods implemented by this resource
  // but leave the structures (math)
  vp = getVarPool()->getRootVarPool();
  if (vp != NULL)
    vp->deleteMethods(this);
}

////////////////////////////////////////////////////

bool UCalc::addMathMethods()
{
  UVarPool * vp, *vpm;
  int n = -1;
  //
  vp = getVarPool()->getRootVarPool();
  if (vp != NULL)
  {
    vpm =  vp->getStruct("math");
    if (vpm == NULL)
      vpm = vp->addStructLocal("math", "global constants and methods", false);
    // global constants
    vpm->addVar("false", 0.0, "d", "Constant false is the same as 0");          //
    vpm->addVar("true", 1.0, "d", "Constant true is the same as 1");          //
    vpm->addVar("pi", M_PI, "d", "Constant pi is the same as 3.14159...");
    // global methods
    vpm->addMethod(this, "limitToPi", "d", "Limits the value to within [+Pi..-Pi[");
    vpm->addMethod(this, "sin", "d", "");
    vpm->addMethod(this, "cos", "d", "");
    vpm->addMethod(this, "hypot", "dd", "");
    vpm->addMethod(this, "acos", "d", "");
    vpm->addMethod(this, "asin", "d", "");
    vpm->addMethod(this, "tan", "d", "");
    vpm->addMethod(this, "atan", "d", "");
    vpm->addMethod(this, "atan2", "dd", "Returns angle within +Pi..-Pi first parameter is y second is x");
    vpm->addMethod(this, "sqrt", "d", "");
    vpm->addMethod(this, "sqr", "d", "Returns value*value");
    vpm->addMethod(this, "abs", "d", "Returns the unsigned (positive) value of parameter");
    vpm->addMethod(this, "max", "dd", "Returns minimum of two values");
    vpm->addMethod(this, "min", "dd", "Returns maximum of two values");
    n = vpm->addMethod(this, "defined", "s", "Returns '1' if a variable or structure with name 's' is defined (global scope)");
  }
  return (n > 0);
}

////////////////////////////////////////////////////


//////////////////////////////////////////////

bool UCalc::evaluateParameters(const char * sourceLine, // source line (for error reporting)
                       const char ** paramStart,   // position of parameter start i.e. an '('
                       double * paramValues,      // array of double parameters
                       int * paramValuesCnt,   // count of double sized parameters found
                       char ** paramStr,          // array of string parameters
                       char * paramOrder,         // order - e.g. "ds" for (double, string)
                       int maxParamCount,         // max size of parameter array
                       int maxParamStrCount,
                       int maxParamStrLength)      // max size of string param list
{
  bool result = true;
  const char * p1 = *paramStart;
  const char * p2;
  bool isOK;
  int strCnt = 0;
  int parCnt = 0;
  int dblCnt = 0;
  //
  if (*p1 == '(')
    p1++; // advance past open bracket
  while(result)
  { // test if there is a parameter
    while (isspace(*p1))
      p1++;
    if ((*p1 == '"') or (*p1 == '\''))
    {
      isOK = evaluateToString(sourceLine, p1, &p2,
                              paramStr[strCnt], maxParamStrLength);
      if (isOK and (strCnt < maxParamStrCount) and (parCnt < (maxParamCount - 1)))
      {
        paramOrder[parCnt++] = 's';
        strCnt++;
      }
    }
    else
    {
      paramValues[dblCnt] = evaluate(sourceLine, p1, &p2, &isOK, false);
      if (isOK and (parCnt < (maxParamCount - 1)))
      {
        paramOrder[parCnt++] = 'd';
        dblCnt++;
      }
    }
    // zero terminate
    paramOrder[parCnt] = '\0';
    // skip optional white space
    while (isspace(*p2))
      p2++;
    // only valid end is a ')'
    if (*p2 == ')')
      break;
    else if (*p2 != ',')
      result = false;
    // restart after komma
    p1 = p2 + 1;
  }

  if (*p2 == ')')
    // save end of parameter lins
    *paramStart = p2 + 1;
  if (paramValuesCnt != NULL)
    *paramValuesCnt = dblCnt;
  return result;
}


//////////////////////////////////////////////

bool UCalc::evaluateToString(const char * sourceLine, // source line (for error reporting)
                             const char * paramStart, // position of where to start in source
                             const char ** nextChar,  // first unused character in line
                             char * dest,             // string for resultant evaluated string
                             int destLength)          // max length of destination string
{
  bool result = true;
  const char *p1 = paramStart;
  const char *p2, *p3;
  const char * pEnd;
  int n = 0, m;
  bool isOK;
  double value;
  char quote;
  const char *specChar = "@�$#&%?�!��:;.,\t"; // not evaluated
  const char *operatorChar = "+-=*/<>%! "; // operator treated as whitespace
  const int MSL = 200;
  char s[MSL];
  bool isStringP1P2;
  bool gotValue;
  bool allEval = true;
  //
  while (isspace(*p1))
    p1++; // advance past white space
  pEnd = &sourceLine[strlen(sourceLine)];
  result = (p1 < pEnd) and (dest != NULL);
  while (result and (p1 < pEnd))
  { // test if there is a parameter
    isStringP1P2 = false;
    gotValue = false;
    if ((*p1 == '"') or (*p1 == '\''))
    { // literals are skipped
      quote = *p1++;
      p2 = strchr(p1, quote);
      if (p2 == NULL)
        p2 = (char *)pEnd;
      isStringP1P2 = true;
    }
    else
    { // find end of name, end may be \0, space or '('
      if (*p1 == '(')
      {
        p3 = p2;
        value = evaluate(sourceLine, &p1[1], &p3, &isOK, false);
        if (*p3 == ')')
        { // could be evaluated to next end bracket
          gotValue = true;
          p2 = p3 + 1;
        }
      }
      if (not gotValue)
      { // try other options
        if (true)
        {
          value = evaluate(sourceLine, p1, &p2, &result, false);
          if (p2 == p1)
          {
            snprintf(errorTxt, MAX_ERROR_SIZE, 
                     "Evaluate to string error near '%c' at %d",
                     *p1, p2 - sourceLine);
            result = false;
          }
          else if (result)
            gotValue = true;
        }
        else
        {
          p2 = (char*)p1;
          while ((isalnum(*p2) or (strchr(specChar, *p2) != NULL)) and
                  (p2 < pEnd) and (isalnum(*p1)))
            p2++;
          m = p2 - p1;
          if (m > 0)
          { // may be an evaluable sequence - try
            if (m > MSL - 1) m = MSL - 1;
            // get name of first value
            strncpy(s, p1, m);
            s[m] = '\0';
            // May be a known variable or a function call
            {
              gotValue = isdigit(*p1);
              if (not gotValue)
              { // may be a variable name or a function call
                p3 = p2;
                gotValue = getVarPool()->getGlobalValue(s, &value); //, &p3, false);
              }
              if (gotValue)
                // is an evaluatable value, so evaluate (to first unused character)
                value = evaluate(sourceLine, p1, &p2, &isOK, false);
            }
          }
          else
          { // unknown character - assume string to next real space (or end)
            while ((p2 < pEnd) and not isspace(*p2))
            { // stop if an operator is passed
              if (strchr(operatorChar, *p2++) != NULL)
                break;
            }
            allEval = false;
            snprintf(errorTxt, MAX_ERROR_SIZE,
                    "Evaluate to string error near '%c' at %d",
                    *p1, p2 - sourceLine);
          }
        }
      }
    }
    if (gotValue)
    { // send value to destination
      if ((n + 12) < destLength)
      { // space for value - print
        snprintf(&dest[n], destLength - n, "%g", value);
        n = strlen(dest);
      }
    }
    else if (result)
    { // is a string from p1 to p2
      m = p2 - p1;
      result = (n + m + 1) < destLength;
      if (result)
      {
        strncpy(&dest[n], p1, m);
        n += m;
        //dest[n++] = ' ';
      }
      else
        snprintf(errorTxt, MAX_ERROR_SIZE, "Not enough space to evaluate string");
      if (isStringP1P2)
        // skip end quote/apot
        p2++;
    }
    // skip optional white space
    if (result)
    { // prepare for next variable
      while (isspace(*p2))
        p2++;
      // skip to next parameter
      p1 = p2;
      // test for end of parameter
      if ((*p1 == ',') or (*p1 == ')'))
        // reached a parameter separator, so end here
        break;
    }
  }
  if (result)
  { // terminate string
    dest[n] = '\0';
    *nextChar = p1;
  }
  return result and allEval;
}

////////////////////////////////////////////////////////////////

bool UCalc::getValueAny(UVarPool * scope, const char * name, double * value, const char ** params)
{
  bool result = false;
  const char *p1;
  int parsCnt, n;
  const int MAX_PARS = 10;
  double pars[MAX_PARS];
  UPose pose;
  // string parameters
  const int MAX_STR_PARS = 3;
  const int MSPL = 1000;
  char paramStr[MAX_STR_PARS][MSPL];
  char * parsStr[MAX_STR_PARS];
  // string with parameter order - e.g. "dsd" (double, string, double)
  char parsOrder[MAX_PARS];
  // method pointer
  UVarPool *vpScope;
  bool isFunc;
  bool lastScope = false;
  //
  // try first an ordinary named value in calculator
  // it may be a local variable (a parameter) in a function
  vpScope = scope;
  if (vpScope != NULL)
  { // test if it is a function call with parameters
    isFunc = false;
    if (params != NULL)
    { // test if it is a function with parameters
      // test for function
      // 1. get parameters, 2. call function, 3. return value
      p1 = *params;
      while (isspace(*p1))
        p1++;
      if (*p1 == '(')
      { // a value-parameter function
        parsCnt = 0;
        parsOrder[0] = '\0';
        for (n = 0; n < MAX_STR_PARS; n++)
          parsStr[n] = paramStr[n];
        result = evaluateParameters(*params, &p1, pars, NULL,
                                     parsStr, parsOrder,
                                     MAX_PARS, MAX_STR_PARS, MSPL);
        if (result)
        { // all parameters are evaluated
          // return end position of parameters
          // -- first character after end-bracket
          *params = p1;
          isFunc = true;
        }
      }
    }
    //
    // now test for values in scopes
    // from local scope back to root, and then math scope
    while (vpScope != NULL)
    {
      if (isFunc)
      { // get value from function call
        result = vpScope->callLocal(name, parsOrder, parsStr, pars, value, NULL, NULL);
      }
      else
      { // get value as a variable
        result = vpScope->getLocalValue(name, value);
      }
      if (result)
        // value is found
        break;
      if (lastScope)
        // we are finished
        break;
      // move up one scope level
      vpScope = vpScope->getParentVarPool();
      if (vpScope == NULL)
      {
        vpScope = scope->getRootVarPool()->getStruct("math");
        lastScope = true;
      }
    }
  }
  return result;
}

/////////////////////////////////////////////////////////

bool UCalc::methodCall(const char * name, const char * paramOrder,
                       char ** strings, const double * pars,
                       double * value,
                       UDataBase ** returnStruct,
                       int * returnStructCnt)
{ // implement method call from math
  bool result = true;
  // evaluate standard functions
  if ((strcasecmp(name, "limitToPi") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = limitToPi(pars[0]);
  else if ((strcasecmp(name, "sin") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = sin(pars[0]);
  else if ((strcasecmp(name, "hypot") == 0) and (strcmp(paramOrder, "dd") == 0))
    *value = hypot(pars[0], pars[1]);
  else if ((strcasecmp(name, "cos") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = cos(pars[0]);
  else if ((strcasecmp(name, "acos") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = acos(pars[0]);
  else if ((strcasecmp(name, "asin") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = asin(pars[0]);
  else if ((strcasecmp(name, "tan") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = tan(pars[0]);
  else if ((strcasecmp(name, "atan") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = atan(pars[0]);
  else if ((strcasecmp(name, "sqrt") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = sqrt(pars[0]);
  else if ((strcasecmp(name, "sqr") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = sqr(pars[0]);
  else if ((strcasecmp(name, "abs") == 0) and (strcmp(paramOrder, "d") == 0))
    *value = fabs(pars[0]);
  else if ((strcasecmp(name, "max") == 0) and (strcmp(paramOrder, "dd") == 0))
    *value = maxd(pars[0], pars[1]);
  else if ((strcasecmp(name, "min") == 0) and (strcmp(paramOrder, "dd") == 0))
    *value = mind(pars[0], pars[1]);
  else if ((strcasecmp(name, "atan2") == 0) and (strcmp(paramOrder, "dd") == 0))
    *value = atan2(pars[0], pars[1]);
  else if ((strcasecmp(name, "defined") == 0) and (strcmp(paramOrder, "s") == 0))
  { // is this variable defined
    result = isDefined(getVarPool(), strings[0]);
    *value = double(result);
    result = true;
  }
  else
    result = false;
  if (returnStructCnt != NULL)
    *returnStructCnt = 0;
  return result;
}

////////////////////////////////////////////////////////////////

bool UCalc::isDefined(UVarPool * scope, const char * varName)
{
  bool result;
  UVarPool *vpScope;
  bool lastScope;
  const int MSL = 32;
  char s[MSL];
  double v;
  //
  vpScope = scope;
  // try first a variable
  result = getValueAny(scope, varName, &v, NULL);
  if (not result)
  { // it may also be the name of a structure
    lastScope = false;
    while (vpScope != NULL)
    { // look for a structure in this scope
      strncpy(s, "a", MSL);
        // get struct returns a var pool, and any not found name in 's'
      vpScope->getStructDeep(varName, false, s, MSL);
        // so if 's' is empty, the structure is found
      result = (strlen(s) == 0);
      if (result)
        break;
      if (lastScope)
        break;
        // not found yet, so
        // move up one scope level
      vpScope = vpScope->getParentVarPool();
      if (vpScope == NULL)
      { // add the math
        vpScope = getVarPool()->getRootVarPool()->getStruct("math");
        lastScope = true;
      }
    }
  }
  return result;
}

////////////////////////////////////////////

bool UCalc::getLocalScopeVaiable(const char * name, double * value)
{
  return false;
}

//////////////////////////////////////////////

// const char * UCalc::getGenFunctionHelp(const int index, int * count)
// {
//   const int SYS_FUNCTION_CNT = 11;
//   const char * sysFunHelp[SYS_FUNCTION_CNT] = {
//               "sin('radians')",
//               "cos(radians)",
//               "tan(radians)",
//               "asin('radians')",
//               "acos(value)",
//               "atan(value)",
//               "atan2(y-value, x-value)",
//               "sqrt(value)",
//               "sqr(value)",
//               "abs(value)",
//               "limitToPi(value) (not case sensitive)"
//               };
//   const char * result = NULL;
//   //
//   if ((index >= 0) and (index < SYS_FUNCTION_CNT))
//   {
//     result = sysFunHelp[index];
//   }
//   if (count != NULL)
//     *count = SYS_FUNCTION_CNT;
//   //
//   return result;
// }

//////////////////////////////////////////////

double UCalc::evaluateUnaryVal(const char * line, const char * start,
                            const char ** end, bool * evaluated, bool syntaxCheck)
{ // on return 'end' is first unused character
  double v1 = 0.0, result;
  const char *p1;
  const char *p2;
  bool isOK = true;
  char name[MAX_VARIABLE_NAME_SIZE];
  //
  p1 = start;
  while (isspace(*p1))
    p1++;
  // may have negation, '-', or is a sub expression
  if ((strncmp(p1, "not", 3) == 0) and not isalnum(p1[3]))
  { // unary not
    p1 += 3;
    v1 = not evaluateUnaryVal(line, p1, &p2, &isOK, syntaxCheck);
  }
  else if (*p1 == '!')
  { // unary ! boolean not
    v1 = ! evaluateUnaryVal(line, ++p1, &p2, &isOK, syntaxCheck);
  }
  else if (*p1 == '-')
  { // negative unary operator
    v1 = -evaluateUnaryVal(line, ++p1, &p2, &isOK, syntaxCheck);
  }
  else if (*p1 == '+')
  { // positive unary operator
    v1 = evaluateUnaryVal(line, ++p1, &p2, &isOK, syntaxCheck);
  }
  else if (*p1 == '(')
  { // sub expression
    v1 = evaluate(line, ++p1, &p2, &isOK, syntaxCheck);
    if (isOK)
    { // found value, now expect close bracket
      isOK = (*p2 == ')');
      if (isOK)
        p2++;
      else
        snprintf(errorTxt, MAX_ERROR_SIZE,
              "***eval: expected ')' but found '%c' at %d in '%s'",
              *p2, p2 - line, line);
    }
  }
  else if (isalpha(*p1))
  { // is either a boolean or a double symbol.
    // find end of symbol
    p2 = p1 + 1;
    while (isalnum(*p2) or (*p2 == '_') or (*p2 == '.'))
      p2++;
    // must be a variable or a constant
    strncpy(name, p1, p2 - p1);
    name[p2 - p1] = '\0';
    isOK = getValueAny(getVarPool(), name, &v1, &p2);
    if (not isOK)
    { // write error text
      snprintf(errorTxt, MAX_ERROR_SIZE,
               "***eval: unknown '%s' at %d in '%s'",
               name, p1 - line, line);
    }
  }
  else
  { // must be a constant (or an error)
    v1 = strtod(p1, (char **)&p2);
    isOK = (p1 != p2);
  }
  // return results
  result = v1;
  if (end != NULL)
    *end = p2;
  if (evaluated != NULL)
    *evaluated = isOK;
  return result;
}

/////////////////////////////////////////////

// double UCalc::evaluateOld(const char * line, const char * start,
//                        const char ** end, bool * evaluated, bool syntaxCheck)
// { // expect a line as
//   // "(roadLeft < 20.3 or double[7] > double[6])"
//   // may return false or true, when a ')' or eol is reached
//   double result;
//   const char *p1, *p2;
//   const char *p3, *p4;
//   double v1 = 0.0, v2 = 0.0;
//   char name[MAX_VARIABLE_NAME_SIZE];
//   int oper = 0;
//   bool isOK;
//   bool finished = false;
//   //
//   snprintf(errorTxt, MAX_ERROR_SIZE, "No error");
//   p1 = start;
//   // skip leading space
//   while(isspace(*p1))
//     p1++;
//   v1 = evaluateUnaryVal(line, p1, &p2, &isOK, syntaxCheck);
//   if (not isOK)
//     p4 = p2;
//   result = v1;
//   // evaluate the back part of expression
//   while (isOK and not finished)
//   { // find operator - >=, <= != < >
//     oper = 0;
//     while (isspace(*p2))
//       p2++;
//     if (isalpha(*p2))
//     { // may be and or or
//       if ((strncmp(p2, "and", 3) == 0) and not isalnum(p2[3]))
//       {
//         oper = 14;
//         p2 += 3;
//       }
//       else if ((strncmp(p2, "or", 2) == 0) and not isalnum(p2[2]))
//       {
//         oper = 16;
//         p2 += 2;
//       }
//       else
//       {
//         snprintf(errorTxt, MAX_ERROR_SIZE,
//           "***eval: unknown operator at %d in '%s'",
//           p2 - line, line);
//         isOK = false;
//       }
//     }
//     p3 = p2 + 1;
//     if (oper == 0)
//       switch (*p2)
//       { // other operators
//       case '>': oper = 1; break;
//       case '<': oper = 3; break;
//       case '!': oper = 5; break; // boolean not
//       case '=': oper = 7; break;
//       case '+': oper = 9; break;
//       case '*': oper = 10; break;
//       case '-': oper = 11; break;
//       case '/': oper = 12; break;
//       case '&': oper = 13; break; // bit and
//       case '|': oper = 15; break; // bit or
//       default:
//         // no operator, then value must be result
//         finished = true;
//         // unknown (offending character)
//         // must be maintained
//         p3--;
//       }
//     if (not finished)
//     {
//       if (*p3 == *p2)
//       {
//         if (oper == 12)
//         { // C-style remark
//           finished = true;
//           p3--;
//         }
//         else if ((oper > 12) or (oper == 7))
//         { // boolean operation
//           oper++;
//           p3++;
//         }
//         else if (oper < 5)
//         { // bit shift operation
//           oper += 16;
//           p3++;
//         }
//       }
//       else if ((oper <= 7) and (*p3 == '='))
//       { // one of >= <= != ==
//         oper++;
//         p3++;
//       }
//       else if (oper == 5)
//       { // unary ! (not) not allowed (here)
//         p3--;
//         finished = true;
//         isOK = false;
//         snprintf(errorTxt, MAX_ERROR_SIZE,
//                  "***eval: not valid '!' at %d in '%s'",
//                  p1 - line, line);
//       }
//     }
//     if (finished)
//       // finished save end pointer
//       p4 = p3;
//     //
//     if (isOK and not finished)
//     { // get next value
//       if ((oper == 10) or (oper == 12))
//         // evaluate now '*' or '/'
//         v2 = evaluateUnaryVal(line, p3, &p4, &isOK, syntaxCheck);
//       else
//         // binary or +- - delay operation
//         v2 = evaluate(line, p3, &p4, &isOK, syntaxCheck);
//     }
//     if (isOK and not finished)
//     {
//       switch (oper)
//       {
//         case 1: result = v1 > v2; break;
//         case 2: result = v1 >= v2; break;
//         case 3: result = v1 < v2; break;
//         case 4: result = v1 <= v2; break;
//         case 6: result = v1 != v2; break;
//         case 8: result = v1 == v2; break;
//         case 9: result = v1 + v2; break;
//         case 10: result = v1 * v2; break;
//         case 11: result = v1 - v2; break;
//         case 12: result = v1 / v2; break;
//         case 13: result = double(long(v1) & long(v2)); break;
//         case 14: result = v1 and v2; break;
//         case 15: result = double(long(v1) | long(v2)); break;
//         case 16: result = v1 or v2; break;
//         case 17: result = double(long(v1) >> long(v2)); break;
//         case 19: result = double(long(v1) << long(v2)); break;
//         default:
//           snprintf(errorTxt, MAX_ERROR_SIZE,
//               "***eval: expected an operator '%s' at %d in '%s'",
//               name, p3 - line, line);
//           finished = true;
//           break;
//       }
//     }
//     p2 = p4;
//     v1 = result;
//   }
//   if (end != NULL)
//     *end = p4;
//   if (evaluated != NULL)
//     *evaluated = isOK;
//   return result;
// }

/////////////////////////////////////////////////////////////////

double UCalc::evaluate(const char * line, const char * start,
                       const char ** end, bool * evaluated, bool syntaxCheck)
{ // expect a line as
  // "(roadLeft < 20.3 or double[7] > double[6])"
  // may return false or true, when a ')' or eol is reached
  double result = 0.0;
  const char *p1, *p2;
  const char *p3, *p4;
  double v1 = 0.0, v2 = 0.0;
  bool isOK;
  bool finished = false;
  const int MOC = 20;
  UCalcOper oos[MOC];
  int oosCnt = 0;
  UCalcOper *oV, *oO, *oRoot;

  //
  snprintf(errorTxt, MAX_ERROR_SIZE, "No error");
  p1 = start;
  // skip leading space
  oRoot = &oos[oosCnt++];
  oV = &oos[oosCnt++];
  //strncpy(oRoot->operStr, "root", 5);
  //
  while(isspace(*p1))
    p1++;
  v1 = evaluateUnaryVal(line, p1, &p2, &isOK, syntaxCheck);
  // save potential end character pointer
  p4 = p2;
  oV->setValue(v1, line, p2);
  oV->parent = oRoot;
  oRoot->right = oV;
  // evaluate the back part of expression
  while (isOK and not finished and oosCnt < MOC - 2)
  { // find operator - >=, <= != < >
    oO = &oos[oosCnt++];
    isOK = oO->evaluateOperator(line, p2, &p3,
                         errorTxt, MAX_ERROR_SIZE, syntaxCheck);
    if (not isOK)
    { // no more (usefull) data
      finished = true;
      isOK = true;
    }
    else
    { // continue with operator and next value
      oV->insertOperator(oO);
      // get next value
      v2 = evaluateUnaryVal(line, p3, &p4, &isOK, syntaxCheck);
      if (isOK)
      {
        oV = &oos[oosCnt++];
        oV->setValue(v2, line, p3);
        oV->parent = oO;
        oO->right = oV;
        //oRoot->print("pre ");
      }
      else
      { // print syntax error message
        snprintf(errorTxt, MAX_ERROR_SIZE,
                 "***eval: expected a value but found '%c' at %d in '%s'",
                 p3[0], p3 - line, line);
      }
    }
    p2 = p4;
  }
  if (isOK)
  { // print an error to error text
    //oRoot->print("pre eval: ");
    isOK = oRoot->evaluate();
    //oRoot->print("posteval: ");
    if (isOK)
      result = oRoot->value;
    //printf("evaluated (%s) to %g\n", bool2str(isOK), result);
  }
  if (end != NULL)
    *end = p4;
  if (evaluated != NULL)
    *evaluated = isOK;
  return result;
}

///////////////////////////////////////////////////////////////

bool UCalc::setVariable(const char * cmdLine, bool syntaxCheck)
{
  const int MVL = 50;
  char var[MVL];
  const char * peq;
  const char * p1, * p2 = NULL;
  int n = 0;
  bool result;
  double vald;
  // format 'set speedMax= 77.88'
  // format 'set   speedMax = 77.88'
  // skip the set
  peq = strchr(cmdLine, '=');
  result = (peq != NULL);
  if (not result)
  {
    printf("UCalc:: syntax error in SET command, no '=' sign found in '%s'\n", cmdLine);
    snprintf(errorTxt, MAX_ERROR_SIZE,
         "***eval:: syntax error, no '=' sign found in '%s'", cmdLine);
  }
  if (result and getVarPool() == NULL)
  {
    result = false;
    printf("UCalc:: missing variable resource to evaluate '%s'\n", cmdLine);
    snprintf(errorTxt, MAX_ERROR_SIZE,
             "***eval: missing variable resource to evaluate '%s'", cmdLine);
  }
  if (result)
  {
    if (strncasecmp(cmdLine, "set", 3) == 0)
      p1 = &cmdLine[4];
    else
      p1 = cmdLine;
    p2 = peq - 1;
    // skip space before variable
    while (isspace(*p1))
      p1++;
    // skip spaces before = sign
    while (isspace(*p2))
      p2--;
    n = p2 - p1 + 1;
    result = (n > 0);
    if (not result)
    {
      printf("PLANNER: syntax error in SET command, variable name in '%s'\n", cmdLine);
      snprintf(errorTxt, MAX_ERROR_SIZE,
         "***eval: syntax error, variable name in '%s'\n", cmdLine);
    }
  }
  if (result)
  { // get variable name
    strncpy(var, p1, n);
    var[n] = '\0';
    // no space is allowed in a variable name
    result = (strchr(var, ' ') == NULL);
    if (not result)
      snprintf(errorTxt, MAX_ERROR_SIZE,
            "***eval: found space in variable name in '%s'\n", var);
  }
  if (result)
  {
    p1 = peq + 1;
    while (isspace(*p1))
      p1++;
    vald = evaluate(cmdLine, (char *)p1, NULL, &result, syntaxCheck);
  }
  if (result and not syntaxCheck)
  { // set the variable
    result = getVarPool()->setGlobalVar(var, vald, true);
    if (not result)
      snprintf(errorTxt, MAX_ERROR_SIZE, "***eval: No space for new variable");
  }
  //
  return result;
}

///////////////////////////////////////////////////////////////

bool UCalc::evaluateSystemFunction(const char * name,
                          const double pars[], int parsCnt,
                          double * value, bool syntaxCheck)
{ // no system specific functions here
  // --- should be overwritten
  return false;
}

///////////////////////////////////////////////////////////////

bool UCalc::getSmrdemoValue(const char * name, double * value)
{
  return false;
}

///////////////////////////////////////////////////////////////

bool UCalc::isRemark(const char * string)
{
  bool result;
  const char *p1 = string;
  //
  while (isspace(*p1))
    p1++;
  result = (strchr("#%;", *p1) != NULL)
            or (strncmp(p1, "//", 2) == 0);
  return result;
}

/////////////////////////////////////////////////////////

// void UCalc::logCalcVars(FILE * logcc)
// {
//   int i;
//   //
//   if (logcc != NULL)
//   {
//     fprintf(logcc,"variable values\n");
//     //save all values in calculator
//     for (i = 0; i < varsCnt; i++)
//     {
//       fprintf(logcc, "%s=%g\n", getVarName(i), getValue(i));
//     }
//     fprintf(logcc,"variable values finished\n");
//   }
// }


bool UCalc::setResource(UResBase * resource, bool remove)
{
  bool result;
  UVarMethod * vm;
  //
  result = UResVarPool::setResource(resource, remove);
  if (result)
  {
    getVarPool()->getRootVarPool()->getLocalMethod("math.sin", "d", &vm);
    if (vm == NULL)
    { // math structure not created - do it now
      addMathMethods();
    }
  }
  return result;
}
