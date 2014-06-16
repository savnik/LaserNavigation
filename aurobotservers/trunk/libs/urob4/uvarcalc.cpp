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
#include "uvarpool.h"
#include "uvarcalc.h"

UVarCalcOper::UVarCalcOper()
{
  evalud = false;
  left = NULL;
  right = NULL;
  valueType = 3;
  priority = 0;
  operLine = NULL;
  operPos = 0;
  oper = -1;
  var = NULL;
  varLocal = false;
}

/////////////////////////////////////////////////////

UVarCalcOper::~UVarCalcOper()
{ // nothing to delete
  if (var != NULL and varLocal)
    delete var;
}

////////////////////////////////////////////////////

void UVarCalcOper::evaluateMinusD()
{
  UVarCalcOper * op;
  //
  op = right;
  while (true)
  {
    if (not op->evalud and op->priority > priority)
      // need to be avaluated before conversion
      op->evaluateD();
    if (op->evalud)
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
      op->evaluateMinusD();
    // continue to first evaluated left operand
    op = op->left;
  }
  // make '-' to '+' or '/' to '*'
  // i.e. 11->9 and 12->10
  oper -= 2;
}

/////////////////////////////////////////////////////////////

void UVarCalcOper::evaluateMinusV()
{
  UVarCalcOper * op;
  //
  op = right;
  while (true)
  {
    if (not op->evalud and op->priority > priority)
      // need to be avaluated before conversion
      op->evaluateV();
    if (op->evalud)
    { // convert to positive operator
      if (oper == 11) // a '-'
        op->negateValue();
      else // if (oper == 12) // a '/'
        op->inverseValue();
      // that was it
      break;
    }
    // we are not finished
    if (op->oper == oper)
      // also a '-' or '/' so start a separate conversion from here
      op->evaluateMinusV();
    // continue to first evaluated left operand
    op = op->left;
  }
  // make '-' to '+' or '/' to '*'
  // i.e. 11->9 and 12->10
  oper -= 2;
}

/////////////////////////////////////////////////////////////

bool UVarCalcOper::evaluateD()
{
  bool result;
  //
  result = evalud;
  if (not evalud)
  {
    if (oper == -1)
    { //is the root, it has a right value only
      result = right != NULL;
      if (result)
        result = right->evaluateD();
      evalud = result;
      if (evalud)
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
          evaluateMinusD();
        }
        if (not left->evalud)
          result = left->evaluateD();
        if (result and not right->evalud)
          result = right->evaluateD();
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
                      "UVarCalcOper::evaluate unknown operator '%d' at %d in '%s'",
                      oper, operPos, operLine);
              result = false;
              break;
          }
          evalud = result;
        }
      }
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVarCalcOper::evaluateV()
{
  bool result;
  bool isOK;
  //
  result = evalud;
  if (not evalud)
  {
    if (oper == -1)
    { //is the root, it has a right value only
      result = right != NULL;
      if (result)
        result = right->evaluateV();
      evalud = result;
      if (evalud)
      { // evaluated without error
        value = right->value;
        var = right->var;
        varLocal = false;
      }
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
          evaluateMinusV();
        }
        if (not left->evalud)
          result = left->evaluateV();
        if (result and not right->evalud)
          result = right->evaluateV();
        if (result)
        { // evaluate this expression
          if (var == NULL or not varLocal)
          {
            var = new UVariable(left->var);
            varLocal = true;
          }
          isOK = true;
          switch (oper)
          {
            case 1: isOK = var->setGt(left->var, right->var); break;
            case 2: isOK = var->setGe(left->var, right->var); break;
            case 3: isOK = var->setLt(left->var, right->var); break;
            case 4: isOK = var->setLe(left->var, right->var); break;
            case 6: isOK = var->setNe(left->var, right->var); break;
            case 8: isOK = var->setEq(left->var, right->var); break;
            case 9: isOK = var->setPlus(left->var, right->var); break;
            case 10: isOK = var->setProduct(left->var, right->var); break;
            case 11: isOK = var->setDiff(left->var, right->var); break;
            case 12: isOK = var->setDivide(left->var, right->var); break;
            case 13: isOK = var->setBinaryAnd(left->var, right->var); break; // binary and
            case 14: isOK = var->setLogicalAnd(left->var, right->var); break; // logical and
            case 15: isOK = var->setBinaryOr(left->var, right->var); break; // binary or
            case 16: isOK = var->setLogicalOr(left->var, right->var); break; // logical or
            case 17: isOK = var->setBinaryShiftRight(left->var, right->var); break; // bit shift right
            case 18: isOK = var->setBinaryXor(left->var, right->var); break; // xor
            case 19: isOK = var->setBinaryShiftLeft(left->var, right->var); break; // bit shift left
            case 20: isOK = var->setModulo(left->var, right->var); break; // modulo
            case 21: isOK = var->setPow(left->var, right->var); break; // power (not an 'xor' that can be obtained by characters only)
            default:
              fprintf(stderr,
                      "UVarCalcOper::evaluate unknown operator '%d' at %d in '%s'\n",
                      oper, operPos, operLine);
              result = false;
              break;
          }
          if (not isOK)
          {
            fprintf(stderr,
                    "UVarCalcOper::operator value mismatch '%d' at %d in '%s'\n",
                    oper, operPos, operLine);
            result = false;
          }
          evalud = result;
        }
      }
    }
  }
  return result;
}

////////////////////////////////////////////////////

const char * UVarCalcOper::getOperStr()
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

bool UVarCalcOper::evaluateOperator(const char * line, const char * start,
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
  //snprintf(errorTxt, errorTextCnt, "No error");
  p2 = start;
  operLine = line;
  // evaluate the back part of expression
  // find operator - >=, <= != < >
  oper = 0;
  while (isspace(*p2))
    p2++;
  p3 = p2;
  //
  if (isRem(p2))
    isOK = false;
  else
  { // save start of operator
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
                "UVarCalc::evaluate unknown operator at %d in '%s'",
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
  /*        snprintf(errorTxt, errorTextCnt,
                  "UVarCalc::unknown operator '%c' at %d in '%s'",
                  *p2, p1 - line, line);*/
          break;
      }
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
               "invalid placed '!' at %d in '%s'",
               p1 - line, line);
    }
    else if (oper == 7)
    { // an '=' (expected to be a ==) not strictly legal
      oper = 8;
      snprintf(errorTxt, errorTextCnt,
               "invalid placed '=' (changed to '==') at %d in '%s'",
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

void UVarCalcOper::setValue(double val, const char * line, const char * start)
{
  value = val;
  evalud = true;
  priority = 14;
  operLine = line;
  operPos = start-line;
}

////////////////////////////////////////////////////

void UVarCalcOper::setValue(UVariable * val, bool asLocal, const char * line, const char * start)
{
  if (asLocal)
  { // take ownership of this variable
    if (var != NULL and varLocal)
      delete var;
    var = val;
    varLocal = true;
  }
  else
  {
    if (var == NULL or not varLocal)
    {
      var = new UVariable();
      varLocal = true;
    }
    var->copy(val, false);
  }
  evalud = true;
  priority = 14;
  operLine = line;
  operPos = start-line;
}

////////////////////////////////////////////////////

// void UVarCalcOper::setValue(UVariable * val, const char * line, const char * start)
// {
// /*  if (var != NULL)
//     delete var;
//   var = val; */
//   evaluated = true;
//   priority = 14;
//   operLine = line;
//   operPos = start-line;
// }

////////////////////////////////////////////////////

void UVarCalcOper::insertOperator(UVarCalcOper * oper)
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

void UVarCalcOper::print(const char * preStr)
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
    if (evalud)
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

bool UVarCalcOper::isRem(const char * r)
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

////////////////////////////////////////////////////

void UVarCalcOper::negateValue()
{
  if (not varLocal)
  {
    var = new UVariable(var);
    varLocal = true;
  }
  var->negate();
}

////////////////////////////////////////////////////

void UVarCalcOper::inverseValue()
{
  if (not varLocal)
  {
    var = new UVariable(var);
    varLocal = true;
  }
  var->inverse();
}

////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////


UVarCalc::UVarCalc()
{
  errorTxt[0] = '\0';
}

////////////////////////////////////////////////////

UVarCalc::~UVarCalc()
{
  //UVarPool * vp;
  // remove methods implemented by this resource
  // but leave the structures (math)
  //vp = getRootVarPool();
/*  if (vp != NULL)
    vp->deleteMethods(this);*/
}

////////////////////////////////////////////////////

// bool UVarCalc::addMathMethods()
// {
//   UVarPool * vp, *vpm;
//   int n = -1;
//   //
//   vp = getRootVarPool();
//   if (vp != NULL)
//   {
//     vpm =  vp->getStruct("math");
//     if (vpm == NULL)
//       vpm = vp->addStructLocal("math", "global constants and methods");
//     // global constants
//     vpm->addVar("false", 0.0, "d", "Constant false is the same as 0");          //
//     vpm->addVar("true", 1.0, "d", "Constant true is the same as 1");          //
//     vpm->addVar("pi", M_PI, "d", "Constant pi is the same as 3.14159...");
//     // global methods
//     vpm->addMethod(this, "limitToPi", "d", "Limits the value to within [+Pi..-Pi[");
//     vpm->addMethod(this, "sin", "d", "");
//     vpm->addMethod(this, "cos", "d", "");
//     vpm->addMethod(this, "hypot", "dd", "");
//     vpm->addMethod(this, "acos", "d", "");
//     vpm->addMethod(this, "asin", "d", "");
//     vpm->addMethod(this, "tan", "d", "");
//     vpm->addMethod(this, "atan", "d", "");
//     vpm->addMethod(this, "atan2", "dd", "Returns angle within +Pi..-Pi first parameter is y second is x");
//     vpm->addMethod(this, "sqrt", "d", "");
//     vpm->addMethod(this, "sqr", "d", "Returns value*value");
//     vpm->addMethod(this, "abs", "d", "Returns the unsigned (positive) value of parameter");
//     vpm->addMethod(this, "max", "dd", "Returns minimum of two values");
//     vpm->addMethod(this, "min", "dd", "Returns maximum of two values");
//     n = vpm->addMethod(this, "defined", "s", "Returns '1' if a variable or structure with name 's' is defined (global scope)");
//   }
//   return (n > 0);
// }

////////////////////////////////////////////////////


//////////////////////////////////////////////

bool UVarCalc::evaluateParametersD(const char * sourceLine, // source line (for error reporting)
                       const char ** paramStart,   // position of parameter start i.e. an '('
                       double * paramValues,      // array of double parameters
                       int * paramValuesCnt,   // count of double sized parameters found
                       char ** paramStr,          // array of string parameters
                       char * paramOrder,         // order - e.g. "ds" for (double, string)
                       int maxParamCount,         // max size of parameter array and double array
                       int maxParamStrCount,
                       int maxParamStrLength)      // max size of string param list
{
  bool result = true;
  const char * p1 = *paramStart;
  const char * p2 = p1;
  bool isOK;
  int strCnt = 0;
  int parCnt = 0;
  int dblCnt = 0;
  bool inBracket;
  //
  inBracket = (*p1 == '(');
  if (inBracket)
    p1++; // advance past open bracket
  while(result)
  { // test if there is a parameter
    while (isspace(*p1))
      p1++;
    if ((*p1 == '"') or (*p1 == '\''))
    {
      isOK = evaluateToStringD(sourceLine, p1, &p2,
                              paramStr[strCnt], maxParamStrLength);
      if (isOK and (strCnt < maxParamStrCount) and (parCnt < (maxParamCount - 1)))
      {
        paramOrder[parCnt++] = 's';
        strCnt++;
      }
    }
    else if (not inBracket and isRemark(p1))
    { // end of parameter list - the rest is a remark
      // zero terminate
      paramOrder[parCnt] = '\0';
      break;
    }
    else
    { // an expected double value
      paramValues[dblCnt] = evaluateD(sourceLine, p1, &p2, &isOK, false);
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
    // expecting parameter separator
    if (*p2 == ',')
      p1 = p2 + 1;
    else if (inBracket and *p2 == ')')
    { // advance past close bracket
      p2++;
      break;
    }
    else if (not inBracket)
      break;
    else
    { // syntax error - no end bracket
      result = false;
      snprintf(errorTxt, MAX_ERROR_SIZE,
               "Expected end of parameter list, but found '%c' at %d in %s",
               *p1, p2 - sourceLine, sourceLine);
    }
  }
  // save end of parameter pointer
  *paramStart = p2;
  // and count of parameters
  if (paramValuesCnt != NULL)
    *paramValuesCnt = dblCnt;
  return result;
}

//////////////////////////////////////////////

bool UVarCalc::evaluateParametersV(const char * sourceLine, // source line (for error reporting)
                           const char ** paramStart, // position of parameter start i.e. an '('
                           char * paramOrder,        // order - e.g. "ds" for (double, string)
                           UVariable * params[],     // array of parameter variables
                           int * paramsCnt,          // count of parameters found
                           int maxParamsCnt)         // max number of parameters
{
  bool result = true;
  const char * p1 = *paramStart;
  const char * p2 = p1;
  bool isOK;
  int parCnt = 0;
  bool inBracket;
  const int MSL = 500;
  char s[MSL];
  //
  inBracket = (*p1 == '(');
  if (inBracket)
    p1++; // advance past open bracket
  while(result)
  { // test if there is a parameter
    while (isspace(*p1))
      p1++;
    if ((*p1 == '"') or (*p1 == '\''))
    {
      isOK = evaluateToStringV(sourceLine, p1, &p2, s, MSL);
      if (isOK and (parCnt < (maxParamsCnt - 1)))
      {
        if (params[parCnt] == NULL)
          params[parCnt] = new UVariable();
        params[parCnt]->setValues(s, 0, true);
        paramOrder[parCnt++] = 's';
      }
    }
    else if (not inBracket and isRemark(p1))
    { // end of parameter list - the rest is a remark
      // zero terminate
      paramOrder[parCnt] = '\0';
      break;
    }
    else
    { // an expected double value
      if (params[parCnt] == NULL)
        params[parCnt] = new UVariable();
      isOK = evaluateV(sourceLine, p1, &p2, params[parCnt], false);
      if (isOK and (parCnt < (maxParamsCnt - 1)))
      {
        if (params[parCnt]->isDouble())
        { // is a double, or an array of doubles
          if (params[parCnt]->getElementCnt() > 1)
            paramOrder[parCnt] = 'c';
          else
            paramOrder[parCnt] = 'd';
        }
        else
        { // is a string, but may be longer
          paramOrder[parCnt] = 's';
          // get the rest of the string
/*          p1 = p2;
          isOK = evaluateToStringV(sourceLine, p1, &p2, s, MSL);
          if (strlen(s) > 0)
            // add the rest
            params[parCnt]->setValues(s, params[parCnt]->getElementCnt(), true);*/
        }
        parCnt++;
      }
    }
    // zero terminate
    paramOrder[parCnt] = '\0';
    // skip optional white space
    while (isspace(*p2))
      p2++;
    // expecting parameter separator
    if (*p2 == ',')
      p1 = p2 + 1;
    else if (inBracket and *p2 == ')')
    { // advance past close bracket
      p2++;
      break;
    }
    else if (not inBracket)
      break;
    else
    { // syntax error - no end bracket
      result = false;
      snprintf(errorTxt, MAX_ERROR_SIZE,
               "Expected end of parameter list, but found '%c' at %d in %s",
               *p1, p2 - sourceLine, sourceLine);
    }
  }
  // save end of parameter pointer
  *paramStart = p2;
  // and count of parameters
  if (paramsCnt != NULL)
    *paramsCnt = parCnt;
  return result;
}

//////////////////////////////////////////////

bool UVarCalc::evaluateToStringD(const char * sourceLine, // source line (for error reporting)
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
        value = evaluateD(sourceLine, &p1[1], &p3, &isOK, false);
        if (*p3 == ')')
        { // could be evaluated to next end bracket
          gotValue = true;
          p2 = p3 + 1;
        }
      }
      if (not gotValue)
      { // try other options
        value = evaluateD(sourceLine, p1, &p2, &result, false);
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
      if ((*p1 == ',') or (*p1 == ')') or isRemark(p1))
        // reached a parameter separator, so end here
        break;
    }
  }
  if (true)
  { // terminate string
    dest[n] = '\0'; // found string (or until error)
    *nextChar = p1; // save next (or offending) character
  }
  return result and allEval;
}

///////////////////////////////////////////////////////

bool UVarCalc::evaluateToStringV(const char * sourceLine, // source line (for error reporting)
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
  //bool isOK;
  char quote;
  bool isStringP1P2;
  bool gotValue;
  bool allEval = true;
  UVariable value;
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
        evaluateV(sourceLine, &p1[1], &p3, &value, false);
        if (*p3 == ')')
        { // could be evaluated to next end bracket
          gotValue = true;
          p2 = p3 + 1;
        }
      }
      if (not gotValue)
      { // try other options
        result = evaluateV(sourceLine, p1, &p2, &value, false);
        if (p2 == p1 and (*p2 != ')'))
        { // the last and ^^^^^^^^^^ is a workaraund to avoid errors in string parameter
          snprintf(errorTxt, MAX_ERROR_SIZE,
                    "Evaluate to string error near '%c' at %d",
                    *p1, p2 - sourceLine);
          result = false;
        }
        else if (result)
          gotValue = true;
      }
    }
    if (gotValue)
    { // send value to destination
      if ((n + 12) < destLength)
      { // space for value - print
        value.getValuesAsString(&dest[n], destLength - n, 0);
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
      if ((*p1 == ',') or (*p1 == ')') or isRemark(p1))
        // reached a parameter separator, so end here
        break;
    }
  }
  if (true)
  { // terminate string
    dest[n] = '\0'; // found string (or until error)
    *nextChar = p1; // save next (or offending) character
  }
  return result and allEval;
}

////////////////////////////////////////////////////////////////

bool UVarCalc::getValueAnyD(UVarPool * scope, const char * name, double * value, const char ** params)
{
  bool result = false;
  const char *p1;
  int n;
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
        //parsCnt = 0;
        parsOrder[0] = '\0';
        for (n = 0; n < MAX_STR_PARS; n++)
          parsStr[n] = paramStr[n];
        result = evaluateParametersD(*params, &p1, pars, NULL,
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
      vpScope = vpScope->getParentVarPoolScope();
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

bool UVarCalc::getValueAnyV(UVarPool * scope, const char * name, UVariable * value, const char ** params)
{
  bool result = false;
  const char *p1;
  int parsCnt, n;
  const int MAX_PARS = 10;
  UVariable * pars[MAX_PARS];
  UPose pose;
  // string with parameter order - e.g. "dsd" (double, string, double)
  char parsOrder[MAX_PARS];
  // method pointer
  UVarPool *vpScope;
  bool isFunc;
  bool lastScope = false;
  UVariable * val = value;
  UDataBase * bval[1] = {val};
  int i, elem;
  bool delPars = false;
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
        for (n = 0; n < MAX_PARS; n++)
          pars[n] = NULL;
        delPars = true; // remember to del pars
        result = evaluateParametersV(*params, &p1, parsOrder,
                                      pars, &parsCnt, MAX_PARS);
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
    if (val == NULL)
      n = 0; // no return value required
    else
      n = 1;
    while (vpScope != NULL)
    {
      if (isFunc)
      { // get value from function call
        result = vpScope->callLocalV(name, parsOrder, pars, bval, &n);
      }
      else if (value != NULL)
      { // get value as a variable
        val = vpScope->getLocalVariable(name, &elem);
        result = val != NULL;
        if (result)
        { // copy value of desired element to returned variable
          if (strchr(name, '[') == NULL)
            // no index, so return everything - string or array
            value->copy(val, false);
          else if (val->isString())
            // return substring
            value->setValues(val->getValues(elem), 0, true);
          else
            // return one element only
            value->setValued(val->getValued(elem), 0, true);
        }
      }
      if (result)
        // value is found
        break;
      if (lastScope)
        // we are finished
        break;
      // move up one scope level
      vpScope = vpScope->getParentVarPoolScope();
      if (vpScope == NULL)
      {
        vpScope = scope->getRootVarPool()->getStruct("math");
        lastScope = true;
      }
    }
    // release created parameter variables
    if (delPars)
    {
      for (i = 0; i < MAX_PARS; i++)
      {
        if (pars[i] != NULL)
          delete pars[i];
        else
          break;
      }
    }
  }
  return result;
}

/////////////////////////////////////////////////////////

bool UVarCalc::getVariableAny(UVarPool * scope, const char * name, UVariable * value)
{
  bool result = false;
  UVarPool *vpScope;
  bool lastScope = false;
  UVariable * val = value;
  //
  // try first an ordinary named value in calculator
  // it may be a local variable (a parameter) in a function
  vpScope = scope;
  // test for values in scopes
  // from local scope back to root, and then math scope
  while (vpScope != NULL)
  {
    if (value != NULL)
    { // get value as a variable
      val = vpScope->getLocalVariable(name, NULL);
      result = val != NULL;
      if (result)
        value->copy(val, true);
    }
    if (result)
      // value is found
      break;
    if (lastScope)
      // we are finished
      break;
    // move up one scope level
    vpScope = vpScope->getParentVarPoolScope();
    if (vpScope == NULL)
    {
      vpScope = scope->getRootVarPool()->getStruct("math");
      lastScope = true;
    }
  }
  return result;
}

//////////////////////////////////////////////////////////////////////

bool UVarCalc::getVariableRefAny(UVarPool * scope, const char * name, UVariable ** found, int * nameIndex)
{
  bool result = false;
  UVarPool *vpScope;
  bool lastScope = false;
  UVariable * val;
  //
  // try first an ordinary named value in calculator
  // it may be a local variable (a parameter) in a function
  vpScope = scope;
  // test for values in scopes
  // from local scope back to root, and then math scope
  while (vpScope != NULL)
  { // get value as a variable
    val = vpScope->getLocalVariable(name, nameIndex);
    result = val != NULL;
    if (result and found != NULL)
      *found = val;
    if (result)
      // value is found
      break;
    if (lastScope)
      // we are finished
      break;
    // move up one scope level
    vpScope = vpScope->getParentVarPoolScope();
    if (vpScope == NULL)
    {
      vpScope = scope->getRootVarPool()->getStruct("math");
      lastScope = true;
    }
  }
  return result;
}

/////////////////////////////////////////////////////////

// bool UVarCalc::getValueAny(UVarPool * scope, const char * name, UVariable * value, const char ** params)
// {
//   bool result = false;
//   const char *p1;
//   int parsCnt, n;
//   const int MAX_PARS = 10;
//   double pars[MAX_PARS];
//   UPose pose;
//   // string parameters
//   const int MAX_STR_PARS = 3;
//   const int MSPL = 1000;
//   char paramStr[MAX_STR_PARS][MSPL];
//   char * parsStr[MAX_STR_PARS];
//   // string with parameter order - e.g. "dsd" (double, string, double)
//   char parsOrder[MAX_PARS];
//   // method pointer
//   UVarPool *vpScope;
//   bool isFunc;
//   bool lastScope = false;
//   double v;
//   UDataBase * returnVar;
//   UVariable * lv;
//   //
//   // try first an ordinary named value in calculator
//   // it may be a local variable (a parameter) in a function
//   vpScope = scope;
//   returnVar = value;
//   if (vpScope != NULL)
//   { // test if it is a function call with parameters
//     isFunc = false;
//     if (params != NULL)
//     { // test if it is a function with parameters
//       // test for function
//       // 1. get parameters, 2. call function, 3. return value
//       p1 = *params;
//       while (isspace(*p1))
//         p1++;
//       if (*p1 == '(')
//       { // a value-parameter function
//         parsCnt = 0;
//         parsOrder[0] = '\0';
//         for (n = 0; n < MAX_STR_PARS; n++)
//           parsStr[n] = paramStr[n];
//         result = evaluateParameters(*params, &p1, pars, NULL,
//                                      parsStr, parsOrder,
//                                      MAX_PARS, MAX_STR_PARS, MSPL);
//         if (result)
//         { // all parameters are evaluated
//           // return end position of parameters
//           // -- first character after end-bracket
//           *params = p1;
//           isFunc = true;
//         }
//       }
//     }
//     //
//     // now test for values in scopes
//     // from local scope back to root, and then math scope
//     while (vpScope != NULL)
//     {
//       if (isFunc)
//       { // get value from function call
//         n = 1;
//         result = vpScope->callLocal(name, parsOrder, parsStr, pars, &v, &returnVar, &n);
//       }
//       else
//       { // get value as a variable
//         lv = vpScope->getLocalVariable(name);
//         result = (lv != NULL);
//         if (result and value != NULL)
//           value->copy(lv);
//       }
//       if (result)
//         // value is found
//         break;
//       if (lastScope)
//         // we are finished
//         break;
//       // move up one scope level
//       vpScope = vpScope->getParentVarPoolScope();
//       if (vpScope == NULL)
//       {
//         vpScope = scope->getRootVarPool()->getStruct("math");
//         lastScope = true;
//       }
//     }
//   }
//   return result;
// }

/////////////////////////////////////////////////////////

// bool UVarCalc::methodCall(const char * name, const char * paramOrder,
//                        char ** strings, const double * pars,
//                        double * value,
//                        UDataBase ** returnStruct,
//                        int * returnStructCnt)
// { // implement method call from math
//   bool result = true;
//   // evaluate standard functions
//   if ((strcasecmp(name, "limitToPi") == 0) and (strcmp(paramOrder, "d") == 0))
//     *value = limitToPi(pars[0]);
//   else if ((strcasecmp(name, "sin") == 0) and (strcmp(paramOrder, "d") == 0))
//     *value = sin(pars[0]);
//   else if ((strcasecmp(name, "hypot") == 0) and (strcmp(paramOrder, "dd") == 0))
//     *value = hypot(pars[0], pars[1]);
//   else if ((strcasecmp(name, "cos") == 0) and (strcmp(paramOrder, "d") == 0))
//     *value = cos(pars[0]);
//   else if ((strcasecmp(name, "acos") == 0) and (strcmp(paramOrder, "d") == 0))
//     *value = acos(pars[0]);
//   else if ((strcasecmp(name, "asin") == 0) and (strcmp(paramOrder, "d") == 0))
//     *value = asin(pars[0]);
//   else if ((strcasecmp(name, "tan") == 0) and (strcmp(paramOrder, "d") == 0))
//     *value = tan(pars[0]);
//   else if ((strcasecmp(name, "atan") == 0) and (strcmp(paramOrder, "d") == 0))
//     *value = atan(pars[0]);
//   else if ((strcasecmp(name, "sqrt") == 0) and (strcmp(paramOrder, "d") == 0))
//     *value = sqrt(pars[0]);
//   else if ((strcasecmp(name, "sqr") == 0) and (strcmp(paramOrder, "d") == 0))
//     *value = sqr(pars[0]);
//   else if ((strcasecmp(name, "abs") == 0) and (strcmp(paramOrder, "d") == 0))
//     *value = fabs(pars[0]);
//   else if ((strcasecmp(name, "max") == 0) and (strcmp(paramOrder, "dd") == 0))
//     *value = maxd(pars[0], pars[1]);
//   else if ((strcasecmp(name, "min") == 0) and (strcmp(paramOrder, "dd") == 0))
//     *value = mind(pars[0], pars[1]);
//   else if ((strcasecmp(name, "atan2") == 0) and (strcmp(paramOrder, "dd") == 0))
//     *value = atan2(pars[0], pars[1]);
//   else if ((strcasecmp(name, "defined") == 0) and (strcmp(paramOrder, "s") == 0))
//   { // is this variable defined
//     result = isDefined(getVarPool(), strings[0]);
//     *value = double(result);
//     result = true;
//   }
//   else
//     result = false;
//   if (returnStructCnt != NULL)
//     *returnStructCnt = 0;
//   return result;
// }

////////////////////////////////////////////////////////////////

bool UVarCalc::isDefined(UVarPool * scope, const char * varName)
{
  bool result;
  UVarPool *vpScope;
  bool lastScope;
  const int MSL = 32;
  char s[MSL];
  UVariable v;
  //
  vpScope = scope;
  // try first a variable
  result = getVariableAny(scope, varName, &v);
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
      vpScope = vpScope->getParentVarPoolScope();
      if (vpScope == NULL)
      { // add the math
        vpScope = getRootVarPool()->getStruct("math");
        lastScope = true;
      }
    }
  }
  return result;
}

////////////////////////////////////////////

bool UVarCalc::getLocalScopeVaiable(const char * name, double * value)
{
  return false;
}

//////////////////////////////////////////////

// const char * UVarCalc::getGenFunctionHelp(const int index, int * count)
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

double UVarCalc::evaluateUnaryValD(const char * line, const char * start,
                            const char ** end, bool * evaluated, bool syntaxCheck)
{ // on return 'end' is first unused character
  double v1 = 0.0, result;
  const char *p1, *p2;
  bool isOK = true;
  const int MNL = MAX_VARIABLE_NAME_SIZE * 20;
  char name[MNL];
  int n;
  double vidx;
  //
  p1 = start;
  while (isspace(*p1))
    p1++;
  // may have negation, '-', or is a sub expression
  if ((strncmp(p1, "not", 3) == 0) and not isalnum(p1[3]))
  { // unary not
    p1 += 3;
    v1 = not evaluateUnaryValD(line, p1, &p2, &isOK, syntaxCheck);
  }
  else if (*p1 == '!')
  { // unary ! boolean not
    v1 = ! evaluateUnaryValD(line, ++p1, &p2, &isOK, syntaxCheck);
  }
  else if (*p1 == '-')
  { // negative unary operator
    v1 = -evaluateUnaryValD(line, ++p1, &p2, &isOK, syntaxCheck);
  }
  else if (*p1 == '+')
  { // positive unary operator
    v1 = evaluateUnaryValD(line, ++p1, &p2, &isOK, syntaxCheck);
  }
  else if (*p1 == '(')
  { // sub expression
    v1 = evaluateD(line, ++p1, &p2, &isOK, syntaxCheck);
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
    n = p2 - p1;
    strncpy(name, p1, n);
    name[n] = '\0';
    // test for array index
    while (isspace(*p2))
      p2++;
    if (*p2 == '[')
    { // there is an index
      p1 = p2 + 1;
      vidx = evaluateD(line, p1, &p2, &isOK, syntaxCheck);
      if (*p2 != ']')
        snprintf(errorTxt, MAX_ERROR_SIZE,
                 "***eval: index error at %d in '%s'",
                 p1 - line, line);
      else
      { // build the full variable name to find, including index
        snprintf(&name[n], MNL - n, "[%d]", roundi(vidx));
        // move past the close bracket
        p2++;
      }
    }
    isOK = getValueAnyD(this, name, &v1, &p2);
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

bool UVarCalc::evaluateUnaryValV(const char * line, const char * start,
                                  const char ** end, UVariable * value, bool syntaxCheck)
{ // on return 'end' is first unused character
  UVariable *v1 = value;
  const char *p1, *p2;
  bool isOK = true;
  const int MNL = MAX_VARIABLE_NAME_SIZE * 20;
  char name[MNL];
  int n;
  double v, vidx;
  const int MSL = 1000;
  char s[MSL];
  //
  p1 = start;
  while (isspace(*p1))
    p1++;
  //v1 = new UVariable();
  if (v1 != NULL)
  {
    // may have negation, '-', or is a sub expression
    if ((strncmp(p1, "not", 3) == 0) and not isalnum(p1[3]))
    { // unary not
      p1 += 3;
      isOK = evaluateUnaryValV(line, p1, &p2, v1, syntaxCheck);
      v1->setNot();
    }
    else if (*p1 == '!')
    { // unary ! boolean not
      isOK = evaluateUnaryValV(line, ++p1, &p2, v1, syntaxCheck);
      v1->setNot();
    }
    else if (*p1 == '-')
    { // negative unary operator
      isOK = evaluateUnaryValV(line, ++p1, &p2, v1, syntaxCheck);
      v1->negate();
    }
    else if (*p1 == '+')
    { // positive unary operator
      isOK = evaluateUnaryValV(line, ++p1, &p2, v1, syntaxCheck);
    }
    else if (*p1 == '(')
    { // sub expression
      isOK = evaluateV(line, ++p1, &p2, v1, syntaxCheck);
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
      n = p2 - p1;
      strncpy(name, p1, n);
      name[n] = '\0';
      // test for array index
      while (isspace(*p2))
        p2++;
      if (*p2 == '[')
      {
        p1 = p2 + 1;
        isOK = evaluateV(line, p1, &p2, v1, syntaxCheck);
        vidx = v1->getValued();
        if (*p2 != ']')
          snprintf(errorTxt, MAX_ERROR_SIZE,
                  "***eval: index error at %d in '%s'",
                  p1 - line, line);
        else
        {
          snprintf(&name[n], MNL - n, "[%d]", roundi(vidx));
          // advance end pointer past close bracket
          p2++;
        }
      }
      // potential parameters start at p2, and p2 is after parameters at return
      isOK = getValueAnyV(this, name, v1, &p2);
      if (not isOK)
      { // write error text
        snprintf(errorTxt, MAX_ERROR_SIZE,
                "***eval: unknown '%s' at %d in '%s'",
                name, p1 - line, line);
      }
      else if (v1->isString())
      { // the result value is found - return as is
        //p1 = p2;
        //isOK = evaluateToStringV(line, p1, &p2, s, MSL);
        //if (isOK)
        //  v1->setValues(s, v1->getElementCnt(), true);
        //else
          // no more string is OK too
        // debug
          isOK = true;
        // debug end
      }
    }
    else
    { // must be a constant (or an error)
      v = strtod(p1, (char **)&p2);
      isOK = (p1 != p2);
      if (isOK)
        v1->setValued(v, 0, true);
      else if (*p1 == '"' or *p1 == '\'')
      { // variable value is a string
        isOK = evaluateToStringV(line, p1, &p2, s, MSL);
        v1->setTypeChar("s");
        v1->setValues(s, 0, true);
      }
    }
  }
  // return results end next unused character
  if (end != NULL)
    *end = p2;
  return isOK;
}

/////////////////////////////////////////////

// UVariable * UVarCalc::evalUnaryVal(const char * line, const char * start,
//                                   const char ** end, bool * evaluated, bool syntaxCheck)
// { // on return 'end' is first unused character
//   UVariable *v1 = NULL, *result;
//   const char *p1;
//   const char *p2;
//   char *p3;
//   bool isOK = true;
//   char name[MAX_VARIABLE_NAME_SIZE];
//   //
//   p1 = start;
//   while (isspace(*p1))
//     p1++;
//   // may have negation, '-', or is a sub expression
//   if ((strncmp(p1, "not", 3) == 0) and not isalnum(p1[3]))
//   { // unary not
//     p1 += 3;
//     v1 = evalUnaryVal(line, p1, &p2, &isOK, syntaxCheck);
//     if (v1 != NULL)
//       v1->negateBool(0);
//   }
//   else if (*p1 == '!')
//   { // unary ! boolean not
//     v1 = evalUnaryVal(line, ++p1, &p2, &isOK, syntaxCheck);
//   }
//   else if (*p1 == '-')
//   { // negative unary operator
//     v1 = evalUnaryVal(line, ++p1, &p2, &isOK, syntaxCheck);
//     if (v1 != NULL)
//       v1->negated(0);
//   }
//   else if (*p1 == '+')
//   { // positive unary operator
//     v1 = evalUnaryVal(line, ++p1, &p2, &isOK, syntaxCheck);
//   }
//   else if (*p1 == '(')
//   { // sub expression
//     v1 = eval(line, ++p1, &p2, &isOK, syntaxCheck);
//     if (isOK)
//     { // found value, now expect close bracket
//       isOK = (*p2 == ')');
//       if (isOK)
//         p2++;
//       else
//         snprintf(errorTxt, MAX_ERROR_SIZE,
//                  "***eval: expected ')' but found '%c' at %d in '%s'",
//                  *p2, p2 - line, line);
//     }
//   }
//   else if (isalpha(*p1))
//   { // is either a boolean or a double symbol.
//     // find end of symbol
//     p2 = p1 + 1;
//     while (isalnum(*p2) or (*p2 == '_') or (*p2 == '.'))
//       p2++;
//     // must be a variable or a constant
//     strncpy(name, p1, p2 - p1);
//     name[p2 - p1] = '\0';
//     isOK = getValueAny(this, name, v1, &p2);
//     if (not isOK)
//     { // write error text
//       snprintf(errorTxt, MAX_ERROR_SIZE,
//                "***eval: unknown '%s' at %d in '%s'",
//                name, p1 - line, line);
//     }
//   }
//   else
//   { // must be a constant (or an error)
//     v1 = new UVariable();
//     v1->setType(UVariable::d);
//     v1->setValued(strtod(p1, &p3), 0);
//     isOK = (p1 != p3);
//   }
//   // return results
//   result = v1;
//   if (end != NULL)
//     *end = p2;
//   if (evaluated != NULL)
//     *evaluated = isOK;
//   return result;
// }

/////////////////////////////////////////////

// double UVarCalc::evaluateOld(const char * line, const char * start,
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

bool UVarCalc::evaluateV(const char * line, const char * start,
                       const char ** end, UVariable * value, bool syntaxCheck)
{ // expect a line as
  // "(roadLeft < 20.3 or double[7] > double[6])"
  // may return false or true, when a ')' or eol is reached
  const char *p1, *p2;
  const char *p3, *p4;
  UVariable v1, v2;
  bool isOK;
  bool finished = false;
  const int MOC = 20;
  UVarCalcOper oos[MOC];
  int oosCnt = 0, i;
  UVarCalcOper *oV, *oO, *oRoot;
  const int MSL = 10000;
  char s[MSL];
  //
  errorTxt[0] = '\0';
  p1 = start;
  oRoot = &oos[oosCnt++];
  //strncpy(oRoot->operStr, "root", 5);
  // debug
  for (i = 0; i < MOC; i++)
    oos[i].num = i;
  // debug end
  //
  while(isspace(*p1))
    p1++;
  isOK = evaluateUnaryValV(line, p1, &p2, &v1, syntaxCheck);
  // save potential end character pointer
  p4 = p2;
  // may be a string
  if (isOK and v1.isString())
  {
    if (value != NULL)
      value->copy(&v1, false);
    s[0] = '\0';
    while (isspace(*p2))
      p2++;
    p1 = p2;
    if (isalnum(*p1) or *p1 == '\'' or *p1 == '"' or *p1 == '+' or *p1 == '-')
      // next may be a sontinued concatenated string
      isOK = evaluateToStringV(line, p1, &p2, s, MSL);
    if (isOK and strlen(s) > 0)
      // add the rest
      value->setValues(s, value->getElementCnt(), true);
    else
      // no more string is OK too
      isOK = true;
    if (end != NULL)
      *end = p2;
  }
  else
  {
    // do not let operator keep this v1 value
    oV = &oos[oosCnt++];
    oV->setValue(&v1, false, line, p2);
    oV->parent = oRoot;
    oRoot->right = oV;
    // evaluate the back part of expression
    while (isOK and not finished and oosCnt < MOC - 2)
    { // find operator - >=, <= != < > ...
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
        isOK = evaluateUnaryValV(line, p3, &p4, &v2, syntaxCheck);
        if (isOK)
        {
          oV = &oos[oosCnt++];
          oV->setValue(&v2, false, line, p3);
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
    { // OK, now find value of expression
      //oRoot->print("pre eval: ");
      isOK = oRoot->evaluateV();
      //oRoot->print("posteval: ");
    }
    if (end != NULL)
      *end = p4;
    if (isOK and value != NULL)
      value->copy(oRoot->getVar(), false);
  }
  // debug
  // explicit tdelete of var to trap error
  oV = oos;
  for (i = 0; i < oosCnt; i++)
  {
    oV->remVar();
    oV++;
  }
  // debug end
  //
  return isOK;
}

////////////////////////////////////////////////////////////

double UVarCalc::evaluateD(const char * line, const char * start,
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
  UVarCalcOper oos[MOC];
  int oosCnt = 0;
  UVarCalcOper *oV, *oO, *oRoot;

  //
  errorTxt[0] = '\0';
  p1 = start;
  // skip leading space
  oRoot = &oos[oosCnt++];
  oV = &oos[oosCnt++];
  //strncpy(oRoot->operStr, "root", 5);
  //
  while(isspace(*p1))
    p1++;
  v1 = evaluateUnaryValD(line, p1, &p2, &isOK, syntaxCheck);
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
      v2 = evaluateUnaryValD(line, p3, &p4, &isOK, syntaxCheck);
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
    isOK = oRoot->evaluateD();
    //oRoot->print("posteval: ");
    if (isOK)
      result = oRoot->getValue();
    //printf("evaluated (%s) to %g\n", bool2str(isOK), result);
  }
  if (end != NULL)
    *end = p4;
  if (evaluated != NULL)
    *evaluated = isOK;
  return result;
}

//////////////////////////////////////////////////////////

// UVariable *  UVarCalc::eval(const char * line, const char * start,
//                           const char ** end, bool * evaluated, bool syntaxCheck)
// { // expect a line as
//   // "(roadLeft < 20.3 or double[7] > double[6])"
//   // may return false or true, when a ')' or eol is reached
//   double result = 0.0;
//   const char *p1, *p2;
//   const char *p3, *p4;
//   UVariable *v1 = NULL, *v2 = NULL;
//   bool isOK;
//   bool finished = false;
//   const int MOC = 20;
//   UVarCalcOper oos[MOC];
//   int oosCnt = 0;
//   UVarCalcOper *oV, *oO, *oRoot;
//   UVariable * res = NULL;
//   //
//   errorTxt[0] = '\0';
//   p1 = start;
//   // skip leading space
//   oRoot = &oos[oosCnt++];
//   oV = &oos[oosCnt++];
//   //strncpy(oRoot->operStr, "root", 5);
//   //
//   while(isspace(*p1))
//     p1++;
//   v1 = evalUnaryVal(line, p1, &p2, &isOK, syntaxCheck);
//   // save potential end character pointer
//   p4 = p2;
//   oV->setValue(v1, line, p2);
//   oV->parent = oRoot;
//   oRoot->right = oV;
//   // evaluate the back part of expression
//   while (isOK and not finished and oosCnt < MOC - 2)
//   { // find operator - >=, <= != < >
//     oO = &oos[oosCnt++];
//     isOK = oO->evaluateOperator(line, p2, &p3,
//                                 errorTxt, MAX_ERROR_SIZE, syntaxCheck);
//     if (not isOK)
//     { // no more (usefull) data
//       finished = true;
//       isOK = true;
//     }
//     else
//     { // continue with operator and next value
//       oV->insertOperator(oO);
//       // get next value
//       v2 = evalUnaryVal(line, p3, &p4, &isOK, syntaxCheck);
//       if (isOK)
//       {
//         oV = &oos[oosCnt++];
//         oV->setValue(v2, line, p3);
//         oV->parent = oO;
//         oO->right = oV;
//         //oRoot->print("pre ");
//       }
//       else
//       { // print syntax error message
//         snprintf(errorTxt, MAX_ERROR_SIZE,
//                  "***eval: expected a value but found '%c' at %d in '%s'",
//                  p3[0], p3 - line, line);
//       }
//     }
//     p2 = p4;
//   }
//   if (isOK)
//   { // print an error to error text
//     //oRoot->print("pre eval: ");
//     isOK = oRoot->evaluate();
//     //oRoot->print("posteval: ");
//     if (isOK)
//     {
//       result = oRoot->var;
//       oRoot->var = NULL;
//     }
//     //printf("evaluated (%s) to %g\n", bool2str(isOK), result);
//   }
//   if (end != NULL)
//     *end = p4;
//   if (evaluated != NULL)
//     *evaluated = isOK;
//   return result;
// }

///////////////////////////////////////////////////////////////

// bool UVarCalc::getAssignment(const char * cmdLine, const char ** end, char * name, const int nameCnt, double * value)
// {
//   const int MVL = MAX_VAR_NAME_SIZE;
//   const char * p1, * p2 = NULL;
//   int n = 0;
//   bool result;
//   double vald;
//   // format 'set speedMax= 77.88'
//   // format 'set   speedMax = 77.88'
//   // skip the set
//   p1 = cmdLine;
//   while (isspace(*p1))
//     p1++;
//   result = getIdentifier(p1, MVL, "_", &p2);
//   // get identifier length
//   n = p2 - p1;
//   result = n > 0;
//   if (result)
//   { // skip space before '='
//     while (isspace(*p2))
//       p2++;
//     result = (*p2 == '=');
//   }
//   if (not result)
//   {
//     snprintf(errorTxt, MAX_ERROR_SIZE,
//              "***eval:: syntax error, no '=' or length > %d in '%s'\n",
//              MVL, cmdLine);
//   }
//   if (result)
//   { // get variable name
//     n = mini(nameCnt - 1, n);
//     if (name != NULL)
//     {
//       strncpy(name, p1, nameCnt);
//       name[n] = '\0';
//     }
//     // now get value
//     p1 = p2 + 1;
//     while (isspace(*p1))
//       p1++;
//     vald = evaluate(cmdLine, (char *)p1, &p2, &result, false);
//   }
//   if (result)
//   { // set the result, name, value and last character used
//     if (value != NULL)
//       *value = vald;
//     if (end != NULL)
//       *end = p2;
//   }
//   //
//   return result;
// }

///////////////////////////////////////////////////////////////

bool UVarCalc::getIdentifier(const char * id, int idCnt,
                             const char * allowed,
                             const char ** oChar)
{ // is the next n characters a valid identifier
  bool result;
  const char * p1, *p2;
  int i;
  //
  p1 = id;
  result = idCnt > 0 and isalpha(*id);
  if (result)
  {
    p1++;
    for (i = 1; i < idCnt; i++)
    { // test remaining characters
      p2 = strchr(allowed, *p1);
      if (*p1 == '\0' or (p2 == NULL and not isalnum(*p1)))
        break;
      p1++;
    }
  }
  if (oChar != NULL)
    *oChar = p1;
  return result;
}

///////////////////////////////////////////////////////////////

// bool UVarCalc::evaluateSystemFunction(const char * name,
//                           const double pars[], int parsCnt,
//                           double * value, bool syntaxCheck)
// { // no system specific functions here
//   // --- should be overwritten
//   return false;
// }

///////////////////////////////////////////////////////////////

// bool UVarCalc::getSmrdemoValue(const char * name, double * value)
// {
//   return false;
// }

///////////////////////////////////////////////////////////////

bool UVarCalc::isRemark(const char * string)
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

// void UVarCalc::logCalcVars(FILE * logcc)
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


// bool UVarCalc::setResource(UResBase * resource, bool remove)
// {
//   bool result;
//   UVarMethod * vm;
//   //
//   result = UResVarPool::setResource(resource, remove);
//   if (result)
//   {
//     getVarPool()->getRootVarPool()->getLocalMethod("math.sin", "d", &vm);
//     if (vm == NULL)
//     { // math structure not created - do it now
//       addMathMethods();
//     }
//   }
//   return result;
// }
