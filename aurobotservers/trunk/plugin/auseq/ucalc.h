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
#ifndef UCALC_H
#define UCALC_H

#include <stdio.h>

#include <urob4/uvarpool.h>
#include <urob4/uresvarpool.h>

/**
Number of variables in the calculator */
#define MAX_VARIABLES_IN_CALCULATOR 500
/**
Name length of a variable */
//#define MAX_VARIABLE_NAME_SIZE MAX_VAR_NAME_SIZE
/**
Maximum length of an error */
#define MAX_ERROR_SIZE 200


/**
A waiting operator */
class UCalcOper
{
public:
  /** constructor */
  UCalcOper();
  /** destructor */
  ~UCalcOper();
  /** evaluate */
  bool evaluate();
  /**
   * \brief decode an operator
   * \param line is the original line to decode
   * \param start is the position where to search for the operator
   * \param end returns the next unused character after the operator.
   * \returns true if an operator is found.
  */
  bool evaluateOperator(const char * line, const char * start,
               const char ** end,
               char * errorTxt, const int errorTextCnt,
               bool syntaxCheck);
  /** set base values */
  void setValue(double value, const char * line, const char * start);
  /** print the operator status */
  void print(const char * preStr);
  /** insert an new operator relative to this */
  void insertOperator(UCalcOper * oper);

protected:
  /**
   * convert all minus operators to + by changing sign of the first right value
   * or convert division to multiplication  by a 1/value operation */
  void evaluateMinus();
  /**
   * \returns operator as string */
  const char * getOperStr();

public:
  /** Value after evaluation */
  double value;
  /** Evaluated */
  bool evaluated;
  /** Value type 1=bool, 2=int, 3=double */
  int valueType;
  /** Priority 1 boolean, 2 addition, 3 mult, 4 exp */
  int priority;
  /** operator */
  int oper;
  /** operator position in string */
  int operPos;
  /** operation line */
  const char * operLine;
  /** operator string */
  //char operStr[5];
  /** left side value */
  UCalcOper * left;
  /** right side value */
  UCalcOper * right;
  /** parent operator */
  UCalcOper * parent;
};

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////


/**
Class to act as a calculator */
class UCalc : public UResVarPool
{
public:
  /**
  Constructor */
  UCalc();
  /**
  Destructor */
  virtual ~UCalc();
  /**
  Ass standard math methods */
  bool addMathMethods();
  /**
   * \brief Called by server core when new resources are available.
   * \param resource is the offered resource pointer
   * \param remove is true if the resource is unloaded and thuis no longer valid
   * \return true is resouurce is used
   * Save a pointer to the resource as needed. */
  bool setResource(UResBase * resource, bool remove);
  /**
   * \brief Get value for the variable or function call with this name
   * Variables and functions in the local scope has first priority, then
   * gradually one scope at a time until root scope. After that the math scope is tested.
   * \Returns true if found.
   * \param scope the scope to be searched for value (normally just from 'getVarPool()')
   * \param value the found (double) value is returned here (if not NULL).
   * \param params at this position are the parameters for the function, if
   * parameters are used (and can be evaluated) then the character position
   * are modified to just passed the end bracket for the parameter. */
  bool getValueAny(UVarPool * scope, const char * name, double * value, const char ** params);
  /**
   * \brief Evaluate an expression
   * \param line is the source line
   * \param start is where the expression starts in the line
   * \param next points to the first unused character after evaluation
   * \returns a double, that may hold a boolean value.
   * \param evaluated returns false if string fails to evaluate 'evaluated' (if not NULL)
   * \param syntaxCheck when true the evaluation is evaluated partially, as far as to the point where values or actions are initiated  */
  double evaluate(const char * line, const char * start,
                  const char ** end, bool * evaluated, bool syntaxCheck);
  /**
   * \brief Evaluate an expression
   * \param line is the source line
   * \param start is where the expression starts in the line
   * \param next points to the first unused character after evaluation
   * \returns a double, that may hold a boolean value.
   * \param evaluated returns false if string fails to evaluate 'evaluated' (if not NULL)
   * \param syntaxCheck when true the evaluation is evaluated partially, as far as to the point where values or actions are initiated  */
//  double evaluateOld(const char * line, const char * start,
//                  const char ** end, bool * evaluated, bool syntaxCheck);
  /**
  Evaluate a single value, that may have a unary operator in front of the value.
  The syntax may be: ['not' | '-'] (symbol | constant | '(' expression ')' | functionCall ).
  Returns the value found, or 0 on error.
  'line' is full line that is evaluated upon.
  'start' is where to start (on this line).
  'evaluated is set true, if a value is found.
  'end' is set to first unused character.
  'syntaxCheck' is passed on to any expression of functionCall evaluation. */
  double evaluateUnaryVal(const char * line, const char * start,
                       const char ** end, bool * evaluated, bool syntaxCheck);
  /**
  Evaluate a number [0..maxParamCount] of parameters -- for a function call.
  Returns true if not syntax error were found.
  The number of found parameters gets returned in 'paramCnt', and
  the values are present in 'paramValues'.
  Eks.:
  sourceLine: "phi = 23.7 + atan2(y * sin(h), (x - 0.1) * cos(h))", and then
  paramStart: "(y * sin(h), (x - 0.1) * cos(h))"
  REturn will be paramCnt = 2 and paramValue[0] and [1] will be assigned values.
  'paramStart' will point at first character after parameter list - just after the ')' char,
  i.e. no white space check. */
  bool evaluateParameters(const char * sourceLine, // source line (for error reporting)
                                 const char ** paramStart,   // position of parameter start i.e. an '('
                                 double * paramValues,      // array of double parameters
                                 int * paramValuesCnt,   // count of double sized parameters found
                                 char ** paramStr,          // array of string parameters
                                 char * paramOrder,         // order - e.g. "ds" for (double, string)
                                 int maxParamCount,         // max size of parameter array
                                 int maxParamStrCount,      // max size of string param list
                                 int maxParamStrLength);    // max length of a string param
  /**
  Evaluate a string of mixed unknown values and values to evaluate to a new evaluated string,
  that can be send to a subsystem - e.g. a SMRCL command.
  If part of the command is in quotes ("), then it is not evaluated.
  e.g. turnr 1.2 calcH -0.1 '"rad"' @ v speed ":($odoth >" calcH ")"
  and speed=1.2 and calcH=0.5 and there is no value v, then it is evaluated to:
  turnr 1.2 0.4 "rad" @ v 0.5 :($odoth > 0.5)
  Returns true if evaluated and there is space in 'dest' string.
  */
  bool evaluateToString(const char * sourceLine, // source line (for error reporting)
                               const char * paramStart, // position of where to start in source
                               const char ** nextChar,        // first unused character in line
                               char * dest,      // string for resultant evaluated string
                               int destLength);   // max length of destination string
  /**
  Set variable as specified in this line.
  returns true if variable is set with no syntax error, on
  syntax error the errorText() is set with a description.. */
  bool setVariable(const char * cmdLine, bool syntaxCheck);
  /**
  Get error text */
  const char * getErrorTxt()
    { return errorTxt; };
  /**
  Get a (or try to) value from the connected MRC.
  Returns false if variable is unknown or MRC is not connected */
  virtual bool getSmrdemoValue(const char * name, double * value);
  /**
  Is this string to be regarded as a remark line?
  Returns true if so.
  A remark line start with one of "%#;" or "//" */
  bool isRemark(const char * string);
  /**
  Copy values of all variables to logfile */
  void logCalcVars(FILE * logcc);
  /**
  The varPool has methods, and a call to one of these are needed.
  Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed. */
  virtual bool methodCall(const char * name, const char * paramOrder,
                          char ** strings, const double * doubles,
                          double * value,
                          UDataBase ** returnStruct = NULL,
                          int * returnStructCnt = NULL);

protected:
  /**
  Evaluate system specific functions - that need to know
  position of planner etc.
  The called function name is in 'name' the parameters in 'pars'
  and if the function returns a value it
  must be put in 'value' */
  virtual bool evaluateSystemFunction(const char * name,
                          const double pars[], int parsCnt,
                          double * value, bool syntaxCheck);
  /**
  Get value from a local function variable (a parameter).
  Returns true if a variable exist, and then the value
  is returned in 'value' */
  virtual bool getLocalScopeVaiable(const char * name, double * value);
  /**
   * \brief Is this variable defined in the scope.
   * The test is for variables and structures, and in the provided scope
   * as well as any parent scope. further the math scope is tested too.
   * \param scope the scope variable pool at the deepest level.
   * \param varName variable name (or struct name) that may include a '.' for sub-structure.
   * \returns true if found. */
  bool isDefined(UVarPool * scope, const char * varName);

protected:
  /**
  Error string, when an error is detected */
  char errorTxt[MAX_ERROR_SIZE];
};

#endif
