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
#ifndef UVARCALC_H
#define UVARCALC_H

#include <stdio.h>

#include "uvarpool.h"

/**
Number of variables in the calculator */
#define MAX_VARIABLES_IN_CALCULATOR 500
/**
Name length of a variable */
#define MAX_VARIABLE_NAME_SIZE 32
/**
Maximum length of an error */
#define MAX_ERROR_SIZE 200


/**
A waiting operator */
class UVarCalcOper
{
public:
  /** constructor */
  UVarCalcOper();
  /** destructor */
  ~UVarCalcOper();
  /** evaluate using double values */
  bool evaluateD();
  /** evaluate using var values */
  bool evaluateV();
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
  /**
   * Set value for this operator
   * \param val is the source value
   * \param asLocal should operator take resoncibility for deletion of variable?
   * \param line source line for variable
   * \param start position of evaluation */
  void setValue(UVariable * val, bool asLocal, const char * line, const char * start);
  /**
   * Set a UVariable value in this operator structure */
//  void setValue(UVariable * val, const char * line, const char * start);
      /** print the operator status */
  void print(const char * preStr);
  /** insert an new operator relative to this */
  void insertOperator(UVarCalcOper * oper);
  /**
   * Get pointer to value variable */
  inline UVariable * getVar() const
  { return var; };
  /**
   * Get value */
  inline double getValue()
  { return value; };
  /**
   * remove var structure
   * */
  void remVar()
  {
    if (var != NULL and varLocal)
    {
      delete var;
      var = NULL;
      varLocal = false;
    }
  };
  /**
   * Do an element-wise inverse of the variable */
  void inverseValue();
  /**
   * Do an element-wise negation of the variable */
  void negateValue();


protected:
  /**
   * convert all minus operators to + by changing sign of the first right value
   * or convert division to multiplication  by a 1/value operation */
  void evaluateMinusD();
  /**
   * convert all minus operators to + by changing sign of the first right value
   * or convert division to multiplication  by a 1/value operation (no good for vectors) */
  void evaluateMinusV();
  /**
   * \returns operator as string */
  const char * getOperStr();
  /**
   * Is the string at this position a remark?
   * A remark starts at a '#', a ';' or a '//'
   * \returns true if a remark starts here. */
  bool isRem(const char * r);

protected:
  /** Value after evaluation */
  double value;
  /** Value as variable reference */
  UVariable * var;
  /**
   * Is the variable locally created - and thus should be deleted */
  bool varLocal;
  /** Evaluated */
  bool evalud;
  /** Value type 1=bool, 2=int, 3=double, 4=variable */
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
public:
  /** left side value */
  UVarCalcOper * left;
  /** right side value */
  UVarCalcOper * right;
  /** parent operator */
  UVarCalcOper * parent;
  /// value as UVariable structure
  //UVariable * var;
  int num;
};

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////


/**
Class to act as a calculator */
class UVarCalc : public UVarPool
{
public:
  /**
  Constructor */
  UVarCalc();
  /**
  Destructor */
  virtual ~UVarCalc();
  /**
  Add standard math methods */
  //bool addMathMethods();
  /**
   * \brief Called by server core when new resources are available.
   * \param resource is the offered resource pointer
   * \param remove is true if the resource is unloaded and thuis no longer valid
   * \return true is resouurce is used
   * Save a pointer to the resource as needed. */
  bool setResource(UResBase * resource, bool remove);
  /**
   * \brief Get the variable with this name in the specified scope (incl. math)
   * Variables in the local scope has first priority, then
   * gradually one scope at a time until root scope. After that the math scope is tested.
   * \Returns true if found.
   * \param scope the scope to be searched for value (normally just from 'getVarPool()')
   * \param name is the name of the variable
   * \param value the found (double) value is returned here (if not NULL). */
  bool getVariableAny(UVarPool * scope, const char * name, UVariable * value);
  /**
   * \brief Get reference to variable with this name in the specified scope (incl. math)
   * Variables in the local scope has first priority, then
   * gradually one scope at a time until root scope. After that the math scope is tested.
   * \Returns true if found.
   * \param scope the scope to be searched for value (normally just from 'getVarPool()')
   * \param name is the name of the variable (may include index - is returned in nameIndex
   * \param found the found variable is returned here (if not NULL).
   * \param nameIndex is any optional index in the variable name string (if not NULL) if variable is found, but no index, then set to -1.
                       if no variable, then not changed. */
  bool getVariableRefAny(UVarPool * scope, const char * name, UVariable ** found, int * nameIndex);
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
  bool getValueAnyD(UVarPool * scope, const char * name,
                   double * value, const char ** params);
  /**
   * \brief Get value for the variable or function call with this name
   * Variables and functions in the local scope has first priority, then
   * gradually one scope at a time until root scope. After that the math scope is tested.
   * \Returns true if found.
   * \param scope the scope to be searched for value (normally just from 'getVarPool()')
   * \param value the found value is returned here (if not NULL).
   * \param params at this position are the parameters for the function, if
   * parameters are used (and can be evaluated) then the character position
   * are modified to just passed the end bracket for the parameter. */
  bool getValueAnyV(UVarPool * scope, const char * name,
                    UVariable * value, const char ** params);
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
/*  bool getValueAny(UVarPool * scope, const char * name,
                   UVariable * value, const char ** params);*/
  /**
   * \brief Evaluate an expression
   * \param line is the source line
   * \param start is where the expression starts in the line
   * \param next points to the first unused character after evaluation
   * \returns a double, that may hold a boolean value.
   * \param evaluated returns false if string fails to evaluate 'evaluated' (if not NULL)
   * \param syntaxCheck when true the evaluation is evaluated partially, as far as to the point where values or actions are initiated  */
  double evaluateD(const char * line, const char * start,
                  const char ** end, bool * evaluated, bool syntaxCheck);
  /**
   * \brief Evaluate an expression
   * \param line is the source line
   * \param start is where the expression starts in the line
   * \param next points to the first unused character after evaluation
   * \param value returns the resulting value here (if not NULL)
   * \param syntaxCheck when true the evaluation is evaluated partially, as far as to the point where values or actions are initiated
   * \returns true is evaluated, false if syntax errors (description in errorTxt) */
  bool evaluateV(const char * line, const char * start,
                  const char ** end, UVariable * value, bool syntaxCheck);
  /**
   * \brief Evaluate an expression
   * \param line is the source line
   * \param start is where the expression starts in the line
   * \param next points to the first unused character after evaluation
   * \param evaluated returns false if string fails to evaluate 'evaluated' (if not NULL)
   * \param syntaxCheck when true the evaluation is evaluated partially, as far as to the point where values or actions are initiated
   * \returns a pounter to a variable on the heap, receiver must delete. */
/*  UVariable *  eval(const char * line, const char * start,
                              const char ** end, bool * evaluated, bool syntaxCheck);*/
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
   * \param line is full line that is evaluated upon.
   * \param start is where to start (on this line).
   * \param evaluated is set true, if a value is found.
   * \param end is set to first unused character.
   * \param syntaxCheck is passed on to any expression of functionCall evaluation.
   * \returns the result as a double */
  double evaluateUnaryValD(const char * line, const char * start,
                       const char ** end, bool * evaluated, bool syntaxCheck);
  /**
  Evaluate a single value, that may have a unary operator in front of the value.
  The syntax may be: ['not' | '-'] (symbol | constant | '(' expression ')' | functionCall ).
  Returns the value found, or 0 on error.
   * \param line is full line that is evaluated upon.
   * \param start is where to start (on this line).
   * \param end is set to first unused character.
   * \param value is the value found (if not NULL).
   * \param syntaxCheck is passed on to any expression of functionCall evaluation.
   * \returns the true if a value is found */
  bool evaluateUnaryValV(const char * line, const char * start,
                          const char ** end, UVariable * value, bool syntaxCheck);
  /**
   * evaluate a single variable
   * \param line is full line that is evaluated upon.
   * \param start is where to start (on this line).
   * \param evaluated is set true, if a value is found.
   * \param end is set to first unused character.
   * \param syntaxCheck is passed on to any expression of functionCall evaluation.
   * \returns a variable with the result */
/*  UVariable * evalUnaryVal(const char * line, const char * start,
                       const char ** end, bool * evaluated, bool syntaxCheck);*/
  /**
  Evaluate a number [0..maxParamCount] of parameters -- for a function call.
  The number of found parameters gets returned in length of 'paramStr', and
  the values are present in 'paramValues' and 'paramStr'.
  Eg:
  sourceLine: "phi = 23.7 + atan2(y * sin(h), (x - 0.1) * cos(h))", and
  paramStart: "(y * sin(h), (x - 0.1) * cos(h))"
  Return will be paramValues = 2, paramValue[0] and [1] will be assigned values,
  'paramStart' will point at first character after parameter list - just after the ')' char
   * \param source line is used when reporting syntax error only
   * \param paramStart is a pointer into the place in 'line' where the parameter analysis should start.
   * \param paramValues is an array of doubles where the double sized parameters are returned, the size is in maxParamCount
   * \param paramValuesCnt is a pointer to an interger where the found number of double parameters are returned.
   * \param paramStr is a pointer to an array of string pointers. This is where the string parameters are returned. The number of strings available is in maxParamStrCount and the length of each string is in maxParamStrLength
   * \param paramOrder is a string where the sequence of parameters are returned, e.g. "dsd" means the first parameter is a double, returned in paramValue[0], the next is a string returned in paramStr[0] and the third parameter is a double returned in paramValue[1].
   * \param maxParamCount is the size of the paramOrder string AND the size of the paramValues array
   * \param maxParamStrCount max number of string parameters allowed
   * \param maxParamStrLength is the max length of evaluated string parameters (size of string buffer)
   * \Returns true if not syntax error were found. */
  bool evaluateParametersD(const char * sourceLine, // source line (for error reporting)
                                 const char ** paramStart,   // position of parameter start i.e. an '('
                                 double * paramValues,      // array of double parameters
                                 int * paramValuesCnt,   // count of double sized parameters found
                                 char ** paramStr,          // array of string parameters
                                 char * paramOrder,         // order - e.g. "ds" for (double, string)
                                 int maxParamCount,         // max size of parameter array and double array
                                 int maxParamStrCount,      // max size of string param list
                                 int maxParamStrLength);    // max length of a string param
  /**
  Evaluate a number [0..maxParamCount] of parameters -- for a function call.
  The number of found parameters gets returned in length of 'paramStr', and
  the values are present in 'paramValues' and 'paramStr'.
  Eg:
  sourceLine: "phi = 23.7 + atan2(y * sin(h), (x - 0.1) * cos(h))", and
  paramStart: "(y * sin(h), (x - 0.1) * cos(h))"
  Return will be paramValues = 2, paramValue[0] and [1] will be assigned values,
  'paramStart' will point at first character after parameter list - just after the ')' char
   * \param source line is used when reporting syntax error only
   * \param paramStart is a pointer into the place in 'line' where the parameter analysis should start.
   * \param paramOrder is a string where the sequence of parameters are returned, e.g. "dsd" means the first parameter is a double, the next is a string and the third parameter is a double.
   * \param params is a pointer to an array of variables.
   *               The number of variable pointers available is in maxParamsCount.
   *               Assumed to be NULL pointers, an caller must delete variables created.
   * \param paramsCnt is a pointer to an interger where the found number of parameters are returned.
   * \param maxParamsCount max number of string parameters allowed
   * \Returns true if not syntax error were found. */
  bool evaluateParametersV(const char * sourceLine, // source line (for error reporting)
                          const char ** paramStart, // position of parameter start i.e. an '('
                          char * paramOrder,        // order - e.g. "ds" for (double, string)
                          UVariable * params[],          // array of parameter variables
                          int * paramsCnt,          // count of parameters found
                          int maxParamsCnt);        // max number of parameters
  /**
  Evaluate a string of mixed unknown values and values to evaluate to a new evaluated string,
  that can be send to a subsystem - e.g. a SMRCL command.
   * Uses evaluateD(...) to evaluate non-strings.
  If part of the command is in quotes ("), then it is not evaluated.
  e.g. 'turnr 1.2 ' calcH -0.1 ' "rad" @v' speed ":($odoth >" calcH ")"
  and speed=0.3 and calcH=0.5, then it is evaluated to:
  turnr 1.2 0.4 "rad" @v 0.3 :($odoth > 0.5)
   * \param paramStart the start position (in line) where to start the string extraction
   * \param nextChar returns the first position after the string, e.g. a ',' or a ')'
   * \param dest the destination string
   * \param destLength is the length of the string buffer
  \Returns true if evaluated and there is space in 'dest' string.
  */
  bool evaluateToStringD(const char * sourceLine, // source line (for error reporting)
                               const char * paramStart, // position of where to start in source
                               const char ** nextChar,        // first unused character in line
                               char * dest,      // string for resultant evaluated string
                               int destLength);   // max length of destination string
  /**
  Evaluate a string of mixed unknown values and values to evaluate to a new evaluated string,
  that can be send to a subsystem - e.g. a SMRCL command.
   * Uses evaluateV(...) to evaluate non-strings.
  If part of the command is in quotes ("), then it is not evaluated.
  e.g. 'turnr 1.2 ' calcH -0.1 ' "rad" @v' speed ":($odoth >" calcH ")"
  and speed=0.3 and calcH=0.5, then it is evaluated to:
  turnr 1.2 0.4 "rad" @v 0.3 :($odoth > 0.5)
   * \param paramStart the start position (in line) where to start the string extraction
   * \param nextChar returns the first position after the string, e.g. a ',' or a ')'
   * \param dest the destination string
   * \param destLength is the length of the string buffer
  \Returns true if evaluated and there is space in 'dest' string.
   */
  bool evaluateToStringV(const char * sourceLine, // source line (for error reporting)
                        const char * paramStart, // position of where to start in source
                        const char ** nextChar,        // first unused character in line
                        char * dest,      // string for resultant evaluated string
                        int destLength);   // max length of destination string
  /**
   * Get variable assignment as specified in this line.
   * \returns true if variable is set with no syntax error.
   * \param cmdLine The line from where to start the evaluation (line is not changed)
  syntax error the errorText() is set with a description.. */
/*  bool getAssignment(const char * cmdLine, const char ** end, char * name, const int nameCnt, double * value);*/
  //bool setVariable(const char * cmdLine, bool syntaxCheck);
  /**
  Get error text */
  char * getErrorTxt()
  { return errorTxt; };
  /**
  Get error text max length */
  int getErrorTxtMaxCnt()
  { return MAX_ERROR_SIZE; };
  /**
  Returns a help string for general function 'index' and
  the number of general functions in 'count'. */
  //const char * getGenFunctionHelp(const int index, int * count);
  /**
  Get a (or try to) value from the connected MRC.
  Returns false if variable is unknown or MRC is not connected */
  //virtual bool getSmrdemoValue(const char * name, double * value);
  /**
  Is this string to be regarded as a remark line?
  Returns true if so.
  A remark line start with one of "%#;" or "//" */
  bool isRemark(const char * string);
  /**
   * Is the next n characters from 'var' a valid identifier.
   * an identifier starts with a character and may continue with
   * a sequence of characters, numbers or one of '_$'
   * \param id pointer to the first character to be evaluated - assumed not to be zero terminated.
   * \param idCnt maximum number of characters to be evaluated.
   * \param allowed is any special characters allowed after initial alpha character - typical "_"
   * \param oChar is set to the first char not belonging to a identifier
   * \returns true if 'id' starts with a valid identifier  */
  bool getIdentifier(const char * id, int idCnt,
                               const char * allowed,
                               const char ** oChar);

  /**
  Copy values of all variables to logfile */
  //void logCalcVars(FILE * logcc);
  /**
  The varPool has methods, and a call to one of these are needed.
  Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed. */
//   virtual bool methodCall(const char * name, const char * paramOrder,
//                           char ** strings, const double * doubles,
//                           double * value,
//                           UDataBase ** returnStruct = NULL,
//                           int * returnStructCnt = NULL);
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
  Evaluate system specific functions - that need to know
  position of planner etc.
  The called function name is in 'name' the parameters in 'pars'
  and if the function returns a value it
  must be put in 'value' */
/*  virtual bool evaluateSystemFunction(const char * name,
                          const double pars[], int parsCnt,
                          double * value, bool syntaxCheck);*/
  /**
   * Get value from a local function variable (a parameter).
   * \Returns true if a variable exist, and
   * \returns the value in '*value' */
  virtual bool getLocalScopeVaiable(const char * name, double * value);

protected:
  /**
  Error string, when an error is detected */
  char errorTxt[MAX_ERROR_SIZE];
};

#endif
