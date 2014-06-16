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
#ifndef UMISSION_H
#define UMISSION_H

#include <stdio.h>
#include <string>
#include <regex.h>

#include <urob4/uresvarpool.h>
#include <ugen4/udatabase.h>
#include <urob4/usmlfile.h>
#include <urob4/ulogfile.h>

//#include <urob4/uvarcalc.h>

using namespace std;

/**
Name length of a variable */
//#define MAX_VARIABLE_NAME_SIZE 32
/**
Maximum length of an error */
#define MAX_ERROR_SIZE 200

class UVarCalc;

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds a statement item of any type, blocked or single line  */
class UMisItem : public UDataBase
{
public:
  /**
   * Constructor */
  UMisItem();
  /**
   * DEstructor */
  virtual ~UMisItem();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misItem";
  };
  /** Possible return values of an evaluation */
  typedef enum {
    RV_EMPTY,
    RV_OK,            // item completed as expected - continue
    RV_OK_FALSE,      // as RV_OK, but with result FALSE
    RV_OK_AGAIN,      // item is a control statement that is not finished
    RV_SYNTAX_ERROR,  // there is syntax problems in this line
    RV_FAILED,        // the line failed for other reasons than syntax (e.g. no connection to device)
    RV_IF_TRUE,       // result of condition statement (true)
    RV_IF_FALSE      // result of condition statement (false)
  } ResultValue;
  /** execute item.
   * \Returns result of executing this line normal is RV_OK, an error ir RV_SYNTAX_ERROR. The error text is
   * to be returned in the errTxt buffer. */
  virtual ResultValue execute(UVarCalc * calc, int state);
  /**
   * Check the syntax validity of the statement.
   * \param cnn is a pointer to the source - may be NULL if the source is unavailable
   * \returns true if no errors were found. */
//  virtual bool checkSyntax(USmlSource * cnn);
  /**
   * Set the command line */
  virtual bool setLine(const int lineNum, const char * value, UVarCalc * calc)
  { lineNumber = lineNum; return true; } ;
  /**
   * Print a mission item to buffer
   * \param preStr a string to add before the plan lines
   * \param buf a string buffer to hold the plan
   * \param bufCnt the size of the buffer  */
  virtual const char * print(const char * preStr, char * buf, const int bufCnt);
  /**
   * Find a caracter outside any strings or (sub) brackets
   * Looking for a ':' in 'smr.do('drive : sin($odoth) < 0') : false' will
   * find the second ':' as the first one is within a string delimiter (and in a bracket).
   * Looking for a ')' in the same string will find the last ')' for the same
   * reason.
   * Looking for a ')' in 'atan2(sin(th), cos(45*(pi/180.0))) + 1)' will find
   * the last bracket (after the 1), as the others are matched.
   * Embedded strings in strings (or brackets) are allowed.
   * Embedded brackets in strings are ignored.
   * The string characters are quotes (") and apostrof (') - these MUST be matched.
   * \param source is the string to look in
   * \returns a pointer to the found stop character, or NULL if not found. */
  const char * findStopChar(const char * source, char stop);
  /**
   * Get line number of this mission statement (or first line, if multi line statement) */
  int getLineNumber()
  { return lineNumber; };
  /**
   * Set source line number for this item */
  void setLineNumber(const int line)
  { lineNumber = line; };
  /**
   * Is this line of this specified type.
   * \returns true if the type match. */
  bool isA(const char * matchType);
  /**
   * Is this character (and possibly the next) a start of remark character
   * A remark is the rest of a line starting with a ';' a '#' or a '//' pair */
  static bool isRem(const char * r);

protected:
  /**
   * Find the number of characters from the start of this string */
  int symbolLength(const char * exp);

public:
  /** Next line in a line sequence */
  UMisItem * next;

protected:
  /** source line number in mission file */
  int lineNumber;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds a non-block statement */
class UMisLineItem : public UMisItem
{
  public:
  /**
   * Constructor */
  UMisLineItem();
  /**
   * Constructor */
//  UMisLineItem(const char * exp);
  /**
   * Destructor */
  virtual ~UMisLineItem();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misLineItem";
  };
  /**
   * Set the command line */
  virtual bool setLine(const int lineNum, const char * value, UVarCalc * calc);
  /**
     * Print if statement to buffer
     * \param preStr a string to add before the plan lines
     * \param buf a string buffer to hold the plan
   * \param bufCnt the size of the buffer  */
  virtual const char * print(const char * preStr, char * buf, const int bufCnt);
  /**
   * Get a pointer to the full source line */
  const char * getLine()
  { return line; };

  protected:
    /** the command line */
    char * line;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds an assignment statement */
class UMisAssign : public UMisLineItem
{
  public:
  /**
   * Constructor */
  UMisAssign();
  /**
   * Constructor */
//  UMisAssign(const char * exp);
  /**
   * DEstructor */
  virtual ~UMisAssign();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misAssign";
  };
  /**
    * Set the command line */
  virtual bool setLine(const int lineNum, const char * value, UVarCalc * calc);
  /** execute item.
   * \param calc is the calculateor - and scope for variables and methods
   * \param strate is an integer for the sequencer state (init, main or post)
   * \Returns result of executing this line normal is RV_OK, an error ir RV_SYNTAX_ERROR. The error text is
   * to be returned in the errTxt buffer. */
  virtual ResultValue execute(UVarCalc * calc, int state);

protected:
  /** the name to be assigned to - not zero terminated - use length */
  const char * name;
  /** name length */
  int nameCnt;
  /** index expression to be evaluated before assignment */
  const char * indexExpr;
  /** the assigned value to be evaluated - a zero terminated string */
  const char * assignment;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds a non-block if statement */
class UMisIf : public UMisLineItem
{
  public:
  /**
   * Constructor */
  UMisIf();
  /**
   * Destructor */
  virtual ~UMisIf();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misIf";
  };
  /**
   * Set the command line */
  virtual bool setLine(const int lineNum, const char * value, UVarCalc * calc);
  /** execute item.
   * \param calc is the calculateor - and scope for variables and methods
   * \param state is the processing state - either init, main or post
   * \Returns result of executing this line RV_OK or RV_SYNTAX_ERROR. The error text is
   * returned in the errTxt buffer of calculator. */
  virtual ResultValue execute(UVarCalc * calc, int state);

protected:
  /** the command line */
  const char * condition;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds a non-block if statement */
class UMisIfElse : public UMisLineItem
{
  public:
  /**
   * Get (end) type of this structure */
    virtual const char * getDataType()
    {
      return "misIfElse";
    };
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds a non-block if statement */
class UMisBreak : public UMisLineItem
{
  public:
  /**
   * Constructor */
    UMisBreak();
  /**
   * Constructor */
//  UMisBreak(const char * exp);
  /**
   * Destructor */
    virtual ~UMisBreak();
  /**
   * Get (end) type of this structure */
    virtual const char * getDataType()
    {
      return "misBreak";
    };
    /**
     * Is the call a continue
     * \returns true if a continue statement */
    inline bool isContinue()
    { return strncasecmp(line, "continue", 8) == 0; };
    /**
     * Is the call a continue
     * \returns true if a continue statement */
    inline bool isBreak()
    { return strncasecmp(line, "break", 5) == 0; };
    /**
     * Set line and find start of parameter */
    virtual bool setLine(const int lineNum, const char * value, UVarCalc * calc);
    /**
     * Get a copy of just the break parameter */
    const char * getParamCopy(char * dest, const int destCnt);
    /**
     * Get a pointer to the start of the parameter (first charanter) on the line, or
     * a NULL if no parameter is available. */
    const char * getParam()
    { return param; };

  protected:
    /** the parameter following the break keyword - is NULL if no parameter or just a comment*/
    const char * param;
    /** length of the parameter - till first space or remark or end of line */
    int paramCnt;
}; // std::string

/**
 * Class that holds a enable-disable statement */
class UMisEnable : public UMisLineItem
{
public:
  /**
   * Constructor */
  UMisEnable();
  /**
   * Destructor */
  virtual ~UMisEnable();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misEnable";
  };
  /**
    * Set line and find start of parameter */
  virtual bool setLine(const int lineNum, const char * value, UVarCalc * calc);
  /**
    * Get a copy of just the break parameter */
  const char * getParamCopy(char * dest, const int destCnt);
  /**
    * Get a pointer to the start of the parameter (first charanter) on the line, or
    * a NULL if no parameter is available. */
  const char * getParam()
  { return param; };
  /** is the statement an Enable */
  bool isEnable()
  { return anEnable; };

protected:
  /** the parameter following the break keyword - is NULL if no parameter or just a comment*/
  const char * param;
  /** length of the parameter - till first space or remark or end of line */
  int paramCnt;
  /** is it an enable or disable call */
  bool anEnable;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds a loop, either a for-loop or a while-loop  statement */
class UMisLoop : public UMisLineItem
{
  public:
  /**
   * Constructor */
  UMisLoop();
  /**
   * Destructor */
  virtual ~UMisLoop();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misLoop";
  };
  /**
   * Set line and find start of parameter */
  virtual bool setLine(const int lineNum, const char * value, UVarCalc * calc);
  /**
   * GExecute the init assignment for the loop (if any)
   * \param calc is the UVarCalc object with the local variables
   * \returns RV_OK if assignment is an success, else RV_SYNTAX_ERROR  */
  UMisItem::ResultValue runInitAssignment(UVarCalc * calc);
  /**
   * GExecute the init assignment for the loop (if any)
   * \param calc is the UVarCalc object with the local variables
   * \returns RV_OK if assignment is an success, else RV_SYNTAX_ERROR  */
  UMisItem::ResultValue runLoopAssignment(UVarCalc * calc);
  /**
   * evaluate the loop condition
   * \param calc is the UVarCalc object with the local variables
   * \param val is where the boolean result is returned
   * \returns RV_OK if expression is evaluated, else RV_SYNTAX_ERROR   */
  UMisItem::ResultValue evalCondition(UVarCalc * calc, bool * val);

protected:
  /** The initial assignment - possibly creating a new local variable */
  UMisAssign * initAssign;
  /** The assignment to be done after each loop */
  UMisAssign * loopAssign;

public:
  /** The condition to continue the loop */
  const char * condition;
  /** The loop expression itself, set during execution */
  UMisItem * loopLine;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds a case  statement */
class UMisCaseSwitch : public UMisLineItem
{
public:
  /**
   * Constructor */
  UMisCaseSwitch();
  /**
   * Destructor */
  virtual ~UMisCaseSwitch();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misSwitch";
  };
  /**
   * Set line and find start of parameter */
  virtual bool setLine(const int lineNum, const char * value, UVarCalc * calc);
  /**
   * Evaluate the integer expression to bi compared with case statements */
  int evaluateExpr(UVarCalc * calc);

  protected:
    /** The case block */
    const char * switchExpression;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds a case  statement */
class UMisCase : public UMisLineItem
{
public:
  /**
   * Constructor */
  UMisCase();
  /**
   * Destructor */
  virtual ~UMisCase();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misCase";
  };
  /**
   * Set line and find start of parameter */
  virtual bool setLine(const int lineNum, const char * value, UVarCalc * calc);
  /**
   * Check if a value is in the list of case values
   * \param value the integer from the switch statement
   * \returns true if the value is in the list, else false */
  bool isInList(int value);

protected:
    /** The case block */
    int * caseList;
    /**
     * Number of list values */
    int caseListCnt;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds a case  statement */
class UMisCaseDefault : public UMisLineItem
{
  public:
  /**
   * Constructor */
    UMisCaseDefault(){};
  /**
   * Destructor */
    virtual ~UMisCaseDefault(){};
  /**
   * Get (end) type of this structure */
    virtual const char * getDataType()
    {
      return "misDefault";
    };
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

class UMisRule;



/**
 * Class that holds a simple function call statement  */
class UMisCall : public UMisLineItem
{
  public:
  /**
   * Constructor */
  UMisCall();
  /**
   * Constructor */
//  UMisCall(const char * exp);
  /**
   * DEstructor */
  virtual ~UMisCall();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misCall";
  };
  /**
   * Set the command line */
  virtual bool setLine(const int lineNum, const char * value, UVarCalc * calc);
  /**
   * Get a pointer to the actual parameters of the call  */
  const char * getParameters()
  { return parameters; };
  /**
   * Get a pointer to the function or method name.
   *  NB! not zero terminated - use nameCnt */
  const char * getName()
  { return name; };
  /** Get number of characters in the name */
  int getNameCnt()
  { return nameCnt; };
  /**
   * Get a zero terminated copy of name to this char buffer
   * \param dest is an array of character.
   * \param destCnt is the size of the provided buffer
   * \returns a zero terminated string in the buffer, and if
   * the buffer is of sufficient length (name length + 1) then
   * the full name is in the buffer  */
  const char * getNameCopy(char * dest, const int destCnt);
  /**
   * Does the name match this name */
  bool isA(const char * thisName);

protected:
  /** the call identifier - not zero terminated - use nameCnt*/
  const char * name;
  /** name length */
  int nameCnt;
  /**
   * Parameters - a pointer to the '('  */
  const char * parameters;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds a control statement, that is a call
 * that is called repeatingly until a continue condition is fullfilled */
class UMisControl : public UMisCall
{
  public:
  /**
   * Constructor */
  UMisControl();
  /**
   * Constructor */
//  UMisControl(const char * exp);
  /**
   * DEstructor */
  virtual ~UMisControl();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misControl";
  };
  /**
   * Set the command line */
  virtual bool setLine(const int lineNum, const char * value, UVarCalc * calc);
  /** execute item.
   * \param calc is the calculateor - and scope for variables and methods
   * \param repeat is the repeat call sequence 0 for first >0 for subsequent and -1 for last call
   * \Returns result of executing this line normal is RV_OK if control line is finished, or
   * \returns RV_SYNTAX_ERROR. The error text is returned in the errTxt buffer.
   * \returns RV_OK_AGAIN when the call is not finished. */
  virtual ResultValue execute(UVarCalc * calc, int repeat);
  /** \brief evaluate the condition of the control statement.
   * \param calc is the calculateor - and scope for variables and methods
   * \returns RV_OK if condition evaluated anything but false (rounds to 0),
   * returns RV_ otherwise 
   * returns RV_SYNTAX_ERROR if condition can not be evaluated - syntax error.
   * Puts a statement in log if syntax error. */
  UMisItem::ResultValue evaluateCondition(UVarCalc * calc);
  /**
   * Get explicit condition
   * \returns pointer to condition string */
  const char * getCondition()
  { return strCond; };

protected:
  /** explicit condition statement */
  const char * strCond;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds a comment - ignored during execution  */
class UMisRemark : public UMisLineItem
{
  public:
  /**
   * Constructor */
  UMisRemark();
  /**
   * Constructor */
//  UMisRemark(const char * exp);
  /**
   * DEstructor */
  virtual ~UMisRemark();
  /**
   * Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misRemark";
  };
  /**
   * A remark requires no action */
  virtual UMisItem::ResultValue execute(UVarCalc * calc, int state)
  { return RV_OK; };

protected:
  /** the remark line */
  string line;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

/**
 * Class that holds a comment - ignored during execution  */
class UMisString : public UMisLineItem
{
  public:
  /**
   * Constructor */
    UMisString();
  /**
   * Constructor */
//  UMisRemark(const char * exp);
  /**
   * DEstructor */
    virtual ~UMisString();
  /**
   * Get (end) type of this structure */
    virtual const char * getDataType()
    {
      return "misString";
    };

  protected:
    /** the command line */
    //string line;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////


/**
 * Class that holds the base for a mission or a mission segment  */
class UMisRule : public UMisItem
{
public:
  /**
   * Constructor */
  UMisRule();
  /**
   * DEstructor */
  virtual ~UMisRule();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "misRule";
  };
  /** Get number of ini-lines
   * returns number of ini-lines */
  inline int getIniLinesCnt()
  { return getLinesCnt(iniLines); };
  /** Get number of main-lines
   * returns number of main-lines */
  inline int getMainLinesCnt()
  { return getLinesCnt(mainLines); };
  /** Get number of post-lines
   * returns number of post-lines */
  inline int getPostLinesCnt()
  { return getLinesCnt(postLines); };
  /** Add a line to the ini-segment */
  inline void addIniLine(UMisItem * newLine)
  { addLine(iniLines, newLine); };
  /** Add a line to the ini-segment */
  inline void addMainLine(UMisItem * newLine)
  { addLine(mainLines, newLine); };
  /** Add a line to the ini-segment */
  inline void addPostLine(UMisItem * newLine)
  { addLine(postLines, newLine); };
  /**
   * Get plan name
   * \returns a pointer to the name attribute */
  const char * getName()
  { return name; };
  /**
   * Get plan name
   * \returns a pointer to the filename string */
  const char * getFileName()
  { return fileName; };
  /**
   * unpack from XML source */
  bool unpackRule(USmlSource * cnn, USmlTag * tag);
  /**
   * Print mission plan to buffer
   * \param preStr a string to add before the plan lines
   * \param buf a string buffer to hold the plan
   * \param bufCnt the size of the buffer  */
  virtual const char * print(const char * preStr, char * buf, const int bufCnt);
  /**
   * Get the first ini-line
   * \returns NULL is no init lines are defined. */
  UMisItem * getIniLines()
  { return iniLines; };
  /**
   * Get the first main-line
   * \returns NULL is no init lines are defined. */
  UMisItem * getMainLines()
  { return mainLines; };
  /**
   * Get the first post-line
   * \returns NULL is no init lines are defined. */
  UMisItem * getPostLines()
  { return postLines; };
  /**
   * set syntax check var pool calculator */
  void setCalc(UVarCalc * calculator)
  { calc = calculator; };
  /**
   * Set just name pf plan */
  void setName(const char * toName)
  { strncpy(name, toName, MAX_VAR_NAME_SIZE); };
  /**
   * Adjust the number of times this plan is beeing used by the
   * sequencer - add 1 every time it is used in a plan state and remove one
   * when plan state is released
   * \returns 0 busy is not implemented at this level. */
  virtual int setBusyCnt(int delta) { return 0; };
  /**
   * Set logfile */
  void setLogFile(ULogFile * logFile)
  { logm = logFile; };
  /**
   * Is this plan in editing mode - i.e. append is allowed
   * \returns true if plan is marked as under edit, i.e. not runable and not active */
  virtual bool isEdit()
  { return false; };

protected:
  /** Possible return values of an evaluation */
  typedef enum {
    MP_INIT,
    MP_MAIN,
    MP_POST
  } MisPart;
  /**
   * Count number of (remaining lines fstarting at this line */
  int getLinesCnt(UMisItem * fromLine);
  /**
   * Add a line to this segment */
  void addLine(UMisItem * elemList, UMisItem * newLine);
  /**
   * Unpack a block structure - either a command or an block (if, loop or otherwise)
   * This is the same as a plan, but without the init and post structures.
   * \param cnn is the source for more data (the XML file)
   * \param tag is the block start tag
   * \param itemList is the position of the pointer, where the block is to be added
   * \returns true */
  bool unpackBlock(USmlSource * cnn, USmlTag * tag, UMisItem ** itemList);
  /**
   * Unpacks a sequence of statements until the end-tag is discovered.
   * The statements are added to the pointer in the item list.
   * \param cnn is the source for more data (the XML file)
   * \param tag is the block start tag
   * \param itemList is the position of the pointer, where the block is to be added
   * \param iniBlock flag for specific block part 0=main, 1=ini, 2=post
   * \returns true */
  bool unpackInitPostBlock(USmlSource * cnn, USmlTag * tag, UMisItem ** itemList,
                           MisPart iniBlock);
  /**
   * Unpacks a sequence of statements until the end-tag is discovered.
   * The statements are added to the pointer in the item list.
   * \param cnn is the source for more data (the XML file)
   * \param lines text buffer with line separated expressions
   * \param itemList is the position of the pointer, where the block is to be added
   * \param iniBlock flag for specific block part 0=main (default), 1=ini, 2=post
   * \returns true */
  bool unpackSequence(USmlSource * cnn, char * lines, UMisItem ** itemList, int lineNum, MisPart iniBlock = MP_MAIN);
  /**
   * Unpack a text line to any of the allowed line types, that is
   * if, call, remark, control or assignment.
   * \param line the line to unpack
   * \param itemNext the pointer that should point to the new line
   * \param iniBlock flag for specific block part 0=main, 1=ini, 2=post
   * \returns true if a line is added, otherwise syntax error */
  bool unpackLine(USmlSource * cnn, char * line, UMisItem ** itemNext, int lineNum, MisPart iniBlock);
  /**
   * TRim off all pre and post white space and line feeds
   * \param s c-string to trim
   * \returns length of trimmed string  */
  int trimWhiteSpace(char * s);
  /**
   * Check the syntax validity of the block statement (not the lines in the block).
   * \param cnn is a pointer to the source - may be NULL if the source is unavailable
   * \returns true if no errors were found. */
  virtual bool checkSyntax(USmlSource * cnn);
  /**
   * Get the edit lines (un-implemented parts of a plan)
   * \returns NULL if no edit lines are available. */
  virtual const char * getEditLines()
  { return NULL; };

protected:
  /**
   * Name of rule or block */
  char name[MAX_VAR_NAME_SIZE];
  /**
   * filename */
  char fileName[MAX_FILENAME_LENGTH];
  /** First ini-line */
  UMisItem * iniLines;
  /** First main-line */
  UMisItem * mainLines;
  /** First post-line */
  UMisItem * postLines;
  /**
   * Syntax check calculator */
  UVarCalc * calc;

public:
  /**
   * Description string */
  string description;
  /**
   * Condition string at start of block
   * Must be evaluated to true to execute block */
  char * condition;
  /**
   * Parameter string to be implemented at start of block
   * Could be 'x=1 y=2 th=0.57' these should be created on varpool
   * with these default values. */
  char * parameters;
  /**
   * number of parameters found in plan */
  int parametersCnt;
  /**
   * destination if the block is to be implemented by another command enterpreter.
   * This could be 'server.send' or 'smr.send' */
  char * destination;
  /**
   * Dependency, this is a list of plugin dependecies - a
   * space separated list of plug-in modules that need to be present
   * for this mission to succeed.
   * If some of the listed plug-ins
   * are optional, there should be a note in the description, so
   * that the user may select to start the mission anyhow. The list is
   * therefore not enforced as mandatory, but a tool for the user. */
  string dependPlugin;
  /**
   * \brief Priority order of this plan - relative to other (sub) plans.
   * This value controls in wich order the list of (sub) plans are
   * assigned processing time, i.e. if a number of sub-plans evaluate some
   * value - e.g. the robot pose - then thise can be assigned to be evaluated before
   * another subplan, that combines or selects the usable results from the others.
   * A priority with a low number is executed first.
   * The value range is 1..100, where 1 is executed before the others. if two
   * plans have the same priority order, then the order is un-determined.
   * Default value is 50. */
  int order;
  /**
   * Is this a full rule or just a block (isPlan == false) - only full rules have init and post structures */
  bool isFull;

//protected:
  /**
   * Is this plan a top level executable mission */
  bool isTop;
  /**
   * Is this plan a flagged to run when loaded */
  bool isRun;
  /** Is this block a loop */
  bool isLoop;
  /**
   * Is this plan a rule - has an if attrubute */
  bool isRule;
  /**
   * It this block part of a switch statement */
  bool isSwitch;
  /**
   * The switch value */
  int switchValue;
  /** Logfile for any mission load statements */
  ULogFile * logm;
protected:
  /** is the current plan in edit-append mode.
   * this will affect the handling of xml blocks
   * to allow end of (file) with no XML end tag.
   */
  bool inEditAppend;
};

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////


/**
 * Class that holds a case  statement */
// class UMisCaseBlock : public UMisRule
// {
//   public:
//   /**
//    * Constructor */
//     UMisCaseBlock();
//   /**
//    * Destructor */
//     virtual ~UMisCaseBlock();
//   /**
//    * Get (end) type of this structure */
//     virtual const char * getDataType()
//     {
//       return "misCaseBlock";
//     };
//   /**
//    * Set case variable */
//     void setCaseExpression(const char * expr)
//     {
//       caseExpression = expr;
//     }
//   protected:
//     /** The expression to be evaluated as an integer */
//     const char * caseExpression;
// };

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

#endif
