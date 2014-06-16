/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                   *
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
#ifndef USEQUENCER_H
#define USEQUENCER_H

#include <ugen4/ulock.h>
#include <ugen4/utime.h>
#include <urob4/uclienthandler.h>
#include <urob4/uvarcalc.h>

#include "ucalcseq.h"
//#include "usmrclmmr.h"
//#include "ummrdrive.h"


// Stop conditions
const int NUMBER_OF_STOPCONDITIONS = 7;
#define ODO_STOP_DISTANCE      0
#define GPS_STOP_DISTANCE      1
#define EKF_STOP_DISTANCE      2
#define ODO_STOP_WP            3
#define GPS_STOP_WP            4
#define EKF_STOP_WP            5
#define STOP_TIME              6

// how to follow a road (WPF)
#define WPF_ROUTE_GO_LEFT      0
#define WPF_ROUTE_GO_RIGHT     1
#define WPF_ROUTE_GO_TOP       2
#define WPF_ROUTE_GO_POSITION  3
#define WPF_ROUTE_GO_DIRECT    4
#define WPF_ROUTE_NOGO         5

// drive system mode and states
#define DRIVE_MODE_IDLE             0
#define DRIVE_MODE_SMRCL            1
#define DRIVE_MODE_WPF              2
// actual state of system
#define DRIVE_STATE_IDLE            0
#define DRIVE_STATE_SEQ             1
#define DRIVE_STATE_WPF             2
#define DRIVE_STATE_NO_CONNECTION   3


/**
Max number of lines in sequencer */
#define MAX_SQE_LINE_CNT 10000
/**
Max number of watch lines */
#define MAX_SEQ_WATCH_LINES 200
/**
Max size of sequencer stack (return point after a function call) */
#define MAX_SEQ_STACK_SIZE 1000
/**
Max Parameter values used in function cals (doubles) */
#define MAX_SEQ_PARAM_STACK_SIZE 1000
/**
Max number ofdefinable functions */
#define MAX_SEQ_FUNCTIONS 100
/**
Max number of parameters to a function */
#define SEQ_MAX_FUNCTION_PARAMS 15
/**
Maximum length of a line in simulated logfile */
#define MAX_SIMLINE_SIZE 1300

enum USeqLineType {SQ_UNKNOWN,
              SQ_ASSIGN,
              SQ_DRIVE_STATEMENT,
              // SQ_DRIVE_FWD, SQ_DRIVE_TURN, SQ_DRIVE_GOTOWAYPOINT,
              // SQ_DRIVE_IDLE, SQ_DRIVE_SMRCL,
              SQ_FUNCTION, SQ_FUNCTION_RETURN,
              SQ_LABEL,
              SQ_FLOW_IF, SQ_FLOW_GOTO, SQ_FLOW_CALL,
              SQ_FLOW_SKIP, SQ_FLOW_SKIP_CALL,
              SQ_WATCH, SQ_UNWATCH,
              /*SQ_SUPPORT_CAM, SQ_SUPPORT_ARM, SQ_SUPPORT_SAY,*/
              SQ_PRINT,
              SQ_REMARK
};

/**
USeqLine holds a line in the sequencer */
class USeqLine
{
public:
  /**
  Constructor */
  USeqLine();
  /**
  Destructor */
  ~USeqLine();
  /**
  Clear line to unknown state */
  void clear();
  /**
  Allocate space for this line and set line type */
  bool setLine(const char * line, UVarPool * varPool, int lineNum, FILE * logS);
  /**
  Determine line type from first part of line */
  bool setLineType(int lineNum, UVarPool * varPool, FILE * logS);
  /**
  Get raw command line */
  inline char * getCmdLine()
    { return cmd; };
  /**
  Set syntax error position */
  inline void setSyntaxErr(int atPosition)
    { syntaxErr = atPosition; };
  /**
  get syntax error position */
  inline int getSyntaxErrorPosition()
    { return syntaxErr; };
  /**
  Get position of first paraketer after the statement keyword
  of the open bracket with parameters. */
  inline char * getParams()
    { return params; };
  /**
  Get position of first paraketer after the statement keyword
  of the open bracket with parameters. */
  inline void setParams(char * paramsStartPosition)
  { params = paramsStartPosition; };
  /**
  Get position of explicit stop criteria or to-do part of
  flow or watch statement */
  inline char * getToDo()
    { return toDo; };
  /**
  Set to-Do pointer */
  inline void setToDo(char * toDoPartOfLine)
    {toDo = toDoPartOfLine; };
  /**
  Set line type */
  inline void setType(USeqLineType toType)
    { typ = toType; };
  /**
  Set line type */
  inline USeqLineType getType()
    { return typ; };
  /**
  Get identifier (an alpha followed by alha-num) and return in 'name',
  and set 'next' to the first non-space character following identifier. */
  char * getIdentifier(const char * string,
                       char * buffer,
                       int bufferLng,
                       char ** next);
  /**
  Test if this is a label with this name.
  the provided name may end with an optional ':' */
  bool isThisLabel(const char * labelName);
  /**
  Is this line one of the SKIP types,
  Returns true if so */
  bool isASkipLine();
  /**
  Is this line legal as a then art in an if statement */
  bool isLegalThenStatement();
  /**
  Get sequence of parameters witout evaluation of values.
  Not fail safe, as missing operators in expressions are not evaluated */
  bool enumerateParameters(const char * start, char * parSeq, const int parSeqCnt, const char ** nextChar);

protected:
  /**
  Line that holds the command */
  char * cmd;
  /**
  Line of allocated memory */
  int buffSize;
  /**
  Type of line */
  USeqLineType typ;
  /**
  Syntax error found at character position */
  int syntaxErr;
  /**
  The 'params' pointer points to the first non-space
  character after the initial keyword, i.e. the '(' anter an if or a function call, or
  the first parameter in a drive command.
  The value is set by the 'setLineType' function */
  char * params;
  /**
  Explicit part og statement.
  This will be
  the statement to execute in a watch, or
  the statement to execute in an if statement.
  The value is set by the sequencer (during syntax control) */
  char * toDo;
};

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

/**
Class with information on watches in addition to USeqLine */

class USeqWatch
{
public:
  /**
  Constructor */
  USeqWatch();
  /**
  Destructor */
  ~USeqWatch();
  /**
  Is watch active */
  inline bool isActive()
  { return active; };
  /**
  Set watch active flag */
  void setActive(bool value);
  /**
  Get first line number of watch function */
  inline USeqLine * getLine()
  { return &line; };
  /**
  Get name of watch */
  inline const char * getName()
  { return name; };
  /**
  Is it time to evaluate.
  Returns true if time to execute watch, and if so
  the ttw is advanced */
  bool isTimeToWatch(UTime toTime);
  /**
  Set watch values */
  void set(bool asActive,
          double sampleInterval,
          const char * watchName);

  protected:
  /**
  Is watch active */
  bool active;
  /**
  Related sequencer line */
  USeqLine line;
  /**
  Do not evaluate before this time */
  UTime ttw;
  /**
  watch with this interval (in decmal seconds).
  if negative, then every time */
  double sampleTime;
  /**
  Watch name */
  char name[MAX_VARIABLE_NAME_SIZE];
};

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

/**
* This class holds information on functions defined in the sequencer language.
*/
class USeqFunction
{
public:
  /**
  Constructor */
  USeqFunction();
  /**
  Destructor */
  ~USeqFunction();
  /**
  Get function name */
  inline const char * getName()
  { return name; };
  /**
  Get number of parameters */
  inline int getParamCnt()
  { return paramsCnt; };
  /**
  Get line number with return statement.
  NB! return line is -1 if not evaluated yet. */
  inline int getReturnLine()
  { return last; };
  /**
  Get line with function declaration */
  inline int getStartLine()
  { return first; };
  /**
  Set function values.
  Resets last line to -1 (unknown) and
  resets param count to -1 (unknown) */
  bool set(USeqLine * functionLine,
                         int lineNum,
                         FILE * log);
  /**
  Set line number of function return statement */
  inline void setReturnLine(int lastLineNum)
  { last = lastLineNum; };
  /**
  Set number of parameters required by function */
  inline void setParamCnt(int numberOfParams)
  { paramsCnt = numberOfParams; };
  /**
  Get value of function parameter.
  The values are found relative to the value stack top.
  Returns true if the value is found, and in this case the value is assigned
  to 'value', otherwise the function returns false. */
  bool getValue(const char * name, double * value, double * valueStackTop, int valueStackCnt);

protected:
  /**
  Name of function */
  char name[MAX_VARIABLE_NAME_SIZE];
  /**
  Program line with function declaration */
  int first;
  /**
  Program line with return statement */
  int last;
  /**
  Name of parameters */
  char params[SEQ_MAX_FUNCTION_PARAMS][MAX_VARIABLE_NAME_SIZE];
  /**
  Number of params required */
  int paramsCnt;
};

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

/**
USequencer holds the list of executable lines, program counter watch list etc.

@author Christian Andersen
*/
class USequencer
{
public:
  /**
  Constructor */
  USequencer();
  /**
  Destructor */
  virtual ~USequencer();
  /**
  Reset status to no lines, no functions no nothing */
  void reset();
  /**
  Load plan from file */
  bool loadPlan(char * filenamebool, FILE * log, bool append);
  /**
  Clear current plan, and skip to line 0.
  Sequencer is locked while dooing tyis. */
  void clear();
  /**
  Add a line at the end of the sequencer program.
  If 'lineNum' is not NULL, then the line number is returned here. */
  bool add(const char * line, FILE * logf, int * lineNum);
  /**
  Add a label at the end if the sequencer with a default name,
  and return a pointer to the name.
  Af not possible (no more space) a NULL is returned. */
  const char * addLabel();
  /**
  skip execution to this labeled line.
  If 'asCall' is true then the current active line is
  pushed on the return stack,
  else the callstack is cleared! */
  bool skipToLabel(const char * label, bool asCall);
  /**
  Set pointer to calculator object */
/*  inline void setCalculator(UCalcMmr * calculator)
    { calc = calculator; };*/
  /**
  Set pointer to connected smr object */
/*    inline void setSmrClObj(USmrClMmr * smrObj)
    { smr = smrObj;  };*/
  /**
  Set reference to drive system */
/*  inline void setDriveSystem(UMmrDrive * driveSystem)
  { drive = driveSystem; };*/
  /**
  Get name of last loaded plan */
  inline const char * getPlanName()
  { return loadedPlanName; };
  /**
  Get name of last loaded plan */
  inline int getLineCnt()
  { return linesCnt; };
  /**
  Parse en execute statement.
  If 'syntaxCheck' is true, then the line is only checked for
  syntax error. */
  bool parseStatement(USeqLine * line,
                      int lineNum,
                      FILE * log,
                      bool syntaxCheck, int * nextLine);
  /**
  Execute all pending watches.
  may set active line if a watch executes a skip statement.
  Returns true if no errors were found. */
  bool doWatches(UTime sampleTime);
  /**
  Execute a function.
  Function may return a value (double), this
  is returned in 'returnValue' and is valid if
  'returnValueValid' is true. */
  bool doExecuteFunction(USeqLine * fromLine,
                         double * returnValue,
                         bool * returnValueValid);
  /**
  Evaluate a script function as a value returning function, in e.g. an
  assignment statement.
  Returns true if it exist.
  If not just 'syntaxCheck' and if not syntax errors are found, then
  the value is returned in 'value' */
  virtual bool evaluateSequencerFunction(const char * name,
                                         const double pars[], int parsCnt,
                                         double * value, bool syntaxCheck);
  /**
  Get reference to line */
  USeqLine * getLine(int lineNum);
  /**
  Open logfile */
  bool setLogFile(bool sim,
                  const char * simPath,
                  const char * simLogfile,
                  const char * logPath,
                  const char * logfile);
  /**
  Set simulation flag.
  If false, then sequencer follows loaded program, (as if) for love
  run.
  If true, processed line is taken form logfile, and watches are
  not executed (monitor what happend from logfile - mode) */
  inline void setSimulated(bool value)
  { simulated = value; };
  /**
  Set active line */
  void setActiveLine(const int lineNum);
  /**
  Get active line number */
  inline int getActiveLine()
  { return lineActive; };
  /**
  Is the simulated flag set */
  inline bool isSimulated()
  { return simulated; };

  /**
  Get the linenumber of a label.
  Returns -1 if label is undefined */
  int findLabelNum(const char * labelName);

  /**
  Get value from a local function variable (a parameter).
  Returns true if a variable exist, and then the value
  is returned in 'value' */
  virtual bool getLocalScopeVaiable(const char * name, double * value);
  /**
  Send this text message (assumed to be coded in XML format with
  a linefeed added at the end) to all active clients attached to server.
  Returns true if send to at least one client */
  virtual bool sendToAll(const char * message, int lockedUser);
  /**
  Pack this text in a \<push text="%s"/\> XML element and send to all active clients.
  returns true if send to at least one active client */
  virtual bool sendInfoToAll(const char * message, int lockedUser);
  /**
  Get pointer to sequencer calculator */
  inline UCalcSeq * getCalc()
  { return calc; };

protected:
  /**
  Push a line number on the stack */
  void pushStack(int number);
  /**
  Get the top value from the call stack.
  returns -1 if stack is empty. */
  int popStack();
  /**
  Push a double value on the parameter stack.
  returns false and report an error if stack is full */
  bool pushParam(double value);
  /**
  Get the top value on the parameter stack.
  if empty, then 0.0 is returned and an error is reporteg (and logged) */
  double popParam();
  /**
  Handle an assign statement, or just check syntax if syntax check is true.
  if 'log' is not NULL, then report errors here.
  Returns false if statement failed (syntax or otherwise) */
  bool parseAssignStatement(USeqLine * line,
                           int lineNum,
                           FILE * log,
                           bool syntaxCheck);
  /**
  Parse command lines to other servers.
  Provisitions are made for camera server only, with
  the initial keyword 'cam'.
  Returns true if command is send successfully. */
  bool parseSupportStatement(USeqLine * line,
                    int lineNum, FILE * logf, bool syntaxCheck);
  /**
  Handle a flow statement, or just check syntax if syntax check is true.
  if 'log' is not NULL, then report errors here.
  Returns false if statement failed (syntax or otherwise). */
  bool parseFlowStatement(USeqLine * line,
                          int lineNum,
                          FILE * log,
                          bool syntaxCheck,
                          int * nextLine);
  /**
  Handle a skip statement, or just check syntax if syntax check is true.
  if 'log' is not NULL, then report errors here.
  Returns false if statement failed (syntax or otherwise). */
  bool parseSkipStatement(USeqLine * line,
                          int lineNum,
                          FILE * log,
                          bool syntaxCheck,
                          int * nextLine);
  /**
  Handle a drive statement, or just check syntax if syntax check is true.
  if 'log' is not NULL, then report errors here.
  Returns false if statement failed (syntax or otherwise) */
  bool parseDriveStatement(USeqLine * line,
                           int lineNum,
                           FILE * log,
                           bool syntaxCheck,
                           int * nextLine);
  /**
  Handle a function statement, or just check syntax if syntax check is true.
  if 'log' is not NULL, then report errors here.
  Returns false if statement failed (syntax or otherwise)
  'NextLine' is set to first line after function return */
  bool parseFunctionStatement(USeqLine * line,
                              int lineNum,
                              FILE * log,
                              bool syntaxCheck,
                              int * nextLine);
  /**
  Handle a function return statement.
  The parameter values are removed from the value (params) stack.
  The optional return value is extracted and the index
  to save this value is pop'ed from the stack, and saved here (if not -1).
  The return line is pop'ed from the stack and set as next line.
  Returns true if no errors were detected.
  Errors get reported to log if (log != NULL).
  If 'syntaxCheck', then no stack operations are performed.
  If the function returns a value, this is returned in 'returnValue' and
  'returnValueValid' is in this case set to true.
  Default is skip return value. */
  bool parseFunctionReturn(USeqLine * line,
                              int lineNum,
                              FILE * log,
                              bool syntaxCheck,
                              int * nextLine,
                              double * returnVale,
                              bool * returnValueValid);
  /**
  Parse this function call line with an initialized
  function structure 'func'. The function parameter values [0..10] starts with
  an open '(' at 'paramStart'.
  If 'log'-file is open (not NULL) errors get reported here, if 'syntaxError'
  then stntax is checked only, and nothing is changed.
  Returns true if succesfull, and next line to execute is returned in
  'nextLine', and parameters are pushed on parameter stack, and
  return line on call stack.
  'paramStart' is modified to point just after parameter list (after the ')' char) */
  bool parseFunctionCall(USeqLine * line,
                         int lineNum,
                          USeqFunction * func,
                          const char ** paramStart,
                          int * nextLine,
                          FILE * log,
                          bool syntaxCheck);
  /**
  Parse an if statement line.
  Returnes true is no syntax error.
  if 'syntaxCheck', then nothing is implemented.
  if 'log'file is open (not NULL), then errors get reported here.
  'NextLine' is set to line+1 if not a call or goto, else as needed */
  bool parseIfStatement(USeqLine * line,
                        int lineNum,
                          int * nextLine,
                          FILE * log,
                          bool syntaxCheck);
  /**
  Handle a drive statement, or just check syntax if syntax check is true.
  if 'log' is not NULL, then report errors here.
  Returns false if statement failed (syntax or otherwise) */
  bool parseWatchStatement(USeqLine * line,
                           int lineNum,
                           FILE * log,
                           bool syntaxCheck);
  /**
  Parse a print statement, if OK send message to all connected clients
  in the form \<push text="text-part" value="result-of-expression"/> the
  syntax for the print is: print text expression\n, as an example
  print "took wrong turn after [m]" distSoFar */
  bool parsePrintStatement(USeqLine * line,
                          int lineNum,
                          FILE * slog,
                          bool syntaxCheck);
  /**
  Handle a drive statement, or just check syntax if syntax check is true.
  if 'log' is not NULL, then report errors here.
  Returns false if statement failed (syntax or otherwise) */
  bool parseLabelStatement(USeqLine * line,
                           int lineNum,
                           FILE * log,
                           bool syntaxCheck);
  /**
  Start a forward drive command. Command is not issued, but just checked, if
  syntaxCheck is true.
  Returns true if no error occured.
  Returns false if an error is found, and errorTxt is set with a description. */
  bool parseFwdCmd(USeqLine * line,
                   const int lineNum,
                   bool syntaxCheck);
  /**
  Start a gotowaypoint command. Command is not issued, but just checked, if
  syntaxCheck is true.
  Returns true if no error occured.
  Returns false if an error is found, and errorTxt is set with a description. */
/*  bool parseGotoWaypointCmd(USeqLine * line,
                            const int lineNum,
                            bool syntaxCheck);*/
  /**
  Start a turn command. Command is not issued, but just checked, if
  syntaxCheck is true.
  Returns true if no error occured.
  Returns false if an error is found, and errorTxt is set with a description. */
  bool initiateTurnCmd(USeqLine * line,
                       const int lineNum,
                       bool syntaxCheck);
  /**
  Inwoke a gotowaypont command to this odometry waypoint.
  opt should be ODO, DIRECT or GPS. For GPS is (x,y) parameters in UTM coordinates (E,N).
  Returns true if initiated correctly. */
  bool initiateGotowaypointCmd(const char * opt, double x, double y, double h);
  /**
  Send a smrcl command to MRC.
  Returns true if send (and link to MRC is active) */
  bool initiateSmrClCmd(const char * cmd);
  /**
  Test implicit stop condition, as per the testXxxx variable set at initiation of command.
  Returns true of at least one stop condition is met.
  SMRCL commands can return syntx error, if so the
  'syntaxErr' is set to true. */
  bool testImplicitStopCond(bool * syntaxErr);
  /**
  Test explicit stop condition, as a boolean statement pointed to by 'explicitStopCond'
  if 'syntaxError' then syntax is tested only - no implementation */
  bool testExplicitStopCond(const char * explicitStopCond, bool * syntaxError);
  /**
  Get pointer of function description with this name.
  Returns pointer to description or NULL if nunction is not found. */
  USeqFunction * getFunction(const char * name);
  /**
  Get pointer of function description at this line number.
  Returns pointer to description or NULL if nunction is not found. */
  USeqFunction * getFunction(int lineNum);
  /**
  Get index of this function pointer in the function list */
  int getFunction(USeqFunction * func);
  /**
  Get line number for declaration of current function */
  int getFunctionStart(int lineNum);
  /**
  Get pointer to function structure for current function */
  USeqFunction * getFunctionCurrent(int lineNum);
  /**
  Get pointer to a named watch structure */
  USeqWatch * getWatch(const char * name);
  /**
  Get a (or try to) value from the connected MRC.
  Returns false if variable is unknown or MRC is not connected */
  virtual bool getSmrdemoValue(const char * name, double * value);
  /**
  Open simulation file, and if successfull set simulated to true.
  Returns true is in simulated mode. */
  bool simOpenReplayFile();
  /**
  Get next replay line up to this time, load the lines in
  sequencer program, and return line number of line
  to execute (return 'lineActive' if no newer lines are loaded.
  Returns 'linesCnt' if no more lines are available in replay file.
  Reads from the file handle 'simc', opened by 'simOpenReplayFile()' */
  int getSimLine(UTime toTime);

public:
  /**
  Lock for sequencer, to ensure integrity */
  ULock seqLock;

protected:
  /**
  The program to sequence */
  USeqLine lines[MAX_SQE_LINE_CNT];
  /**
  Number of lines used in sequence */
  int linesCnt;
  /**
  Active line in sequencer (main process, not watches= */
  int lineActive;
  /**
  Watch list */
  USeqWatch watch[MAX_SEQ_WATCH_LINES];
  /**
  Number of used watch lines */
  int watchCnt;
  /**
  The sequencer stack */
  int stack[MAX_SEQ_STACK_SIZE];
  /**
  Size of used stack */
  int stackCnt;
  /**
  Parameter stack - for use in function cals */
  double params[MAX_SEQ_PARAM_STACK_SIZE];
  /**
  Number of used params in stack */
  int paramsCnt;
  /**
  List of functions */
  USeqFunction funcs[MAX_SEQ_FUNCTIONS];
  /**
  Number of defined functions */
  int funcsCnt;
  /**
  Reference to function and variable calculator */
  UCalcSeq * calc;
  /**
  Space for error text in parsing statements */
  char errorTxt[MAX_ERROR_SIZE];
  /**
  Is the current line an activated drive command.
  If so the implicit and explicit stop conditions must be
  tested.
  Is set by USequencer::parseDriveStatement(...). */
  int inDriveCmd;
  /**
  Line in wich the current drive command takes place.
  Is set by parseDriveStatement(...). */
  int inDriveCmdLine;
  /**
  Max size of cmd identifier - used by inDriveCmdThis[.] */
  static const int MAX_CMD_IDENTIFIER_SIZE = 100;
  /**
  Copy of identifier for current drive command */
  char inDriveCmdThis[MAX_CMD_IDENTIFIER_SIZE + 1];
  //
  /**
  simulation filename (for status print) */
  char simFileName[MAX_FILENAME_SIZE];
  /**
  Logfile filename (for status print) */
  char logFileName[MAX_FILENAME_SIZE];
  /**
  Is the run simulated (replay or use live command) */
  bool simulated;
  /**
  Name of last loaded plan (for status print) */
  char loadedPlanName[MAX_FILENAME_SIZE];
  /**
  Current function */
  USeqFunction * currentFunction;
  /**
  Skip line to be executed to end an idle sequence (if valid) */
  USeqLine skipLine;
  /**
  Logfile for sequencer */
  FILE * logSeq;

private:
  /**
  File handle for simulated file commands */
  FILE * simc;
  /**
  Last line read from simfile. This is the next line to use */
  char simLine[MAX_SIMLINE_SIZE];
  /**
  Time extracted from simLine */
  UTime simLineTime;
};

#endif

