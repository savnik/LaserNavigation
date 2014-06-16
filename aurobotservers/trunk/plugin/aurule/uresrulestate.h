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

#ifndef URESRULESTATE_H
#define URESRULESTATE_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/usmrcl.h>
#include <urob4/uresbase.h>
#include <urob4/uvarcalc.h>
#include <urob4/ulogfile.h>

#include "urule.h"

class UResRule;

/**
 * Class that holds the current execution state of a mission plan.
 * The state has pointers to all sub-plans, that (if active) are to
 * be executed at every new step.
 * The state holds info of any calls to other plans, that are to be
 * executed as part of the main part of the plan.
 * The state also has a error report in 'errLevel' (0=OK,
 * 1=warning (continues), 2=error (execution may be halted), and
 * an error text in errorTxt.
 * The class generates (or may generate) a number of local variables, and will
 * be created on the global variable stack. */

class UMisRuleState : public UVarCalc, public UVarMethodImplement
{
public:
  /**
   * Constructor */
  UMisRuleState();
  /**
   * Destructor */
  ~UMisRuleState();
  /**
   * Execute a step in this plan. This means
   * allow all active subplans to step first, then
   * make a step of the main function of this plan.
   * \param breakLevels the levels og plans that are to be removed, i.e. if
   * breakLevels returns 1 when this level is completed, that is all
   * postLines are executed and all postLines of subRules are executed too.
   * \param aLastCall when true, this is the last call to this plan, and
   * the plan should do a last call and run the post-lines.
   * \returns true if the step did not meet an execution error
  */
  bool step(int * breakLevels, const bool aLastCall);
  /**
   * Set this plan state to the plan described in this plan structure.
   * \param plan plan to prepare state for.
   * \param hasParent is the pointer to the parent plan - or NULL if
   * this is the root plan.
   * \param logFile is a pointer to a logfile where sufficient information is to be saved to allow replay
   * \returns true if successfull */
  bool setRule(UMisRule * plan, UMisRuleState * hasParent, ULogFile * logFile);
  /**
   * Remove all state info and all state variables related to this mission state */
  void remove();
  /**
   * Remove this named subplan from the state structure (and all its sub-structures)
   * \param name is the plan name to be removed from the statate structure
   * \returns true if plan is found and removed. */
  bool remove(const char * name);
  /**
   * Get a pointer to current plan
   * \returns NULL if no plan is loaded. */
  UMisRule * getRule()
  { return misRule; };
  /**
   * Activate or inactivate this plan.
   * if set active (doStart=true) and is active already, then state is not changed
   * if doStart=false and is active, then run post conde and set to inactive
   * \param doStart sets the state to init if true, else set the state to inavtive.
   * \param onceOnly activate the plan once only, i.e. no automatic repeat.
   * \returns true */
  bool setActive(bool doStart, bool onceOnly);
  /**
   * Add this plan line as new sub-plan to this sequencer state.
   * \param plan is a pointer to the plan to be added.
   * \param activate, if true the plan is set for automatic restart and active
   * \returns true if the plan was added with no errors.
   * else the errors are reported in errorTxt. */
  bool addRule(UMisRule * plan, bool activate);
  /**
   * A varPool method with this class as implementor is called.
   * \param name is the name of the called function
   * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
   * \param params is an array of variable pointers with the actual parameters, in the order specified by order
   * \param returnStruct is an array of class object pointers that can return values or objects (may be NULL) if no result value is needed (a procedure call)
   * \param returnStructCnt is the number of objects in the returnStruct buffer
   * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
  virtual bool methodCallV(const char * name, const char * paramOrder,
                           UVariable * params[],
                           UDataBase ** returnStruct = NULL,
                           int * returnStructCnt = NULL);
  /**
   * A varPool method with this class as implementor is called.
   * \param name is the name of the called function
   * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
   * \param strings is an array of string pointers for the string type parameters (may be NULL if not used)
   * \param doubles is an array with double typed parameters (may be NULL if not used)
   * \param value is the (direct) result of the class, either a double value, or 0.0 for false 1.0 for true
   *              (2.0 for implicit stop if a controll call from mission sequencer).
   * \param returnStruct is an array of class object pointers that can be used as parameters or return objects (may be NULL)
   * \param returnStructCnt is the number of objects in the returnStruct buffer
   * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
  virtual bool methodCall(const char * name, const char * paramOrder,
                          char ** strings, const double * doubles,
                          double * value,
                          UDataBase ** returnStruct = NULL,
                          int * returnStructCnt = NULL);
  /**
   * Set the value of the actual parameters.
   * \param dpar is an array of doubles with the new value for the parameters
   * \param dparCnt is the number of valid parameters.
   * \returns true if all parameters were set. */
  bool setParameters(double dpar[], int dparCnt);
  /**
   * Set the value of the actual parameters.
   * \param params is an array of UVariable with the new value for the parameters
   * \param paramsCnt is the number of valid parameters.
   * \returns true if all parameters were set. */
  bool setParameters(UVariable * params[], int paramsCnt);
  /**
   * Get a state string, with the state of all subplans.
   * \param pre is a prestring printed before all lines
   * \param buff is the character buffer where to put the data
   * \param buffCnt is the length of the buff prameter
   * \returns false if buffer overflows */
  bool getStateStr(const char * pre, char * buff, const int buffCnt);
  /**
   * get number of subplans */
  int getSubRulesCnt()
  { return subRulesCnt; };
  /**
   * Set logfile */
  void setLogFile(ULogFile * logFile)
  { logf = logFile; };
  /**
   * Send a string to the logfile
   * This function can tolerate that the logfile pointer is not set and the logfile is not open. */
  void toLog(const char * logString);
  /**
   * Is the logfile open.
   * \returns false if the logfile pointer is not set and if the logfile is closed. */
  bool isLogOpen();
  /**
   * \brief Start or stop one of the subplans for this mission
   * Stating a started or stopping a stopped plan makes no change
   * Stopping a started plan makes the plan execute any post-lines before inactivation
   * Starting an rule sets also the auto-repeat flag
   * \param planName is the plan name to be manipulated
   * \param start determines wheather the plan is to be activated or deactivated  */
  bool startStopRule(const char * planName, bool start);
  /**
   * \brief is the subplan with this name active
   * \param planName name of subplan.
   * \returns true if the plan exist and is not inactive */
  bool isActive(const char * planName);
  /**
   * Is the state of this plan active
   * \returns true if the state is not inactive */
  bool isActive()
  { return blockState != inactive; };
  /**
   * \brief get the state structure of the first caller of the plan with this name.
   * The caller makes the plan busy, so it is nice to know who is calling this plan.
   * \param name the name of the plan
   * \returns a pointer to the state structure of the direct caller of the plan */
  UMisRuleState * getCaller(const char * name);
  /**
   * \brief Get called plan state
   * If this plan is waiting in a called plan, get the called plan state
   * \returns state of the called, or NULL if not in a call. */
  UMisRuleState * getCalled()
  { return inCall; };


protected:
  /**
   * Create variables and local functions */
  virtual void createBaseVar();
  /**
   * run lines starting at seqLine
   * \param breakLevels [out] set to number of levels to break
   * if a break statement is reached.
   * \param maxStatements is -1 of no limit, else max number of statements, used by loops to allow just one statement.
   * \returns RV_OK ot RV_AGAIN if no errors are found
   * else RV_SYNTAX_ERROR and the errors are reported in errorTxt. */
  UMisItem::ResultValue runLines(int * breakLevels, int maxStatements);
  /**
   * Execute the line in seqLine.
   * \param lastCall is used if the line is a control statement, where
   * a last call must be executed when leaving the statement.
   * \param breakLevels is the result of a break level
   * \returns ar result value normally RV_OK_DONE or
   * RV_OK_AGAIN if in a control statement */
  UMisItem::ResultValue runLine(bool lastCall, int * breakLevels);
  /**
   * Convert parameter line to variables.
   * \param syntaxCheck when true no variables are created, just convert to names and values.
   * \returns true if all values can be converted to double and all identifiers are valid. */
  bool createParameterVar(bool syntaxCheck);
  /**
   * Prepare a plan (a call or an in-line block) for a step.
   * \param inLineBlock the call is an in-line block rather than a call to a plan
   * \param extraPar set true if in a control call, then the repeat count is added
   * \param extraParVal is the vale added if extra parameter is needed
   * \returns RV_OK if call is an available call (e.g. sin(0.56)) or RV_OK_AGAIN if call is a plan and needs a step. */
  UMisItem::ResultValue initCall(bool inLine, bool extraPar, int extraParVal);
  /**
   * Get related plan, try:
   * 1 daughter plans,
   * 2 parent's daughter plans (sisters),
   * 3-N grand-parent's daughters until no more parents
   * \param name name of the plan
   * \returns a pointer to the first found plan */
  virtual UMisRule * getSubRule(const char * name);
  /**
   * Get related plan state, looking in this order:
   * 1 daughter plans,
   * 2 parent's daughter plans (sisters),
   * 3-N grand-parent's daughters until no more parents
   * \param name name of the plan
   * \returns a pointer to the first found plan state */
  UMisRuleState * getSubRuleState(const char * name);
  /**
   * Is the state main, that is sequencer is in a control-command
   * \returns true if state is main */
  bool isInMain()
  { return blockState == main; };
  /**
   * Enable the plan state with this name, if it is within scope.
   * Scope is current plans subplans and any direct parent's subplan.
   * \param planName the name of the plan
   * \param value is true when the plan should be enables and false if to be disabled.
   * \returns true if a plan of that name is found */
  UMisItem::ResultValue runEnable(UMisEnable * eLine);
  /**
   * \brief Implement a wait control method
   * when called first time (repeat == 0) the start time is stored.
   * on subsequent calles (repeat > 0) then time passed is calculated in return value
   * is set accordingly.
   * \param secs, then time to wait (may change from call to call)
   * \param repeat count - assumed to be incremented for each wait call
   * \returns 1 if time has not passed and 2 if the time has passed */
  int methodWait(double secs, int repeat);
  /**
   * Execute a break
   * This is done by finding the level of wich to break.
   * If level is undefined, then default is 1 level, i.e. break current plock
   * If level is a number, then this number determines the break level (0 is no break)
   * if level is a plan (or block) identifier, then the levels to this block is used.
   * \param breakLevels is set after the call to the number of levels to break.
   * \returns RV_OK if no syntax error is found. */
  UMisItem::ResultValue runBreak(int * breakLevels);
  /**
   * \brief Execute a call to a rule
   * \param breakLevel should return the number of levels to break, and left
   * unchanged if no break levels were executed.
   * \param lastCall if set to true, then a control statement is terminated and any post code run
   * \returns RV_OK if completed or RV_AGAIN if stuck in
   * a control statement, the control statement is the (or in) the
   * statement in loopLine. May also return RV_SYNTAX_ERROR */
  UMisItem::ResultValue runCall(int * breakLevels, bool lastCall);
  /**
   * \brief Execute a control call
   * \param breakLevel should return the number of levels to break, and left
   * unchanged if no break levels were executed.
   * \param lastCall if set to true, then a control statement is terminated and any post code run
   * \returns RV_OK if completed or RV_AGAIN if stuck in
   * a control statement, the control statement is the (or in) the
   * statement in loopLine. May also return RV_SYNTAX_ERROR */
  UMisItem::ResultValue runControl(int * breakLevels, bool lastCall);
  /**
   * Run the main part of the plan starting at the line in seqLine.
   * \param breakLevel should return the number of levels to break, and left
   * unchanged if no break levels were executed.
   * \param lastCall when true, the lines should be get the final call, as needed before the post lines.
   * \returns true if no syntax error were detected (in main lines) */
  bool runMainLines(int * breakLevel, const bool lastCall);
  /**
   * Run main lines in a plan with just command parameters
   * \returns true if all lines could be fitted as parameter to the call */
  bool runRuleCommands();
  /**
   * Log the actual call to a function with all parameters evaluated
   * \param lineNumber is the line number from source
   * \param funcName is the name of the function (before ())
   * \param parOrder is the parameter order (only s and d are valid)
   * \param sppar is an array of string parameters
   * \param dpar is an array of double parameters */
  void logActualCall(const int lineNumber,
                      const char * funcName,
                      const char * parOrder,
                      char * sppar[],
                      double dpar[]);
  /**
   * Log the actual call to a function with all parameters evaluated
   * \param lineNumber is the line number from source
   * \param funcName is the name of the function (before ())
   * \param parOrder is the parameter order (only s and d are valid)
   * \param params is the parameter array of UVariable values */
  void logActualCallV(const int lineNumber,
                      const char * funcName,
                      const char * parOrder,
                      UVariable ** params);
  /**
   * Skip a statement after an if or an else statement
   * \param anIf is true if it was just after an 'if' statement
   * \returns true if skipped as requested. */
  bool skipOneStatement(bool anIf);
  /**
   * Run this loop statement.
   * \param lLine is the loop definition line
   * \param breakLevels set to 1..N if a break line is found
   * \returns RV_OK if completed or RV_AGAIN if stuck in
   * a control statement, the control statement is the (or in) the
   * statement in loopLine. May also return RV_SYNTAX_ERROR */
  UMisItem::ResultValue runLoop(int * breakLevels, UMisLoop * lLine);
  /**
   * Run a switch statement, evaluation the switch value and setting the value
   * in the next block structure */
  UMisItem::ResultValue runSwitch(UMisItem * nextLine);
  /**
   * Advance program counter to after the right case statement */
  void advanceToCase();
  /**
   * A case statement is found terminating the statements in a case construction
   * \param breakLevels is set to 1 on exit
   * \returns RV_OK */
  UMisItem::ResultValue runCase(int * breakLevels);

public:
  typedef enum {inactive, init, main, post} BlockState;

protected:
  /**
   * state of this plan block */
  BlockState blockState;
  /**
   * parent block that called this plan */
  UMisRuleState * parent;
  /**
   * Mission plan definition */
  UMisRule * misRule;
  /**
   * max number of subplans allown in any plan */
  static const int MAX_SUBPLANS = 100;
  /**
   * Subplans defined by this plan */
  UMisRuleState * subRules[MAX_SUBPLANS];
  /**
   * Number of subplans defined */
  int subRulesCnt;
  /**
   * Next line to be executed in this  */
  UMisItem * seqLine;
  /**
   * Counter of the number of times a process line is called */
  int seqAgainCnt;
  /**
   * When next line is in a (nested) call, then the
   * called function is block in its own right and the state
   * of this block is at the end of this pointer. Ef not in a call, then
   * this pointer is NULL */
  UMisRuleState * inCall;
  /**
   * The block where an error is reported - NULL if no error */
  UMisRuleState * errBlock;
  /**
   * the error level - 0 is no error, 1 is warning, 2 is error */
  int errLevel;
  /**
   * Error count, to suppress excessive printout and log */
  int errCnt;
  /**
   * Set to automatic restart.
   * Automatic restart is enabled as a separate thing to do 'all the time'
   * This will reset the state to 'init' after the 'post' is finished,
   * otherwise the state will change to 'inactive' after 'post' is finished */
  bool automaticRestart;
  /**
   * Cycle counter for this automatic restart plan */
  int automaticRestartCnt;
  /**
   * Index to remaining wait time */
  UVariable * varWait;
  /**
   * start time for wait method */
  UTime waitStart;
  /**
   * index to parameter variables - the count is in misRule->parametersCnt */
  UVariable ** varParameters;
  /**
   * logfile to log any execution statements  */
  ULogFile * logf;
  /**
   * Maximum number of nested if statements */
  static const int inIfCntMax = 10;
  /**
   * Result of last if evaluation */
  bool inIf[inIfCntMax];
  /**
   * nesting level of if statements */
  int inIfCnt;
  /// is the rule initialized
  bool initialized;
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class UMisSeqRoot : public UMisRuleState
{
public:
  /**
   * Constructor */
  UMisSeqRoot()
  {
    missions = NULL;
    createBaseVar();
  };
  /**
   * Destructor */
  ~UMisSeqRoot() {};
  /**
   * Set the pointer to the mission resource */
  inline void setMissions(UResRule * mis)
  { missions = mis; };
  /**
   * get the pointer to the mission resource */
  inline UResRule *  getMissions()
  { return missions; };
  /**
   * Get related plan, try:
   * 1 daughter plans,
   * 2 parent's daughter plans (sisters),
   * 3-N grand-parent's daughters until no more parents
   * 4 search root level missions, that has is not known to the sequencer yet.
   * \param name name of the plan
   * \returns a pointer to the first found plan */
  virtual UMisRule * getSubRule(const char * name);
  /**
   * Execute a step in this plan. This means
   * allow all active subplans to step first, then
   * make a step of the main function of this plan.
   * \param breakLevels the levels og plans that are to be removed, i.e. if
   * breakLevels returns 1 when this level is completed, that is all
   * postLines are executed and all postLines of subRules are executed too.
   * \param aLastCall when true, this is the last call to this plan, and
   * the plan should do a last call and run the post-lines.
   * \returns true if the step did not meet an execution error
   */
  bool step(int * breakLevels, const bool aLastCall);
  /**
   * A varPool method with this class as implementor is called.
   * \param name is the name of the called function
   * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
   * \param strings is an array of string pointers for the string type parameters (may be NULL if not used)
   * \param doubles is an array with double typed parameters (may be NULL if not used)
   * \param value is the (direct) result of the class, either a double value, or 0.0 for false 1.0 for true
   *              (2.0 for implicit stop if a controll call from mission sequencer).
   * \param returnStruct is an array of class object pointers that can be used as parameters or return objects (may be NULL)
   * \param returnStructCnt is the number of objects in the returnStruct buffer
   * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
//   virtual bool methodCall(const char * name, const char * paramOrder,
//                           char ** strings, const double * doubles,
//                           double * value,
//                           UDataBase ** returnStruct = NULL,
//                           int * returnStructCnt = NULL);

protected:
  /**
   * Create variables and local functions */
  virtual void createBaseVar();

protected:
  /**
   * Pointer to the missions resource, to get hold of other root plans */
  UResRule * missions;
  /**
   * Index to step counter */
  UVariable * varSteps;
};


/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResRuleState : public UResVarPool,
                   protected UServerPush,
                   public ULogFile
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResRuleState) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResRuleState()
  { // these first two lines is needed
    // to save the ID, version number and description
    setResID("rulestate", 205);
    // set description for global variables owned by this resource (optional)
    setDescription("rule sequencer", false);
    // set logfile name to the same as this resource
    setLogName(getResID());
    //openLog();
    // create global variables for this resource
    createBaseVar();
    // client to get print statements
    printClientNumber = -1;
    // start sequencer interval thread
    running = false;
    terminate = false;
    // name of (fake) mission plan used to populate the root mission state
    rootRule.setName("state");
    // start the sequencer loop thread
    start();
  };
  /**
  Destructor */
  virtual ~UResRuleState();
  /**
  Called by the server core, when a new resource is
  available (or is removed), local pointers to the resource should be
  updated as appropriate. */
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);

public:
  /**
   * Add this plan as a plan that is ready to be executed
   * If a plan is set already, this plan is terminated and removed.
   * \param plan is a pointer to the plan to be set. */
  bool addRule(UMisRule * plan);
  /**
   * Get number of plans loaded for execution */
  int getRulesCnt();
  /**
   * Remove the plan with this name and delete all sub state structures and variables
   * \param name of the plan to be removed
   * \returns true if plan is found (and removed) */
  bool removeRule(const char * name);
  /**
   * Sequencer run loop - must be public, as it is called from external C-function */
  void run();
  /**
   * Make one sequencer iteration - usually called after a sequencer time event */
  bool seqIterateStep();
  /**
   * Set number of iteration steps to run */
  void setSteps(int value);
  /**
   * Is the sequencer in stepping mode.
   * \returns true if not in continuous mode */
  bool isStepping();
  /**
   * Is the sequencer in stepping mode and all steps finished (paused)
   * \returns true if no steps left (and execution not resumed) */
  bool isStopped();
  /**
   * A varPool method with this class as implementor is called.
   * \param name is the name of the called function
   * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
   * \param params is an array of variable pointers with the actual parameters, in the order specified by order
   * \param returnStruct is an array of class object pointers that can return values or objects (may be NULL) if no result value is needed (a procedure call)
   * \param returnStructCnt is the number of objects in the returnStruct buffer
   * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
  virtual bool methodCallV(const char * name, const char * paramOrder,
                          UVariable * params[],
                          UDataBase ** returnStruct = NULL,
                          int * returnStructCnt = NULL);
  /**
   * Add the standard math methods - if they are not there already - and set
   * the implementation object to this resource.
   * \returns true if the methods are added or existed already */
  bool addMathMethods();
  /**
   * Get a state string, with the state of all subplans.
   * \param pre is a prestring printed before all lines
   * \param buff is the character buffer where to put the data
   * \param buffCnt is the length of the buff prameter
   * \returns false if buffer overflows */
  bool getStateStr(const char * pre, char * buff, const int buffCnt);
  /**
   * Send this sequencer text to this client
   * \param client is the socket connected client (or fake client)
   * \param helpTxt is the text to be send. */
  void sendToClientAsHelp(const int client, const char * helpTxt);
  /**
   * Set reply client number.
   * This is used when the sequencer has a message to the used, e.g. a print or a syntax error.
   * \param client the client number - index to the clients socket connection. */
  void setPrintClientNumber(int client)
  { printClientNumber = client; };
  /**
   * Get iteration count number */
  int getIterationCnt()
  { return varIterations->getInt(); };
  /**
   * \brief Start or stop a plan.
   * Rule is set active or inactive acording to 'start' value.
   * If stopping a plan that is started, then the post part of the plan is executed before inactivation
   * If the plan is a rule (has if="" part) then the plan is started with automatic repeat */
  bool startStopRule(const char * planName, bool start);
  /**
   * \brief Is this (top-level) plan active or not
   * Tests the state flag, and return true if not inactive.
   * \param planName name of one of the sub-plans
   * \returns true if plan exist and state is not inactive */
  bool isActive(const char * planName);
  /**
   * \brief get the state structure of the first caller of the plan with this name.
   * The caller makes the plan busy, so it is nice to know who is calling this plan.
   * \param name the name of the plan
   * \returns a pointer to the state structure of the direct caller of the plan */
  UMisRuleState * getCaller(const char * name);

protected:
  // Locally used methods
  /**
  Create the smrif related variables */
  void createBaseVar();
  /**
   * Start sequencer run loop */
  bool start();
  /**
   * Stop sequencer run loop */
  virtual void stop(bool andWait);

protected:
  /** index to sample time for sequencer */
  UVariable * varSampleTime;
  /** index to mission start time */
  UVariable * varStartTime;
  /** index to time into mission */
  UVariable * varMissionTime;
  /** index to sequence iteration counter */
  UVariable * varIterations;
  /** if stepping, this is the number of steps remaining */
  UVariable * varStepsLeft;
  /**
   * line parsed currently */
  UMisSeqRoot state;
  /**
   * The (empty) root plan, that just allows loaded subplans to be run */
  UMisRule rootRule;
  /**
   * terminate control loop.
   * Should be false at all times, except at shutdown. */
  bool terminate;
  /**
   * Is the thread running */
  bool running;
  /**
   * Starttime of last loop */
  UTime loopStartTime;
  /**
   * Thread handle for loop-thread */
  pthread_t thSeq;
  /**
   * Number of steps to be executed -1 is continue, 0 is no steps */
  int stepsRemaining;
  /**
   * client number of the connection who should get then print calls. */
  int printClientNumber;
};

#endif

