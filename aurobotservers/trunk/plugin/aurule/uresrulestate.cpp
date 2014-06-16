/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
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

#include <stdio.h>
#include <math.h>

#include <urob4/uvarcalc.h>
#include <ugen4/usmltagin.h>
#include <urob4/ucmdexe.h>
#include <ugen4/u2dline.h>

#include "uresrulestate.h"
#include "uresrule.h"


UMisRuleState::UMisRuleState()
{
  subRulesCnt = 0;
  parent = NULL;
  inCall = NULL;
//  varPaused = -1;
  blockState = inactive;
  errBlock = NULL;
  errLevel = 0;
  errCnt = 0;
  seqAgainCnt = 0;
  automaticRestart = false;
  createBaseVar();
  varParameters = NULL;
  automaticRestartCnt = 0;
  logf = NULL;
  inIfCnt = 0;
  misRule = NULL;
  initialized = false;
}


///////////////////////////////////////////

UMisRuleState::~UMisRuleState()
{
  int i;
  //
  if (varParameters != NULL)
    free(varParameters);
  for (i = subRulesCnt-1; i >= 0; i--)
    delete subRules[i];
  subRulesCnt = 0;
}

///////////////////////////////////////////

void UMisRuleState::createBaseVar()
{
  varWait = addVar("waitLeft", 0, "d", "(r) Remaining wait period in seconds, used by wait method");
    // add methods
  addMethod(this, "wait", "dd", "wait a number of seconds (parameter 1), parameter 2 is repeat count starting at zero. "
        "Returns 1 if wait period is not expired, 2 if it is expired, 0 if parameter error");
  addMethod(this, "wait", "d", "wait endlessly, parameter 2 is repeat count. Returns 1");
  addMethod(this, "if", "", "Returns the result of the rule condition (i.e. the if attribute in the rulde definition, if rule has no if, then if returns false.");
  addMethod(this, "eval", "s", "Evaluate this string as an ordinary legal expression.");
  addMethodV(this, "eval", "s", "Evaluate this string as an ordinary legal expression.");
  addMethodV(this, "name", "", "Return rule name as a string");
}


///////////////////////////////////////////

bool UMisRuleState::setActive(bool doStart, bool onceOnly)
{
  int br = 0;
  //
  if (doStart and blockState == inactive)
  { // OK activate
    blockState = init;
  }
  else if (blockState == main and not doStart)
  { // stopping this rule, so go a last round first
    step(&br, true);
    blockState = inactive;
  }
  else if (not doStart)
  { // not in main, but set to inactive anyhow
    // could - most likely - be in init state
    blockState = inactive;
  }
  automaticRestart = not onceOnly;
  if (automaticRestart)
    automaticRestartCnt = 0;
  //
  return true;
}

///////////////////////////////////////////

bool UMisRuleState::step(int * breakLevels, const bool aLastCall)
{
  bool result = true;
  int myBreakLevel = 0;
  bool lastCall = aLastCall;
  UVariable * dva = NULL;
  const char * p1;
  bool isOK, conditionOK;
  UMisItem::ResultValue rv;
  //
  *breakLevels = 0;
  if (blockState != inactive)
  {
    if (blockState == init and not lastCall)
    {
      if (misRule->condition != NULL)
      {
        dva = new UVariable();
        isOK = evaluateV(misRule->condition, misRule->condition, &p1, dva, false);
        conditionOK = isOK and absf(dva->getValued()) > 0.5;
        delete dva;
        if (not isOK)
        { // error reporting
          if (errCnt < 5)
          { // limit the number of error reports
            printf("SYNTAX error rule condition line %d: %s\n", misRule->getLineNumber(), errorTxt);
            logf->toLog("# SYNTAX error rule condition", misRule->getLineNumber(), errorTxt);
          }
          else if (errCnt == 5)
          {
            printf("SYNTAX error rule condition (last error report in): %s\n", errorTxt);
            logf->toLog("# SYNTAX error rule condition (last error report in)", misRule->getLineNumber(), errorTxt);
          }
          errCnt++;
        }
      }
      else
        conditionOK = true;
      // change state to main
      // init block is executed only once, not in a step
      if (conditionOK)
      { // rule (condition) is OK, so do the main section
        if (not initialized)
        { // run the constructor lines
          seqLine = misRule->getIniLines();
          rv = runLines(&myBreakLevel, -1);
          result = rv != UMisItem::RV_SYNTAX_ERROR;
          initialized = true;
        }
        if (myBreakLevel > 0 or not result)
        { // syntax error in init-code or
          // a break in the init-code, skip to pose code and exit
          blockState = post;
          if (result)
            myBreakLevel--;
        }
        else
        { // normal case - no break in init block
          blockState = main;
          seqLine = misRule->getMainLines();
          if (misRule->isSwitch)
          { // skip until the right case is found
            advanceToCase();
          }
        }
        seqAgainCnt = 0;
      }
    }
    if (blockState == main)
    { // execute or continue main part of rule (or block)
      if (misRule->destination != NULL)
      { // block is just a list of parameters
        result = runRuleCommands();
        blockState = post;
      }
      else
        // plan is executable statements
        result = runMainLines(&myBreakLevel, lastCall);
      if ((seqLine == NULL or (myBreakLevel > 0)) and blockState == main)
      { // all main lines are finished - change to post
        blockState = post;
        if (myBreakLevel > 0 and (misRule->isFull or misRule->isSwitch))
          myBreakLevel--;
      }
    }
    if (result and blockState == post)
    { // this is the final set of lines
      seqLine = misRule->getPostLines();
      rv = runLines(&myBreakLevel, -1);
      result = (rv != UMisItem::RV_SYNTAX_ERROR);
      if (seqLine == NULL)
      { // all post lines are finished - terminate or restart
        if (automaticRestart)
        {
          blockState = init;
          automaticRestartCnt++;
        }
        else
        { // terminate rule
          blockState = inactive;
          //
          //printf("UMisRuleState::step: line %d finished the %s plan (now set to inactive)\n",
          //       getRule()->getLineNumber(),  misRule->getName());
        }
      }
    }
  }
  if (myBreakLevel > 0)
    // return break level to next level if unhandled
    *breakLevels = myBreakLevel;
  return result;
}

///////////////////////////////////////////

bool UMisRuleState::runMainLines(int * breakLevel, const bool lastCall)
{
  bool result = true;
  int i, bl;
  UMisItem::ResultValue rv;
  UMisRuleState * sp;
  bool isSorted = false;
  //
  while (not isSorted)
  { // do a boble sort - fast, when almost sorted
    isSorted = true;
    for (i = 0; i < subRulesCnt - 1; i++)
    { // ensure rules are in execution order
      sp = subRules[i+1];
      if (subRules[i]->getRule()->order > sp->getRule()->order)
      { // swap the two rules
        subRules[i+1] = subRules[i];
        subRules[i] = sp;
        isSorted = false;
      }
    }
  }
  for (i = 0; i < subRulesCnt; i++)
  { // allow any sub-plans to procede if active
    bl = 0;
    subRules[i]->step(&bl, lastCall);
    if (bl > 0)
      // make the most demanding break
      *breakLevel = max(*breakLevel, bl);
  }
  if (*breakLevel > 0)
  { // a break is detected that breaks this plan too
    if (not lastCall)
    { // subplans need to have a last call
      for (i = 0; i < subRulesCnt; i++)
      {
        bl = 0;
        subRules[i]->step(&bl, true);
        if (bl > 0)
          // make the most demanding break
          *breakLevel = max(*breakLevel, bl);
      }
    }
    blockState = post;
    if (*breakLevel > 0 and misRule->isFull)
      (*breakLevel)--;
  }
  if (lastCall)
  { // the block is to be terminated - stop it here
    if (seqLine != NULL)
    { // if a control line, then run it one more time
      if (seqLine->isA("misControl"))
      { // run this control line a last time
        rv = runLine(lastCall, breakLevel);
        if (not (rv == UMisItem::RV_OK or rv == UMisItem::RV_OK_AGAIN))
          result = false;
      }
    }
    // skip to post-lines
    blockState = post;
  }
  else
  { // continue the main block as normal
    rv = runLines(breakLevel, -1);
    result = (rv != UMisItem::RV_SYNTAX_ERROR);
  }
  return result;
}

//////////////////////////////////////////

void UMisRuleState::advanceToCase()
{
  UMisCase * lc;
  int swv;
  //
  swv = getRule()->switchValue;
  while (seqLine != NULL)
  {
    if (seqLine->isA("misCase"))
    {
      lc = (UMisCase*)seqLine;
      if (lc->isInList(swv))
        break;
    }
    else if (seqLine->isA("misDefault"))
      break;
    seqLine = seqLine->next;
  }
  if (seqLine != NULL)
    // advance to the line after the case
    seqLine = seqLine->next;
}

///////////////////////////////////////////

bool UMisRuleState::runRuleCommands()
{
  const int MPL = 30;
  UVariable * param[MPL];
  int paramCnt;
  char parOrder[MPL];
  UMisString * seqStr;
  const char * p2, *p1;
  bool result = true;
  //
  seqStr = (UMisString*) seqLine;
  while (seqStr != NULL)
  {
    p1 = seqStr->getLine();
    p2 = p1;
    paramCnt = 0;
    result = evaluateParametersV(p1,       // source line (for error reporting)
                                &p2,      // position of parameter start i.e. an '(' - returns at end ')'
                                parOrder, // order - e.g. "ds" for (double, string)
                                param,     // array of double parameters
                                &paramCnt, // count of double sized parameters found
                                MPL       // max size of parameter array
                                );
    if (strlen(parOrder) > 0 or not isRemark(p2))
    { // not just a remark
      logActualCallV(seqStr->getLineNumber(), misRule->destination, parOrder, param);
      if (not result)
      {
        printf("Expected parameter list, found %c at %d in %s\n", *p2, p2 - p1, p1);
        logf->toLog("# call SYNTAX ERROR in parameter list", seqStr->getLineNumber(), p2 - p1, p1);
      }
      // try to spot a C-implemented function in current scope
      result = callScopeV(misRule->destination, parOrder, param, NULL, 0);
      //
      if (not result)
      { // syntax error - stop?
        if (errCnt < 5)
        {
          printf("# call SYNTAX ERROR: line %d, parameter list error: %s(%s) from %s(%s)\n",
                seqStr->getLineNumber(), misRule->destination, parOrder, misRule->destination, p1);
          logf->toLog("# call SYNTAX ERROR parameter list", seqStr->getLineNumber(), p2 - p1, p1);
        }
        else if (errCnt == 5)
        {
          printf("# call SYNTAX ERROR (last): line %d, parameter list error: %s(%s) from %s(%s)\n",
                 seqStr->getLineNumber(), misRule->destination, parOrder, misRule->destination, p1);
          logf->toLog("# call SYNTAX ERROR (last) parameter list", seqStr->getLineNumber(), p2 - p1, p1);
        }
        errCnt++;
        //break;
      }
    }
    seqStr = (UMisString*)seqStr->next;
  }
  return result;
}

// bool UMisRuleState::runRuleCommandsD()
// {
//   const int MDP = 30;
//   double dpar[MDP];
//   const int MSP = 3;
//   const int MSL = 300;
//   char spar[MSP][MSL];
//   char * sppar[MSP] = {spar[0], spar[1], spar[2]};
//   int dparCnt;
//   char parOrder[MDP + MSP];
//   UMisString * seqStr;
//   const char * p2, *p1;
//   bool result = true;
//   double res;
//   //
//   seqStr = (UMisString*) seqLine;
//   while (seqStr != NULL)
//   {
//     p1 = seqStr->getLine();
//     p2 = p1;
//     dparCnt = MDP;
//     result = evaluateParameters(p1,       // source line (for error reporting)
//                                 &p2,      // position of parameter start i.e. an '(' - returns at end ')'
//                                 dpar,     // array of double parameters
//                                 &dparCnt, // count of double sized parameters found
//                                 sppar,    // array of string parameters
//                                 parOrder, // order - e.g. "ds" for (double, string)
//                                 MDP + MSP,// max size of parameter array
//                                 MSP,      // max number of string parameters
//                                 MSL);      // max size of string param list
//     if (strlen(parOrder) > 0 or not isRemark(p2))
//     { // not just a remark
//       logActualCall(seqStr->getLineNumber(), misRule->destination, parOrder, sppar, dpar);
//       if (not result)
//       {
//         printf("Expected parameter list, found %c at %d in %s\n", *p2, p2 - p1, p1);
//         logf->toLog("# call SYNTAX ERROR in parameter list", seqStr->getLineNumber(), p2 - p1, p1);
//       }
//       // try to spot a C-implemented function in current scope
//       result = callScope(misRule->destination, parOrder, sppar, dpar, &res, NULL, 0);
//       //
//       if (not result)
//       { // syntax error - stop?
//         if (errCnt < 5)
//         {
//           printf("# call SYNTAX ERROR: line %d, parameter list error: %s(%s) from %s(%s)\n",
//                  seqStr->getLineNumber(), misRule->destination, parOrder, misRule->destination, p1);
//           logf->toLog("# call SYNTAX ERROR parameter list", seqStr->getLineNumber(), p2 - p1, p1);
//         }
//         else if (errCnt == 5)
//         {
//           printf("# call SYNTAX ERROR (last): line %d, parameter list error: %s(%s) from %s(%s)\n",
//                  seqStr->getLineNumber(), misRule->destination, parOrder, misRule->destination, p1);
//           logf->toLog("# call SYNTAX ERROR (last) parameter list", seqStr->getLineNumber(), p2 - p1, p1);
//         }
//         errCnt++;
//         //break;
//       }
//     }
//     seqStr = (UMisString*)seqStr->next;
//   }
//   return result;
// }

///////////////////////////////////////////

bool UMisRuleState::addRule(UMisRule * plan, bool activate)
{
  bool result = false;
  UMisRuleState * ps;
  //
  if (subRulesCnt < MAX_SUBPLANS)
  {
    ps = new UMisRuleState();
    ps->setRule(plan, this, logf);
    subRules[subRulesCnt++] = ps;
    result = true;
    if (activate)
      // activate acording to plan settings
      ps->setActive(plan->isRule or plan->isRun, not plan->isRule);
  }
  return result;
}

//////////////////////////////////////////////

bool UMisRuleState::setRule(UMisRule * plan, UMisRuleState * hasParent, ULogFile * logFile)
{
  bool result = true;
//  int i = 0;
//  UMisItem::ResultValue rv;
  //
  parent = hasParent;
  if (parent != NULL)
  { // and var pool parent too - set already for root plan
    setPreName(plan->getName());
    setParentVarPool(parent);
  }
  // save pointer to logfile
  logf = logFile;
  // add this plan name as structure in parent var pool
  getParentVarPool()->addStruct(plan->getName(), this);
  // create space for local variables
  createVarSpace(10, 5,0, "rule local variables", false);
  // ini-block
  misRule = plan;
  errLevel = 0;
  inCall = NULL;
  // prepare any sub-plans
  blockState = init;
  // create parameter variables
  if (misRule->parameters != NULL)
    result = createParameterVar(false);
  //
/*  if (result)
  {
    seqLine = misRule->getIniLines();
    // run the constructor lines
    rv = runLines(&i, -1);
    result = rv != UMisItem::RV_SYNTAX_ERROR;
  }*/
  // return to inactive state
  blockState = inactive;
  //
  return result;
}

///////////////////////////////////////////////

bool UMisRuleState::createParameterVar(bool syntaxCheck)
{
  const char *p2;
  const int MNL = MAX_SML_NAME_LENGTH;
  char att[MNL];
  const int MDL = 500; // max length of value buffer
  char val[MDL];
  char ptag[MDL];
  bool result = true;
  USmlTagIn tag;
  int n = 0, m = 0;
  UVariable v;
  // parameters are is in a XML tag format, but needs the frame
  snprintf(ptag, MDL, "<p %s/>", misRule->parameters);
  //
  tag.setTag(ptag);
  //
  if (misRule->parametersCnt > 0)
  {
    varParameters = (UVariable **)malloc(misRule->parametersCnt * sizeof(UVariable *));
  }
  // get parameters in this tag - if any
  while (tag.getNextAttribute(att, val, MDL))
  {
    result = getIdentifier(att, MNL, "_", &p2);
    if (result)
    {
      n = p2 - att;
      result = (*p2 == '\0') and (n <= MAX_VAR_NAME_SIZE);
    }
    if (not result)
    {
      if (syntaxCheck)
        snprintf(errorTxt, MAX_ERROR_SIZE,
               "***param:: syntax error, parameter identifier is invalid in: (%s)\n", att);
      break;
    }
    // get value of expression
    result = evaluateV(val, val, &p2, &v, syntaxCheck);
    // implement
    if (result)
    { // add variable to var-pool
      if (not syntaxCheck and (m < misRule->parametersCnt))
        varParameters[m] = addVar(att, &v, "rule parameter");
      // count parameters
      m++;
    }
    else
      break;
  }
  if (result)
  { // do we have the expected number of parameters
    result = (m == misRule->parametersCnt);
    if (not result)
      printf("Error in parameter %d!=%d evaluation - management error - should be OK\n", m, misRule->parametersCnt);
  }
  return result;
}

///////////////////////////////////////////////

UMisItem::ResultValue UMisRuleState::runLines(int * breakLevels, int maxStatements)
{
  const int MSL = 200;
  char sBuff[MSL];
  int m = maxStatements;
  //  bool result = true;
  UMisItem::ResultValue rv = UMisItem::RV_OK;
  const char * err = "# SYNTAX ERROR";
  const char * warn = "# SYNTAX WARNING";
  const char * et; // error text type
  //
  errLevel = 0;
  while (seqLine != NULL and m != 0)
  { // test for structure creators
    if (blockState == init and seqLine->isA("misRule"))
    { // create a sub-structure as appropriate
      addRule((UMisRule*)seqLine, true);
      logf->toLog("misRule", seqLine->getLineNumber(), getFullPreName(sBuff, MSL), ((UMisRule*)seqLine)->getName());
    }
    else
    { // ordinary line - just run it
      rv = runLine(false, breakLevels);
    }
    if (errorTxt[0] != '\0')
    { // report syntax error or warning - always for a line item
      if (rv == UMisItem::RV_SYNTAX_ERROR)
        et = err;
      else
        et = warn;
      logf->toLog(et, seqLine->getLineNumber(), rv, errorTxt);
      // debug
      if (errCnt < 5)
        printf("%s line %d: %s\n", et, seqLine->getLineNumber(), errorTxt);
      // debug end
      errCnt++;
      errorTxt[0] = '\0';
    }
    if (rv == UMisItem::RV_SYNTAX_ERROR)
    { // raise the error level and stop this block
      errLevel = 1;
      break;
    }
    if (rv == UMisItem::RV_OK_AGAIN)
    { // in a control line, and not finished
      break;
    }
    if (*breakLevels > 0)
    { // stop executing lines in this sequence
      // debug
      // printf("UMisRuleState::runLines: Break detected\n");
      // debug end
      break;
    }
    // may be an if statement
    if (rv == UMisItem::RV_IF_TRUE)
    {
      inIf[inIfCnt++] = true;
    }
    else if (rv == UMisItem::RV_IF_FALSE)
    { // set in if statement
      inIf[inIfCnt++] = false;
      // jump one line
      skipOneStatement(false);
    }
    if (rv != UMisItem::RV_IF_TRUE and inIfCnt > 0)
    { // reduce if nesting
      while (inIfCnt > 0)
      { // potentially skip all else-statements
        inIfCnt--;
        if (seqLine != NULL)
          if (seqLine->next != NULL)
          { // there is a statement
            if (seqLine->next->isA("misIfElse"))
            { // skip the else statement
              seqLine = seqLine->next;
              // log the skipped else line
              logf->toLog("# else", seqLine->getLineNumber(), ((UMisLineItem*)seqLine)->getLine());
              if (inIf[inIfCnt])
              { // jump one statement in addition to else
                skipOneStatement(false);
              }
              else
                // else part should be executed
                break;
            }
          }
      }
    }
    // limit max number of is statements
    if (inIfCnt > inIfCntMax)
    { // report error
      logf->toLog("# SYNTAX ERROR", seqLine->getLineNumber(),
                  inIfCntMax, "Exceeded maximum number of nested IF statements!");
      // debug
      if (errCnt < 5)
        printf("# SYNTAX ERROR line %d: Exceeded maximum number of nested IF statements (>%d)!\n", seqLine->getLineNumber(), inIfCntMax);
      // debug end
      errLevel = 1;
      inIfCnt--;
      errCnt++;
    }
    //
    if (seqLine != NULL)
    { // go to next line
      if (m != 1)
        // last line should not be advanced, as
        // it will be after this loop
        seqLine = seqLine->next;
      seqAgainCnt = 0;
      m--;
    }
  }
  return rv;
}

//////////////////////////////////////////

bool UMisRuleState::skipOneStatement(bool anIf)
{
  bool result = true;
  //
  if (seqLine->next != NULL)
  {
    seqLine = seqLine->next;
    if (seqLine->isA("misIf"))
      skipOneStatement(true);
    else if (anIf and seqLine != NULL)
    { // we are skipping a full if - both if and a potential else part
      if (seqLine->next != NULL)
        if (seqLine->next->isA("misIfElse"))
        { // skip statement before else (the if statement)
          seqLine = seqLine->next;
          // and skip the else part too (may be nested further)
          skipOneStatement(false);
        }
    }
  }
  else
  { // report error
    if (seqLine->isA("misIf"))
      logf->toLog("# SYNTAX ERROR", seqLine->getLineNumber(),
                UMisItem::RV_SYNTAX_ERROR,
                "Expected a statement after an if statement, none found!");
    else
      logf->toLog("# SYNTAX ERROR", seqLine->getLineNumber(),
                  UMisItem::RV_SYNTAX_ERROR,
                  "Expected a statement after an else statement, none found!");
    errLevel = 1;
    result = false;
    errCnt++;
  }
  return result;
}

///////////////////////////////////////////

UMisItem::ResultValue UMisRuleState::runLine(bool lastCall, int * breakLevels)
{
  UMisItem::ResultValue rv;
//  int n;
  const int MSL = 500;
  char s[MSL];
  UMisLineItem * ml = (UMisLineItem*)seqLine;
  //
  // log line prior to execution (if not a plan - plans are not a line item)
  if (not seqLine->isA("misRule"))
    logf->toLog(ml->getDataType(), ml->getLineNumber(),
              getFullPreName(s, MSL), ml->getLine());
  //
  if (seqLine->isA("misControl"))
  { // run a control line
    rv = runControl(breakLevels, lastCall);
/*    if (lastCall)
      n = -1;
    else
      n = seqAgainCnt;
    rv = seqLine->execute(this, n);
    if (rv == UMisItem::RV_OK_AGAIN)
    {  // in a control line, and not finished
      seqAgainCnt++;
    }
    else if (rv == UMisItem::RV_FAILED)
    {
      if (errCnt < 50)
        printf("Call completed, but failed for some reason - no connection? - continues\n");
      logf->toLog("# Call found but failed", seqLine->getLineNumber(), rv,
                  " for some reason - no connection? - continues anyhow\n");
      errCnt++;
    }*/
  }
  else if ((seqLine->isA("misCall")) or (seqLine->isA("misRule")))
  { // a call or in a called plan (explicit or in-line) run this
    rv = runCall(breakLevels, lastCall);
  }
  else if (seqLine->isA("misBreak"))
  { // is a break call
    //printf("UMisRuleState::runLines: break not implemented yet\n");
    rv = runBreak(breakLevels);
/*    logf->toLog("break", seqLine->getLineNumber(), rv,
                getFullPreName(s, MSL), ((UMisLineItem*)seqLine)->getLine());*/
  }
  else if (seqLine->isA("misEnable"))
  { // is an enable or disable call
    rv = runEnable((UMisEnable*)seqLine);
  }
  else if (seqLine->isA("misLoop"))
  { // is a loop, this requires a next statement
    rv = runLoop(breakLevels, (UMisLoop*)seqLine);
  }
  else if (seqLine->isA("misSwitch"))
  { // is a switch statement, this requires a next statement (that must be a block)
    rv = runSwitch((UMisRule*)seqLine->next);
  }
  else if (seqLine->isA("misCase") or seqLine->isA("misDefault"))
  { // a case statement or a case default is taken as a break statement, as
    // the right case to execute is found in the step() by advanceToCase()
    rv = runCase(breakLevels);
  }
  else
  { // ordinary line - just run it
    rv = seqLine->execute(this, blockState);
  }
  return rv;
}

///////////////////////////////////////////

// UMisItem::ResultValue UMisRuleState::runControl(int * breakLevels, bool lastCall)
// {
//   UMisItem::ResultValue rv;
//   int n;
//   //
//   if (lastCall)
//     n = -1;
//   else
//     n = seqAgainCnt;
//   rv = seqLine->execute(this, n);
//     // log
// /*    logf->toLog("control", seqLine->getLineNumber(), rv, getFullPreName(s, MSL),
//   ((UMisLineItem*)seqLine)->getLine());*/
//     // log end
//   if (rv == UMisItem::RV_OK_AGAIN)
//   {  // in a control line, and not finished
//     seqAgainCnt++;
//   }
//   else if (rv == UMisItem::RV_FAILED)
//   {
//     if (errCnt < 50)
//       printf("Call completed, but failed for some reason - no connection? - continues\n");
//     logf->toLog("# Call found but failed", seqLine->getLineNumber(), rv,
//                 " for some reason - no connection? - continues anyhow\n");
//     errCnt++;
//   }
//   return rv;
// }

///////////////////////////////////////////

UMisItem::ResultValue UMisRuleState::runControl(int * breakLevels, bool lastCall)
{
  UMisItem::ResultValue rv;
  int n;
  bool inLine;
  bool isOK;
  UMisControl * mc;
  //
  mc = (UMisControl *) seqLine;
  inLine = mc->isA("misRule");
  if (lastCall)
    n = -1;
  else
    n = seqAgainCnt;
  // do the call
  // every call is to be reinitialized to get new parameters, but call-state may be reused
  rv = initCall(inLine, true, n);
  //
  if (rv == UMisItem::RV_FAILED)
  {
    if (errCnt < 5)
      printf("Call '%s' completed, but failed - continues\n", getRule()->getName());
    logf->toLog("# Call found but failed", mc->getLineNumber(), rv,
                " for some reason - no connection? - continues anyhow\n");
    errCnt++;
  }
  //
  // if a rule, then the call is ready, but not implemented
  if (inCall != NULL and rv == UMisItem::RV_OK_AGAIN)
  { // all is ready for an additional (or first) step of the the established plan
    isOK = inCall->step(breakLevels, lastCall);
    if (isOK and inCall->isInMain())
    { // waiting at a control line
      rv = UMisItem::RV_OK_AGAIN;
    }
    else
      // call is finished
      rv = UMisItem::RV_OK;
  }
  //
  // if not finished, then we need to increase seqAgainCnt
  if (rv == UMisItem::RV_OK_AGAIN)
  { // in a control line, and not finished.
    // the explicit condition needs to be evaluated.
    rv = mc->evaluateCondition(this);
        // if result of condition is false, then keep RV_OK_AGAIN,
        // else either terminate now or syntax error
    if (rv == UMisItem::RV_OK_FALSE)
      // evaluated false, we are not finished yet
      rv = UMisItem::RV_OK_AGAIN;
    else if (inCall != NULL)
      // this terminates the call - run post code
      isOK = inCall->step(breakLevels, true);
    // increase loop coiunt
    seqAgainCnt++;
    // error handling
    if (rv == UMisItem::RV_FAILED)
    {
      if (errCnt < 5)
        printf("Line %d, explicit condition '%s' did not evaluate\n",
               mc->getLineNumber(), mc->getCondition());
      logf->toLog("# Call found but failed", mc->getLineNumber(), rv,
                  " for some reason - no connection? - continues anyhow\n");
      errCnt++;
    }
  }
  // clean up
  if (inCall != NULL and rv != UMisItem::RV_OK_AGAIN)
  { // call has terminated - release
    // decrease busy count for the plan - relevant if loaded plan only
    inCall->getRule()->setBusyCnt(-1);
    // delete structure created by call
    deleteStruct(inCall->getRule()->getName());
    // finished with the call - release the called plan and its state resource
    inCall->remove();
    // remove the state object itself
    delete inCall;
    // mark as not in a call
    inCall = NULL;
  }
  return rv;
}

///////////////////////////////////////////

UMisItem::ResultValue UMisRuleState::runCall(int * breakLevels, bool lastCall)
{
  bool inLine;
  UMisItem::ResultValue rv;
  bool isOK;
  //
  inLine = seqLine->isA("misRule");
  if (inCall == NULL)
  { // make the call or create a sub-structure ready for a step
    rv = initCall(inLine, false, 0);
      //logf->toLog("initCall", rv, getFullPreName(s, MSL), ((UMisLineItem*)seqLine)->getLine());
  }
  else
      // in call already, so just call again
    rv = UMisItem::RV_OK_AGAIN;
  //
  if (inCall != NULL and rv == UMisItem::RV_OK_AGAIN)
  { // step the established plan
    isOK = inCall->step(breakLevels, lastCall);
    if (isOK and inCall->isInMain())
    { // waiting at a control line
      rv = UMisItem::RV_OK_AGAIN;
      seqAgainCnt++;
    }
    else
    { // call has terminated - release
      rv = UMisItem::RV_OK;
        // decrease busy count for the plan - relevant if loaded plan only
      inCall->getRule()->setBusyCnt(-1);
        // delete structure created by call
      deleteStruct(inCall->getRule()->getName());
        // finished with the call - release the called plan and its state resource
      inCall->remove();
        // remove the state object itself
      delete inCall;
        // mark as not in a call
      inCall = NULL;
    }
  }
  return rv;
}

/////////////////////////////////////////////

UMisItem::ResultValue UMisRuleState::runBreak(int * breakLevels)
{
  UMisBreak * br;
  UMisItem::ResultValue rv;
  const int MNL = MAX_VAR_NAME_SIZE;
  char bp[MNL];
  int i, n;
  char *p1;
  UMisRuleState * sst;
  //
  br = (UMisBreak *) seqLine;
  rv = UMisItem::RV_OK;
  if (br->getParam() == NULL)
  {
    *breakLevels = 1;
    rv = UMisItem::RV_OK;
  }
  else
  {
    br->getParamCopy(bp, MNL);
    //
    p1 = bp;
    i = strtol(p1, &p1, 0);
    if (p1 != bp)
    {
      *breakLevels = i;
      // debug
      printf("UMisRuleState::runBreak: Break to number of levels (%d)\n", i);
      // debuge end
    }
    else
    { // break to a named level
      sst = this;
      p1 = bp;
      n = strlen(bp);
      // use first parameter only - the rest is treated as remarks
      for (i = 0; i < n; i++)
      {
        if (UMisItem::isRem(p1) or (*p1 <= ' '))
          break;
        p1++;
      }
      *p1 = '\0';
      i = 1;
      while (sst != NULL)
      {
        if (strcasecmp(bp, sst->misRule->getName()) == 0)
          break;
        if (sst->misRule->isFull or sst->misRule->isSwitch)
          // only rule and switch blocks count as a break level (not if block or other blocks)
          i++;
        if (sst->misRule->isTop)
          // we are at the top of the scope, so stop here
          sst = NULL;
        else
          // more scope available - try next level
          sst = sst->parent;
      }
      if (sst != NULL)
      {
        *breakLevels = i;
        // debug
        // printf("UMisRuleState::runBreak: found at level %d\n", i);
        // debug end
      }
      else
      {
        *breakLevels = 1;
        snprintf(errorTxt, MAX_ERROR_SIZE,
                 "***break:: Break level is not in current scope: %s", bp);
        rv = UMisItem::RV_SYNTAX_ERROR;
        // print to console too
        if (errCnt < 5)
          printf("SYNTAX ERROR line %d: %s\n", br->getLineNumber(), errorTxt);
        errCnt++;
      }
    }
  }
  return rv;
}

////////////////////////////////////////////

UMisItem::ResultValue UMisRuleState::runCase(int * breakLevels)
{ // a case statement - and a case default is handled as a break
  // the right start line is handled in the step() function and adcanceToCase()
  UMisItem::ResultValue rv;
  //
  *breakLevels = 1;
  rv = UMisItem::RV_OK;
  return rv;
}

////////////////////////////////////////////

UMisItem::ResultValue UMisRuleState::initCall(bool inLine, bool extraPar, int extraParVal)
{
  UMisItem::ResultValue rv;
  bool result;
  const char * p2;
  UMisCall * call;
  const int MPL = 30;
  char parOrder[MPL];
  UMisRule * mRule;
  const int MNL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES; //allow struct.name notation
  char ruleName[MNL] = "";
  bool gotValidPars;
  UVariable * params[MPL];
  int paramsCnt = 0;
  UVariable * ev;
  UDataBase *res;
  UVariable * resV = NULL;
  int n, i;
  //
  // convert line to right type
  call = (UMisCall *) seqLine;
  // is it a real call, or an inline block structure
  rv = UMisItem::RV_EMPTY;
  // any possibility for parameters?
  if (not inLine)
  { // yes, call has a name and a parameter string
    for (i = 0; i < MPL; i++)
      params[i] = NULL;
    // get name of method to call
    call->getNameCopy(ruleName, MNL);
    // a call may have parameters
    // get start (and end) of parameters
    p2 = call->getParameters();
    gotValidPars = evaluateParametersV(call->getLine(), // source line (for error reporting)
                                &p2,      // position of parameter start i.e. an '(' - returns at end ')'
                                parOrder, // order - e.g. "ds" for (double, string)
                                params,     // array of double parameters
                                &paramsCnt, // count of double sized parameters found
                                MPL);     // max number of parameters
    //
    if (not gotValidPars)
    { // error handling - error text is in place
      rv = UMisItem::RV_SYNTAX_ERROR;
    }
    if (extraPar and paramsCnt < MPL)
    { // add the (optional) extra parameter
      if (params[paramsCnt] == NULL)
        params[paramsCnt] = new UVariable();
      ev = params[paramsCnt++];
      ev->setValued((double)extraParVal, 0, true);
      strcat(parOrder, "d");
    }
    // log actual call parameters
    logActualCallV(call->getLineNumber(), ruleName, parOrder, params);
    //
    if (inCall == NULL)
    { // not an established rule-call
      // make a result value
      resV = new UVariable();
      res = resV;
      // try a C-implemented function in current scope
      n = 1;
      result = callScopeV(ruleName, parOrder, params, &res, &n);
      if (result)
      { // call is finished and all is well, just continue
        if (extraPar and n == 1)
        { // the return value decides outcome
          switch (roundi(resV->getValued()))
          { // 0 meanes that the function is failed
            // but for control, we do not abort here - try again
            case 0: rv = UMisItem::RV_OK_AGAIN; /*UMisItem::RV_FAILED; */ break;
            // succeded, but not finished
            case 1: rv = UMisItem::RV_OK_AGAIN; break;
            // res=2 succeded and finished (implicit continue)
            default: rv = UMisItem::RV_OK; break;
          }
        }
        else
          // normal call - all is fine - call is complete
          rv = UMisItem::RV_OK;
      }
      delete resV;
    }
  }
  //
  if (rv == UMisItem::RV_EMPTY)
  { // not handled
    if (inLine)
    { // an in-line plan (a block), so the plan is found already
      mRule = (UMisRule *)call;
    }
    else
    { // find the plan
      // try to find a plan with this name
      // may be daughters, sisters or grand-sisters
      mRule = getSubRule(ruleName);
      // error handling
      if (mRule == NULL)
      { // no such function
        snprintf(errorTxt, MAX_ERROR_SIZE,
                 "***call:: syntax error, plan or method not found: %s(%s)\n", ruleName, parOrder);
        rv = UMisItem::RV_SYNTAX_ERROR;
      }
      else if (mRule->parametersCnt < (int)strlen(parOrder))
      { // there is too many parameters
        snprintf(errorTxt, MAX_ERROR_SIZE,
                 "***call:: too many actual parameters %s(%s) only %d specified in rule\n",
                 ruleName, call->getParameters(), mRule->parametersCnt);
        rv = UMisItem::RV_SYNTAX_ERROR;
      }
    }
    if (mRule != NULL and rv != UMisItem::RV_SYNTAX_ERROR)
    { //load plan into new 'inCall' structure
      if (inCall == NULL)
      { // a call need a state structure
        inCall = new UMisRuleState();
        // load the plan as a called plan (not a sub-plan)
        inCall->setRule(mRule, this, logf);
        // increase busy count
        inCall->getRule()->setBusyCnt(1);
        // make it runable - state to init and once only
        inCall->setActive(true, true);
      }
      if (not inLine)
      { // plans may have parameters in-line blocks do not
        if (extraPar)
          n = 1;
        else
          n = 0;
        inCall->setParameters(params, paramsCnt - n);
      }
      // call is now ready for a step
      rv = UMisItem::RV_OK_AGAIN;
    }
  }
  for (i = 0; i < paramsCnt; i++)
  {
    if (params[i] != NULL)
      delete params[i];
    params[i] = NULL;
  }
  //
  return rv;
}

/////////////////////////////////////////////////////////////

void UMisRuleState::logActualCall(const int lineNumber,
                                 const char * funcName,
                                 const char * parOrder,
                                 char * sppar[],
                                 double dpar[])
{
  int i,j,k,n;
  char * p1;
  const int MLL = 300;
  char sl[MLL];
  //
  p1 = sl;
  snprintf(p1, MLL, "%s(", funcName);
  j = 0;
  k = 0;
  n = strlen(p1);
  p1 = &sl[n];
  for (i = 0; i < (int)strlen(parOrder); i++)
  {
    if (i > 0)
    {
      strncat(p1, ", ", MLL);
      p1 += 2;
      n += 2;
    }
    if (parOrder[i] == 's')
      snprintf(p1, MLL - n, "'%s'", sppar[j++]);
    else if (parOrder[i] == 'd')
      snprintf(p1, MLL - n, "%g", dpar[k++]);
    n += strlen(p1);
    p1 = &sl[n];
  }
  strncat(p1, ")", MLL - n);
  logf->toLog("# actual call:", lineNumber, sl);
}

/////////////////////////////////////////////////////////////

void UMisRuleState::logActualCallV(const int lineNumber,
                                  const char * funcName,
                                  const char * parOrder,
                                  UVariable ** params)
{
  int i,n;
  char * p1;
  const int MLL = 500;
  char sl[MLL];
  char s[MLL];
  //
  p1 = sl;
  snprintf(p1, MLL, "%s(", funcName);
  n = strlen(p1);
  p1 = &sl[n];
  for (i = 0; i < (int)strlen(parOrder); i++)
  {
    if (i > 0)
    {
      strncat(p1, ", ", MLL);
      p1 += 2;
      n += 2;
    }
    if (params[i]->isString())
      snprintf(p1, MLL - n, "'%s'", params[i]->getValueBuffer());
    else if (params[i]->isDouble())
      snprintf(p1, MLL - n, "%s", params[i]->getValuesdAsString(s, MLL, 0));
    n += strlen(p1);
    p1 = &sl[n];
  }
  strncat(p1, ")", MLL - n);
  logf->toLog("# actual call:", lineNumber, sl);
}

////////////////////////////////////////////

UMisItem::ResultValue UMisRuleState::runEnable(UMisEnable * eLine)
{
  UMisRuleState * ms;
  UMisItem::ResultValue result;
  const int MSL = MAX_VAR_NAME_SIZE + 1;
  char s[MSL];
  const char *p1, *p2;
  //
  if (eLine->getParam() == NULL)
    ms = this;
  else
  { // get terminated copy of plan name ...
    p1 = eLine->getParam();
    if (*p1 == '"' or *p1 == '\'')
      // rule may be a combination with a number
      evaluateToStringV(eLine->getLine(), p1, &p2, s, MSL);
    else
      eLine->getParamCopy(s, MSL);
    // ... to get state pointer
    ms = getSubRuleState(s);
  }
  if (ms != NULL)
  { // set active or inactive
    ms->setActive(eLine->isEnable(), ms->getRule()->isRule);
    //ms->setActive(eLine->isEnable(), false);
    result = UMisItem::RV_OK;
  }
  else
  { // rule not found
    result = UMisItem::RV_SYNTAX_ERROR;
    snprintf(errorTxt, MAX_ERROR_SIZE,
             "No such rule: '%s'", s);
  }
  return result;
}

////////////////////////////////////////////

UMisItem::ResultValue UMisRuleState::runLoop(int * breakLevels, UMisLoop * lLine)
{
  UMisItem::ResultValue rv = UMisItem::RV_SYNTAX_ERROR;
  bool val = true;
  //
  if (lLine->loopLine == NULL)
  { // not started yet get the next line
    lLine->loopLine = seqLine->next;
    // execute the init - assignment
    rv = lLine->runInitAssignment(this);
  }
  else if (seqAgainCnt > 0)
  { // in a control statement, so
    rv = UMisItem::RV_OK_AGAIN;
  }
  while (rv != UMisItem::RV_SYNTAX_ERROR)
  { // continue loop until break
    if (seqAgainCnt == 0)
    { // we are not in a control statement, so evaluate condition
      rv = lLine->evalCondition(this, &val);
      val &= (rv == UMisItem::RV_OK);
    }
    if (not val)
    { // condition no longer satisfied - exit loop
      // reset loop to init state
      lLine->loopLine = NULL;
      break;
    }
    // run the loop expression
    seqLine = lLine->loopLine;
    // run just 1 line - may be a block, and may be a control statement
    rv = runLines(breakLevels, 1);
    if (rv == UMisItem::RV_OK_AGAIN)
    { // we have hit a control statement
      if (not seqLine->isA("misControl"))
        // the control statement is in a sub-block, so count here too
        seqAgainCnt++;
      // the loop definition line needs to be run again
      seqLine = lLine;
      // stop the loop temporary
      break;
    }
/*    else
      seqAgainCnt = 0;*/
    // test break levels
    if (*breakLevels > 0)
    { // break line found - count loop as a level
      (*breakLevels)--;
      // loop is finished in any case
      break;
    }
    // not a control statement - do the loop assignment
    rv = lLine->runLoopAssignment(this);
  }
  return rv;
}

////////////////////////////////////////////

UMisItem::ResultValue UMisRuleState::runSwitch(UMisItem * nextLine)
{
  UMisItem::ResultValue rv = UMisItem::RV_OK;
  UMisCaseSwitch * sw = (UMisCaseSwitch *) seqLine;
  UMisRule * nr;
  //
  while (nextLine != NULL)
  { // allow remarks between switch and block
    if (not nextLine->isA("misRemark"))
      break;
    nextLine = nextLine->next;
  }
  if (nextLine != NULL)
  { // there is a next line
    if (nextLine->isA("misRule"))
    { // an in-line block
      nr = (UMisRule*) nextLine;
      nr->isSwitch = true;
      nr->switchValue = sw->evaluateExpr(this);
    }
    else
    {
      rv = UMisItem::RV_SYNTAX_ERROR;
      snprintf(errorTxt, MAX_ERROR_SIZE,
               "Expected block after switch, but found: '%s'", nextLine->getDataType());
    }
  }
  else
  {
    rv = UMisItem::RV_SYNTAX_ERROR;
    snprintf(errorTxt, MAX_ERROR_SIZE,
             "Expected block after switch, but found nothing");
  }
  return rv;
}

////////////////////////////////////////////

bool UMisRuleState::remove(const char * name)
{ // remove one of the subplans
  UMisRuleState * ms;
  bool found = false;
  int i, j;
  //const char * ruleName;
  //
  for (i = 0; i < subRulesCnt; i++)
  {
    ms = subRules[i];
    if (strcasecmp(ms->getRule()->getName(), name) == 0)
    {
      found = true;
      break;
    }
    ms++;
  }
  if (found)
  { // let the rule state remuve all sub-elements
    ms->remove();
    // remove the structure from the var pool
    deleteStruct(name);
    // move all remaining rules down
    for (j = i + 1; j < subRulesCnt; j++)
    {
      subRules[j-1] = subRules[j];
    }
    subRulesCnt--;
    // delete empty structure itself
    delete ms;
  }
  return found;
}

////////////////////////////////////////////

void UMisRuleState::remove()
{
  int i;
  //
  blockState = inactive;
  seqLine = NULL;
  errBlock = NULL;
  errLevel = 0;
  seqAgainCnt = 0;
  // delete all subRules
  for (i = subRulesCnt - 1; i >= 0; i--)
  { // subplans can not be root missions
    subRules[i]->remove();
    subRulesCnt--;
    delete subRules[i];
  }
  if (inCall != NULL)
  { // remove in-call sub structures
    // step a last time to run post-cond
    inCall->step(&i, true);
    // decrease busy count for the plan - relevant if loaded plan only
    inCall->getRule()->setBusyCnt(-1);
    // delete structure (local variables) created by call
    deleteStruct(inCall->getRule()->getName());
    // finished with the call - release the called plan and its state resource
    inCall->remove();
    // remove the state object itself
    delete inCall;
    inCall = NULL;
  }
  // cut link to mission definition
  misRule = NULL;
}

///////////////////////////////////////////

UMisRule * UMisRuleState::getSubRule(const char * name)
{
  UMisRule * result = NULL;
  int i;
  UMisRuleState * ms;
  //
  for (i = 0; i < subRulesCnt; i++)
  {
    ms = subRules[i];
    if (strcasecmp(ms->getRule()->getName(), name) == 0)
    {
      result = ms->getRule();
      break;
    }
  }
  if (result == NULL and parent != NULL)
  { // ask parent
    result = parent->getSubRule(name);
  }
  return result;
}

///////////////////////////////////////////

UMisRuleState * UMisRuleState::getCaller(const char * name)
{
  UMisRuleState * result = NULL;
  int i;
  //
  if (inCall != NULL)
  { // may be this is a call to the searched plan
    if (strcasecmp(name, inCall->getRule()->getName()) == 0)
      result = this;
  }
  if (result == NULL)
  { // not found here, so try sub-plans
    for (i = 0; i < subRulesCnt; i++)
    { // ask each of the sub-plans
      result = subRules[i]->getCaller(name);
      if (result != NULL)
        break;
    }
  }
  return result;
}

///////////////////////////////////////////

UMisRuleState * UMisRuleState::getSubRuleState(const char * name)
{
  UMisRuleState * result = NULL;
  int i;
  UMisRuleState * ms;
  //
  for (i = 0; i < subRulesCnt; i++)
  {
    ms = subRules[i];
    if (strcasecmp(ms->getRule()->getName(), name) == 0)
    {
      result = ms;
      break;
    }
  }
  if (result == NULL and parent != NULL)
  { // try parent
    result = parent->getSubRuleState(name);
  }
  return result;
}

///////////////////////////////////////////

bool UMisRuleState::methodCallV(const char * name, const char * paramOrder,
                                UVariable * params[],
                                UDataBase ** returnStruct,
                                int * returnStructCnt)
{ // implement method call from math
  bool result = true;
  UVariable *var, *dva;
  const char *p1;
  double value = 0.0; // return value - one double value
  bool setStructCnt = false; // is return struct count set
  //
  if ((strcasecmp(name, "eval") == 0) and (strcmp(paramOrder, "s") == 0))
  { // get distance from source to destination
    if (misRule != NULL)
    {
      dva = new UVariable();
      result = evaluateV(params[0]->getValues(), params[0]->getValues(), &p1, dva, false);
      if (result)
      { // get result of evaluation - as double
        if (dva->isDouble())
          value = dva->getValued();
        if (returnStruct[0] != NULL)
        { // set result to result variable
          if (returnStruct[0]->isA("var"))
          {
            var = (UVariable*)returnStruct[0];
            var->copy(dva, false);
            if (returnStructCnt != NULL)
              *returnStructCnt = 1;
            value = 1;
            setStructCnt = true;
          }
        }
      }
      delete dva;
    }
    else
      result = false;
  }
  else if ((strcasecmp(name, "name") == 0) and (strlen(paramOrder) == 0))
  {
    if (returnStruct[0] != NULL)
    { // set result to result variable
      if (returnStruct[0]->isA("var") and misRule != NULL)
      {
        var = (UVariable*)returnStruct[0];
        var->setValues(misRule->getName(), 0, true);
        if (returnStructCnt != NULL)
          *returnStructCnt = 1;
        value = 1;
        setStructCnt = true;
      }
    }
  }
  else
    result = false;
  // just set a default value
  if (not setStructCnt and returnStructCnt != NULL)
  { // returnStructCnt is not set, so set return value to result
    if (returnStruct[0] != NULL)
    {
      if (returnStruct[0]->isA("var"))
      {
        var = (UVariable*)returnStruct[0];
        var->setValued(value, 0, true);
        *returnStructCnt = 1;
      }
    }
  }
  return result;
}

///////////////////////////////////////////

bool UMisRuleState::methodCall(const char * name, const char * paramOrder,
                              char ** strings, const double * doubles,
                              double * value,
                              UDataBase ** returnStruct,
                              int * returnStructCnt)
{
  bool result = true;
  UVariable * dva;
  bool isOK;
  const char * p1;
  // find right method
  if ((strcasecmp(name, "wait") == 0) and (strcmp(paramOrder, "dd") == 0))
  {
    *value = methodWait(doubles[0], roundi(doubles[1]));
  }
  else if ((strcasecmp(name, "wait") == 0) and (strcmp(paramOrder, "d") == 0))
  { // called with no parameter, just wait
    *value = 1.0;
  }
  else if ((strcasecmp(name, "if") == 0) and (strlen(paramOrder) == 0))
  { // called with no parameter, just wait
    if (misRule != NULL)
    {
      dva = new UVariable();
      isOK = evaluateV(misRule->condition, misRule->condition, &p1, dva, false);
      *value = isOK and dva->getBool();
      delete dva;
    }
    else
      *value = false;
  }
  else if ((strcasecmp(name, "eval") == 0) and (strcmp(paramOrder, "s") == 0))
  { // called with no parameter, just wait
    if (misRule != NULL)
    {
      dva = new UVariable();
      isOK = evaluateV(strings[0], strings[0], &p1, dva, false);
      if (isOK and dva->isDouble())
        *value = dva->getValued();
      else
        *value = false;
      delete dva;
    }
    else
      *value = false;
  }
  else
    result = false;
  return result;
}

////////////////////////////////////////////////

int UMisRuleState::methodWait(double secs, int repeat)
{
  int result = 1;
  double dt;
  //
  if (repeat == 0)
  { // first call
    waitStart.now();
    varWait->setValued(secs, 0);
    if (secs <= 0.0)
      result = 2;
  }
  else if (repeat > 0)
  {
    dt = waitStart.getTimePassed();
    if (dt > secs)
      result = 2;
    // set remaining wait time
    varWait->setValued(secs - dt, 0);
  }
  return result;
}

///////////////////////////////////////////

bool UMisRuleState::setParameters(double dpar[], int dparCnt)
{
  int i;
  bool result = true;
  //
  for (i = 0; i < dparCnt; i++)
  {
    if (i < misRule->parametersCnt)
      varParameters[i]->setValued(dpar[i], 0, true);
    else
    {
      if (errCnt < 5)
        printf("UMisRuleState::setParameters: "
            "failed to set parameter %d of %d in %s - too many\n",
            i, misRule->parametersCnt, misRule->getName());
      result = false;
      errCnt++;
    }
  }
  return result;
}

///////////////////////////////////////////

bool UMisRuleState::setParameters(UVariable * params[], int paramsCnt)
{
  int i;
  bool result = true;
  //
  for (i = 0; i < paramsCnt; i++)
  {
    if (i < misRule->parametersCnt)
      varParameters[i]->setValue(params[i], 0);
    else
    {
      if (errCnt < 5)
        printf("UMisRuleState::setParameters: "
            "failed to set parameter %d of %d in %s - too many\n",
            i, misRule->parametersCnt, misRule->getName());
      result = false;
      errCnt++;
    }
  }
  return result;
}

///////////////////////////////////////////

bool UMisRuleState::getStateStr(const char * pre, char * buff, const int buffCnt)
{
  int n = 0;
  char * p1 = buff;
  const int MPL = 50;
  char pre2[MPL];
  char pre3[MPL];
  const int MRNL = 250;
  const char * subRuleNames[MRNL];
  int subRuleNamesCnt;
  int i, j;
  UMisLineItem * ml;
  bool isRoot;
  UMisRuleState * sp;
  UVarPool * vp;
  bool found;
  bool result = true;
  //
  if (buffCnt < 100)
  { // no more buffer space - exit
    snprintf(p1, buffCnt, "Buffer overflow\n");
    result = false;
  }
  else
  {
    isRoot = misRule->getLineNumber() < 0;
    *p1 = '\0';
    if (not isRoot)
    {
      if (blockState == inactive)
        snprintf(p1, buffCnt - n, "%srule %s (line %d) is inactive\n",
                pre, misRule->getName(), misRule->getLineNumber());
      else
        snprintf(p1, buffCnt - n, "%srule %s (line %d) is active\n",
                pre, misRule->getName(), misRule->getLineNumber());
    }
    snprintf(pre2, MPL, "%s  ", pre);
    //
    if (blockState != inactive and not isRoot)
    {
      if (blockState == init)
        ml = (UMisLineItem*)misRule->getMainLines();
      else
        ml = (UMisLineItem*)seqLine;
      n += strlen(p1);
      p1 = &buff[n];
      if (errCnt > 0)
      {
        snprintf(p1, buffCnt - n, "%s- error count is %d (see %s.log)\n", pre, errCnt, logf->getLogName());
        n += strlen(p1);
        p1 = &buff[n];
      }
      if (blockState == init and misRule->condition != NULL)
        snprintf(p1, buffCnt - n, "%s- waiting at condition\n", pre);
      else if (blockState == init)
        snprintf(p1, buffCnt - n, "%s- and running (cycle %d)\n", pre, automaticRestartCnt);
      else
      { // at a control statement in main
        if (ml != NULL)
          snprintf(p1, buffCnt - n, "%s- at line %d/%d: %s\n",
                  pre, ml->getLineNumber(), seqAgainCnt, ml->getLine());
        else
          snprintf(p1, buffCnt - n, "%s- at line %d: not a line item?\n", pre, ml->getLineNumber());
        if (inCall != NULL)
        {
          n += strlen(p1);
          p1 = &buff[n];
          strcat(pre2, "  ");
          result = inCall->getStateStr(pre2, p1, buffCnt - n);
        }
      }
    }
    // print also variable values
    if (result and (automaticRestartCnt > 0 or blockState != inactive))
    { // print also state of variables and parameters
      n += strlen(p1);
      p1 = &buff[n];
      listVars(pre2, p1, buffCnt - n, false);
      // get list of subrule names - not to be included in struct list
      subRuleNamesCnt = 0;
      for (i = 0; i < subRulesCnt; i++)
      {
        sp = subRules[i];
        subRuleNames[i] = sp->getRule()->getName();
        subRuleNamesCnt++;
      }
      for (i = 0; i < getStructCnt(); i++)
      {
        vp = getStruct(i);
        found = false;
        if (vp != NULL)
        { // not deleted! (is that possible?)
          for (j = 0; j < subRuleNamesCnt; j++)
          { // is the struct local vars for a sub-rule
            if (strcmp(subRuleNames[j], vp->getPreName()) == 0)
            {
              found = true;
              break;
            }
          }
          if (not found)
          { // a local struct - include in list
            snprintf(pre3, MPL, "%s%s.", pre2, vp->getPreName());
            n += strlen(p1);
            p1 = &buff[n];
            vp->listVars(pre3, p1, buffCnt - n, true);
          }
        }
      }
    }
    if (result)
    {
      for (i = 0; i < subRulesCnt; i++)
      {
        sp = subRules[i];
        if (isRoot)
        {
          n += strlen(p1);
          p1 = &buff[n];
          snprintf(p1, buffCnt - n, "%sfrom file %s\n", pre2, sp->misRule->getFileName());
        }
        n += strlen(p1);
        p1 = &buff[n];
        result = sp->getStateStr(pre2, p1, buffCnt - n);
        if (not result)
          break;
      }
    }
  }
  return result;
}

///////////////////////////////////////////////

void UMisRuleState::toLog(const char * logString)
{
  if (isLogOpen())
    logf->toLog(logString);
}

///////////////////////////////////////////////

bool UMisRuleState::isLogOpen()
{
  bool result = false;
  if (logf != NULL)
  {
    result = logf->isLogOpen();
  }
  return result;
}

///////////////////////////////////////////////

bool UMisRuleState::startStopRule(const char * ruleName, bool start)
{
  UMisRuleState * ms;
  //
  ms = getSubRuleState(ruleName);
  if (ms != NULL)
    ms->setActive(start, not getRule()->isRule);
  return ms != NULL;
}

///////////////////////////////////////////////

bool UMisRuleState::isActive(const char * ruleName)
{
  UMisRuleState * ms;
  bool result;
  //
  ms = getSubRuleState(ruleName);
  if (ms != NULL)
    result = ms->isActive();
  else
    result = false;
  return result;
}

///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////

UMisRule * UMisSeqRoot::getSubRule(const char * name)
{
  UMisRule * result = NULL;
  // search known subplans
  result = UMisRuleState::getSubRule(name);
  if (result == NULL and missions != NULL)
  { // search the top level missions
    // of the unloaded missions list (but only runable plans)
    result = missions->getRule(name, true);
  }
  return result;
}

////////////////////////////////////////////

bool UMisSeqRoot::step(int * breakLevels, const bool aLastCall)
{
  varSteps->add(1.0, 0);
  return UMisRuleState::step(breakLevels, aLastCall);
}

////////////////////////////////////////////

void UMisSeqRoot::createBaseVar()
{
  varSteps = addVar("misSteps", 0, "d", "Number of steps performed by sequencer");
    // add methods
  //addMethod(this, "print", "s", "print this string to the console");
}

///////////////////////////////////////////

// bool UMisSeqRoot::methodCall(const char * name, const char * paramOrder,
//                               char ** strings, const double * doubles,
//                               double * value,
//                               UDataBase ** returnStruct,
//                               int * returnStructCnt)
// {
//   bool result = true;
//   // find right method
//   if ((strcasecmp(name, "print") == 0) and (strcmp(paramOrder, "s") == 0))
//   {
//     printf("%s\n", strings[0]);
//     *value = 1;
//   }
//   else
//     // may be implemented by a normal plan state implementation
//     result = UMisRuleState::methodCall(name, paramOrder, strings, doubles,
//                           value, returnStruct, returnStructCnt);
//   return result;
// }


///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////

UResRuleState::~UResRuleState()
{ // stop sequencer loop and wait for it to finish
  stop(true);
}

///////////////////////////////////////////

bool UResRuleState::setResource(UResBase * resource, bool remove)
{
  bool result;
  result  = UServerPush::setResource(resource, remove);
  result &= UResVarPool::setResource(resource, remove);
  if (resource->isA(UResVarPool::getResClassID()))
  { // the global varpool exist, so add the math functions
    // if not already there
    addMathMethods();
  }
  return result;
}

///////////////////////////////////////////

void UResRuleState::createBaseVar()
{
  //varLineParsed = addVar("lineParsed", lineParsed, "d", "Current line in sequencer");
  varSampleTime =  UResVarPool::addVar("sampleTime",  0.25, "d", "(r/w) Time between rule sequencer iterations (in seconds)");
  varStartTime =   UResVarPool::addVar("startTime",   0.0,  "d", "(r) Timestamp for rule start - the last run command");
  varMissionTime = UResVarPool::addVar("missionTime", 0.0,  "d", "(r) Time of last rule iteration since mission start (in seconds)");
  varIterations =  UResVarPool::addVar("iterations",  0.0,  "d", "(r) Rule iteration count - since last run");
  varStepsLeft =   UResVarPool::addVar("steps",       0.0,  "d", "(r/w) Do this many iteration steps and then stop (-1 is no stop)");
    // add methods
  addMethodV("print", "s", "print this string to console (and to client)");
  addMethodV("print", "sd", "print this string - used as control statement");
  addMethodV("print", "c", "print this string to console (parameter is a UVariable)");
  addMethodV("stop", "", "Debug stop execution - setting the system in step-mode, "
                         "ready for state inspection. "
                        "The program can be resumed by a 'mis step=-1' command");
  addMethodV("stop", "d", "Same as stop(), but used as a control statement.");
  addMethodV("defined", "s", "Returns '1' if a variable or structure with name 's' is defined (global scope)");
  addMethodV("size", "s", "Returns number of elements in variable, string length for strings, -1 if not found (scope)");
  addMethodV("isString", "s", "Returns 1 if variable is a string, 0 if not, and -1 if not found (scope)");
  addMethodV("isDouble", "s", "Returns 1 if variable is a based on doubles, 0 if not, and -1 if not found (scope)");
}

///////////////////////////////////////////

const char * UResRuleState::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  snprintf(buff, buffCnt, "%s Rule sequencer is OK?\n", preString);
  return buff;
}

///////////////////////////////////////////

bool UResRuleState::removeRule(const char * misName)
{ // stop processing

  toLog("removed", misName);
  return state.remove(misName);
}

///////////////////////////////////////////

bool UResRuleState::addRule(UMisRule * plan)
{
  bool result;
  //
  if (state.getRule() == NULL)
  { // initialize the sequencer root state
    state.setParentVarPool(getVarPool());
    // set the (dummy) root plan and transfer the logfile
    state.setRule(&rootRule, NULL, this);
    // root plan must be active
    state.setActive(true, false);
  }
  // set plan and let the parent as this root sequencer state
  result = state.addRule(plan, true);
  if (result)
    toLog("loaded", plan->getName());
  else
    toLog("failed to load", plan->getName());
  return result;
}

///////////////////////////////////////////

int UResRuleState::getRulesCnt()
{
  return state.getSubRulesCnt();
}

///////////////////////////////////////////

void *startMisSeq(void *ptr)
{
  UResRuleState * obj;
  // convert pointer to planner object
  obj = (UResRuleState *) ptr;
  // run sequencer loop
  obj->run();
  //
  return NULL;
}

////////////////////////////////////////////

bool UResRuleState::start()
{
  bool result = true;
  pthread_attr_t  thConAttr;
  if (not running)
  { // Starts socket server thread 'runSockServer'
    pthread_attr_init(&thConAttr);
    // disable stop flag
    terminate = false;
    // create socket server thread
    pthread_create(&thSeq, &thConAttr, &startMisSeq, (void *)this);
    pthread_attr_destroy(&thConAttr);
  }
  return result;
}

///////////////////////////////////////////

void UResRuleState::run()
{
  double sampleTime;
  UServerInMsg msg;
  // the push command that performs the next sequencer iteration
  const char * cmd = "mseq i=1 cmd='rule seqIterate'";
//  int stepsLeft;
  bool cmdOK = false;
  bool added;
  //
  running = true;
  // add a push command to perform the next sequencer iteration
  msg.setMessage(-1, cmd, strlen(cmd), false);
  while (not cmdOK and not terminate)
  { // command will not be added until resources for push operation are in place
    cmdOK = addPushCommand(&msg);
    if (not cmdOK)
      Wait(1);
  }
  // now the loop that triggers the iteration command
  while (not terminate)
  { // time to start a new loop
    stepsRemaining = varStepsLeft->getInt();
    if (stepsRemaining != 0)
    { // either steps left to do or in continous mode (stepsLeft=-1)
      added = setUpdated("");
    }
    else
      added = false;
    // decrease step count
    if (added)
    { // the step is added, so decrease remaining steps
      stepsRemaining = varStepsLeft->getInt();
      if (stepsRemaining > 0)
        varStepsLeft->add(-1.0, 0);
    }
    // the sample time for this loop
    sampleTime = varSampleTime->getValued();
    Wait(sampleTime);
  }
  running = false;
}

///////////////////////////////////////////

bool UResRuleState::isStepping()
{
  int stepsLeft;
  stepsLeft = varStepsLeft->getInt();
  return stepsLeft >= 0;
}

///////////////////////////////////////////

bool UResRuleState::isStopped()
{
  int stepsLeft;
  stepsLeft = varStepsLeft->getInt();
  return stepsLeft == 0;
}

///////////////////////////////////////////

bool UResRuleState::seqIterateStep()
{
  bool result = true;
  int breakLevel = 0;
  UTime intoMission;
  UResRule * mis;
  const int MSL = 500;
  char s[MSL] = "";
  //
  if (state.getMissions() == NULL)
  { // get rule resource - getStaticResource(name, mayCreate, staticOnly)
    mis = (UResRule *)getStaticResource("rule", false, false);
    // set rule pointer in this state structure
    state.setMissions(mis);
  }
  loopStartTime.Now();
  // increase iteration count
  varIterations->add(1.0, 0);
  //toLog(UResVarPool::getLocalValueInt(varIterations), "step", "start");
  // step the plan
  state.step(&breakLevel, terminate);
  if (breakLevel > 0 and not terminate)
  { // stop mission globally
    state.setActive(false, false);
    toLog("break", state.getFullPreName(s, MSL));
  }
  if (varIterations->getInt() % 100 == 0)
  { // reduce logging to once every 100th iteration
    snprintf(s, MSL, "took %f ms", loopStartTime.getTimePassed() * 1000.0);
    toLog("# --------------- step", varIterations->getInt(), s);
  }
  if (stepsRemaining >= 0)
  {
    snprintf(s, MSL, "took %f ms", loopStartTime.getTimePassed() * 1000.0);
    printf("# --------------- step %d %s\n", varIterations->getInt(), s);
  }
  return result;
}

///////////////////////////////////////////


void UResRuleState::setSteps(int value)
{
  varStepsLeft->setValued(value, 0);
}

////////////////////////////////////////

void UResRuleState::stop(bool andWait)
{
  terminate = true;
  if (running)
  { // thread will stop in a while
    if (andWait)
    { // wait for thread to terminae
      printf("UResRuleState:: stopping seqencer loop ...");
      fflush(NULL);
      pthread_join(thSeq, NULL);
      printf(" [OK]\n");
    }
    else
      Wait(0.07);
  }
}

////////////////////////////////////////////////////

bool UResRuleState::addMathMethods()
{
  UVarPool * vp, *vpm;
  int n = -1;
  //
  vp = getVarPool()->getRootVarPool();
  if (vp != NULL)
  {
    vpm =  vp->getStruct("math");
    if (vpm == NULL)
    { // not defined, so make it now
      vpm = vp->addStructLocal("math", "global constants and methods", false);
      // global constants
      vpm->addVar("false", 0.0, "d", "Constant false is the same as 0");          //
      vpm->addVar("true", 1.0, "d", "Constant true is the same as 1");          //
      vpm->addVar("pi", M_PI, "d", "Constant pi is the same as 3.14159...");
      // global methods
      vpm->addMethodV(this, "limitToPi", "d", "Limits the value to within [+Pi..-Pi[");
      vpm->addMethodV(this, "sin", "d", "Sine of a value in radians");
      vpm->addMethodV(this, "cos", "d", "Cosine of a value in radians");
      vpm->addMethodV(this, "hypot", "dd", "Evaluates the squareroot of the sum of two squared values");
      vpm->addMethodV(this, "acos", "d", "Returns a value in radians");
      vpm->addMethodV(this, "asin", "d", "");
      vpm->addMethodV(this, "tan", "d", "Tangent value of an angle in radians");
      vpm->addMethodV(this, "atan", "d", "Returns angle in radians");
      vpm->addMethodV(this, "atan2", "dd", "Returns angle within +Pi..-Pi first parameter is y second is x");
      vpm->addMethodV(this, "sqrt", "d", "Returns the squareroot");
      vpm->addMethodV(this, "sqr", "d", "Returns value*value");
      vpm->addMethodV(this, "abs", "d", "Returns the unsigned (positive) value of parameter");
      vpm->addMethodV(this, "max", "dd", "Returns minimum of two values");
      vpm->addMethodV(this, "min", "dd", "Returns maximum of two values");
      vpm->addMethodV(this, "pow", "dd", "Implements the c-call pow(d,d)");
      n = vpm->addMethodV(this, "defined", "s", "Returns '1' if a variable or structure with name 's' is defined (global scope)");
      n = vpm->addMethodV(this, "size", "s", "Returns number of elements in variable, string length for strings, -1 if not found (global scope)");
      n = vpm->addMethodV(this, "isString", "s", "Returns 1 if variable is a string, 0 if not, and -1 if not found (global scope)");
      n = vpm->addMethodV(this, "isDouble", "s", "Returns 1 if variable is a based on doubles, 0 if not, and -1 if not found (global scope)");
      vpm->addMethodV(this, "now", "", "Returns time now, as double (since 1 jan 1970)");
      vpm->addMethodV(this, "poseToMap", "dddddd", "First 3 parameters (x,y,h) is current pose and next 3 parameters (x,y,h) is the pose that is to be converted to map coordinates (returns variable struct with 3 elements)");
      vpm->addMethodV(this, "poseToMap", "cc", "The 2 parameters are current pose and the pose that is to be converted to map coordinates (returns pose in variable with 3 elements)");
      vpm->addMethodV(this, "poseToMap", "cddd", "The first parameters is current pose and the rest is the pose that is to be converted to map coordinates (returns pose in variable with 3 elements)");
      vpm->addMethodV(this, "mapToPose", "dddddd", "First 3 parameters (x,y,h) is current pose and next 3 parameters (x,y,h) is the map pose that is to be converted to local coordinates (returns variable struct with 3 elements)");
      vpm->addMethodV(this, "mapToPose", "cc", "The 2 parameters are current pose and the map pose that is to be converted to local coordinates (returns pose in variable with 3 elements)");
      vpm->addMethodV(this, "mapToPose", "cddd", "The first parameters is current pose and the rest the map pose that is to be converted to local coordinates (returns pose in variable with 3 elements)");
      vpm->addMethodV(this, "distToPoseLineSigned", "ddddd", "Find signed position to line. The first three parameters define the line (x,y,th), then next 2 is the position (x,y)");
      vpm->addMethodV(this, "dist2d", "cc", "Find distance between these two (2D) positions or poses, i.e. hypot(a[0] - b[0], a[1] - b[0])");
      vpm->addMethodV(this, "poseOnLine", "ccd", "Find the pose on a line between these two points (2D points or poses), point 1 is source, point 2 is destination, parameter 3 is the desired distance from source towards destination.");
      vpm->addMethodV(this, "poseOnLine", "cd", "Find the pose on the line defined by the 'c' pose, parameter 2 is the desired distance from the pose.");
      vpm->addMethodV(this, "distOnLine", "cdd", "Find the distance along the line defined by the 'c' pose from the closest position on the line from the point (x,y) defined by the 2nd and 3rd parameter. Returns a negative value if behind.");
      vpm->addMethodV(this, "distOnLine", "cc", "Find the distance along the line defined by the 'c' pose from the "
                            "closest position on the line from the point (x,y) defined by the 2nd parameter. "
                            "Returns a negative value if behind.");
      vpm->addMethodV(this, "getVar", "s", "Get the variable with the name in the parameter (from rule only)");
      vpm->addMethodV(this, "setVar", "sc", "Set the variable with the name in the parameter to the provided value(s) (from rule only)");
      vpm->addMethodV(this, "setVar", "sd", "Set the variable with the name in the parameter to this value (from rule only), the destination string may hold an index, "
                            "e.g. setVar('global.test.m[3]',77.1) sets the fourth value of global.test.m to 77.1");
    }
    else
      n = 1;
  }
  // return true if methods were added or exist already
  return (n > 0);
}

//////////////////////////////////////////////

bool UResRuleState::methodCallV(const char * name, const char * paramOrder,
                       UVariable * params[],
                       UDataBase ** returnStruct,
                       int * returnStructCnt)
{ // implement method call from math
  bool result = true;
  UTime t;
  UPose pose1, pose2, pose3;
  UVariable * var;
  bool setStructCnt = false;
  double h, d, value = 0.0;
  const int MSL = 1000;
  char s[MSL];
  U2Dlined lin;
  // evaluate standard functions
  if ((strcasecmp(name, "limitToPi") == 0) and (strcmp(paramOrder, "d") == 0))
    value = limitToPi(params[0]->getValued());
  else if ((strcasecmp(name, "sin") == 0) and (strcmp(paramOrder, "d") == 0))
    value = sin(params[0]->getValued());
  else if ((strcasecmp(name, "hypot") == 0) and (strcmp(paramOrder, "dd") == 0))
    value = hypot(params[0]->getValued(), params[1]->getValued());
  else if ((strcasecmp(name, "cos") == 0) and (strcmp(paramOrder, "d") == 0))
    value = cos(params[0]->getValued());
  else if ((strcasecmp(name, "acos") == 0) and (strcmp(paramOrder, "d") == 0))
    value = acos(params[0]->getValued());
  else if ((strcasecmp(name, "asin") == 0) and (strcmp(paramOrder, "d") == 0))
    value = asin(params[0]->getValued());
  else if ((strcasecmp(name, "tan") == 0) and (strcmp(paramOrder, "d") == 0))
    value = tan(params[0]->getValued());
  else if ((strcasecmp(name, "atan") == 0) and (strcmp(paramOrder, "d") == 0))
    value = atan(params[0]->getValued());
  else if ((strcasecmp(name, "sqrt") == 0) and (strcmp(paramOrder, "d") == 0))
    value = sqrt(params[0]->getValued());
  else if ((strcasecmp(name, "sqr") == 0) and (strcmp(paramOrder, "d") == 0))
    value = sqr(params[0]->getValued());
  else if ((strcasecmp(name, "abs") == 0) and (strcmp(paramOrder, "d") == 0))
    value = fabs(params[0]->getValued());
  else if ((strcasecmp(name, "max") == 0) and (strcmp(paramOrder, "dd") == 0))
    value = maxd(params[0]->getValued(), params[1]->getValued());
  else if ((strcasecmp(name, "min") == 0) and (strcmp(paramOrder, "dd") == 0))
    value = mind(params[0]->getValued(), params[1]->getValued());
  else if ((strcasecmp(name, "atan2") == 0) and (strcmp(paramOrder, "dd") == 0))
    value = atan2(params[0]->getValued(), params[1]->getValued());
  else if ((strcasecmp(name, "pow") == 0) and (strcmp(paramOrder, "dd") == 0))
    value = pow(params[0]->getValued(), params[1]->getValued());
  else if ((strcasecmp(name, "now") == 0) and (strlen(paramOrder) == 0))
  {
    t.now();
    value = t.getDecSec();
  }
  else if ((strcasecmp(name, "defined") == 0) and (strcmp(paramOrder, "s") == 0))
  { // is this variable defined
    result = getVarPool()->isDefined(getVarPool(), params[0]->getValueBuffer());
    value = double(result);
    result = true;
  }
  else if ((strcasecmp(name, "size") == 0) and (strcmp(paramOrder, "s") == 0))
  { // is this variable defined
    var = getVarPool()->getGlobalVariable(params[0]->getValueBuffer(), NULL);
    if (var == NULL)
      value = -1.0;
    else
      value = var->getElementCnt();
    result = true;
  }
  else if ((strcasecmp(name, "isString") == 0) and (strcmp(paramOrder, "s") == 0))
  { // is this variable defined
    var = getVarPool()->getGlobalVariable(params[0]->getValueBuffer(), NULL);
    if (var == NULL)
      value = -1.0;
    else
      value = var->isString();
    result = true;
  }
  else if ((strcasecmp(name, "isDouble") == 0) and (strcmp(paramOrder, "s") == 0))
  { // is this variable defined
    var = getVarPool()->getGlobalVariable(params[0]->getValueBuffer(), NULL);
    if (var == NULL)
      value = -1.0;
    else
      value = var->isDouble();
    result = true;
  }
  else if ((strcasecmp(name, "print") == 0) and (strncmp(paramOrder, "s", 1) == 0))
  { // send also to client
    sendToClientAsHelp(printClientNumber, params[0]->getValueBuffer());
    value = 1.0;
  }
  else if ((strcasecmp(name, "print") == 0) and (strcmp(paramOrder, "c") == 0))
  { // send also to client
    sendToClientAsHelp(printClientNumber, params[0]->getValuesAsString(s, MSL, 0));
    value = 1.0;
  }
  else if (strcasecmp(name, "stop") == 0)
  { // send also to client
    varStepsLeft->setValued(0.0, 0);
    value = 1.0;
  }
  else if ((strcasecmp(name, "getvar") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    int idx = 0;
    var = NULL;
    bool isOK = getVarPool()->getVariableRefAny(getVarPool(), params[0]->getValueBuffer(), &var, &idx);
    if (isOK)
    {
      if (var->isDouble())
      {
        if (idx >= 0)
          value = var->getDouble(idx);
        else
          value = var->getDouble(idx); // return first value
      }
      else // a string
        value = 0.0; // false
    }
    else
      value = 0.0;
    if (isOK and returnStructCnt != NULL and returnStruct[0] != NULL)
    { // return also into variable struct
      UVariable * dst;
      if (returnStruct[0]->isA("var"))
      {
        dst = (UVariable*)returnStruct[0];
        if (idx < 0 or var->isString())
          dst->copy(var, false);
        else
          dst->setValued(var->getDouble(idx));
      }
      if (var != NULL)
        *returnStructCnt = 1;
      else
        *returnStructCnt = 0;
      setStructCnt = true;
    }
  }
  else if ((strcasecmp(name, "setvar") == 0) and (strcmp(paramOrder, "sc") == 0))
  { // set variable with name in staring from another vaiable
    var = NULL;
    int idx = 0;
    // get variable from string
    bool isOK = getVarPool()->getVariableRefAny(getVarPool(), params[0]->getValueBuffer(), &var, &idx);
    if (isOK and var != NULL)
    { // set all or just part
      if (idx < 0)
        var->copy(params[1], false);
      else
        // set only one value
        var->setValue(params[1], idx);
      value = 1.0;
    }
    else
      value = 0.0;
  }
  else if ((strcasecmp(name, "setvar") == 0) and (strcmp(paramOrder, "sd") == 0))
  { // set variable from single double value
    var = NULL;
    int idx = 0;
    // get variable from string
    bool isOK = getVarPool()->getVariableRefAny(getVarPool(), params[0]->getValueBuffer(), &var, &idx);
    if (isOK and var != NULL)
    { // set all or just part
      var->setValue(params[1], idx);
      value = 1.0;
    }
    else
      value = 0.0;
  }
  else if ((strcasecmp(name, "poseToMap") == 0) and (strcmp(paramOrder, "dddddd") == 0))
  {
    pose1.set(params[0]->getValued(), params[1]->getValued(), params[2]->getValued());
    pose2.set(params[3]->getValued(), params[4]->getValued(), params[5]->getValued());
    // @todo fix this conversion routine
    pose3 = pose1.getPoseToMapPose(&pose2);
    value = 0;
    if (returnStruct[0] != NULL)
    {
      if (returnStruct[0]->isA("var"))
      {
        var = (UVariable*)returnStruct[0];
        var->setPose(&pose3);
        var->setType(UVariable::pose);
        var->setName("mapPose");
        if (returnStructCnt != NULL)
          *returnStructCnt = 1;
        value = 1;
        setStructCnt = true;
      }
    }
  }
  else if ((strcasecmp(name, "poseToMap") == 0) and (strcmp(paramOrder, "cc") == 0))
  {
    pose1 = params[0]->getPose();
    pose2 = params[1]->getPose();
    // @todo fix this conversion routine
    pose3 = pose1.getPoseToMapPose(&pose2);
    value = 0;
    if (returnStruct[0] != NULL)
    {
      if (returnStruct[0]->isA("var"))
      {
        var = (UVariable*)returnStruct[0];
        var->setPose(&pose3);
        var->setType(UVariable::pose);
        var->setName("mapPose");
        if (returnStructCnt != NULL)
          *returnStructCnt = 1;
        value = 1;
        setStructCnt = true;
      }
    }
  }
  else if ((strcasecmp(name, "poseToMap") == 0) and (strcmp(paramOrder, "cddd") == 0))
  {
    pose1 = params[0]->getPose();
    pose2.set(params[1]->getValued(), params[2]->getValued(), params[3]->getValued());
    // @todo fix this conversion routine
    pose3 = pose1.getPoseToMapPose(&pose2);
    value = 0;
    if (returnStruct[0] != NULL)
    {
      if (returnStruct[0]->isA("var"))
      {
        var = (UVariable*)returnStruct[0];
        var->setPose(&pose3);
        var->setType(UVariable::pose);
        var->setName("mapPose");
        if (returnStructCnt != NULL)
          *returnStructCnt = 1;
        value = 1;
        setStructCnt = true;
      }
    }
  }
  else if ((strcasecmp(name, "mapToPose") == 0) and (strcmp(paramOrder, "dddddd") == 0))
  {
    pose1.set(params[0]->getValued(), params[1]->getValued(), params[2]->getValued());
    pose2.set(params[3]->getValued(), params[4]->getValued(), params[5]->getValued());
    // @todo fix this conversion routine
    pose3 = pose1.getMapToPosePose(&pose2);
    value = 0;
    if (returnStruct[0] != NULL)
    {
      if (returnStruct[0]->isA("var"))
      {
        var = (UVariable*)returnStruct[0];
        var->setPose(&pose3);
        var->setType(UVariable::pose);
        var->setName("localPose");
        if (returnStructCnt != NULL)
          *returnStructCnt = 1;
        value = 1;
        setStructCnt = true;
      }
    }
  }
  else if ((strcasecmp(name, "mapToPose") == 0) and (strcmp(paramOrder, "cc") == 0))
  {
    pose1 = params[0]->getPose();
    pose2 = params[1]->getPose();
    // @todo fix this conversion routine
    pose3 = pose1.getMapToPosePose(&pose2);
    value = 0;
    if (returnStruct[0] != NULL)
    {
      if (returnStruct[0]->isA("var"))
      {
        var = (UVariable*)returnStruct[0];
        var->setPose(&pose3);
        var->setType(UVariable::pose);
        var->setName("localPose");
        if (returnStructCnt != NULL)
          *returnStructCnt = 1;
        value = 1;
        setStructCnt = true;
      }
    }
  }
  else if ((strcasecmp(name, "mapToPose") == 0) and (strcmp(paramOrder, "cddd") == 0))
  {
    pose1 = params[0]->getPose();
    pose2.set(params[1]->getValued(), params[2]->getValued(), params[3]->getValued());
    // @todo fix this conversion routine
    pose3 = pose1.getMapToPosePose(&pose2);
    value = 0;
    if (returnStruct[0] != NULL)
    {
      if (returnStruct[0]->isA("var"))
      {
        var = (UVariable*)returnStruct[0];
        var->setPose(&pose3);
        var->setType(UVariable::pose);
        var->setName("localPose");
        if (returnStructCnt != NULL)
          *returnStructCnt = 1;
        value = 1;
        setStructCnt = true;
      }
    }
  }
  else if ((strcasecmp(name, "distToPoseLineSigned") == 0) and (strcmp(paramOrder, "ddddd") == 0))
  {
    h = params[2]->getValued();
    lin.setPV(params[0]->getValued(), params[1]->getValued(), cos(h), sin(h));
    value = lin.distanceSigned(params[3]->getValued(), params[4]->getValued());
    // value is also set in first return parameter
  }
  else if ((strcasecmp(name, "dist2d") == 0) and (strcmp(paramOrder, "cc") == 0))
  {
    value = hypot(params[0]->getValued(0) - params[1]->getValued(0),
                  params[0]->getValued(1) - params[1]->getValued(1));
    // value is also set in first return parameter
  }
  else if ((strcasecmp(name, "poseOnLine") == 0) and (strcmp(paramOrder, "ccd") == 0))
  { // get distance from source to destination
    pose1 = params[0]->getPose();
    pose2 = params[1]->getPose();
    h = params[2]->getValued();
    d = hypot(pose2.y - pose1.y, pose2.x - pose1.x);
    if (d > 0.1)
    { // set pose on line between these two poses (2D positions)
      value = 1.0;
      pose3.x = pose1.x + (pose2.x - pose1.x) * h / d;
      pose3.y = pose1.y + (pose2.y - pose1.y) * h / d;
      pose3.h = atan2(pose2.y - pose1.y, pose2.x - pose1.x);
    }
    else
    {
      value = 0.0;
      pose3 = pose2;
    }
    if (returnStruct[0] != NULL)
    {
      if (returnStruct[0]->isA("var"))
      {
        var = (UVariable*)returnStruct[0];
        var->setPose(&pose3);
        var->setType(UVariable::pose);
        var->setName("pose on line");
        if (returnStructCnt != NULL)
          *returnStructCnt = 1;
        value = 1;
        setStructCnt = true;
      }
    }
  }
  else if ((strcasecmp(name, "poseOnLine") == 0) and (strcmp(paramOrder, "cd") == 0))
  { // get pose on the line defined by pose, but d meter in x direction
    pose1 = params[0]->getPose();
    pose2.clear();
    pose2.x = params[1]->getValued();
    pose3 = pose1.getPoseToMapPose(&pose2);
    if (returnStruct[0] != NULL)
    {
      if (returnStruct[0]->isA("var"))
      {
        var = (UVariable*)returnStruct[0];
        var->setPose(&pose3);
        var->setType(UVariable::pose);
        var->setName("pose on pose line");
        if (returnStructCnt != NULL)
          *returnStructCnt = 1;
        value = 1;
        setStructCnt = true;
      }
    }
  }
  else if ((strcasecmp(name, "distOnLine") == 0) and (strcmp(paramOrder, "cdd") == 0))
  { // get distance to closest point on line - signed
    pose1 = params[0]->getPose();
    // define a line across the pose
    lin.setPH(pose1.x, pose1.y, pose1.h - M_PI / 2.0);
    // the distance to this line is positive if to the left of the line
    // and thus in front of the specified pose
    value = lin.distanceSigned(params[1]->getValued(), params[2]->getValued());
    // returns a single value
  }
  else if ((strcasecmp(name, "distOnLine") == 0) and (strcmp(paramOrder, "cc") == 0))
  { // get distance to closest point on line - signed
  pose1 = params[0]->getPose();
  pose2 = params[1]->getPose();
  // define a line across the pose
  lin.setPH(pose1.x, pose1.y, pose1.h - M_PI / 2.0);
  // the distance to this line is positive if to the left of the line
  // and thus in front of the specified pose
  value = lin.distanceSigned(pose2.x, pose2.y);
  // returns a single value
  }
  else
  {
    result = false;
    value = 0;
  }
  if (not setStructCnt and returnStructCnt != NULL)
  { // returnStructCnt is not set, so set return value to result
    if (returnStruct[0] != NULL)
    {
      if (returnStruct[0]->isA("var"))
      {
        var = (UVariable*)returnStruct[0];
        var->setValued(value, 0, true);
        *returnStructCnt = 1;
      }
    }
  }
  return result;
}

////////////////////////////////////////

void UResRuleState::sendToClientAsHelp(const int client, const char * helpTxt)
{
  const int MRL = 400;
  char reply[MRL];
  UCmdExe * core;
  // print always to console
  printf("%s\n", helpTxt);
  if (client >= 0)
  {
    core = getCorePointer();
    if (core != NULL)
    { // send to client
      snprintf(reply, MRL, "<help print=\"%s\"/>\n", helpTxt);
      core->sendMsg(printClientNumber, reply);
    }
  }
}

/////////////////////////////////////////

bool UResRuleState::getStateStr(const char * pre, char * buff, const int buffCnt)
{ // ensure root plan is added (var pool must be ready before this)
  bool result = true;
  //
  if (state.getSubRulesCnt() > 0)
    result = state.getStateStr(pre, buff, buffCnt);
  else
    snprintf(buff, buffCnt, "%sNo rules defined\n", pre);
  return result;
}

///////////////////////////////////////////////

bool UResRuleState::startStopRule(const char * ruleName, bool start)
{
  return state.startStopRule(ruleName, start);
}

///////////////////////////////////////////////

bool UResRuleState::isActive(const char * ruleName)
{
  return state.isActive(ruleName);
}

///////////////////////////////////////////////

UMisRuleState * UResRuleState::getCaller(const char * name)
{
  return state.getCaller(name);
}

////////////////////////////////////////////////


