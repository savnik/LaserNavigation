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
#include <ctype.h>

#include <ugen4/usmltagin.h>
#include <urob4/uvarcalc.h>

//#include "constants.h"
#include "usequencer.h"

/////////////////////////////////////////////////////

USeqLine::USeqLine()
{
  cmd = NULL;
  clear();
}

/////////////////////////////////////////////////////

USeqLine::~USeqLine()
{
  if (cmd != NULL)
    delete cmd;
}

//////////////////////////////////////////////////////////

void USeqLine::clear()
{
  syntaxErr = -1;
  typ = SQ_UNKNOWN;
  buffSize = 0;
  params = NULL;
}

//////////////////////////////////////////////////////////

bool USeqLine::setLine(const char * line, UVarPool * varPool, int lineNum, FILE * logS)
{
  bool result;
  int n = 0;
  const char * p1, *p2;
  //
  p1 = line;
  result = line != NULL;
  if (result)
  { // remove leading space
    while (isspace(*p1))
      p1++;
    n = strlen(p1);
    p2 = strchr(p1, '\n');
    if (p2 > p1)
      n = p2 - p1;
    result = (n > 0);
  }
  if (result)
  { // allocate buffer space
    n++; // add space for the zero as well
    if (n > buffSize)
    { // release old space - if any
/*      if (cmd != NULL)
        free(cmd);*/
      // allocate
      cmd = (char *) realloc(cmd, n);
      if (cmd != NULL)
        buffSize = n;
      else
      { // failed - report major error
        buffSize = 0;
        result = false;
        if (logS != NULL)
          fprintf(logS, "*** %d No malloc(%d) space\n", lineNum, n);
        printf("USeqLine::setLine *** %d No malloc(%d) space\n", lineNum, n);
      }
    }
  }
  syntaxErr = -1;
  toDo = NULL;
  params = NULL;
  typ = SQ_UNKNOWN;
  if (result)
  {
    strncpy(cmd, p1, n);
    cmd[n-1] = '\0';
    result = setLineType(lineNum, varPool, logS);
  }
  //
  return result;
}

////////////////////////////////////////////////////

bool USeqLine::setLineType(int lineNum, UVarPool * varPool, FILE * logS)
{ // scan for syntax error
  bool result = true;
  char *p1, *p2, *p3;
  bool done = false;
  const int MNL = MAX_VARIABLE_NAME_SIZE;
  char name[MNL];
  char parstr[MNL];
  //
  if ((strchr("#%;", cmd[0]) != NULL)
       or (strncmp(cmd,"//",2) == 0))
  {
    p2 = &cmd[1];
    setType(SQ_REMARK);
    done = true;
  }
  if (not done)
  { // must be a statement, get first identifier (must exist)
    p1 = getIdentifier(cmd, name, MNL, &p2);
    result = (p1 != NULL);
    done = not result;
    if (not result)
    { // report error
      if (logS != NULL)
        fprintf(logS, "*** Line %d do not start with an identifier\n", lineNum);
      printf("USeqLine::setLineType: *** Line %d do not start with an identifier\n", lineNum);
    }
  }
  if (not done)
  { // is a label or an assignment
    if (*p2 == ':')
    { // label or an execute statement
      // get first character after ':'
      p3 = p2 + 1;
      // remove white space
      while (isspace(*p3))
        p3++;
      // check for more data
      if ((*p3 < ' ') or (strchr("#%;", *p3) != NULL) or (strncmp(p3, "//", 2) == 0))
      { // no data - or just comment
        done = true;
        p3 = p2;
        // ensure no white space after label and before ':'
        while (*(p2 - 1) == ' ')
          p2--;
        *p3 = ' ';
        *p2 = ':';
        setType(SQ_LABEL);
      }
      else
      { // Sequencer stopper command (typical drive)
        done = true;
        setType(SQ_DRIVE_STATEMENT);
      }
    }
    else if (*p2 == '=')
    { // an assignment statement
      done = true;
      setType(SQ_ASSIGN);
    }
  }
  if (not done)
  { // a keyword based standard function?
    done = true;
    if (strcasecmp(name, "function") == 0)
      setType(SQ_FUNCTION);
    else if (strcasecmp(name, "return") == 0)
      setType(SQ_FUNCTION_RETURN);
    //
    else if (strcasecmp(name, "if") == 0)
      setType(SQ_FLOW_IF);
    else if (strcasecmp(name, "goto") == 0)
      setType(SQ_FLOW_GOTO);
    else if (strcasecmp(name, "skip") == 0)
      setType(SQ_FLOW_SKIP);
    else if (strcasecmp(name, "skipcall") == 0)
      setType(SQ_FLOW_SKIP_CALL);
    else if (strcasecmp(name, "call") == 0)
      setType(SQ_FLOW_CALL);
    // WATCH
    else if (strcasecmp(name, "watch") == 0)
      setType(SQ_WATCH);
    else if (strcasecmp(name, "unwatch") == 0)
      setType(SQ_UNWATCH);
    //
    // PRINT
    else if (strcasecmp(name, "print") == 0)
      setType(SQ_PRINT);
    else
      done = false;
  }
  if (not done)
  { // may be a function call, that do not return a value
    if (*p2 == '(')
    { // NB! this must be checked after drive commands, as the first
      // parameter of a drive command may start with an '(' too
      enumerateParameters(p2, parstr, MNL, (const char **)&p3);
      while (isspace(*p3))
        p3++;
      if (*p3 == ':')
        setType(SQ_DRIVE_STATEMENT);
      else
        // a function call with no return value is a
        // flow_call - without changing the flow
        setType(SQ_FLOW_CALL);
      // p2 (params) must point to function name
      p2 = cmd;
      done = true;
    }
  }
  if (not done)
  {
    result = false;
    if (logS != NULL)
      fprintf(logS, "*** Line %d unknown type of statement, near '%c' (pos %d)\n",
          lineNum, *p2, p2 - cmd);
    printf("USeqLine::setLineType: *** Line %d unknown type of statement, before '%c'\n", lineNum, *p2);
    syntaxErr = p2 - cmd;
  }
  if (done)
    // save pointer to further parameters of the line
    params = p2;
  //
  return result;
}

///////////////////////////////////////////////////////////////


char * USeqLine::getIdentifier(const char * string,
                            char * buffer,
                            int bufferLng,
                            char ** next)
{
  const char * p1, *p2;
  int n;
  char * result = NULL;
  //
  p1 = string;
  while (isspace(*p1))
    p1++;
  if (isalnum(*p1) or (*p1 == '-'))
  { // get rest of identifier
    p2 = p1 + 1;
    while (isalnum(*p2) or (*p2 == '_') or (*p2 == '.'))
      p2++;
    // make a copy of name
    n = mini(p2 - p1, bufferLng - 1);
    strncpy(buffer, p1, n);
    buffer[n] = '\0';
    result = buffer;
    //
  }
  else
    p2 = p1;
  if (next != NULL)
  {
    while(isspace(*p2))
      p2++;
    *next = (char *) p2;
  }
  return result;
}

/////////////////////////////////////////////////////

bool USeqLine::isThisLabel(const char * labelName)
{
  bool result;
  const char * p1;
  int n;
  //
  result = (typ == SQ_LABEL);
  if (result)
  {
    p1 = strchr(labelName, ':');
    if (p1 != NULL)
      n = p1 - labelName;
    else
      n = strlen(labelName);
    result = (n == (params - cmd));
    if (result)
      result = (strncasecmp(labelName, cmd, n) == 0);
  }
  return result;
}

/////////////////////////////////////////////////////

bool USeqLine::isASkipLine()
{
  bool result;
  result = ((typ == SQ_FLOW_SKIP) or (typ == SQ_FLOW_SKIP_CALL));
  return result;
}

/////////////////////////////////////////////////////

bool USeqLine::isLegalThenStatement()
{
  bool result;
  result = ((typ > SQ_LABEL) and (typ < SQ_REMARK)) or (typ == SQ_ASSIGN);
  return result;
}

////////////////////////////////////////////////////////

bool USeqLine::enumerateParameters(const char * start, char * parSeq, const int parSeqCnt, const char ** nextChar)
{
  const char * p1;
  char * p2;
  int pCnt = 0; // paranthesis cnt
  bool inStr = false;
  char inStrChar = '_';
  bool result = true;
  char parType = 'd';
  int parCnt = 0;
  //
  p1 = start;
  p2 = parSeq;
  if (*p1 == '(')
    p1++;
  while (*p1 >= ' ')
  { // tab, \n \r or any other control character not allowed
    if (inStr)
    {
      if (*p1 == inStrChar)
        inStr = false;
    }
    else if (*p1 == '\'' or *p1 == '"')
    { // a string start is detected
      inStr = true;
      // save type of string
      // "aa'bbb'aa" is a legal string
      // 'bb"aaa"bb' is a legal string
      // "bb'aa"cc"aa'bb" is NOT legal,
      // but seen as 2 strings with a cc inbetween!!
      // see
      inStrChar = *p1;
      // change parameter type to string
      parType = 's';
      if (parCnt == 0)
        parCnt = 1;
    }
    else if (*p1 == '(')
    { // start of a bracket sequence -
      // may contain strings or any character but a ')'
      pCnt++;
    }
    else if (*p1 == ')')
    {
      if (pCnt == 0)
      { // end of parameter list
        // (close parenthesis with no start)
        p1++;
        break;
      }
      else
        // just end of matching brackets
        pCnt--;
    }
    else if (*p1 == ',' and pCnt == 0)
    { // not a string or in a bracket, so parameter separator
      parCnt++;
      // save parameter type (s or d)
      *p2++ = parType;
      // assume next parameter is a double (d)
      parType = 'd';
      if (parCnt >= parSeqCnt - 1)
      { // no space for more parameters - error
        result = false;
        break;
      }
    }
    else if (*p1 > ' ' and parCnt == 0)
      // first non-space character
      parCnt = 1;
    p1++;
  }
  if (parCnt > 0)
    *p2++ = parType;
  *p2 = '\0';
  if (nextChar != NULL)
    *nextChar = p1;
  //
  return result;
}

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

USeqWatch::USeqWatch()
{
  active = false;
  sampleTime = -1.0;
  strncpy(name, "none", MAX_VARIABLE_NAME_SIZE);
}

/////////////////////////////////////////////////////

USeqWatch::~ USeqWatch()
{
}

/////////////////////////////////////////////////////

void USeqWatch::setActive(bool value)
{
  active = value;
  ttw.Now();
}

/////////////////////////////////////////////////////

bool USeqWatch::isTimeToWatch(UTime toTime)
{
  bool result = false;
  if ((toTime - ttw) > 0.0)
  {
    if (sampleTime > 0.0)
    {
      ttw += sampleTime;
      if ((ttw - toTime) + sampleTime < 0.0)
        // behind schedule - catch up
        ttw = toTime + sampleTime;
    }
    result = true;
  }
  return result;
}

/////////////////////////////////////////////////////

void USeqWatch::set(bool asActive,
                double sampleInterval,
                const char * watchName)
{
  active = asActive;
  sampleTime = sampleInterval;
  strncpy(name, watchName, MAX_VARIABLE_NAME_SIZE);
}

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

USeqFunction::USeqFunction()
{
  name[0] = '\0';
  first = -1;
  last = -1;
  paramsCnt = 0;
}

/////////////////////////////////////////////////////

USeqFunction::~USeqFunction()
{
}

/////////////////////////////////////////////////////

bool USeqFunction::set(USeqLine * functionLine,
                       int lineNum,
                      FILE * log)
{
  bool result;
  char * p1, * p2;
  // get name of function
  p1 = functionLine->getIdentifier(
                  functionLine->getParams(),
                  name, MAX_VARIABLE_NAME_SIZE, &p2);
  result = (p1 != NULL);
  if (not result and (log != NULL))
    fprintf(log, "*** %d error: error: expected function identifier at pos %d\n", lineNum,
                        functionLine->getParams() - functionLine->getCmdLine());
  if (result)
  {
    result = (*p2 == '(');
    if (not result and (log != NULL))
      fprintf(log, "*** %d error: expected '(' after function name, got '%c'\n", lineNum, *p2);
    if (result)
    { // parse parameter names and count
      functionLine->setParams(p2);
      // count number of parameters
      p2++;
      paramsCnt = 0;
      if (*p2 != ')')
      { // get name and count of parameters
        while (paramsCnt < SEQ_MAX_FUNCTION_PARAMS)
        { // get name
          p1 = functionLine->getIdentifier(p2, params[paramsCnt], MAX_VARIABLE_NAME_SIZE, &p2);
          result = (p1 != NULL);
          if (not result and log != NULL)
          {
            fprintf(log, "*** %d error: invalid function parameter name at pos %d\n",
                          lineNum, p2 - functionLine->getCmdLine());
            break;
          }
          paramsCnt++;
          if (*p2 == ',')
            p2++;
          else
            break;
        }
        if (result)
        {
          result = (*p2 == ')');
          if (not result and log != NULL)
            fprintf(log, "*** %d error: expected ')' after %d function parameters, got '%c'\n",
                  lineNum, paramsCnt, *p2);
        }
      }
    }
  }
  first = lineNum;
  // last line in function is not yet found
  last = -1;
  //
  return result;
}

/////////////////////////////////////////////////////

bool USeqFunction::getValue(const char * name,    // name of variable (zero terminated)
                            double * value,       // where to put value
                            double * valueStackTop, // stack with values
                            int valueStackCnt)      // number of elements in stack
{
  bool result;
  char * p1;
  int i;
  double * val;
  //
  result = (paramsCnt > 0) and (valueStackCnt >= paramsCnt);
  if (result)
  {
    result = false;
    for (i = 0; i < paramsCnt; i++)
    {
      p1 = params[i];
      if (strcasecmp(name, p1) == 0)
      {
        result = true;
        break;
      }
    }
  }
  if (result and (value != NULL))
  { // get value
    val = valueStackTop - paramsCnt + i;
    *value = *val;
  }
  return result;
}




/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

USequencer::USequencer()
{
  //calc = this;
  errorTxt[0] = '\0';
  simulated = false;
  reset();
  logFileName[0] = '\0';
  currentFunction = NULL;
  skipLine.clear();
  simc = NULL;
  calc = new UCalcSeq();
  inDriveCmd = 0;
  inDriveCmdThis[0] = '\0';
  inDriveCmdLine = -1;
}

/////////////////////////////////////////////////////

USequencer::~USequencer()
{
  if (simc != NULL)
    fclose(simc);
  //delete calc;
}

/////////////////////////////////////////////////////

void USequencer::reset()
{
  watchCnt = 0;
  linesCnt = 0;
  paramsCnt = 0;
  stackCnt = 0;
  funcsCnt = 0;
  lineActive = 0;
  strncpy(loadedPlanName, "none", MAX_FILENAME_SIZE);
}

/////////////////////////////////////////////////////

void USequencer::clear()
{
  seqLock.lock();
  reset();
  seqLock.unlock();
}

/////////////////////////////////////////////////////

bool USequencer::loadPlan(char * filename, FILE * log, bool append)
{
  bool result;
  FILE * fp = NULL;
  const int MLL = 500;
  char cl[MLL];
  bool isOK;
  int n, m = 0, errCnt = 0;
  UTime t;
  const int STL = 30;
  char st[STL];
  char * p1;
  //
    // start logfile for syntax error
  if (log != NULL)
  {
    t.Now();
    t.getDateString(cl, true);
    t.getTimeAsString(st, true);
    fprintf(log, "# %s %s Analyzing as MMR (OA) planner file %s\n",
            cl, st, filename);
  }
  result = (filename != NULL);
  if (not result and (log != NULL))
    fprintf(log, "No filename specified\n");
  if (result)
  {
    fp = fopen(filename, "r");
    result = (fp != NULL);
    if (not result and log != NULL)
      fprintf(log, "**** File not found '%s'\n", filename);
  }
  if (result)
  { // lock sequencer for now
    seqLock.lock();
    if (not append)
    { // remove old plan if not appending
      reset();
    }
    // save filename
    strncpy(loadedPlanName, filename, MAX_FILENAME_SIZE);
    //
    while (not feof(fp))
    {
      p1 = fgets(cl, MLL, fp);
      n = strlen(cl);
      m++;
      if (n < (MLL - 1))
      { // OK line - not too long
        isOK = add(cl, log, NULL);
        if (not isOK and (n > 1))
          errCnt++;
      }
      else
      { // too long
        fprintf(log, "*** Line %d too long (%d chars)\n", m, n);
        printf("USequencer::loadPlan: Line %d of %s too long (%d chars)\n", m, filename, n);
        errCnt++;
      }
    }
    seqLock.unlock();
  }
  if (log != NULL)
    fprintf(log, "# Found %d errors in %d lines\n", errCnt, m);
  if (fp != NULL)
  {
    fclose(fp);
    fp = NULL;
  }
  if (result)
    result = (errCnt == 0);
  //
  return result;
}

///////////////////////////////////////////////////////////////

bool USequencer::add(const char * line, FILE * logf, int * lineNum)
{
  bool result;
  USeqLine * sl;
  //
  result = (linesCnt < MAX_SQE_LINE_CNT);
  if (not result and (logf != NULL))
    fprintf(logf, "*** %d No more space for lines\n", linesCnt);
  if (result)
  {
    sl = &lines[linesCnt];
    result = sl->setLine(line, calc->getVarPool(), linesCnt, logf);
  }
  if (result)
  {
    if (lineNum != NULL)
      *lineNum = linesCnt;
    linesCnt++;
  }
  //
  return result;
}

/////////////////////////////////////////////////////////////

const char * USequencer::addLabel()
{
  const char * result = NULL;
  USeqLine * sl;
  const int MLL = 100;
  char line[MLL];
  bool isOK = false;
  //
  if (linesCnt < MAX_SQE_LINE_CNT);
  {
    snprintf(line, MLL, "label%d:", linesCnt);
    sl = &lines[linesCnt];
    isOK = sl->setLine(line, calc->getVarPool(), linesCnt, logSeq);
  }
  if (isOK)
  {
    linesCnt++;
    result = sl->getCmdLine();
  }
  //
  return result;
}

/////////////////////////////////////////////////////////////

bool USequencer::skipToLabel(const char * label, bool asCall)
{ // used from remote control, when loading new script
  bool result;
  int lineNum;
  //
  seqLock.lock();
  lineNum = findLabelNum(label);
  result = (lineNum >= 0);
  if (result)
  {
    if (asCall)
      pushStack(lineActive);
    else
    { // empty stack - exit all functions
      stackCnt = 0;
      paramsCnt = 0;
    }
    setActiveLine(lineNum);
  }
  seqLock.unlock();
  //
  return result;
}

/////////////////////////////////////////////////////////////

bool USequencer::parseStatement(USeqLine * line,
                       int lineNum,
                       FILE * logf,
                       bool syntaxCheck,
                       int * nextLine)
{
  bool result = true;
  char *p1;
  USeqLine * cl = NULL;
  UTime t;
  //
  if (syntaxCheck)
    strncpy(errorTxt, "no error", MAX_ERROR_SIZE);
  //
  cl = line;
  p1 = cl->getParams();
  // default is continue to next line
  *nextLine = lineNum + 1;
  //
  switch (cl->getType())
  {
    case SQ_REMARK:
      break;
    case SQ_UNKNOWN:
      printf("USequencer::parseStatement: %d *** error: unknown line type '%s'\n",
             lineNum, cl->getCmdLine());
      break;
    case SQ_ASSIGN:
      result = parseAssignStatement(cl, lineNum, logf, syntaxCheck);
      break;
    case SQ_DRIVE_STATEMENT:
      result = parseDriveStatement(cl, lineNum, logf, syntaxCheck, nextLine);
      break;
    case SQ_FUNCTION:
      result = parseFunctionStatement(cl, lineNum, logf, syntaxCheck, nextLine);
      break;
    case SQ_FUNCTION_RETURN:
      // ignore return value when parsed as a drive function
      result = parseFunctionReturn(cl, lineNum, logf, syntaxCheck, nextLine, NULL, NULL);
      break;
    case SQ_LABEL:
      result = parseLabelStatement(cl, lineNum, logf, syntaxCheck);
      break;
    case SQ_FLOW_IF:
    case SQ_FLOW_GOTO:
    case SQ_FLOW_CALL:
      result = parseFlowStatement(cl, lineNum, logf, syntaxCheck, nextLine);
      break;
    case SQ_FLOW_SKIP:
    case SQ_FLOW_SKIP_CALL:
      if (syntaxCheck)
        result = parseSkipStatement(cl, lineNum, logf, syntaxCheck, nextLine);
      else
        // copy to buffer, awating skip execution later
        skipLine.setLine(cl->getCmdLine(), calc->getVarPool(), lineNum, NULL);
      break;
    case SQ_WATCH:
    case SQ_UNWATCH:
      result = parseWatchStatement(cl, lineNum, logf, syntaxCheck);
      break;
/*    case SQ_SUPPORT_CAM:
    case SQ_SUPPORT_ARM:
    case SQ_SUPPORT_SAY:
      result = parseSupportStatement(cl, lineNum, logf, syntaxCheck);
      break;*/
    case SQ_PRINT:
      result = parsePrintStatement(cl, lineNum, logf, syntaxCheck);
      break;
    default:
      if (logf != NULL)
        fprintf(logf, "*** %d Unknown SQ_??? line type (%d)\n", lineNum, cl->getType());
      printf("USequencer::parseStatement %d Unknown SQ_??? line type (%d)\n", lineNum, cl->getType());
      result = false;
      break;
  }
  // log for simulation replay
  if (logf != NULL and ((cl->getType() != SQ_DRIVE_STATEMENT) or inDriveCmd == 0))
  {
    t.Now();
    fprintf(logf, "%lu.%03lu Line %d: '%s' [%s]\n",
        t.getSec(), t.getMilisec(),
        lineNum, cl->getCmdLine(),
        bool2str(result));
  }
//   // log end
  return result;
}

///////////////////////////////////////////////////////////////////////////////

bool USequencer::parseAssignStatement(USeqLine * line,
                    int lineNum, FILE * logf, bool syntaxCheck)
{
  bool result;
  USeqLine * cl = line;
  //
  result = calc->setVariable(cl->getCmdLine(), syntaxCheck);
  if (not result)
  {
    if (logf != NULL)
      fprintf(logf, "*** %d error: %s\n", lineNum, calc->getErrorTxt());
    printf("USequencer:: *** %d error: %s\n", lineNum, calc->getErrorTxt());
  }
  //
  return result;
}

///////////////////////////////////////////////////////////////////////////////

// bool USequencer::parseSupportStatement(USeqLine * line,
//                     int lineNum, FILE * logf, bool syntaxCheck)
// {
//   bool result = false;
//   USeqLine * cl = line;
//   const int MCL = 300;
//   char cmd[MCL];
//   char reply[MCL];
//   char * par, *p1;
//   const char *p2;
//   //
//   par = line->getParams();
//   switch (cl->getType())
//   {
//     case SQ_SUPPORT_CAM:
//       // evaluate parameters in SMRCL command
//       result = calc->evaluateToString(par, par, &p2, cmd, MCL - 1);
//       //
//       if (simulated)
//       {
//         p1 = strcasestr(cmd, "gmkget");
//         if (p1 != NULL)
//         { // insert an 'img="5"'
//           p1 += 6;
//           *p1++ = '\0';
//           strcpy(reply, cmd);
//           strcat(reply, " poolImg=\"5\" ");
//           strcat(reply, p1);
//         }
//       }
//       // debug
//       printf("Support command: '%s' eval (%s) to '%s'\n", par, bool2str(result), cmd);
//       // debug end
//       //
//       // add a linefeed to initiate evaluation at support server
//       strcat(cmd, "\n");
//       if (result)
//         result = (imgClient != NULL);
//       if (result and not syntaxCheck)
//       {
//         imgClient->tx.lock();
//         imgClient->sendMsg(cmd);
//         imgClient->tx.unlock();
//       }
//       break;
//     case SQ_SUPPORT_ARM:
//     case SQ_SUPPORT_SAY:
//       fprintf(stdout, "USequencer:: *** Not supported yet: '%s'\n",
//         cl->getCmdLine());
//       break;
//     default:
//       fprintf(stdout, "USequencer:: *** Not a known support statement\n");
//   }
//   if (not result)
//   {
//     if (logf != NULL)
//       fprintf(logf, "*** %d error: %s\n", lineNum, calc->getErrorTxt());
//     printf("USequencer:: *** %d error: %s\n", lineNum, calc->getErrorTxt());
//   }
//   //
//   return result;
// }

///////////////////////////////////////////////////////////////////////////////

bool USequencer::parseFlowStatement(USeqLine * line,
                                    int lineNum,
                                    FILE * logf,
                                    bool syntaxCheck,
                                    int * nextLine)
{
  bool result = true;
  USeqLine * cl;
  const char * p1;
  const char * p2;
  USeqFunction * func;
  const int MNL = MAX_VARIABLE_NAME_SIZE;
  char name[MNL];
  int toLine;
  double d;
  //
  cl = line;
  //
  switch (cl->getType())
  {
    case SQ_FLOW_CALL:
      // copy function name to 'name'
      cl->getIdentifier(cl->getParams(), name, MNL, (char **)&p1);
      // get function description
      func = getFunction(name);
      result = (func != NULL);
      p2 = p1;
      if (result)
        result = parseFunctionCall(cl, lineNum, func, &p2, nextLine,
                        logf, syntaxCheck);
      else
      { // may be a C-code system function (with unused return value)
        result = calc->getValueAny(calc->getVarPool(), name, &d, &p1);
        if (not result and logf != NULL)
          fprintf(logf, "*** %d error: %s\n", lineNum, calc->getErrorTxt());
        if (not result)
          printf("USequencer::parseFlowStatement *** %d (error: %s) or function not found (cmd=%s)\n", lineNum, calc->getErrorTxt(), line->getCmdLine());
        if (nextLine != NULL)
          *nextLine = lineNum + 1;
      }
      break;
    case SQ_FLOW_IF:
      result = parseIfStatement(cl, lineNum, nextLine, logf, syntaxCheck);
      break;
    case SQ_FLOW_GOTO:
      // copy function name to 'name'
      cl->getIdentifier(cl->getParams(), name, MNL, (char **)&p1);
      // find label line
      toLine = findLabelNum(name);
      result = (toLine >= 0);
      if (not result and (logf != NULL))
        fprintf(logf, "*** %d error: no such label '%s'\n", lineNum, name);
      if (result and not syntaxCheck and (nextLine != NULL))
        *nextLine = toLine;
      break;
    default:
      result = false;
      break;
  }
  return result;
}

///////////////////////////////////////////////////////////////////////////////

bool USequencer::parseSkipStatement(USeqLine * line,
                                    int lineNum,
                                    FILE * logf,
                                    bool syntaxCheck,
                                    int * nextLine)
{
  bool result = true;
  USeqLine * cl;
  char * p1;
  const char * p2;
  USeqFunction * func;
  const int MNL = MAX_VARIABLE_NAME_SIZE;
  char name[MNL];
  int toLine;
  UTime t;
  double v;
  //
  cl = line;
  //
  switch (cl->getType())
  { // this is valid at end of idle period only (or for syntax check)
    case SQ_FLOW_SKIP:
      // copy label name to 'name'
      cl->getIdentifier(cl->getParams(), name, MNL, &p1);
      // find label line
      if (p1 != cl->getParams())
        toLine = findLabelNum(name);
      else
        // no label, then just skip to next line
        toLine = getActiveLine() + 1;
      result = (toLine >= 0);
      if (result and not syntaxCheck)
      { // set line in drive script to execute next
        if (nextLine != NULL)
          // continue watch at next line
          *nextLine = toLine;
      }
      break;
    case SQ_FLOW_SKIP_CALL:
      // this is valid as watch statements only, and must be
      // executed just befor next drive statement only
      // copy function name to 'name'
      cl->getIdentifier(cl->getParams(), name, MNL, &p1);
      // get function description
      func = getFunction(name);
      p2 = p1;
      if (result)
        result = parseFunctionCall(cl, getActiveLine() - 1, func, &p2, nextLine,
                                   logf, syntaxCheck);
      break;
    default:
      result = false;
      break;
  }
  // debug log
  if (logSeq != NULL)
  {
    calc->getLocalValue("posehist.time", &v);
    t.setTime(v);
    fprintf(logSeq, "%lu.%03lu executed '%s'\n",
            t.getSec(), t.getMilisec(),
            line->getCmdLine()
            );
  }
  // debug log end
  return result;
}

///////////////////////////////////////////////////////////////////////////////

USeqFunction * USequencer::getFunction(const char * name)
{
  USeqFunction * result = NULL;
  USeqFunction * fun;
  int i;
  //
  fun = funcs;
  for (i = 0; i < funcsCnt; i++)
  {
    if (strcasecmp(fun->getName(), name) == 0)
    {
      result = fun;
      break;
    }
    fun++;
  }
  //
  return result;
}

///////////////////////////////////////////////////////////////////////////////

int USequencer::getFunction(USeqFunction * func)
{
  int result = -1;
  int i;
  USeqFunction * fp = funcs;
  //
  for (i = 0; i < funcsCnt; i++)
  {
    if (fp == func)
    {
      result = i;
      break;
    }
    fp++;
  }
  return result;
}

///////////////////////////////////////////////////////////////////////////////

USeqFunction * USequencer::getFunction(int lineNum)
{
  USeqFunction * result = NULL;
  USeqFunction * fun;
  int i;
  //
  fun = funcs;
  for (i = 0; i < funcsCnt; i++)
    if (fun->getStartLine() == lineNum)
    {
      result = fun;
      break;
    }
  //
  return result;
}

///////////////////////////////////////////////////////////////////////////////

int USequencer::getFunctionStart(int lineNum)
{
  int result = -1;
  USeqLine * line;
  int i;
  //
  i = lineNum;
  for (i = lineNum; i >= 0; i--)
  { // go bach and find function declaration line
    line = getLine(i);
    if (line->getType() == SQ_FUNCTION)
    { // function is found
      result = i;
      break;
    }
    else if ((i != lineNum) and
             (line->getType() == SQ_FUNCTION_RETURN))
      // this is not inside a function
      break;
  }
  //
  return result;
}

///////////////////////////////////////////////////////////////////////////////

USeqFunction * USequencer::getFunctionCurrent(int lineNum)
{
  USeqFunction * result = NULL;
  int n;
  //
  n = getFunctionStart(lineNum);
  if (n >= 0)
    result = getFunction(n);
  //
  return result;
}

///////////////////////////////////////////////////////////////////////////////

bool USequencer::parseDriveStatement(USeqLine * line,
                                     int lineNum,
                                     FILE * logf,
                                     bool syntaxCheck,
                                    int * nextLine)
{
  bool result = true;
  UTime t;
  USeqLine * cl;
//  double secs;
//  bool isOK;
  const char * ps;
  char * par;
  const int MCL = 100;
  char sc[MCL];
  UVarPool * vp;
  const int MPN = SEQ_MAX_FUNCTION_PARAMS;
  double paramValues[MPN];
  int paramValuesCnt; // count of double sized parameters
  const int MSL = 1000;
  const int MSLN = 3; // max 3 string parameters
  char parStr[MSLN][MSL];
  char * strPars[MSLN] = { parStr[0], parStr[1], parStr[2]};
  char parOrder[MPN];
  double value = 0.0;
  double explicitStop = 0.0;
  //
  cl = line;
  par = cl->getParams();
  //
  // get pointer to (optional) stop conditions
  ps = strchr(par, ':');
  if (ps != NULL)
    // set in parsed line
    ps++;
  // set pointer to explicit stop conditions (or NULL if none)
  cl->setToDo((char*)ps);
  //
  // resetset flags for implicit stop criteria possibilities
  line->getIdentifier(line->getCmdLine(), sc, MCL, &par);
  if (not syntaxCheck)
  {
    if ((inDriveCmdLine != lineActive) or (strncmp(sc, inDriveCmdThis, MAX_CMD_IDENTIFIER_SIZE) != 0))
    { // first call for this drive command
      strncpy(inDriveCmdThis, sc, MAX_CMD_IDENTIFIER_SIZE);
      inDriveCmd = 0;
      inDriveCmdLine = lineNum;
    }
    else
      inDriveCmd++;
  }
  // find drive function to call
  result = line->enumerateParameters(par, parOrder, MPN, &ps);
  if (result)
  {
    vp = calc->getVarPool();
    result = (vp != NULL);
  }
  if (result and not syntaxCheck)
  { // now evaluate the double and string parameters for the function
    result = calc->evaluateParameters(
                    line->getCmdLine(), // source line
                    (const char **)&par,// start of param values - should point at '('
                    paramValues,        // value array (doubles)
                    &paramValuesCnt,      // found count of double sized params
                    strPars,            // string parameters
                    parOrder,           // parameter order - e.g. "dd" for 2 doubles.
                    MPN,                // max parameter count
                    MSLN,               // max number of string parameters
                    MSL);               // max string parameter length
    if (not result)
      snprintf(errorTxt, MAX_ERROR_SIZE,
                "Syntax error - in parameters at %d not found in %s",
                par - line->getCmdLine(), line->getCmdLine());
  }
  if (result)
  { // we need to add the repeat count to the double parameter list
    strcat(parOrder, "d");
    paramValues[paramValuesCnt] = inDriveCmd;
    // do the drive call
    result = vp->callGlobal(inDriveCmdThis, parOrder, strPars,
                            paramValues, &value, NULL, NULL);
    if (not result)
      snprintf(errorTxt, MAX_ERROR_SIZE,
         "Syntax error - driver call '%s' with param order '%s' not found!",
         inDriveCmdThis, parOrder);
  }
  if (result and value < 1.5)
  { // implicit stop condition is false,
    // so the explisit stop conditions must be evaluated too.
    explicitStop = calc->evaluate(line->getCmdLine(), line->getToDo(), (const char **)&sc,
                           &result, syntaxCheck);
  }
  if (not syntaxCheck and nextLine != NULL)
  { // result is ready - test for implicit and explicit stop (or failed result)
    if ((value >= 1.5) or (explicitStop > 0.5) or not result)
      *nextLine = lineNum + 1;
    else
      // no stop, so continue this line
      *nextLine = lineNum;
  }
  //
  if (not result and (logf != NULL))
    fprintf(logf, "*** %d error: %s\n", lineNum, errorTxt);
  //
  return result;
}

///////////////////////////////////////////////////////////////////////////////

bool USequencer::parseFunctionStatement(USeqLine * line,
                                        int lineNum,
                                        FILE * logf,
                                        bool syntaxCheck,
                                        int * nextLine)
{
  bool result = false;
  USeqLine * cl;
  USeqFunction * func;
  int i;
  // function name(par1, par2)
  // line->params = "name(..."
  cl = line;
  func = getFunction(cl->getParams());
  if (func == NULL)
  { // function structure not created yet
    result = (funcsCnt < MAX_SEQ_FUNCTIONS);
    if (not result)
      if (logf != NULL)
        fprintf(logf, "*** %d error: %s\n", lineNum, "No space for function declaration");
    if (result)
    { // get a free function structure
      func = &funcs[funcsCnt++];
      // set parameter count and names
      result = func->set(cl, lineNum, logf);
    }
  }
  if (result)
  {
    if (func->getReturnLine() < 0)
    { // return line is not found, do so now
      cl = &lines[lineNum];
      for (i = lineNum; i < linesCnt; i++)
      {
        if (cl->getType() == SQ_FUNCTION_RETURN)
        {
          func->setReturnLine(i);
          break;
        }
        cl++;
      }
    }
    if (func->getReturnLine() > 0)
      // set next line to just after return
      *nextLine = func->getReturnLine() + 1;
    else if (not syntaxCheck)
      // return line not loaded yet, so use last line for now
      *nextLine = linesCnt;
    else if (logf != NULL)
      // no return, so possibly a stray function declaration
      fprintf(logf, "*** %d warning: missing return statement for "
                   "function '%s'\n", lineNum, func->getName());
  }
  //
  return result;
}

//////////////////////////////////////////////////////

bool USequencer::parseFunctionReturn(USeqLine * line,
                                    int lineNum,
                                    FILE * logf,
                                    bool syntaxCheck,
                                    int * nextLine,
                                    double * returnVale,
                                    bool * returnValueValid)
{
  bool result;
  USeqLine * cl;
  USeqFunction * func;
  bool isOK;
  double value;
  const char * p1;
  int fn;
  // function name(par1, par2)
  cl = line;
  func = currentFunction;
  result = (func != NULL);
  if (not result and (logf != NULL))
  { // function structure not created yet
    fprintf(logf, "*** %d error: Return outside function\n", lineNum);
  }
  if (result)
  { // get return value
    value = calc->evaluate(cl->getCmdLine(), cl->getParams(), &p1, &isOK, syntaxCheck);
    if (p1 <= cl->getParams())
      // use default return value
      value = 0.0;
    if (returnVale != NULL)
      *returnVale = value;
    if (returnValueValid != NULL)
      *returnValueValid = (p1 > cl->getParams());
    // remove function parameters from stack
    if (not syntaxCheck)
    { // remove local variables from param stack
      paramsCnt -= func->getParamCnt();
      result = (paramsCnt >= 0);
      if (not result)
      {
        if (logf != NULL)
          fprintf(logf, "*** %d error: Stack underflow!\n", lineNum);
        printf("USequencer::parseFunctionReturn *** %d error: Stack underflow! at '%s'\n",
                    lineNum, cl->getCmdLine());
        paramsCnt = 0;
      }
    }
  }
  if (result and not syntaxCheck and nextLine != NULL)
  { // get return line
    *nextLine = popStack();
    // get function that is returned into (with local variable scope)
    currentFunction = NULL;
    fn = popStack();
    if (fn >= 0)
      currentFunction = &funcs[fn];
  }
  //
  return result;
}

///////////////////////////////////////////////////////////////////////////////

bool USequencer::parseWatchStatement(USeqLine * line,
                                     int lineNum,
                                     FILE * logf,
                                     bool syntaxCheck)
{
  bool result = false;
  USeqWatch * wa;
  const int MNL = MAX_VARIABLE_NAME_SIZE + 2;
  char name[MNL];
  const char * p1, * p2;
  double wTime = 0.0;
  USeqLine * wLine;
  USeqFunction * func;
  //
  result = line->getIdentifier(line->getParams(), name, MNL, (char **)&p1);
  if (not result and (logf != NULL))
    fprintf(logf, "*** %d error: not a valid identifier\n", lineNum);
  if (result)
  {
    wa = getWatch(name);
    if (wa == NULL)
    { // not created yet
      result = (watchCnt < MAX_SEQ_WATCH_LINES);
      if (not result and logf != NULL)
        fprintf(logf, "*** %d error: no more space for watches\n", lineNum);
      if (result)
        wa = &watch[watchCnt++];
    }
  }
  if (result)
  {
    switch (line->getType())
    {
    case SQ_WATCH:
      if (strncasecmp(p1, "every ", 6) == 0)
      { // timed watch
        p2 = p1 + 6;
        wTime = calc->evaluate(line->getCmdLine(), p2, &p1, NULL, syntaxCheck);
        result = (p1 > p2);
        if (not result and (logf != NULL))
          fprintf(logf, "*** %d error: %s\n", lineNum, calc->getErrorTxt());
      }
      if (result)
      { // set active and load name
        wa->set(false, wTime, name);
        // get line to start watch
        if ((strlen(p1) <= 1) or (*p1 == '('))
        { // watch name is a function name?
          func = getFunction(name);
          if (func != NULL)
          {
            strcat(name, "()");
            p1 = name;
          }
        }
        wLine = wa->getLine();
        result = wLine->setLine(p1, calc->getVarPool(), lineNum, logf);
        if (not result)
        { // watch command is not valid
          if (logf != NULL)
            fprintf(logf, "*** %d error: watch line not valid - the '%s' part\n",
                    lineNum, wLine->getCmdLine());
          printf("USequencer::parseWatchStatement: *** %d error: watch line not valid - the '%s' part\n",
                  lineNum, wLine->getCmdLine());
        }
        if (result and not syntaxCheck)
          wa->setActive(true);
      }
      break;
    case SQ_UNWATCH:
      wa->setActive(false);
      break;
    default:
      result = false;
      break;
    }
  }
  //
  return result;
}

////////////////////////////////////////////////////////////////////////////

bool USequencer::parsePrintStatement(USeqLine * line,
                                     int lineNum,
                                     FILE * logf,
                                     bool syntaxCheck)
{
  bool result = true;
  const int MNL = 150;
  char name[MNL] = "";
  const int MRL = 200;
  char reply[MRL];
  char * p1;
  const char *p2;
  UTime t;
  //
  // format: print text value
  // is send as <push text="name" value="value"/>
  // last value part is included if a valid expression follows the 'text' only
  if (true)
  {
    p1 = line->getParams();
    result = calc->evaluateToString(line->getCmdLine(), p1, &p2, name, MRL);
  }
/*  else
  {
    p1 = line->getParams();
    if ((*p1 == '"') or (*p1 == '\''))
    {
      p2 = p1 + 1;
      p2 = strchr(p2, *p1);
      if (p2 > p1)
      { // skip quote apos
        p1++;
        // copy content
        strncpy(name, p1, p2 - p1);
        // zero terminate
        name[p2 - p1] = '\0';
        p2++;
      }
    }
    else
    { // get as string
      sscanf(p1, "%s", name);
      p2 = p1 + strlen(name);
    }
    //
    result = strlen(name) > 0;
    //
    if (result)
    { // make name XML-safe
      str2xml(name, MNL, name);
      // get value
      value = calc->evaluate(line->getCmdLine(), p2, &p3, &isOK, syntaxCheck);
      n = mini(p3-p2, MRL);
      strncpy(var, p2, n);
      var[n] = '\0';
    }
  }*/
  //
  if (result)
  {
/*    if (isOK)
      snprintf(reply, MRL, "<help info=\"%s (%s)=%.12g\"/>\n",
                    name, var, value);
    else*/
    if (not syntaxCheck)
      snprintf(reply, MRL, "<help info=\"%s\"/>\n",
                    name);
    // send to all clients and to sequencer log
    if (logf != NULL)
    {
      t.Now();
      fprintf(logf, "%lu.%03lu %s", t.getSec(), t.getMilisec(), reply);
    }
    // send to connected clients
    result = sendToAll(reply, -1);
  }
  //
  return result;
}

//////////////////////////////////////////////////////////////////////////////

bool USequencer::parseLabelStatement(USeqLine * line,
                                     int lineNum,
                                     FILE * logf,
                                     bool syntaxCheck)
{ // works like a function with no return and no parameters
  // so nothing to do here
  return true;
}

///////////////////////////////////////////////////////////////

bool USequencer::initiateSmrClCmd(const char * cmd)
{
  bool result = false;
  const int MSL = 200;
  char s[MSL];
  const char * p2;
  //
  // evaluate parameters in SMRCL command
  result = calc->evaluateToString(cmd, cmd, &p2, s, MSL);
  // debug
  printf("InitSMRCL: <%s> conv to <%s>\n", cmd, s);
  printf("InitSMRCL: ******************** NOT SUPPORTED P.T. ******************\n");
  result = false;
  // debug end
  //
  return result;
}

///////////////////////////////////////////////////////////////

bool USequencer::doWatches(UTime sampleTime)
{
  bool result = true;
  int w;
  USeqWatch * wp;
  bool isOK;
  USeqLine * line;
  double v;
  bool vOK;
  //
  wp = watch;
  for (w = 0; w < watchCnt; w++)
  {
    if (wp->isActive())
    {
      if (wp->isTimeToWatch(sampleTime))
      {
        line = wp->getLine();
        // execute function, but ignore return value
        vOK = false;
        isOK = doExecuteFunction(line, &v, &vOK);
        if (not isOK)
          result = false;
        // log
/*        if (logc != NULL)
          fprintf(logc, "%lu.%03lu watch %d '%s' returned %.4g (%s) [%s]\n",
                  sampleTime.getSec(), sampleTime.getMilisec(),
                  w, wp->getName(), v,
                  bool2str(vOK), bool2str(result));*/
        // log end
      }
    }
    wp++;
  }
  //
  return result;
}

///////////////////////////////////////////////////////////////

bool USequencer::doExecuteFunction(USeqLine * fromLine,
                                  double * returnValue,
                                  bool * returnValueValid)
{
  bool result = true;
  USeqLine * lp;
  int nextLine;
  int thisLine;
  int n = 0;
  //
  // parse watch statement
  nextLine = -1;
  result = parseStatement(fromLine, -2, NULL, false, &nextLine);
  while (nextLine >= 0)
  {
    thisLine = nextLine;
    lp = getLine(thisLine);
    if (lp->getType() == SQ_DRIVE_STATEMENT)
    { // there should not be drive statements in a watch
      if (logSeq != NULL)
        fprintf(logSeq, "*** %d drive statement in watch is ignored!!!\n", nextLine);
      printf("*** %d drive statement in watch is ignored!!!\n", nextLine);
      // check for plan just cleared (in another thread)
      if (watchCnt == 0)
        break;
      // exit endless loop
      n++;
      if (n > 20)
        break;
    }
    else if (lp->getType() == SQ_FUNCTION_RETURN)
    { // return lines may include a return value
      result = parseFunctionReturn(lp, thisLine, NULL, false, &nextLine,
                                   returnValue, returnValueValid);
    }
    else
    { // normal statement
      result = parseStatement(lp, thisLine, NULL, false, &nextLine);
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////////////

bool USequencer::evaluateSequencerFunction(const char * name,
                                         const double pars[],
                                         int parsCnt,
                                         double * value,
                                         bool syntaxCheck)
{
  bool result;
  USeqLine line;
  USeqFunction * func;
  double val = -1.0;
  bool valValid = true;
  const int MFCL = 200;
  char fcl[MFCL];
  int n, i;
  //
  func = getFunction(name);
  result = (func != NULL);
  if (result and not syntaxCheck)
  { // make a dummy function call line
    snprintf(fcl, MFCL, "%s(", name);
    n = strlen(fcl);
    for (i = 0; i < parsCnt; i++)
    {
      snprintf(&fcl[n], MFCL - n, "%.12g, ", pars[i]);
      n = strlen(fcl);
    }
    if (parsCnt > 0)
      n -= 2;
    snprintf(&fcl[n], MFCL - n, ")\n");
    // put in call line structure
    line.setLine(fcl, calc->getVarPool(), -2, NULL);
    // call function
    result = doExecuteFunction(&line, &val, &valValid);
    if (result)
      result = valValid;
  }
  if (result and (value != NULL))
    *value = val;
  //
  return result;
}

///////////////////////////////////////////////////////////////

USeqLine * USequencer::getLine(int lineNum)
{
  USeqLine * result = NULL;
  //
  if ((lineNum >= 0) and (lineNum < linesCnt))
    result = &lines[lineNum];
  //
  return result;
}

/////////////////////////////////////////////

bool USequencer::setLogFile(bool sim,
                           const char * simSubDir,
                           const char * simLogfile,
                           const char * logPath,
                           const char * logfile)
{
  bool result = true;
  //
  simulated = sim;
  setSimulated(sim);
  if (simLogfile != NULL and sim)
  {
    snprintf(simFileName, MAX_FILENAME_SIZE, "%s/%s/%s",
              logPath, simSubDir, simLogfile);
    simOpenReplayFile();
  }
  if (logSeq != NULL)
  {
    fclose(logSeq);
    logSeq = NULL;
  }
  if (logfile != NULL)
  {
    snprintf(logFileName, MAX_FILENAME_SIZE, "%s/%s", logPath, logfile);
    logSeq = fopen(logFileName, "w");
    result = (logSeq != NULL);
  }
  //
  return result;
}

/////////////////////////////////////////////

bool USequencer::simOpenReplayFile()
{
  if (simc != NULL)
  {
    fclose(simc);
    simc = NULL;
  }
  simc = fopen(simFileName, "r");
  simulated = (simc != NULL);
  if (not simulated)
    printf("USequencer::simOpenReplayFile: "
        "sim file not found: '%s'\n", simFileName);
  if (simulated)
    simLineTime.setTime(0, 0);
  return simulated;
}

///////////////////////////////////////////////////////////////

bool USequencer::testExplicitStopCond(const char * explicitStopCond, bool * syntaxError)
{
  bool result;
  double value;
  bool isOK;
  const char * esptr;
  //
  if (explicitStopCond != NULL)
  {
    value = calc->evaluate(explicitStopCond, explicitStopCond, &esptr, &isOK, false);
    if (not isOK)
    { // stop if syntax error
      result = true;
      snprintf(errorTxt, MAX_ERROR_SIZE,
              "Syntax error in stop condition at %d in %s",
              esptr - explicitStopCond, explicitStopCond);
      if (syntaxError != NULL)
        *syntaxError = true;
    }
    else
      result = (value == true);
  }
  else
    // no stop condition
    result = false;
  //
  return result;
}

///////////////////////////////////////////////////////////////

void USequencer::pushStack(int number)
{
  stack[stackCnt++] = number;
  if (stackCnt >= MAX_SEQ_STACK_SIZE)
  {// overflow
    if (logSeq != NULL)
      fprintf(logSeq, "*** %d USequencer::pushStack: stack overflow\n", lineActive);
    printf("*** %d USequencer::pushStack: stack overflow\n", lineActive);
    stackCnt--;
  }
}

///////////////////////////////////////////////////////////////

int USequencer::popStack()
{
  int result;
  if (stackCnt > 0)
    result = stack[--stackCnt];
  else
  { // underflow
    if (logSeq != NULL)
      fprintf(logSeq, "*** %d USequencer::popStack: stack underflow\n", lineActive);
    printf("*** %d USequencer::popStack: stack underflow\n", lineActive);
    result = -1;
  }
  return result;
}

///////////////////////////////////////////////////////////////

bool USequencer::pushParam(double value)
{
  bool result;
  //
  result = (paramsCnt < MAX_SEQ_PARAM_STACK_SIZE);
  if (result)
    params[paramsCnt++] = value;
  else
  {    // overflow
    if (logSeq != NULL)
      fprintf(logSeq, "*** %d USequencer::pushParam: stack overflow\n", lineActive);
    printf("*** %d USequencer::pushParam: stack overflow\n", lineActive);
    paramsCnt--;
  }
  return result;
}

///////////////////////////////////////////////////////////////

double USequencer::popParam()
{
  double result;
  if (paramsCnt > 0)
    result = params[--paramsCnt];
  else
  { // underflow
    if (logSeq != NULL)
      fprintf(logSeq, "*** %d USequencer::popParam: stack underflow\n", lineActive);
    printf("*** %d USequencer::popParam: stack underflow\n", lineActive);
    result = 0.0;
  }
  return result;
}

///////////////////////////////////////////////////////////////

void USequencer::setActiveLine(const int lineNum)
{
  lineActive = lineNum;
}

///////////////////////////////////////////////////////////////

int USequencer::findLabelNum(const char * labelName)
{
  int result = -1;
  int i;
  USeqLine * sl = lines;
  // find length to compare - do not include the optional ':'
  for (i = 0; i < linesCnt; i++)
  {
    if (sl->isThisLabel(labelName))
    {
      result = i;
      break;
    }
    sl++;
  }
  return result;
}

////////////////////////////////////////////////////////

bool USequencer::parseFunctionCall(USeqLine * line,
                          int lineNum,
                          USeqFunction * func,
                          const char ** paramStart,
                          int * nextLine,
                          FILE * logf,
                          bool syntaxCheck)
{
  bool result;
  const int MPN = SEQ_MAX_FUNCTION_PARAMS;
  double paramValues[MPN];
  int paramCnt = 0;
  int i;
  const int MSL = 1000;
  const int MSLN = 3; // max 3 string parameters
  char parStr[MSLN][MSL];
  char * strPars[MSLN] = { parStr[0], parStr[1], parStr[2]};
  char parOrder[MPN];
  //
  result = (line != NULL) and (func != NULL) and (paramStart != NULL);
  if (result)
  {
    result = calc->evaluateParameters(
                  line->getCmdLine(), // source line
                  paramStart,         // start of param values - should point at '('
                  paramValues, NULL,  // value array (doubles)
                  strPars,            // string parameters
                  parOrder,           // parameter order - e.g. "dd" for 2 doubles.
                  MPN,                // max parameter count
                  MSLN,               // max number of string parameters
                  MSL);               // max string parameter length
    if (not result and (logf != NULL))
      fprintf(logf, "*** %d error: %s\n", lineNum, calc->getErrorTxt());
    if (result)
    { // p.t. only double parameters supported in script functions
      result = (strchr(parOrder, 's') == NULL);
      if (result)
        // no strings allowed
        paramCnt = strlen(parOrder);
    }
  }
  if (result)
  {
    result = (paramCnt == func->getParamCnt());
    if (not result and logf != NULL)
      fprintf(logf, "*** %d error: Expected %d parameters, but found %d\n",
             lineNum, func->getParamCnt(), paramCnt);
  }
  if (result and not syntaxCheck)
  { // push values on param value stack
    for (i = 0; i < paramCnt; i++)
    {
      result = pushParam(paramValues[i]);
      if (not result)
        break;
    }
    if (not result)
    { // roll back all pushed values
      paramCnt = i;
      for (i = paramCnt; i > 0; i--)
        popParam();
      if (logf != NULL)
        fprintf(logf, "*** %d error: %s\n", lineNum, calc->getErrorTxt());
    }
  }
  if (result and not syntaxCheck)
  { // save the function environment that is left behind
    pushStack(getFunction(currentFunction));
    // save line to return to
    pushStack(lineNum + 1); // return to following line
    // continue on first line of function (after declaration)
    *nextLine = func->getStartLine() + 1;
    // save pointer to current function
    currentFunction = func;
  }
  //
  return result;
}

////////////////////////////////////////

bool USequencer::parseIfStatement(USeqLine * line,
                          int lineNum,
                          int * nextLine,
                          FILE * logf,
                          bool syntaxCheck)
{
  bool result;
  USeqLine thenPart;
  const char *p1, *p2;
  double value;
  //
  p1 = line->getParams();
  value = calc->evaluate(line->getCmdLine(), ++p1, &p2, &result, syntaxCheck);
  if (not result and logf != NULL)
    fprintf(logf, "*** %d error: %s on '%s'\n",
            lineNum, calc->getErrorTxt(), line->getCmdLine());
  if (result and ((value == true) or syntaxCheck) and (*p2 == ')'))
  { // make line with then part only
    p2++;
    result = thenPart.setLine(p2, calc->getVarPool(), lineNum, logf);
    if (result)
    {
      if (thenPart.isLegalThenStatement())
        // do as appropriate
        result = parseStatement(&thenPart, lineNum, logf, syntaxCheck, nextLine);
      else
      {
        result = false;
        if (logf != NULL)
          fprintf(logf, "*** %d error: illegal then part of if statement '%s'\n",
                    lineNum, thenPart.getCmdLine());
      }
    }
  }
  //
  return result;
}

////////////////////////////////////////////////////////////////

USeqWatch * USequencer::getWatch(const char * name)
{
  USeqWatch * result = NULL;
  USeqWatch * wa;
  int i;
  //
  wa = watch;
  for (i = 0; i < watchCnt; i++)
  {
    if (strcasecmp(wa->getName(), name) == 0)
    {
      result = &watch[i];
      break;
    }
    wa++;
  }
  //
  return result;
}

////////////////////////////////////////////////////////////////

bool USequencer::getLocalScopeVaiable(const char * name, double * value)
{
  bool result;
  //
  result = (currentFunction != NULL);
  if (result)
  { // get local value - if such exist
    result = currentFunction->getValue(name, value,
                                       &params[paramsCnt],
                                       paramsCnt);
  }
  //
  return result;
}

////////////////////////////////////////////////////////////////

bool USequencer::sendToAll(const char * message, int lockedUser)
{ // do not know clients at this level - should be overwritten at higher level
  printf("%s\n", message);
  return true;
}

////////////////////////////////////////////////////////////////

bool USequencer::sendInfoToAll(const char * message, int lockedUser)
{ // do not know clients at this level - should be overwritten at higher level
  printf("%s\n", message);
  return true;
}

////////////////////////////////////////////////////////////////

bool USequencer::getSmrdemoValue(const char * name, double * value)
{
  const char * validSysNames =
      "$odox " //           x-coordinate of vehicle position (m)
      "$odoy " //           ycoordinate of vehicle position (m)
      "$odoth " //          direction of vehicle (rad)
      "$odovelocity " //     velocity of vehicle (based on odometry measurements)
      "$ododist " //        distance driven since start of vehicle (m)
      "$drivendist " //     distance driven during the current command (m)
      //IR-sensors:
      //all distances are measured from the sensor itself.
      "$irdistleft " //             distance (m) measured by the left side sensor
      "$irdistfrontleft " //                 distance (m) measured by the front left sensor
      "$irdistfrontmiddle " //      distance (m) measured by the front middle sensor
      "$irdistfrontright " //       distance (m) measured by the front right sensor
      "$irdistright " //            distance (m) measured by the right side sensor
      "$cmdtime " //                time spent in the current command (sec)
      "$batteryvoltage " //          batteryvoltage (v) lowpass filtered with a time constant of a
      //"second " //
      "$supplyvoltage " //           voltage of the external power supply (v) lowpass filtered with a
      //                         time constant of 0.1 second
      //Linesensor:
      "$line0 $line1 $line2 $line3 $line4 $line5 $line6 $line7 " //  linesensor values
      "$linepos " //                position of found line
      "$blacklinefound " //         true if a black line is detected ( used to find a line approched from
      //                         the side)
      "$crossingblackline " //       true if a black line is crossing the line the vehicle
      " ";
  bool result;
  const int MVL = 100;
  char vn[MVL];
  const char * p1;
  int n;
  //double val;
  //
  p1 = strstr(validSysNames, name);
  result = (p1 != NULL);
  if (result)
  {
    n = strlen(name);
    result = (p1[n] == ' ');
    if (result)
    {
      p1--;
      n++;
      result = (*p1 == '$');
    }
    if (result)
    {
      strncpy(vn, p1, n + 1);
      vn[n] = '\0';
      printf("USequencer::getSmrdemoValue *************** DRIVE not implemented ***************\n");
      //val = drive->getSmrdemoValue(vn, &result);
      //if (value != NULL)
      //  *value = val;
    }
  }
  return result;
}

////////////////////////////////////////////////////////

// bool USequencer::getSmrdemoPose()
// {
//   bool result;
//   // if result is available, it updates the pose in calc-structure
//   result =  drive->getSmrdemoPose();
//   return result;
// }

////////////////////////////////////////////////////////

int USequencer::getSimLine(UTime toTime)
{
  int result;
  char * p1, *p2;
  bool finished = false;
  bool getNewLine = false;
  int orgLine;
  const int MSL = MAX_SIMLINE_SIZE;
  char line[MSL];
  char s[MSL];
  unsigned long sec, msec;
  int n;
  //
  if (simc == NULL)
    result = linesCnt;
  else
  { // more data in logfile
    result = lineActive;
    while (not finished and (simc != NULL))
    {
      if (simLineTime.getSec() == 0)
        getNewLine = true;
      else
      {
        finished = ((toTime - simLineTime) < 0.0);
        if (not finished)
        { // line is ready to implement
          // add line to program list
          add(simLine, NULL, &result);
          // get next line
          getNewLine = true;
          // finish for now (execute line)
          finished = true;
        }
      }
      while (getNewLine)
      { // no line fetched
        p1 = fgets(line, MSL, simc);
        if (p1 == NULL)
          // no more lines
          break;
        // get line time
        n = sscanf(line, "%lu.%lu %s %d", &sec, &msec, s, &orgLine);
        if ((n == 4) and (strcmp(s, "Line") == 0))
        { // read format up to ':' correctly
          p2 = strchr(line, ':');
          p1 = strchr(line, '\'');
          if ((p1 > p2) and (p2 != NULL))
          { // ": '" is in place too
            p1++;
            // get last '
            p2 = strrchr(p1, '\'');
          }
          if ((p2 != NULL) and (p2 > p1))
          { // copy line to buffer
            n = p2 - p1; // line length
            strncpy(simLine, p1, n);
            simLine[n] = '\0';
            simLineTime.setTime(sec, msec * 1000);
            getNewLine = false;
          }
        }
      }
      if (getNewLine and feof(simc))
      {
        fclose(simc);
        simc = NULL;
        simulated = false;
        break;
      }
    }
  }
  return result;
}



