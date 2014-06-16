/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)                        *
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

#include <stdio.h>

#include "ufunctionseq.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFunctionSeq' with your classname, as used in the headerfile */
  return new UFunctionSeq();
}
//

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFunctionSeq::UFunctionSeq()
{ // initialization of variables in class - as needed
  setCommand("seq", "seq", "seq (sequencer v2.1514) (" __DATE__ " " __TIME__ " by Christian)");
  seq = NULL;       // initially the resource is not created
  //seqLocal = false; // ... and is thus not local
}

///////////////////////////////////////////////////

UFunctionSeq::~UFunctionSeq()
{ // possibly remove allocated variables here - if needed
  if ((seq != NULL) and seqLocal)
    delete seq;
}

///////////////////////////////////////////////////

// const char * UFunctionSeq::name()
// {
//   return "seq (sequencer v1.77) (" __DATE__ " " __TIME__ " by Christian)";
// }

///////////////////////////////////////////////////

// const char * UFunctionSeq::commandList()
// { // space separated list og command keywords handled by this plugin
//   return "seq";
// }

///////////////////////////////////////////////////

// bool UFunctionSeq::setResource(UResBase * resource, bool remove)
// { // load resource as provided by the server (or other plugins)
//   bool result = true;
//   // test if the provided resource is relevant
//   if (resource->isA(UResSeq::getResClassID()))
//   { // pointer to server the resource that this plugin can provide too
//     // but as there might be more plugins that can provide the same resource
//     // use the provided
//     if (seqLocal)
//       // resource is owned by this plugin, so do not change (or delete)
//       result = false;
//     else if (remove)
//       // the resource is unloaded, so reference must be removed
//       seq = NULL;
//     else if (seq != (UResSeq *)resource)
//       // resource is new or is moved, save the new reference
//       seq = (UResSeq *)resource;
//     else
//       // reference is not used
//       result = false;
//   }
//   else
//     // other resource types may be needed by base function.
//     result = UFunctionBase::setResource(resource, remove);
//   return result;
// }

///////////////////////////////////////////////////

void UFunctionSeq::createResources()
{
  if (seq == NULL)
  { // no cam pool loaded - create one now
    seq = new UResSeq();
    addResource(seq, this);
  }
}

///////////////////////////////////////////////////

// UResBase * UFunctionSeq::getResource(const char * resID)
// {
//   UResBase * result = NULL;
//   //
//   if (strcmp(resID, UResSeq::getResClassID()) == 0)
//   {
//     if (seq == NULL)
//     { // no cam pool loaded - create one now
//       seq = new UResSeq();
//       if (seq != NULL)
//       { // mark as locally created (to delete when appropriate)
//         seqLocal = true;
//       }
//     }
//     result = seq;
//   }
//   if (result == NULL)
//     // requested resource may be held by the base class
//     result = UFunctionBase::getResource(resID);
//   //
//   return result;
// }

///////////////////////////////////////////////////

// bool UFunctionSeq::gotAllResources(char * missingThese, int missingTheseCnt)
// { // called by the server core when a status is needed, but may be used
//   // locally from this plugin too (missingThese may be NULL if not needed)
//   bool result = true;
//   bool isOK;
//   int n = 0;
//   char * p1 = missingThese;
//   //
//   isOK = (seq != NULL);
//   if (not isOK)
//   { // missin this resource
//     if  (missingThese != NULL)
//     { // add this resource to the list of missing resources in 'missingThese'
//       // to allow notification of the user.
//       snprintf(p1, missingTheseCnt, " %s", UResSeq::getResClassID());
//       n += strlen(p1);
//       p1 = &missingThese[n];
//     }
//     result = false;
//   }
//   // ask if the function base is OK too
//   isOK = UFunctionBase::gotAllResources(p1, missingTheseCnt - n);
//   return result and isOK;
// }

///////////////////////////////////////////////////

bool UFunctionSeq::handleCommand(UServerInMsg * msg, void * extra)
{ // extract parameters
  bool result = true;
  // decode vars
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 1000;
  char attValue[VAL_BUFF_LNG];
  // camera and source image variables
  bool ask4help = false;
  bool loadFile = false;
  bool appendFile = false;
  bool getMissions = false;
  bool getStatus = false;
  bool getPgm = false;
  const char * label = NULL;
  bool sendReply = false;
  const int MNL = MAX_FILENAME_LENGTH;
  char loadName[MNL];
  FILE * errLog = NULL;
  const char * errLogName = "loadPlanErr.log";
  const int MELL = 300;
  char se[MELL];
  int i, n;
  FILE * mf;
  char * pMf;
  const int MFL = 100;
  char mfn[MFL];
  USeqLine * line;
  const int MRL = 300;
  char reply[MRL];
  char lineXml[MRL];
  //
  // extract parameters
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  { // camera device
    if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strcasecmp(attName, "load") == 0)
    {
      strncpy(loadName, attValue, MNL);
      loadFile = true;
    }
    else if (strcasecmp(attName, "append") == 0)
    {
      strncpy(loadName, attValue, MNL);
      appendFile = true;
    }
    else if (strcasecmp(attName, "add") == 0)
    { // new mission line
      sendReply = addCmdLine(msg, attValue);
    }
    else if (strcasecmp(attName, "goto") == 0)
    { // new mission line
      if (label == NULL)
        label = seq->addLabel();
      sendReply = addCmdLine(msg, attValue);
    }
    else if (strcasecmp(attName, "clear") == 0)
    { // clear current pgm
      seq->clear();
    }
    else if (strcasecmp(attName, "sim") == 0)
    { // set simulation flag - if true, logfile must be set from cmd-line
      seq->setSimulated(str2bool(attValue));
    }
    else if (strcasecmp(attName, "missions") == 0)
      getMissions = true;
    else if (strcasecmp(attName, "status") == 0)
      getStatus = true;
    else if (strcasecmp(attName, "pgm") == 0)
      getPgm = true;
    else if (strcasecmp(attName, "list") == 0)
      getPgm = true;
    else
      // unknown attribute
      result = false;
  }
  //
  if (ask4help)
  {
    sendHelpStart(msg, "SEQ");
    sendText(msg, "---- Available SEQ options:\n");
    sendText(msg, "missions           Get list of all mission files\n");
    sendText(msg, "status             Get plan sequencer status\n");
    sendText(msg, "pgm | list         Get list of all loaded statements\n");
    sendText(msg, "goto=\"cmd\"         Give a manual mission line (in quotes), and skip to this\n");
    sendText(msg, "load=\"file\"        Load a mission file, and skip to this\n");
    sendText(msg, "append=\"file\"      Append a mission file, and skip to this\n");
    sendText(msg, "add=\"command\"      Add a manual command line\n");
    sendText(msg, "stop               Stop current mission (goto end of mission)\n");
    sendText(msg, "clear              remove all mission lines\n");
    sendText(msg, "sim=true | false   Set simulation flag (not implemented yet)\n");
    sendMsg(msg, "</help>\n");
    sendHelpDone(msg);
  }
  else
  {
    if (loadFile or appendFile)
    {
      errLog = fopen(errLogName, "w");
      if (seq->loadPlan(loadName, errLog, appendFile))
      { // loaded OK
        sendInfo(msg, "Loaded without errors");
      }
      else
      {
        if (errLog != NULL)
        {
          fclose(errLog);
          errLog = fopen(errLogName, "r");
          if (errLog != NULL)
          { // there is a file
            sendInfo(msg, "Load error list:");
            while (fgets(se, MELL, errLog) != NULL)
            { // return all filenames in separate frames
              i = strlen(se);
              if (i > 1)
              { // cut off the linefeed at end of line
                se[i-1] = 0;
                result = sendInfo(msg, se);
              }
              if (not result)
                break;
            }
          }
          else
            result = sendWarning(msg, "Could not read error file");
          if (errLog != NULL)
            fclose(errLog);
        }
        else
          sendWarning(msg, "There were errors, but could not create a logfile");
        sendReply = true;
      }
    }
    if (label != NULL)
    { // start execution from label
      result = seq->skipToLabel(label, false);
    }
  }
  if (getMissions)
  {  // return a list of available missions
    n = system("ls *.txt >missions.list");
    if (n >= 0)
    { // there should be a file
      mf = fopen("missions.list", "r");
      n = -1;
      if (mf != NULL)
      { // there is a file
        while (true)
        { // return all filenames in separate frames
          pMf = fgets(mfn, MFL, mf);
          if (pMf == NULL)
            break;
          if (n < 0)
            sendInfo(msg, "List of mission files:");
          i = strlen(pMf);
          if (i > 1)
          { // cut off the linefeed at end of line
            pMf[i-1] = 0;
            result = sendInfo(msg, pMf);
            n++;
          }
          if (not result)
            break;
        }
        if (n < 0)
          result = sendWarning(msg, "No mission files in default dir");
      }
      else
        result = sendWarning(msg, "Could not create listing of mission files");
      if (mf != NULL)
        fclose(mf);
      sendReply = true;
    }
  }
  if (getStatus)
  {
    sendStatusMessage(msg, msg->tag.getTagName());
    sendReply = true;
  }
    // last if - if noone else has send a reply ...
  if (getPgm)
  {
    if (seq->getLineCnt() > 0)
    { // make start tag for lines
      snprintf(reply, MRL,
               "<%s lineCnt=\"%d\" cmdLineNum=\"%d\" file=\"%s\">\n",
               msg->tag.getTagName(),
               seq->getLineCnt(),
               seq->getActiveLine(),
               seq->getPlanName());
      sendMsg(msg, reply);
      for (i = 0; i < seq->getLineCnt(); i++)
      {
        line = seq->getLine(i);
        str2xml(lineXml, MRL, line->getCmdLine());
        snprintf(reply, MRL,
                 "<seqLine ln=\"%d\" cmd=\"%s\"/>\n",
                 i, lineXml);
        result = sendMsg(msg, reply);
        if (not result)
            // stop if line-buffers are full
          break;
      }
        // and end tag (always try)
      snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
      result = sendMsg(msg, reply);
    }
    else
    { // no plan lines - send status only
      snprintf(reply, MRL,
               "<%s lineCnt=\"%d\" cmdLineNum=\"%d\"/>\n",
               msg->tag.getTagName(),
               seq->getLineCnt(),
               seq->getActiveLine());
      sendMsg(msg, reply);
    }
    sendReply = true;
  }
  if (result and not sendReply)
    sendInfo(msg, "done");
  if (not result)
    sendWarning(msg, "One or more attributes to SEQ is not used!");
  return result;
}

///////////////////////////////////////////////////////////////////

bool UFunctionSeq::addCmdLine(UServerInMsg * msg, const char * line)
{
  FILE * errLog;
  bool result = true;
  int m, i = 0;
  const int MRL = 300;
  char reply[MRL];
  const char * logName = "errLog.log";
  bool isOK;
  // make a file for syntax error etc
  errLog = fopen(logName, "w");
  isOK = seq->add(line,  errLog, &m);
  if (errLog != NULL)
    fclose(errLog);
  if (not isOK)
  {
    if (errLog != NULL)
    { // open for read of messages
      errLog = fopen(logName, "r");
      while (fgets(reply, MRL, errLog) != NULL)
      { // send all warning lines
        sendWarning(msg, reply);
        i++;
      }
      fclose(errLog);
    }
    if ((errLog == NULL) or (i == 0))
    {
      snprintf(reply, MRL,
               "Syntax error line %d, but no log - line will be ignored", m);
      sendWarning(msg, reply);
    }
  }
  else
  {
    snprintf(reply, MRL, "added as line %d", m);
    sendInfo(msg, reply);
  }
  //
  return result;
}

///////////////////////////////////////////////////////////////////

bool UFunctionSeq::sendStatusMessage(UServerInMsg * msg,
                                     const char * tagName)
{
  const int MRL = 1000;
  char reply[MRL];
  bool result = true;
  USeqLine * lineActive;
  int lineNum;
  const int MCL = 200;
  char cmd[MCL] = "(none)";
  //
  if (seq != NULL)
  {
    lineNum = seq->getActiveLine();
    lineActive = seq->getLine(lineNum);
    if (lineActive != NULL)
      str2xml(cmd, MCL, lineActive->getCmdLine());
    snprintf(reply, MRL,
            "<%s "
                "file=\"%s\" "
                "cmd=\"%s\" "
                "cmdLineNum=\"%d\" "
                "isIdle=\"%s\" "
                "simulated=\"%s\">\n",
            tagName, seq->getPlanName(),
            cmd, lineNum,
            bool2str(seq->isIdle()),
            bool2str(seq->isSimulated()));
    result = sendMsg(msg, reply);
    // send stop criteria status
    // -- not
    // send end tag
    snprintf(reply, MRL, "</%s>\n", tagName);
    result = sendMsg(msg, reply);
  }
  else
    sendWarning(msg, "Sequencer (seq) module lot loaded");
  return result;
}

///////////////////////////////////////////////////

const char * UFunctionSeq::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s Sequencer loop resource (empty %s)\n",
                  preString, bool2str(seq == NULL));
  return buff;
}

///////////////////////////////////////////////////////////////////

