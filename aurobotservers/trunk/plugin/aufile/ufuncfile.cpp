/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#include "ufuncfile.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncFile' with your classname, as used in the headerfile */
  return new UFuncFile();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncFile::~UFuncFile()
{ // possibly remove allocated variables here - if needed
  if (file != NULL)
    delete file;
}

////////////////////////////////////////////////////////

void UFuncFile::createResources()
{
  file = new UResFile();
  addResource(file, this);
}

///////////////////////////////////////////////////

bool UFuncFile::handleCommand(UServerInMsg * msg, void * extra)
{
  const int MRL = 50000;
  char reply[MRL];
  bool ask4help;
  const int MVL = 1000;
  char val[MVL];
  char addTxt[MVL];
  bool replySend = false;
  bool gotOpenLog;
  bool gotOpenLogValue = false;
  bool gotVerbose;
  bool gotVerboseValue = false;
  bool gotAdd = false;
  bool gotDel = false;
  const int MNL = 500;
  char name[MNL] = "";
  bool gotList = false;
  bool isOK;
  bool gotGet = false;
  bool gotNew = false;
  bool gotClose = false;
  FILE * fh;
  const char * p1;
//  UTime t;
  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (not ask4help)
  { // get all other parameters
    gotOpenLog = msg->tag.getAttValue("log", val, MVL);
    if (gotOpenLog)
      gotOpenLogValue = str2bool2(val, true);
    gotVerbose = msg->tag.getAttValue("verbose", val, MVL);
    if (gotVerbose)
      gotVerboseValue = str2bool2(val, true);
    gotNew = msg->tag.getAttValue("new", name, MNL);
    gotAdd = msg->tag.getAttValue("add", addTxt, MVL);
    gotDel = msg->tag.getAttValue("del", name, MNL);
    gotList = msg->tag.getAttValue("list",  NULL, 0);
    gotGet = msg->tag.getAttValue("get",  name, MVL);
    gotClose = msg->tag.getAttValue("close", NULL, 0);
  }
  // ask4help = false, i.e. no 'help' option.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart( msg, "FILE");
    sendText( msg, "--- available FILE options (file transfer options)\n");
    if (file == NULL)
    {
      sendText( msg, "*** The needed FILE resource is not available ***\n");
      sendText( msg, "help       This message\n");
    }
    else
    { // full functionality
      snprintf(reply, MRL, "list               files (in %s)\n", file->getDirName());
      sendText(msg, reply);
      snprintf(reply, MRL, "new=name           Create (or replace) a file with this name (is %s)\n", file->getFilename(msg->client));
      sendText(reply);
      snprintf(reply, MRL, "add='text'         Add this text to the file (max message length is %d bytes)\n", MAX_MESSAGE_LENGTH_TO_CAM);
      sendText(reply);
      sendText(msg,        "close              Close created file\n");
      sendText(msg,        "del=name           Delete file\n");
      sendText(msg,        "get=name           Get a file as one XML message\n");
      sendText(msg,        "verbose[=false]    Posibly more messages to server console\n");
      sendText(msg,        "log[=false]        Open [or close] logfile\n");
      sendText(msg,        "help               This message\n");
      sendText(msg,        "--------\n");
      sendText(msg,        "See also 'VAR FILE' for file path\n");
    }
    sendHelpDone(msg);
    replySend = true;
  }
  else if (file == NULL)
  {
    sendWarning(msg, "no FILE resource to do that - try unload and reload plug-in");
    replySend = true;
  }
  else
  { // resource is available, so make a reply
    if (gotVerbose)
    {
      file->verbose = gotVerboseValue;
      sendInfo("done");
      replySend = true;
    }
    if (gotOpenLog)
    {
      file->openLog(gotOpenLogValue);
      sendInfo("done");
      replySend = true;
    }
    if (gotList)
    {
      file->getList("", reply, MRL);
      sendHelpStart("file list");
      sendText(reply);
      replySend = sendHelpDone(msg);
    }
    if (gotNew)
    {
      isOK = file->newFile(msg->client, name);
      if (isOK)
      {
        snprintf(reply, MRL, "created %s\n",
                 file->getFilename(msg->client, "", val, MVL));
        replySend = sendInfo(reply);
      }
      else
      {
        snprintf(reply, MRL, "Failed to create %s\n",
                 file->getFilename(msg->client, "", val, MVL));
        replySend = sendWarning(reply);
      }
    }
    if (gotAdd)
    {
      isOK = file->add(msg->client, addTxt);
      if (isOK)
        replySend = sendInfo("added OK");
      else
        replySend = sendWarning("failed - no more space?");
    }
    if (gotDel)
    {
      isOK = file->del(name);
      if (isOK)
      {
        snprintf(reply, MRL, "deleted %s\n",
                 file->getFilename(msg->client, "", val, MVL));
        replySend = sendInfo(reply);
      }
      else
      {
        snprintf(reply, MRL, "Failed to delete %s\n",
                 file->getFilename(msg->client, "", val, MVL));
        replySend = sendWarning(reply);
      }
    }
    if (gotClose)
    {
      isOK = file->closeFile(msg->client);
      if (isOK)
      {
        snprintf(reply, MRL, "closed %s\n",
                 file->getFilename(msg->client, "", val, MVL));
        replySend = sendInfo(reply);
      }
      else
        replySend = sendWarning("No file open");
    }
    if (gotGet)
    {
      if (strlen(name) > 0)
        snprintf(val, MVL, "%s/%s", file->getDirName(), name);
      else
        file->getFilename(msg->client, "", val, MVL);
      fh = fopen(val, "r");
      if (fh != NULL)
      {
        if (strlen(name) > 0)
          strncpy(val, name, MVL);
        else
          strncpy(val, file->getFilename(msg->client), MVL);
        snprintf(reply, MRL, "<%s name=\"%s\" path=\"%s\">\n", msg->tag.getTagName(), val, file->getDirName());
        sendMsg(reply);
        while (not feof(fh))
        {
          p1 = fgets(val, MVL, fh);
          if (p1 == NULL)
            break;
          sendText(p1);
        }
        fclose(fh);
        sendEndTag();
      }
      else
      {
        snprintf(reply, MRL, "File not found %s\n", val);
        replySend = sendWarning(reply);
      }
      replySend = true;
    }
  }
  if (not replySend)
    sendInfo(msg, "no file action performed (no command option?)");
  // return true if the function is handled with a positive result
  return true;
}

