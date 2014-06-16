/***************************************************************************
 *   Copyright (C) 2012 by DTU (Christian Andersen)                        *
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

#include <cstdlib>

#include <urob4/ufuncplugbase.h>
#include <urob4/userverport.h>

#include "urespcp.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * The command interface of a job scheduler that is asunchronous to the the main command queue, and thus long timeconsuming jobs are holding the main command thread.
 * It adds though one additional thread only, and commands are run as pipes.
@author Christian Andersen
*/
class UFuncPcp : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFunctionLine) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncPcp()
  { // command list and version text
    setCommand("pcp", "pcpgons", "storage of pcpgons, pcplines and points");
    pcp = NULL;
    reply = (char *) malloc(15000);
    replyCnt = 15000;
  }
  /**
  Destructor */
  virtual ~UFuncPcp()
  { // possibly remove allocated variables here - if needed
    if (pcp != NULL)
      delete pcp;
    if (reply != NULL)
      free(reply);
  };

  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs and add these to the resource pool
   * to be integrated in the global variable pool and method sharing.
   * Remember to add to resource pool by a call to addResource(UResBase * newResource, this).
   * This module must also ensure that any resources creted are deleted (in the destructor of this class).
   * Resource shut-down code should be handled in the resource destructor.*/
  virtual void createResources()
  {
    pcp = new UResPcp();
    addResource(pcp, this);
  };
  /**
   * Handle incomming commands flagged for this module
   * \param msg pointer to the message and the client issuing the command
   * \return true if the function is handled - otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra)
  {
    bool ask4help;
    const int MVL = 1000;
    char val[MVL];
    bool replySend = false;
    bool gotOpenLog;
    bool gotOpenLogValue = false;
    bool gotVerbose;
    bool gotVerboseValue = false;
    bool gotAdd = false;
    bool gotDel = false;
    bool gotSave = false;
    const int MNL = 100;
    char name[MNL] = "";
    bool gotList = false;
    bool isOK;
    bool gotX, gotY;
    double x, y;
    bool gotGet = false;
    bool gotUpdate = false;
    UTime t;
    int c;
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
      gotAdd = msg->tag.getAttValue("add", name, MNL);
      gotDel = msg->tag.getAttValue("del", name, MNL);
      gotSave = msg->tag.getAttValue("save", name, MNL);
      gotList = msg->tag.getAttValue("list",  NULL, 0);
      gotX = msg->tag.getAttDouble("X",  &x);
      gotY = msg->tag.getAttDouble("Y",  &y);
      if (not gotX)
        gotX = msg->tag.getAttDouble("E",  &x);
      if (not gotY)
        gotY = msg->tag.getAttDouble("N",  &y);
      gotGet = msg->tag.getAttValue("get",  val, MVL);
      gotUpdate = msg->tag.getAttValue("update",  val, MVL);
      if (gotUpdate and strlen(val) > 0)
        gotUpdate = str2bool2(val, true);
    }
    // ask4help = false, i.e. no 'help' option.
    if (ask4help)
    { // create the reply in XML-like (html - like) format
      sendHelpStart("POLY");
      sendText("--- available POLY options (mission pcp lines and points)\n");
      if (pcp == NULL)
      {
        sendText("*** The needed POLY resource is not available ***\n");
        sendText("help       This message\n");
      }
      else
      { // full functionality
        snprintf(reply, replyCnt,
                "list               List all pcpgons (has %d)\n", pcp->getPolysCnt());
        sendText(reply);
        sendText("get                Get all items in one xml message each\n");
        sendText("update             Get all updated items only - first time you get all\n");
        sendText("add=name           Add a new (empty) cloud with this name\n");
        sendText("add=name x=X y=Y   Add one new point point to an item\n");
        sendText("add=name E=X N=Y   Add one new point point to an item\n");
        sendText("del=name           Delete item with this name (may hold one '*' wildcard)\n");
        sendText("save=name          save the cloud with this name as a pcdFile\n");
        sendText("verbose[=false]    Posibly more messages to server console\n");
        sendText("log[=false]        Open [or close] logfile\n");
        sendText("help               This message\n");
        sendText("--------\n");
      }
      sendHelpDone();
      replySend = true;
    }
    else if (pcp == NULL)
    {
      sendWarning("no POLY resource to do that - try unload and reload plug-in");
      replySend = true;
    }
    else
    { // resource is available, so make a reply
      if (gotVerbose)
      {
        pcp->verbose = gotVerboseValue;
        sendInfo("done");
        replySend = true;
      }
      if (gotOpenLog)
      {
        pcp->openLog(gotOpenLogValue);
        sendInfo("done");
        replySend = true;
      }
      if (gotList)
      {
        pcp->getList("pcp", reply, replyCnt);
        sendHelpStart("POLY list");
        sendText(reply);
        replySend = sendHelpDone(msg);
      }
      if (gotAdd)
      {
        UPcpItem * pi = pcp->getItem(name);
        if (pi == NULL)
          pi = pcp->add(name);
        isOK = pi != NULL;
        if (isOK and (gotX or gotY))
        {
          isOK = pcp->add(name, x, y);
        }
        if (isOK)
          replySend = sendInfo("added OK");
        else
          replySend = sendWarning("failed - no more space?");
        pcp->gotNewData();
      }
      if (gotDel)
      {
        pcp->del(name);
        replySend = sendInfo("deleted (or did not exist)");
        pcp->gotNewData();
      }
      if (gotSave)
      {
        const int MFL = MAX_FILENAME_SIZE;
        char fn[MFL];
        makeFilename(fn, MFL, "", name, "pcd");
        bool isOK = pcp->savePCD(name, fn);
        if (isOK)
          snprintf(reply, replyCnt, "saved %s to %s\n", name, fn);
        else
          snprintf(reply, replyCnt, "failed to save %s to %s\n", name, fn);
        replySend = sendInfo(reply);
      }
      if (gotGet or gotUpdate)
      { // calculate needed buffer size
        int ns = replyCnt;
        for (int u = 0; u < pcp->getPolysCnt(); u++)
        {
          UPcpItem * pi = pcp->getItem(u);
          if (pi->getNeededBufferSize() + 100 > ns)
            ns = pi->getNeededBufferSize() + 100;
        }
        if (ns > replyCnt)
        {
          reply = (char *) realloc(reply, ns);
          replyCnt = ns;
        }
        c = msg->client + 20;
        if (gotUpdate and c >= 0)
          // get index to last update timestamp
          t = updateTimes[c];
        else
          // else all updated since start of time (1970)
          t.clear();
        updateTimes[c].now();
        for (int u = 0; u < pcp->getPolysCnt(); u++)
        {
          UPcpItem * pi = pcp->getItem(u);
          pi->lock();
          if (pi != NULL)
            if (pi->updateTime > t)
              if (pcp->codePcpXml(msg->tag.getTagName(), reply, replyCnt, u))
                replySend = sendMsg(reply);
          pi->updateTime = updateTimes[c];
          pi->unlock();
        }
      }
    }
    if (not replySend)
      sendInfo(msg, "no pcp action performed (no command option?)");
    // return true if the function is handled with a positive result
    return true;
  };

  
  /**
  * make filename from basename. if base filename starts with '.', '/' ot '~', then no pre-path is added,
  * else the files are placed in the dataPath path.
  * \param[out] fn is destination filename buffer
  * \param MFL is length of buffer
  * \param add is optional namepart to be added before last ".pcd"
  * \param basefilename is the desired base filename.
  * \param nameType the expected - and default - file name extension - added if not in basename.
  * \returns a pointer to destination buffer. */
  const char * makeFilename(char * fn, int  MFL, const char * add, const char * basefilename, const char * nameType = "pcd")
  {
    const char * p1;
    char * p2;
    int n = 0;
    p2 = fn;
    if (basefilename[0] == '.' or basefilename[0] == '/' or basefilename[0] == '~')
      strncpy(fn, basefilename, MFL);
    else
    {
      snprintf(fn, MFL, "%s/", dataPath);
      n += strlen(p2);
      p2 = &fn[n];
    }
    p1 = strrchr(basefilename, '.');
    strncpy(p2, basefilename, MFL - n);
    n += strlen(p2);
    p2 = &fn[n];
    if (p1 != NULL)
    { // test if . is start of expected file-type
      if (not strcasecmp(&p1[1], nameType) == 0)
        p1 = NULL;
    }
    if (p1 != NULL)
    {
      n += p1 - basefilename;
      p2 = &fn[n];
      snprintf(p2, MFL - n, "%s%s", add, p1);
    }
    else
    {
      snprintf(p2, MFL - n, "%s.%s", add, nameType);
    }
    return fn;
  }

protected:
  /**
  line pointer */
  UResPcp * pcp;
  /** last update for each client */
  UTime updateTimes[MAX_SOCKET_CLIENTS_SERVED + 20];
  /// buffer for sending reply to client
  char * reply;
  /// size of reply buffer
  int replyCnt;
};


#ifdef LIBRARY_OPEN_NEEDED
///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object
UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncPcp' with your classname, as used in the headerfile */
  return new UFuncPcp();
}
#endif

