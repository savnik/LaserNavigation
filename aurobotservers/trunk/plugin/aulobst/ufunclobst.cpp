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

#include "ufunclobst.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncLobst' with your classname, as used in the headerfile */
  return new UFuncLobst();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncLobst::~UFuncLobst()
{ // possibly remove allocated variables here - if needed
  if (lobst != NULL)
    delete lobst;
}

////////////////////////////////////////////////////////

void UFuncLobst::createResources()
{
  lobst = new UResLobst();
  addResource(lobst, this);
}

///////////////////////////////////////////////////

bool UFuncLobst::handleCommand(UServerInMsg * msg, void * extra)
{
  const int MRL = 50000;
  char reply[MRL];
  bool ask4help;
  const int MVL = 1000;
  char val[MVL];
  bool replySend = false;
  bool gotOpenLog;
  bool gotOpenLogValue = false;
  bool gotVerbose;
  bool gotVerboseValue = false;
  bool gotList = false;
  bool gotMake = false;
  bool gotObst = false;
  bool gotSilent = false;
  bool gotAny = false;
  const int MDL = 32;
  char device[MDL] = "-1";
  ULaserData * scan;
  ULaserDevice * scanner;
  int fake = 0;
  int n;
  char * p1;
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
    gotList = msg->tag.getAttValue("list",  NULL, 0);
    gotMake = msg->tag.getAttValue("make",  NULL, 0);
    gotObst = msg->tag.getAttValue("obst",  NULL, 0);
    gotAny = msg->tag.getAttValue("any",  NULL, 0);
    msg->tag.getAttBool("silent", &gotSilent, true);
    msg->tag.getAttValue("device",  device, MDL);
    if (msg->tag.getAttValue("fake",  val, 0))
    {
      if (strlen(val) > 0)
        fake = strtol(val, NULL, 0);
      else
        fake = 3;
    }
  }
  // ask4help = false, i.e. no 'help' option.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart("LOBST");
    sendText("--- available LOBST options (laser ransac obstacles)\n");
    if (lobst == NULL)
    {
      sendText("*** The needed LOBST resource is not available ***\n");
      sendText("help       This message\n");
    }
    else
    { // full functionality
      sendText("device=N   Use scanner device N (else default)\n");
      sendText("fake=F     Fake data from this device - F=1..4\n");
      sendText("any        Use any scan (ignore already used flag)\n");
      sendText("make       Make obstacles from scan\n");
      sendText("obst       Send findings to obstacle resource\n");
      sendText("silent     Do not send obstacles to client\n");
      sendText("list       list newest ransac lines\n");
      sendText("help       This message\n");
      sendText("--------\n");
      sendText("See also 'VAR LOBST' for make parameters\n");
    }
    sendHelpDone();
    replySend = true;
  }
  else if (lobst == NULL)
  {
    sendWarning(msg, "no LOBST resource to do that - try unload and reload plug-in");
    replySend = true;
  }
  else
  { // resource is available, so make a reply
    if (gotVerbose)
    {
      lobst->verbose = gotVerboseValue;
      sendInfo("done");
      replySend = true;
    }
    if (gotOpenLog)
    {
      lobst->openLog(gotOpenLogValue);
      sendInfo("done");
      replySend = true;
    }
    if (gotMake)
    {
      lobst->lock();
      if (gotAny)
        dataBuff.setSerial(0);
      scan = getScan(device, &scanner, (ULaserData*)extra, false, fake);
      lobst->makeObst(scan, scanner);
      if (not gotSilent)
      { // send obstacles as XML tags
        n = 0;
        p1 = reply;
        snprintf(reply, MRL, "<lobst name=\"lobst\" tod=\"%lu.%06lu\">\n",
                scan->getScanTime().getSec(), scan->getScanTime().getMicrosec());
        n += strlen(p1);
        p1 = &reply[n];
        lobst->codeLines(p1, MRL - n);
        n += strlen(p1);
        p1 = &reply[n];
        snprintf(p1, MRL - n, "</lobst>\n");
        sendMsg(reply);
      }
      if (msg->client >= 0 or not gotSilent)
        replySend = sendInfo("done");
      else
        replySend = true;
      lobst->unlock();
    }
    if (gotObst)
    { // transfer obstacles to obst resource
      if (lobst->sendAsObstacles())
      {
        snprintf(reply, MRL, "Send %d lines and %d obstacles to 'obst'", lobst->getLineCnt(), lobst->getObstGrpCnt());
        if (msg->client >= 0 or not gotSilent)
          sendInfo(reply);
      }
      else
        sendWarning("no obstacles or lines saved");
      replySend = true;
    }
    if (gotList)
    {
      lobst->codeLines(reply, MRL);
      sendHelpStart("lobst list");
      sendText(reply);
      replySend = sendHelpDone(msg);
    }
    if (msg->tag.getAttBool("test", NULL))
    {
      testMethodCall();
      sendInfo("done test call - see console printout");
    }
  }
  if (not replySend)
    sendInfo(msg, "no file action performed (no command option?)");
  // return true if the function is handled with a positive result
  return true;
}

////////////////////////////////////////////////////////////////////

void UFuncLobst::testMethodCall()
{
      const int MRL = 20;
      UDataBase * res[MRL]; // result array for found lines
      U2Dseg * seg; // 2D line segment
      U2Dpos p2;
      int n;
      UPoseTime pt;
      //
      // first variable in result must exist, and must be a UPoseTime type
      res[0] = &pt;
      // the rest can be set to nothing (or to *U2Dseg), so keep array - then next call will be faster
      for (n = 1; n < MRL; n++)
        // when NULL, then they will be filled as needed
        res[n] = NULL;
      // the last parameter (n) in the callGlobalV must be set to the size of the pointer array
      n = MRL;
      // get lines from lobst plugin
      bool ok = callGlobalV("lobst.getRansacLines", "", NULL, res, &n);
      if (ok)
      { // the call was successful - but is any lines found?
        printf("Got %d lines\n", n - 1);
        if (n > 0)
        { // first element is robot pose (and scantime)
          UPoseTime * pt = (UPoseTime*) res[0];
          printf("Robot pose at time: %.2fx, %.2fy, %.4fh, time:%.3f\n", pt->x, pt->y, pt->h, pt->t.getDecSec());
        }
        for (int i = 1; i < n; i++)
        { // get the lines - if OK then they are of type U2Dseg
          seg = (U2Dseg*) res[i];
          // get other end of line segment
          p2 = seg->getOtherEnd();
          printf("Line %d at %.2fx, %.2fy, %.4fh, %.2fm to %.2fx, %.2fy\n",
                 i, seg->x, seg->y, seg->getHeading(), seg->length, p2.x, p2.y);
        }
      }
      else
        printf("call failed (should only fail if lobst plug-in is not loaded)\n");
      //
      // remember to delete lines after final use
      for (int i = 1; i < MRL; i++)
        if (res[i] != NULL)
          delete res[i];
        else
          break;
}

///////////////////////////////////////////////////////

ULaserData * UFuncLobst::getScan(const char * device,
                                ULaserDevice ** las,
                                ULaserData * pushData,
                               bool getOnly,
                               int fake)
{
  ULaserPool * lasPool;
  ULaserData * scan = &dataBuff;
  //
  // get pouinter to laser pool (but do not create if not available - should be)
  lasPool = (ULaserPool *)getStaticResource("lasPool", false);
  if (lasPool != NULL)
  { // laser pool is available
    if (not getOnly)
      // get (or use) a new scan
      scan = lasPool->getScan(device, las, &dataBuff, pushData, fake);
    else
      // get laser scanner device only
      lasPool->getScan(device, las, NULL, NULL, 0);
  }
  return scan;
}

////////////////////////////////////////////////////////////////

