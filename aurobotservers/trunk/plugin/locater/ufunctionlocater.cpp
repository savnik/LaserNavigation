/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 * ----------------------------------------------------------------------- *
 *   Plugin for aurobotservers version 2.206.                              *
 *   Does laserbased localisation for a robot running in an orchard        *
 *   environment.                                                          *
 *   Edited by Peter Tjell (s032041) & Søren Hansen (s021751)              *
 * ----------------------------------------------------------------------- *
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
/**
@author Christian Andersen
@editor Peter Tjell / Søren Hansen
*/

#include "ufunctionlocater.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** @todo 'UFunctionExUse' with your classname, as used in the headerfile */
  return new UFunctionLocater();
}

#endif

///////////////////////////////////////////////////

UFunctionLocater::~UFunctionLocater()
{
  // possibly remove allocated variables here - if needed

  printf("UFunctionLocater destructor.\n");
  if(locater != NULL)
    delete locater;
}

///////////////////////////////////////////////////

void UFunctionLocater::createResources()
{ // Creates new resource
  locater = new UResLocater();
  addResource(locater, this);
}

///////////////////////////////////////////////////

bool UFunctionLocater::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  UDataBase * scan = (UDataBase *) extra;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("locater") || msg->tag.isTagA("rf"))
  {
    if (scan != NULL)
      if (not scan->isA("laserdata"))
        scan = NULL;
    result = handleLocater(msg, (ULaserData *) scan);
  }
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFunctionLocater::handleLocater(UServerInMsg * msg, ULaserData * pushData)
{ // send a short reply back to the client requested the 'bark'
  const int MRL = 200;
  char reply[MRL];
  bool ask4help;
  const int MVL = 50;
  char val[MVL];
  char attName[MAX_SML_NAME_LENGTH];

  bool gotOpenlog = false;
  bool gotCloselog = false;
  bool gotlocate = false;
  bool gotutmstate = false;
  bool gotsetposelast = false;
  bool gotdummy = false;
  bool gotUtmToMap = false;
  bool reaction = false;
  bool silent;

  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (not ask4help)
  { // get all other parameters
    if (msg->tag.getAttValue("openlog", val, MVL))
      gotOpenlog = true;
    if (msg->tag.getAttValue("closelog", val, MVL))
      gotCloselog = true;
    if(msg->tag.getAttValue("locate", val, MVL))
      gotlocate = true;
    if(msg->tag.getAttValue("update_allow", val, MVL))
    {
      locater->update_allow = str2bool2(val, true);
      reaction = true;
    }
    if(msg->tag.getAttValue("E_L", val, MVL))
    {
      locater->E_L = strtod(val, NULL);
      reaction = true;
    }
    if(msg->tag.getAttValue("E_A", val, MVL))
    {
      locater->E_A = strtod(val, NULL);
      reaction = true;
    }
    if(msg->tag.getAttValue("E_SA", val, MVL))
    {
      locater->E_SA = strtod(val, NULL);
      reaction = true;
    }
    if(msg->tag.getAttValue("var_d", val, MVL))
    {
      locater->var_d = strtod(val, NULL);
      reaction = true;
    }
    if(msg->tag.getAttValue("var_phi", val, MVL))
    {
      locater->var_phi = strtod(val, NULL);
      reaction = true;
    }
    if(msg->tag.getAttValue("axle_dist", val, MVL))
    {
      locater->axle_dist = strtod(val, NULL);
      reaction = true;
    }
    if(msg->tag.getAttValue("setstate", val, MVL))
    {
      while(msg->tag.getNextAttribute(attName, val, MVL))
      {
        snprintf(reply, MRL, "attName = %s, attValue = %s, VBL = %d", attName, val, MVL);
        sendInfo(msg, reply);

        if((strcasecmp(attName, "x") == 0) || (strcasecmp(attName, "y") == 0) || (strcasecmp(attName, "t") == 0))
          locater->set_state(attName, strtod(val, NULL));
        else
          sendWarning(msg, "Error using setstate");
        reaction = true;
      }
    }
    if(msg->tag.getAttValue("setmatch", val, MVL))
    {
      while(msg->tag.getNextAttribute(attName, val, MVL))
      {
        snprintf(reply, MRL, "attName = %s, attValue = %s, VBL = %d", attName, val, MVL);
        sendInfo(msg, reply);

        if(strcasecmp(attName, "max_ang") == 0)
          locater->max_angle_deviation = strtod(val, NULL);
        else if(strcasecmp(attName, "min_points") == 0)
          locater->min_points = strtol(val, NULL, 0);
        else
          sendWarning(msg, "Error using setmatch");
        reaction = true;
      }
    }
    if(msg->tag.getAttValue("setutmstate", val, MVL))
    {
      gotutmstate = true;
    }
    gotdummy = msg->tag.getAttValue("dummy", NULL, 0);
    gotUtmToMap = msg->tag.getAttValue("utmToMap", NULL, 0);
    gotsetposelast = msg->tag.getAttValue("setposelast", NULL, 0);
    silent = msg->tag.getAttBool("silent", NULL, true);
  }
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendMsg(msg,        "<help subject=\"Locater\" by Soren og Peter :)>\n");
    sendText(msg,       "--- available Locater options\n");
    sendText(msg,       "help            This message\n");
    sendText(msg,       "openlog         Opens the logfiles\n");
    sendText(msg,       "closelog        Save data and close the logfiles\n");
    sendText(msg,       "locate          Estimates the robots position using data from map\n");
    sendText(msg,       "setstate x=[val] y=[val] t=[val]\n");
    sendText(msg,       "                Sets the robots pose\n");
    snprintf(reply, MRL,"utmToMap        Sets mapPose to UTM pose if keepMapPose=false (is %s)\n", bool2str(locater->keepMapPose()));
    sendText(msg, reply);
    sendText(msg,       "setutmstate     Sets locater state (and mapPose if keepMapPose) to UTM pose\n");
    sendText(msg,       "setposelast     Initializes the odometri pose used in prediction\n");
    sendText(msg,       "setmatch max_ang=[val] min_points=[val]\n");
    sendText(msg,       "                Sets the parameters for mathing scandata with map\n");
    snprintf(reply, MRL, "  -- setmatch current: max_ang = %.3f, min_points = %d\n", locater->max_angle_deviation, locater->min_points);
    sendText(msg, reply);
    snprintf(reply, MRL, "update_allow=[true,false]  (Current: %s)\n", bool2str(locater->update_allow));
    sendText(msg, reply);
    sendText(msg,       "                Governs whether locate updates with laser measurements\n");
    snprintf(reply, MRL, "E_L = [val]        (Current: %.3f)\n", locater->E_L);
    sendText(msg, reply);
    sendText(msg,       "                Sets length standard deviation for 1 meters travel\n");
    snprintf(reply, MRL, "E_A = [val]        (Current: %.3f)\n", locater->E_A);
    sendText(msg, reply);
    sendText(msg,       "                Sets angular standard deviation for 1 meters travel\n");
    snprintf(reply, MRL, "E_SA = [val]       (Current: %.3f)\n", locater->E_SA);
    sendText(msg, reply);
    sendText(msg,       "                Sets steering angle standard deviation\n");
    snprintf(reply, MRL, "var_d = [val]      (Current: %.3f)\n", locater->var_d);
    sendText(msg, reply);
    sendText(msg,       "                Sets the laserscanners range variance\n");
    snprintf(reply, MRL, "var_phi = [val]    (Current: %.3f)\n", locater->var_phi);
    sendText(msg, reply);
    sendText(msg,       "                Sets the laserscanners angular variance\n");
    snprintf(reply, MRL, "axle_dist = [val]  (Current: %.3f)\n", locater->axle_dist);
    sendText(msg, reply);
    sendText(msg,       "                Sets used distance between axles\n");
    sendText(msg,       "silent          Print less to console and do not reply if no error\n");
    sendMsg(msg,        "</help>\n");
    sendInfo(msg, "done");

    reaction = true;
  }
  else if (locater == NULL)
    sendWarning(msg, "no varPool resource to access resource functions - try 'module list' for help");
  else
  {
    if (gotOpenlog)
    {
      locater->openlog();
      reaction = true;
    }
    if (gotCloselog)
    {
      locater->closelog();
      reaction = true;
    }
    if(gotlocate)
    {
      locater->locate(pushData, silent);
      reaction = true;
    }
    if(gotutmstate)
    {
      locater->set_utmstate();
      reaction = true;
    }
    if(gotsetposelast)
    {
      locater->set_poselast();
      reaction = true;
    }
    if(gotUtmToMap)
    {
      locater->setUtmPoseToMapPose();
      reaction = true;
    }
    if(gotdummy)
    {
      sendInfo(msg, "Dummy called (used for testing)");
      locater->varprint();
      reaction = true;
    }
  }

  if(reaction == false)
    sendWarning(msg, "Command not recognised");

  return true;
}

////////////////////////////////////////////////////////////

const char * UFunctionLocater::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s Locater functions\n", preString);
  return buff;
}

