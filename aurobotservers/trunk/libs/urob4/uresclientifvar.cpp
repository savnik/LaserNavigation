/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@oersted.dtu.dk
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "uresposehist.h"
#include "uvarcalc.h"
#include "uresclientifvar.h"
#include "usmltag.h"


// UResIfVar::UResIfVar()
// {
//   setResID( getResID());
//   resVersion = getResVersion();
//   verboseMessages = true;
//   poseHist = NULL;
//   poseUtm = NULL;
//   poseMap = NULL;
//   createVarSpace(5, 0, 0, "Variable related to a server interface client");
//   createBaseVar();
//   strncpy(tagList, "var", MAX_TAG_LIST_SIZE);
// }


UResIfVar::~UResIfVar()
{
}

///////////////////////////////////////////////////

const char * UResIfVar::name()
{
  return "to var-pool and pose history";
}

///////////////////////////////////////////////////

const char * UResIfVar::commandList()
{
  return tagList;
}

/////////////////////////////////////////////////////

void UResIfVar::createResources()
{
  createBaseVar();
}

///////////////////////////////////////////////////

void UResIfVar::createBaseVar()
{
  UVarPool * vp;
  // get top level 'directory'
  vp = getVarPool()->getRootVarPool();
  // get or create a subdirectory called var
  vp = vp->getStructDeep("var.", true, NULL, 0);
  // there may be more var clients, so variable mey be created already
  varCallDispOnNewPose = vp->getLocalVariable("callDispOnNewPose", NULL);
  if (varCallDispOnNewPose == NULL)
    varCallDispOnNewPose = vp->addVar("callDispOnNewPose", 0.0, "d", "(rw) if true, then disp.newdata(sd) is called on new odo, map or utm pose update.");
}

///////////////////////////////////////////////////

bool UResIfVar::setResource(UResBase * resource, bool remove)
{
  bool result = true;

  if (resource->isA(UResPoseHist::getOdoPoseID()))
  { // ressource may change
    if (remove)
      poseHist = NULL;
    else if (poseHist != resource)
      poseHist = (UResPoseHist *)resource;
    else
      result = false;
  }
  if (resource->isA(UResPoseHist::getUtmPoseID()))
  { // ressource may change
    if (remove)
      poseUtm = NULL;
    else if (poseUtm != resource)
      poseUtm = (UResPoseHist *)resource;
    else
      result = false;
  }
  if (resource->isA(UResPoseHist::getMapPoseID()))
  { // ressource may change
    if (remove)
      poseMap = NULL;
    else if (poseMap != resource)
      poseMap = (UResPoseHist *)resource;
    else
      result = false;
  }
  result |= UResVarPool::setResource(resource, remove);
  return result;
}

/////////////////////////////////////////////////////////

// bool UResIfVar::gotAllResources(char * missingThese, int missingTheseCnt)
// { // just needs a pointer to core for event handling
//   bool result = true;
//   int n = 0;
//   char * p1 = missingThese;
//   //
//   if (poseHist == NULL)
//   {
//     if (p1 != NULL)
//     {
//       strncpy(p1, UResPoseHist::getOdoPoseID(), missingTheseCnt);
//       n = strlen(p1);
//       p1 = &missingThese[n];
//       result = false;
//     }
//   }
//   result &= UResBase::gotAllResources(p1, missingTheseCnt - n);
//   return result;
// }

//////////////////////////////////////////////////////

void UResIfVar::addTags(const char * tags)
{
  int n;
  char * p1;
  //
  if (not inThisStringList(tags, tagList))
  {
    n = strlen(tagList);
    p1 = &tagList[n];
    snprintf(p1, MAX_TAG_LIST_SIZE - n, " %s", tags);
  }
}

///////////////////////////////////////////////////

void UResIfVar::handleNewData(USmlTag * tag)
{
  if (tag->isTagA("var"))
    handleVar(tag);
  else if (tag->isTagA(UResPoseHist::getOdoPoseID()) or
           tag->isTagA(UResPoseHist::getUtmPoseID()) or
           tag->isTagA(UResPoseHist::getMapPoseID()))
    handlePoseHist(tag);
  else
  { // any other handled message - should not be
    handleOther(tag);
    printf("UResIfVar::handleNewData: ** should never get here - not var and not pose\n");
  }
  msgHandled++;
}

///////////////////////////////////////////////////

void UResIfVar::handleVar(USmlTag * tag)
{ // handle pathGet data
  const int MVL = 2000;
  char val[MVL];
  const int MNL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  char nam[MNL] = "";
  char where[MNL];
  UVarPool * vp;
  const int MTL = 10;
  char type[MTL];
  bool isOK;
  //
  tag->reset();
  vp = getVarPool();
  if (vp != NULL and not tag->isAnEndTag())
  { // a var formatted type of format
    //<var name="aaa.bbb.c"   type="d"   valid="true" value="22.33"
    //<var name="roadTopDist" typ="dq"   valid="true" value="0 0.4" qual="0.4"/>
    //<var name="roadCenter"  typ="3d"   valid="true" value="0.1 0.2 0.3" x="0.1" y="0.2" z="0.3"/>
    //<var name="robot.pose"  typ="pose" valid="true" value="1 2 3" x="1" y="2" th="3"/>
    //<var name="gmks1.gmk"   typ="rot"  valid="true" value="4 5 6" Omega="4" Phi="5" Kappa="6"/>
    vp = vp->getRootVarPool();
    isOK = tag->getAttValue("typ", type, MTL);
    if (isOK)
      isOK = tag->getAttValue("value", val, MVL);
    if (isOK)
    {
      if (tag->getAttValue("dest", where, MNL))
      { // change var-pool to this destination
        // add a dot to mark that this is a structure all the way
        vp = vp->getStructDeep(where, true, nam, MNL);
        isOK = vp != NULL;
        // debug
//         printf("UResIfVar::handleVar variable destination is %s for %s=%s\n",
//                  where, nam, val);
        // debug end
      }
      else
        // put the variable into default var structure
        vp = vp->getStructDeep("var.", true, NULL, 0);
    }
    if (isOK and strlen(nam) == 0)
      // no replacement name, so use source name
      isOK = tag->getAttValue("name", nam, MNL);
    if (isOK)
    {
      UTime updT;
      if (tag->getAttTime("tod", &updT))
        isOK = vp->setLocalVar(nam, val, true, type, &updT);
      else
        isOK = vp->setLocalVar(nam, val, true, type);
      if (isOK and tag->getAttValue("desc", val, MVL))
      {
        UVariable * var = NULL;
        var = vp->getLocalVariable(nam, NULL);
        if (var != NULL)
          var->setDescription(val, true);
      }
    }
    if (not isOK)
    {
      isOK = tag->getAttValue("info", val, MVL);
      if (not isOK)
      { // is it a warning?
        isOK = tag->getAttValue("warning", val, MVL);
        if (isOK)
          tag->print(""); // print warning
      }
      if (not isOK)
      { // not info, not warning, so an error: show part of received var tag
        strncpy(val, tag->getTagStart(), 50);
        val[50] = '\0';
        printf("Failed to set variable from '%s ...'\n", val);
      }
    }
  }
}

//////////////////////////////////////////////////////

void UResIfVar::handleOther(USmlTag * tag)
{ // handle pathGet data
  const int MSL = 100;
  char att[MAX_SML_NAME_LENGTH];
  char val[MSL];
  int n;
  double v;
  const int MNL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  char nam[MNL];
  bool failed = false;
  UVarPool * vp;
  USmlTag nTag;
  bool more;
  const int MRL = 10000;
  char reply[MRL + 1];
  /*
  <camget width="640" height="320"/>
  */
  // unpack polygon - test for result
  tag->reset();
  vp = getVarPool();
  if (vp != NULL and not tag->isAnEndTag())
  { // some other simple set of values that can be handled as single (double) vars
    while (tag->getNextAttribute(att, val, MSL))
    {
      if ((strcasecmp(att, "warning") == 0) or
            (strcasecmp(att, "error") == 0))
        failed = true;
      if (not failed)
      {
        n = 1;
        if (strcasecmp(val, "false") == 0)
          v = 0.0;
        else if (strcasecmp(val, "true") == 0)
          v = 1.0;
        else
          n = sscanf(val, "%lg", &v);
        if ((n == 1) or
              (strcasecmp(val, "false") == 0) or
              (strcasecmp(val, "true") == 0))
        { // there is value, that can evaluate to a double
          snprintf(nam, MNL, "v_%s_%s", tag->getTagName(), att);
          vp->setLocalVar(nam, v, true, UVariable::d);
        }
      }
    }
    if (failed)
      v = 0.0;
    else
      v = 1.0;
    snprintf(nam, MNL, "v_%s", tag->getTagName());
    vp->setLocalVar(nam, v, true, UVariable::d);
    if (tag->cnnVerbose())
    { // echo message to console
      tag->print("");
      if (tag->isAStartTag())
      { // we need to display the rest too
        while (true)
        {
          n = MRL;
          more = tag->getNextTag(&nTag, 100, tag, reply, &n);
          reply[n] = '\0';
          printf("%s", reply);
          if (n < MRL)
            nTag.print("");
          if (not more)
            break;
        }
      }
    }
    else if (tag->isAStartTag())
      tag->skipToEndTag(200);
  }
}

//////////////////////////////////////////////////////

void UResIfVar::handlePoseHist(USmlTag * tag)
{ // handle pathGet data
  const int MVL = 100;
  char val[MVL];
  const int MNL = MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES;
  char name[MNL] = "";
  UVarPool * vp;
  USmlTag nTag;
  UPoseTVQ pose;
  UResPoseHist * ph;
  double v;
  char * tagName;
  /*
  <camget width="640" height="320"/>
  */
  // unpack polygon - test for result
  tag->reset();
  vp = getVarPool();
  if (tag->isTagA(UResPoseHist::getOdoPoseID()))
    ph = poseHist;
  else if (tag->isTagA(UResPoseHist::getUtmPoseID()))
    ph = poseUtm;
  else if (tag->isTagA(UResPoseHist::getMapPoseID()))
    ph = poseMap;
  else
    ph = NULL;
  //
  if (vp != NULL and not tag->isAnEndTag() and ph != NULL)
  { // some other simple set of values that can be handled as single (double) vars
    if (tag->isAStartTag())
    {
      while (tag->getNextTag(&nTag, 200))
      { //<posehist name="poses">
        //<pose name="newest" x="203.256" y="62.632" h="-0.148" tod="1149865580.490413"/>
        //</posehist>
        if (nTag.isTagA("pose"))
        { //
          nTag.getAttValue("name", name, MNL);
          if (strcmp(name, "newest") == 0)
          { // get also velocity
            if (nTag.getAttValue("vel", val, MVL))
              pose.vel = strtod(val, NULL);
            if (nTag.getAttValue("q", val, MVL))
              pose.q = strtod(val, NULL);
            // get other pose attributes
            if (nTag.getPoseT(&pose))
            { // new pose is available - pose to poseHist
              ph->addIfNeeded(pose, -2);
            }
          }
        }
        else if (nTag.isAStartTag())
        // is an unwanted tag goup - skip
          tag->skipToEndTag(200);
        // remember to test for endflag too
        else if (nTag.isTagAnEnd(tag->getTagName()))
          break;
      }
    }
    if (varCallDispOnNewPose->getBool())
    {
      tagName = (char *) tag->getTagName();
      v = ph->getNewestSource();
      callGlobal("disp.newData", "sd", &tagName, &v, &v, NULL, NULL);
    }
  }
  else if (tag->isAStartTag())
      // is an unwanted tag goup - skip
    tag->skipToEndTag(200);
}

/////////////////////////////////////////////

char * UResIfVar::snprint(const char * preString, char * buff, const int buffCnt)
{
  UVarPool * vp;
  char * p1 = buff;
  int n = 0;
  //
  vp = getVarPool();
  if (vp == NULL)
    snprintf(p1, buffCnt, "%s Var pool missing!!! - hard error\n", preString);
  else
  {
    snprintf(p1, buffCnt - n, "%s Handles interface tags: %s\n",
              preString, commandList());
    n = strlen(p1);
    p1 = &buff[n];
    snprintf(p1, buffCnt - n,   "%s Interface handler has (%d/%d var %d/%d structs %d/%d funcs)\n",
             preString,
           vp->getVarsCnt(), vp->getVarMax(),
           vp->getStructCnt(), vp->getStructMax(),
           vp->getMethodCnt(), vp->getMethodMax());
  }
  return buff;
}

