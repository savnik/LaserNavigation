/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
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
#include "usmltag.h"

#include <ugen4/ugmk.h>
#include <umap4/uprobpoly.h>
#include <umap4/umanarc.h>

#include "ucmdexe.h"


const char * USmlTag::codePosition(UPosition * pos, char * buff,
                                   const unsigned int buffCnt,
                                   const char * name, const char * extra)
{
  return pos->codeXml(name, buff, buffCnt, extra);
}

///////////////////////////////////////////////////////////

bool USmlTag::getPosition(UPosition * pos)
{
  const int MVL = 30; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  bool gotX = false;
  bool gotY = false;
  bool gotZ = false;
  double v;
  //
  // <pos3d name="start" x=10.000000 y=200.000000 z=0.050000/>
  // <pos3d name="vector" x=-8.07299604668089e-09 y=1.00000000811000e+00 z=-3.07800055157770e-11/>
  //
  reset();
  if (pos->isA("3d"))
  {
    v = 99;
    pos->y = v + 1;
    v = pos->y;
  }
  while (getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "x") == 0)
    {
      v = strtod(val, NULL);
      pos->x = v;
      gotX = true;
    }
    else if (strcasecmp(att, "y") == 0)
    {
      v = strtod(val, NULL);
      pos->y = v;
      gotY = true;
    }
    else if (strcasecmp(att, "z") == 0)
    {
      v = strtod(val, NULL);
      pos->z = v;
      gotZ = true;
    }
  }
  return (gotX and gotY and gotZ);
}

///////////////////////////////////////////////////////////

bool USmlTag::getPoseV(UPoseV * pose)
{
  const int MVL = 30; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  bool gotX = false;
  bool gotY = false;
  bool gotH = false;
  bool gotV = false;
  //
  // <posev name="start" x="0.034" y="-0.001" h="-0.062" v="0.115"/>
  // <posev name="end" x="3.873" y="-4.784" h="-0.062" v="0.800"/>
  //
  reset();
  while (getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "x") == 0)
    {
      pose->x = strtod(val, NULL);
      gotX = true;
    }
    else if (strcasecmp(att, "y") == 0)
    {
      pose->y = strtod(val, NULL);
      gotY = true;
    }
    else if (strcasecmp(att, "h") == 0)
    {
      pose->h = strtod(val, NULL);
      gotH = true;
    }
    else if (strcasecmp(att, "v") == 0)
    {
      pose->setVel(strtod(val, NULL));
      gotV = true;
    }
  }
  return (gotX and gotY and gotH and gotV);
}

///////////////////////////////////////////////////////////

bool USmlTag::getPose(UPose * pose)
{
  const int MVL = 30; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  bool gotX = false;
  bool gotY = false;
  bool gotH = false;
  //
  // <pose name="start" x="0.034" y="-0.001" h="-0.062""/>
  // <pose name="end" x="3.873" y="-4.784" h="-0.062"/>
  //
  reset();
  while (getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "x") == 0)
    {
      pose->x = strtod(val, NULL);
      gotX = true;
    }
    else if (strcasecmp(att, "y") == 0)
    {
      pose->y = strtod(val, NULL);
      gotY = true;
    }
    else if ((strcasecmp(att, "h") == 0) or
              (strcasecmp(att, "th") == 0))
    {
      pose->h = strtod(val, NULL);
      gotH = true;
    }
  }
  return (gotX and gotY and gotH);
}

///////////////////////////////////////////////////////

bool USmlTag::getPoseT(UPoseTime * pose)
{
  const int MVL = 30; //
  char val[MVL];
  bool gotTod;
  bool gotXYH;
  unsigned long s, m;
  //
  // <pt tod="1234567890.123456" x="0.034" y="-0.001" th="-0.062""/>
  //
  //reset();
  gotTod = getAttValue("tod", val, MVL);
  if (gotTod)
    sscanf(val,"%lu.%lu", &s, &m);
  else
  {
    s = 0;
    m = 0;
  }
  pose->t.setTimeU(s,m);
  gotXYH = getPose(pose);
  return (gotTod and gotXYH);
}

///////////////////////////////////////////////////////

bool USmlTag::getPoseTVQ(UPoseTVQ * pose)
{
  const int MVL = 30; //
  char val[MVL];
  //
  // <pt tod="1234567890.123456" x="0.034" y="-0.001" th="-0.062" vel="1.22" q="2.0""/>
  //
  //reset();
  if (getAttValue("vel", val, MVL))
    pose->vel = strtod(val, NULL);
  if (getAttValue("q", val, MVL))
    pose->q = strtod(val, NULL);
  return getPoseT(pose);
}

///////////////////////////////////////////////////////////

const char * USmlTag::codeRotation(URotation * rot, char * s, const unsigned int bufferLength, const char * name)
{
  const char * result = NULL;
  //
  snprintf(s, bufferLength, "<rot3d name=\"%s\" Omega=\"%.14g\" Phi=\"%.14g\" Kappa=\"%.14g\"/>\n",
           name, rot->Omega, rot->Phi, rot->Kappa);
  if (strlen(s) <= bufferLength)
    result = s;
  return result;
}

///////////////////////////////////////////////////////////

bool USmlTag::getRotation(URotation * rot)
{
  const int MVL = 30; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  bool gotO = false;
  bool gotP = false;
  bool gotK = false;
  //
  // <pos3d name="start" x=10.000000 y=200.000000 z=0.050000/>
  // <pos3d name="vector" x=-8.07299604668089e-09 y=1.00000000811000e+00 z=-3.07800055157770e-11/>
  //
  reset();
  while (getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "Omega") == 0)
    {
      rot->Omega = strtod(val, NULL);
      gotO = true;
    }
    else if (strcasecmp(att, "Phi") == 0)
    {
      rot->Phi = strtod(val, NULL);
      gotP = true;
    }
    else if (strcasecmp(att, "Kappa") == 0)
    {
      rot->Kappa = strtod(val, NULL);
      gotK = true;
    }
  }
  return (gotO and gotP and gotK);
}

///////////////////////////////////////////////////////////

bool USmlTag::codeLineSegment(ULineSegment * seg, char * s, const unsigned int bufferLength, const char * name, const char * extra)
{
  unsigned int n;
  char * p1 = s;
  //
  if (extra == NULL)
    snprintf(p1, bufferLength, "<lineSeg name=\"%s\" length=\"%.14g\">\n", name, seg->length);
  else
    snprintf(p1, bufferLength, "<lineSeg name=\"%s\" length=\"%.14g\" %s>\n",
             name, seg->length, extra);
  n = strlen(p1);
  p1 = &s[n];
  if (n < bufferLength)
  {
    codePosition(&seg->pos, p1, bufferLength - n, "start");
    n += strlen(p1);
    p1 = &s[n];
  }
  if (n < bufferLength)
  {
    codePosition(&seg->vec, p1, bufferLength - n, "vector");
    n += strlen(p1);
    p1 = &s[n];
  }
  if (n < bufferLength)
  {
    snprintf(p1, bufferLength - n, "</lineSeg>\n");
    n += strlen(p1);
  }
  //
  return (n <= bufferLength);
}

//////////////////////////////////////////////////////////////

bool USmlTag::getLineSegment(ULineSegment * seg)
{
  const int MVL = 30; //
//  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  bool gotLen = false;
  bool gotPos = false;
  bool gotVec = false;
  USmlTag nTag;
  bool result;
  /*
  <lineSeg name="passableInterval" length=1.43088471889496e+00>
  <pos3d name="start" x=1.71957826614380e+00 y=-3.28020524978638e+00 z=2.42707570923525e-01/>
  <pos3d name="vector" x=3.05969167544701e-01 y=9.51039274669251e-01 z=-4.36722302177049e-02/>
  </lineSeg>
  */
  reset();
  if (getAttValue("length", val, MVL))
  {
    seg->length = strtod(val, NULL);
    gotLen = true;
  }
  result = (isAStartTag());
  while (result)
  { // get other parameters
    result = cnn->getNextTag(&nTag, 400);
    if (cnn->isVerbose() and result)
      nTag.print("");
    if (not result)
      break;
    else if (nTag.isTagA("pos3d"))
    { // get waypoint attributes
      // <waypoint n=7 x=3.098 y=0.782/>
      if (nTag.getAttValue("name", val, MVL))
      { // decode where to put info
        if (strcasecmp(val, "start") == 0)
          gotPos = nTag.getPosition(&seg->pos);
        else if (strcasecmp(val, "vector") == 0)
          gotVec = nTag.getPosition(&seg->vec);
      }
    }
    else if (nTag.isAStartTag())
        // is an unwanted tag goup - skip
      cnn->skipToEndTag(&nTag, 200);
      // remember to test for endflag too
    else if (nTag.isTagAnEnd(getTagName()))
      // no more data -- should exit here
      break;
    // else just skip
  }
  return (gotLen and gotPos and gotVec);
}

//////////////////////////////////////////////////////////////

bool USmlTag::getProbPoly(UProbPoly * poly)
{
  const int MVL = 30; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  USmlTag nTag;
  bool result;
  UPose zeroPose(0.0, 0.0, 0.0);
  UTime time;
  UPosition pos;
  bool isObst;
  int count = -1;
  //bool andBool = false;
  UPose pose;
  double seedSD = 0.0;
  /*
  <UProbPoly count="5" obstFlags="true">
  <time name="imgTime" sec="1128975800" usec="191676"/>
  <pose name="org" x="0.2" y="-22.5" h="0.03"/>
  <pos2db x="3.145" y="0.0145684" obst="false"/>
  <pos2db x="3.60671" y="0.0149595" obst="true"/>
  <pos2db x="3.14536" y="0.0511105" obst="false"/>
  <pos2db x="3.24907" y="0.664903" obst="false"/>
  <pos2db x="3.15209" y="0.736918" obst="false"/>
  </UProbPoly>
  */
  reset();
  while (getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "count") == 0)
      count = strtol(val, NULL, 10); // number of positions in polygon
    else if (strcasecmp(att, "obstFlags") == 0)
      ; //andBool = str2bool(val);
    // seedCromaSD
    else if (strcasecmp(att, "seedCromaSD") == 0)
      seedSD = strtod(val, NULL);
  }
  result = isAStartTag();
  if (result)
  { // is a (valid) polygon
    poly->clear();
    poly->setCromaSD(seedSD);
  }
  while (result)
  { // get other parameters
    result = cnn->getNextTag(&nTag, 400);
    if (cnn->isVerbose() and result)
      nTag.print("");
    if (not result)
      break;
    else if (nTag.isTagA("time"))
    {
      nTag.getTime(&time);
      poly->setPoly(zeroPose, time);
      poly->setValid(true);
    }
    else if (nTag.isTagA("pos2db"))
    { // get position and boolean flag
      nTag.getPosition(&pos);
      nTag.getAttValue("obst", val, MVL);
      isObst = str2bool(val);
      poly->addPos(pos, isObst);
    }
    else if (nTag.isTagA("pose"))
    {
      nTag.getAttValue("x", val, MVL);
      pose.x = strtod(val, NULL);
      nTag.getAttValue("y", val, MVL);
      pose.y = strtod(val, NULL);
      nTag.getAttValue("h", val, MVL);
      pose.h = strtod(val, NULL);
      poly->setPoseOrg(pose);
    }
    else if (nTag.isTagAnEnd(getTagName()))
      // no more data -- should exit here
      break;
    else if (nTag.isAStartTag())
          // extra grouped info
      cnn->skipToEndTag(&nTag, 200);
    // else just skip
  }
  if (poly->getPointsCnt() != count)
    printf("USmlTag::getProbPoly: Expected %d edges found %d\n", count, poly->getPointsCnt());
  return ((poly->getPointsCnt() == count) and poly->isValid());
}

/////////////////////////////////////////////////////////////////////

const char * USmlTag::codeTime(UTime time, char * s, const unsigned int bufferLength, const char * name)
{
  const char * result = NULL;
  unsigned int n;
  //
  strncpy(s, "<timeofday ", bufferLength);
  n = strlen(s);
  if (n < bufferLength and (name != NULL))
  {
    snprintf(&s[n], bufferLength - n, "name=\"%s\" ", name);
    n = strlen(s);
  }
  if (n < bufferLength)
  {
    snprintf(&s[n], bufferLength - n, "tod=\"%ld.%06ld\"/>\n",
              time.getSec(), time.getMicrosec());
    n = strlen(s);
  }
  if (n < bufferLength)
  {
    result = s;
  }
  return result;
}

/////////////////////////////////////////////////////////////////////////

bool USmlTag::getGmk(UGmk * gmk)
{
  const int MVL = 30; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  USmlTag gtag;
  UTime gmksTime;
  char codeStr[UGmk::MAX_CODE_LENGTH];
  const int MCSL = 32;
  char camStr[MCSL];
  //int camDev;
  bool gotTime = false;
  bool gotPos = false;
  bool gotRot = false;
  /*
  <gmkget gmkCnt="1">
  <gmk code="abc1000" cam="Left" device="1">
  <timeofday tod="1129466404.528579"/>
  <pos3d name="gmkPosition" x="0.66967" y="0.036835" z="0.113016"/>
  <rot3d name="gmkRotation" Omega="1.6143908062221" Phi="0.14146691871041" Kappa="0.087895071018558"/>
  </gmk>
  </gmkget>*/
  //
  reset();
  while (getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "code") == 0)
      strncpy(codeStr, val, UGmk::MAX_CODE_LENGTH);
    else if (strcasecmp(att, "cam") == 0)
      strncpy(camStr, val, MCSL);
    else if (strcasecmp(att, "device") == 0)
      ;// camDev = strtol(val, NULL, 10);
    else
      ; // just ignore
  }
  gmk->set(codeStr);
  if (isAStartTag())
  {
    while (cnn->getNextTag(&gtag, 200))
    {
      if (cnn->isVerbose())
        gtag.print("");
      if (gtag.isTagA("timeofday"))
      {
        gtag.getTime(&gmksTime);
        gmk->setTime(gmksTime);
        gotTime = true;
      }
      else if (gtag.isTagA("pos3d"))
        gotPos = gtag.getPosition(gmk->getPos());
      else if (gtag.isTagA("rot3d"))
        gotRot = gtag.getRotation(gmk->getRot());
      else if (gtag.isAStartTag())
        // is an unwanted tag goup - skip
        cnn->skipToEndTag(&gtag, 200);
      // remember to test for endflag too
      else if (gtag.isTagAnEnd(getTagName()))
        break;
    }
    gmk->setPosValid(gotPos and gotRot);
  }
  return (gotTime and gotPos and gotRot);
}

///////////////////////////////////////////////////////////////

char * USmlTag::codeManoeuvre(UManoeuvre * man, char * buf, int bufCnt, char * name)
{
  char * p1;
  int n;
  double r, a;
  //
  snprintf(buf, bufCnt, "<manoeuvre");
  n = strlen(buf);
  p1 = &buf[n];
  if (name != NULL)
    if (strlen(name) > 0)
  { // include a name
    snprintf(p1, bufCnt - n, " name=\"%s\"", name);
    n += strlen(p1);
    p1 = &buf[n];
  }
  switch (man->getManType())
  { // MAN_NONE, MAN_LINE, MAN_ARC, MAN_STOP
    case UManoeuvre::MAN_STOP:
      snprintf(p1, bufCnt - n, " typ=\"stop\"");
      break;
    case UManoeuvre::MAN_LINE:
      snprintf(p1, bufCnt - n, " typ=\"line\" dist=\"%.3g\"", man->getDistance());
      break;
    case UManoeuvre::MAN_ARC:
      r = ((UManArc *)man)->getTurnRadius();
      a = ((UManArc *)man)->getTurnAngle();
      snprintf(p1, bufCnt - n, " typ=\"arc\" rad=\"%.3g\" arc=\"%.4g\"", r, a);
      break;
    default:
      snprintf(p1, bufCnt - n, " typ=\"none\"");
      break;
  }
  n += strlen(p1);
  p1 = &buf[n];
  snprintf(p1, bufCnt - n, " acc=\"%.5g\" endVel=\"%.5g\"",
           man->getAcc(), man->getVel());
  n += strlen(p1);
  p1 = &buf[n];
  snprintf(p1, bufCnt - n, "/>\n");
  //
  return buf;
}

////////////////////////////////////////////////////////////////

bool USmlTag::codeManPPSeq(UManPPSeq * manseq, char * buf, int bufCnt, char * name)
{
  char * p1;
  int n;
  UManoeuvre * man;
  const char tagname[] = "manseq";
  int i;
  //
  snprintf(buf, bufCnt, "<%s", tagname);
  n = strlen(buf);
  p1 = &buf[n];
  if (name != NULL)
    if (strlen(name) > 0)
  { // include a name
    snprintf(p1, bufCnt - n, " name=\"%s\"", name);
    n += strlen(p1);
    p1 = &buf[n];
  }
  snprintf(p1, bufCnt - n, " cnt=\"%d\" endV=\"%.3f\" dt=\"%.3f\">\n",
           manseq->getSeqCnt(), manseq->getEndVel(), manseq->getManTime());
  n += strlen(p1);
  p1 = &buf[n];
  // start pose
  codePoseV(manseq->getStartPoseV(), p1, bufCnt - n, "start");
  n += strlen(p1);
  p1 = &buf[n];
  // end pose (with desired end velocity, not actual)
  codePoseV(manseq->getEndPoseV(), p1, bufCnt - n, "end");
  n += strlen(p1);
  p1 = &buf[n];
  // now all the somple manoeuvres
  for (i = 0; i < manseq->getSeqCnt(); i++)
  {
    man = manseq->getMan(i);
    if (man != NULL)
    {
      codeManoeuvre(man, p1, bufCnt - n, NULL);
      n += strlen(p1);
      p1 = &buf[n];
    }
  }
  // end tag
  snprintf(p1, bufCnt - n, "</%s>\n", tagname);
  n += strlen(p1);
  // debug
  i = strlen(buf);
  if (i != n)
    printf("Error in buffer length count\n");
  // debug end
  //
  return ((n + 1) < bufCnt);
}

//////////////////////////////////////////////////////////////////////

char * USmlTag::codePoseV(UPoseV pv, char * buf, int bufCnt, const char * name)
{
  char * p1;
  int n;
  //
  snprintf(buf, bufCnt, "<posev");
  n = strlen(buf);
  p1 = &buf[n];
  if (name != NULL)
    if (strlen(name) > 0)
  { // include a name
    snprintf(p1, bufCnt - n, " name=\"%s\"", name);
    n += strlen(p1);
    p1 = &buf[n];
  }
  //
  snprintf(p1, bufCnt - n, " x=\"%.3f\" y=\"%.3f\" h=\"%.3f\" v=\"%.3f\"/>\n",
           pv.x, pv.y, pv.h, pv.getVel());
  //
  return buf;
}

//////////////////////////////////////////////////////////////////////

char * USmlTag::codePose(UPose pose, char * buf, int bufCnt, const char * name, const char * extraAtt)
{
  char * p1;
  int n;
  //
  snprintf(buf, bufCnt, "<pose");
  n = strlen(buf);
  p1 = &buf[n];
  if (name != NULL)
    if (strlen(name) > 0)
  { // include a name
    snprintf(p1, bufCnt - n, " name=\"%s\"", name);
    n += strlen(p1);
    p1 = &buf[n];
  }
  //
  snprintf(p1, bufCnt - n, " x=\"%.3f\" y=\"%.3f\" h=\"%.3f\"",
           pose.x, pose.y, pose.h);
  n += strlen(p1);
  p1 = &buf[n];
  // add extra attributes
  if (extraAtt != NULL)
    snprintf(p1, bufCnt - n, " %s/>\n", extraAtt);
  else
    snprintf(p1, bufCnt - n, "/>\n");
  //
  return buf;
}

//////////////////////////////////////////////////////////////////////

char * USmlTag::codePoseTime1(UPoseTime pt, char * buf, int bufCnt, const char * name, const char * extraAtt)
{
  const int MEL = 150;
  char extra[MEL];
  UPose po;
  // make extra time attribute
  if (extraAtt != NULL)
    snprintf(extra, MEL, "tod=\"%ld.%06ld\" %s",  pt.t.getSec(), pt.t.getMicrosec(), extraAtt);
  else
    snprintf(extra, MEL, "tod=\"%ld.%06ld\"",  pt.t.getSec(), pt.t.getMicrosec());
  // return pose with extra attribute
  po = pt.getPose();
  return codePose(po, buf, bufCnt, name, extra);
}

//////////////////////////////////////////////////////////////////////

char * USmlTag::codePoseTime(UPoseTime pt, char * buf, int bufCnt, const char * name)
{
  char * p1;
  int n;
  const char * POSET = "poset";
  //
  snprintf(buf, bufCnt, "<%s", POSET);
  n = strlen(buf);
  p1 = &buf[n];
  if (name != NULL)
    if (strlen(name) > 0)
  { // include a name
    snprintf(p1, bufCnt - n, " name=\"%s\"", name);
    n += strlen(p1);
    p1 = &buf[n];
  }
  //
  snprintf(p1, bufCnt - n, ">\n");
  n += strlen(p1);
  p1 = &buf[n];
  // add pose
  codePose(pt.getPose(), p1, bufCnt - n, "");
  n += strlen(p1);
  p1 = &buf[n];
  // add time
  codeTime(pt.t, p1, bufCnt - n, NULL);
  n += strlen(p1);
  p1 = &buf[n];
  //
  snprintf(p1, bufCnt - n, "</%s>\n", POSET);
  //
  return buf;
}

////////////////////////////////////////////////////////////////////////

bool USmlTag::sendProbPoly(UProbPoly * poly, UServerInMsg * msg, const char * name, const char * extraAtt, UCmdExe * exe)
{
  bool result = (msg != NULL);
  const int MRL = 2000;
  char reply[MRL];
  //UCmdExe * exe;
  UPosition * pos;
  int i;
  //
  if (result)
    result = (exe != NULL);
  if (result)
  { // Send a reply
    snprintf(reply, MRL, "<%s name=\"%s\" type=\"UProbPoly\">",
             msg->tag.getTagName(), name);
    exe->sendMsg(msg, reply);
    // send UPrbPoly header
    if (extraAtt != NULL)
      snprintf(reply, MRL, "<UProbPoly count=\"%d\" obstFlags=\"true\" seedCromaSD=\"%g\" %s>\n",
               poly->getPointsCnt(), poly->getCromaSD(), extraAtt);
    else
      snprintf(reply, MRL, "<UProbPoly count=\"%d\" obstFlags=\"true\" seedCromaSD=\"%g\">\n",
               poly->getPointsCnt(), poly->getCromaSD());
    exe->sendMsg(msg, reply);
    // send time
    exe->sendMsg(msg, poly->getPoseTime().getAsSml("imgTime", reply, MRL));
    // send source pose
    poly->getPoseOrg().getAsSml("org", reply, MRL);
    exe->sendMsg(msg, reply);
    // send polygon values
    pos = poly->getPoints();
    for (i = 0; i < poly->getPointsCnt(); i++)
    { // send 2D with binary flag
      snprintf(reply, MRL, "<pos2db x=\"%g\" y=\"%g\" obst=\"%s\"/>\n",
               pos->x, pos->y, bool2str(poly->getIsObst()[i]));
      exe->sendMsg(msg, reply);
      pos++;
    }
    // debug
    // printf("UProbPoly send %d vertices to client %d\n", poly->getPolyCnt(), msg);
    // debug end
    // send UProbPoly end tag
    snprintf(reply, MRL, "</UProbPoly>");
    exe->sendMsg(msg, reply);
    // send end tag
    snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
    result = exe->sendMsg(msg, reply);
  }
  //
  return result;
}

////////////////////////////////////////////////////////////////

bool USmlTag::getManSeq(UManSeq * man)
{
  const int MVL = 30; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  USmlTag gtag;
  //int manCnt = 0;
  double endV = 0.0, dt = 0.0;
  UManPPSeq * mpp;
  UManoeuvre * mm;
  bool isOK;
  UPoseV pv;
  double acc = 0.0, r = 0.0, a = 0.0, vel = 0.0;
 /*
  <manseq cnt="2" endV="0.455" dt="24.215">
  <posev name="start" x="0.034" y="-0.001" h="-0.062" v="0.115"/>
  <posev name="end" x="3.873" y="-4.784" h="-0.062" v="0.800"/>
  <manoeuvre typ="arc" rad="2.073" arc="-1.6653" acc="0.0140562" endVel="  0.8"/>
  <manoeuvre typ="arc" rad="2.073" arc="1.6653" acc="0.0140562" endVel="  0.8"/>
  </manseq>
  */
  //
  reset();
  while (getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "cnt") == 0)
      ; //manCnt = strtol(val, NULL, 10);
    else if (strcasecmp(att, "endV") == 0)
      endV = strtod(val, NULL);
    else if (strcasecmp(att, "dt") == 0)
      dt = strtod(val, NULL);
    else
      ; // just ignore
  }
  mpp = man->addP2P();
  if (mpp != NULL)
  {
    mpp->setEndVel(endV);
    mpp->setManTime(dt);
  }
  if (isAStartTag())
  {
    while (cnn->getNextTag(&gtag, 200))
    {
      if (cnn->isVerbose())
        gtag.print("");
      if (gtag.isTagA("posev"))
      {
        isOK = gtag.getAttValue("name", val, MVL);
        if (strcasecmp(val, "start") == 0)
        {
          gtag.getPoseV(&pv);
          mpp->setStartPoseV(pv);
        }
        else if (strcasecmp(val, "end") == 0)
        {
          gtag.getPoseV(&pv);
          mpp->setEndPoseV(pv);
        }
      }
      else if (gtag.isTagA("manoeuvre"))
      {
        if (gtag.getAttValue("acc", val, MVL))
          acc = strtod(val, NULL);
        if (gtag.getAttValue("vel", val, MVL))
          vel = strtod(val, NULL);
        mm = NULL;
        isOK = gtag.getAttValue("typ", val, MVL);
        if (isOK and strcasecmp(val, "arc") == 0)
        {
          if (gtag.getAttValue("rad", val, MVL))
            r = strtod(val, NULL);
          if (gtag.getAttValue("arc", val, MVL))
            a = strtod(val, NULL);
          mm = man->getNewArc(r, a, acc, vel);
        }
        else if (isOK and strcasecmp(val, "line") == 0)
        {
          if (gtag.getAttValue("dist", val, MVL))
            r = strtod(val, NULL);
          mm = man->getNewLine(r, acc, vel);
        }
        else if (isOK and strcasecmp(val, "stop") == 0)
        {
          mm = man->getNewStop();
        }
        if (mm != NULL)
          mpp->add(mm);
      }
      else if (gtag.isAStartTag())
        // is an unwanted tag goup - skip
        cnn->skipToEndTag(&gtag, 200);
      // remember to test for endflag too
      else if (gtag.isTagAnEnd(getTagName()))
        break;
    }
  }
  return mpp != NULL;
}

//////////////////////////////////////////////////////

bool USmlTag::getPoseTime(UPoseTime * pose, char * name)
{
  const int MVL = 30; //
  USmlTag nTag;
  bool gotPose = false;
  bool gotTime = false;
 /*
  <poset>
  <pose name="start" x="0.034" y="-0.001" h="-0.062"/>
  <tod />
  <timeofday tod="12345678.123456"/>\n"
  </poset>
 */
  //
  reset();
  if (name != NULL)
    getAttValue("name", name, MVL);
  if (isAStartTag())
  { // should be a start tag
    while (cnn->getNextTag(&nTag, 200))
    {
      if (cnn->isVerbose())
        nTag.print("");
      if (nTag.isTagA("pose"))
        gotPose = nTag.getPose(pose);
      else if (nTag.isTagA("timeofday"))
        gotTime = nTag.getTimeofday(&pose->t, NULL);
      // remember to test for endflag too
      else if (nTag.isAStartTag())
        // is an unwanted tag goup - skip
        cnn->skipToEndTag(&nTag, 200);
      else if (nTag.isTagAnEnd(getTagName()))
        break;
    }
  }
  return gotPose and gotTime;
}

///////////////////////////////////////////////////////////////////

bool USmlTag::getTimeofday(UTime * time, char * name)
{
  const int MVL = 40; //
  char val[MVL];
  bool gotTime = false;
  int n;
  unsigned long se, us;
  //
  // <timeofday name="start" tod=567123456.123456/>
  //
  reset();
  if (getAttValue("tod", val, MVL))
  {
    n = sscanf(val, "%lu.%lu", &se, &us);
    gotTime = (n == 2);
    if (gotTime)
      time->setTime(se, us);
  }
  if (name != NULL)
    getAttValue("name", name, 30);
  //
  return gotTime;
}

///////////////////////////////////////////////////////////////

bool USmlTag::getObstacleGroup(UObstacleGroup * oGrp, char * name)
{
  const int MVL = 30; //
  char val[MVL];
  USmlTag nTag;
  bool gotSome = false;
  UPoseTime pt;
  UObstacle * obst;
  const int MOC = UObstacleGroup::MAX_OBSTACLES;
  bool upd[MOC];
  int updCnt;
  bool isUpdate = false;
  int i, idx = 0;
  unsigned long oSerial;
  bool noChange;

 /*
  <obstGrp cnt="3" serial="1" update="true">
    <poset name="first">
      <pose x="1162.565" y="1201.157" h="0.822"/>
      <timeofday tod="1148031884.497858"/>
    </poset>
    <poset name="last">
      <pose x="1163.969" y="1202.649" h="0.833"/>
      <timeofday tod="1148031886.772080"/>
    </poset>
    <obst serial="0" noChange="true"/>
    <obst hits="8" valid="true" serial="1">
      <poset name="first">
        <pose x="1162.565" y="1201.157" h="0.822"/>
        <timeofday tod="1148031884.497858"/>
      </poset>
      <poset name="last">
        <pose x="1163.969" y="1202.649" h="0.833"/>
        <timeofday tod="1148031886.772080"/>
      </poset>
      <polygon cnt="14">
        <pos3d x="1162.5180707271" y="1203.6425570087" z="0"/>
        <pos3d x="1163.3963005251" y="1203.7279421302" z="0"/>
        <pos3d x="1163.6512586026" y="1203.85665595" z="0"/>
        ........
        <pos3d x="1162.4201902918" y="1203.6486076783" z="0"/>
      </polygon>
    </obst>
    <obst hits="4" valid="true" serial="4">
      ...............
    </obst>
  </obstGrp>
 */
  reset();
  if (name != NULL)
    getAttValue("name", name, MVL);
  if (getAttValue("update", val, MVL))
    isUpdate = str2bool(val);
  if (isAStartTag())
  { // should be a start tag
    updCnt = oGrp->getObstsCnt();
    if (isUpdate)
    { // there may be obstacles in the group
      // these should be removed if not updated or unchanged
      for (i = 0; i < updCnt; i++)
        upd[i]=false;
    }
    while (cnn->getNextTag(&nTag, 200))
    {
      if (cnn->isVerbose())
        nTag.print("");
      if (nTag.isTagA("poset"))
      { // poseTime tag - either first or last
        nTag.getAttValue("name", val, MVL);
        if (strcasecmp(val, "first") == 0)
          nTag.getPoseTime(oGrp->getPPoseFirst(), NULL);
        else
          // must be last
          nTag.getPoseTime(oGrp->getPPoseLast(), NULL);
      }
      else if (nTag.isTagA("obst"))
      {
        if (nTag.getAttValue("serial", val, MVL))
        { // get obstacle to update
          oSerial = strtol(val, NULL, 10);
          obst = oGrp->getObstacle(oSerial, true, &idx);
        }
        else
          // old format with no serial number - add as new
          obst = oGrp->getNewObst();
        noChange = nTag.getAttValue("noChange", val, MVL);
        if ((obst != NULL) and not noChange)
        { // decode obstacle tag
          obst->lock();
          gotSome = nTag.getObstacle(obst, NULL);
          obst->unlock();
        }
        else if (nTag.isAStartTag())
          // no space, just skip
          cnn->skipToEndTag(&nTag, 200);
        if (isUpdate)
          upd[idx] = true;
      }
      else if (nTag.isAStartTag())
        // is an unwanted tag goup - skip
        cnn->skipToEndTag(&nTag, 200);
      // remember to test for endflag too
      else if (nTag.isTagAnEnd(getTagName()))
        break;
    }
    if (isUpdate)
    { // remove not updated obstacles
      for (i = updCnt - 1; i >= 0; i--)
      {
        if (not upd[i])
          oGrp->removeObst(i);
      }
    }
  }
  return gotSome;
}

/////////////////////////////////////////////////////

bool USmlTag::getObstacle(UObstacle * obst, char * name)
{
  const int MVL = 30; //
  char val[MVL];
  USmlTag nTag;
  bool gotSome = false;
  UPoseTime pt;
  int hits;
  UPolygon * poly;
 /*
  <obst name="maybe" hits="521" valid="true">
    <poset name="first"> ...
    </poset>
    <poset name="last"> ...
    </poset>
    <polygon name="maybe cnt="31">
      <pos3d ... />
    </polygon>
  </obst>
 */
  //reset(); // not needed
  if (name != NULL)
    getAttValue("name", name, MVL);
  hits = 0;
  if (getAttValue("hits", val, MVL))
    hits = strtol(val, NULL, 10);
  obst->setHits(hits);
  if (getAttValue("valid", val, MVL))
    obst->setValid(str2bool(val));
  // get the rest
  if (isAStartTag())
  { // should be a start tag
    while (cnn->getNextTag(&nTag, 200, this))
    {
      if (cnn->isVerbose())
        nTag.print("");
      if (nTag.isTagA("poset"))
      { // poseTime tag - either first or last
        nTag.getAttValue("name", val, MVL);
        if (strcasecmp(val, "first") == 0)
          nTag.getPoseTime(obst->getPPoseFirst(), NULL);
        else
          // must be last
          nTag.getPoseTime(obst->getPPoseLast(), NULL);
      }
      else if (nTag.isTagA("polygon"))
      {
        poly = obst;
        gotSome = nTag.getPolygon(poly, NULL);
      }
      else if (nTag.isAStartTag())
        // is an unwanted tag goup - skip
        cnn->skipToEndTag(&nTag, 200);
      // remember to test for endflag too
      else if (nTag.isTagAnEnd(getTagName()))
        break;
    }
  }
  return gotSome;
}

/////////////////////////////////////////////////////////////////////////

bool USmlTag::getPolygon(UPolygon * poly, char * name)
{
  const int MVL = 30; //
  char val[MVL];
  USmlTag nTag;
  bool gotSome = false;
  UPosition pos;
  //int cnt;
 /*
  <polygon name="maybe cnt="31">
  <pos3d x=123.556 y=123.000 z=123.666 />
  <pos3d ... />
  <pos3d ... />
  </polygon>
 */
  //reset(); // not needed
  poly->UPolygon::clear();
  if (name != NULL)
    getAttValue("name", name, MVL);
//   cnt = 0;
//   if (getAttValue("cnt", val, MVL))
//     cnt = strtol(val, NULL, 10);
  getAttValue("color", poly->color, 8);
  if (getAttValue("closed", val, MVL))
    poly->setAsPolygon(str2bool2(val, true));
  // get the rest
  if (isAStartTag())
  { // should be a start tag
    while (cnn->getNextTag(&nTag, 200))
    {
      if (cnn->isVerbose())
        nTag.print("");
      if (nTag.isTagA("pos3d"))
      { // poseTime tag - either first or last
        if (poly->getPointsCnt() < poly->getPointsMax())
        { // is a 3D position
          gotSome = nTag.getPosition(&pos);
          // add to polygon
          poly->add(pos);
        }
      }
      else if (nTag.isAStartTag())
        // is an unwanted tag goup - skip
        cnn->skipToEndTag(&nTag, 200);
      // remember to test for endflag too
      else if (nTag.isTagAnEnd(getTagName()))
        break;
    }
  }
  return gotSome;
}

//////////////////////////////////////////////////////////////////////

const char * USmlTag::codePolygon(UPolygon * poly, char * buff, int buffCnt, char * name,
                            const char * extra)
{
  return poly->codeXml(name, buff, buffCnt, extra);
}

//////////////////////////////////////////////////////////////////////

const char * USmlTag::codeObstacle(UObstacle * obst, char * buf, int bufCnt, char * name)
{
  char * p1;
  int n;
  const char * TAG_NAME = "obst";
  //
  snprintf(buf, bufCnt, "<%s", TAG_NAME);
  n = strlen(buf);
  p1 = &buf[n];
  if (name != NULL)
    if (strlen(name) > 0)
    { // include a name
      snprintf(p1, bufCnt - n, " name=\"%s\"", name);
      n += strlen(p1);
      p1 = &buf[n];
    }
  // code obstacle type
  snprintf(p1, bufCnt - n, " type=\"%s\"", obst->getDataType());
  n += strlen(p1);
  p1 = &buf[n];
  // code specific attributes for this obstacle type
  obst->codeXmlAttributes(p1, bufCnt - n);
  n += strlen(p1);
  p1 = &buf[n];
  //
  strncpy(p1, ">\n", mini(bufCnt - n, 4));
  n += strlen(p1);
  p1 = &buf[n];
  //
  codePoseTime(obst->getPoseFirst(), p1, bufCnt - n, "first");
  n += strlen(p1);
  p1 = &buf[n];
  //
  codePoseTime(obst->getPoseLast(), p1, bufCnt - n, "last");
  n += strlen(p1);
  p1 = &buf[n];
  //
  codePolygon(obst, p1, bufCnt - n, NULL, NULL);
  n += strlen(p1);
  p1 = &buf[n];
  // end tag
  snprintf(p1, bufCnt - n, "</%s>\n", TAG_NAME);
  //
  return buf;
}

/////////////////////////////////////////////////////////////////

bool USmlTag::getNextTag(USmlTag * tag, int msTimeout,
                  USmlTagIn * failEndTag,
                  char * beforeTagBuffer, int * beforeTagCnt)
{
  if (cnn == NULL)
  {
    printf("USmlTag::getNextTag: no cnn!\n");
    return false;
  }
  else
    return cnn->getNextTag(tag, msTimeout, failEndTag, beforeTagBuffer, beforeTagCnt);
}
