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

#include <ugen4/utime.h>

#include "upose.h"
#include "uposev.h"

////////////////////////////////////////////

UPose::UPose()
{
}

////////////////////////////////////////////

// virtual UPose::~UPose()
// { implemened in .h file
// }

////////////////////////////////////////////

UPose::UPose(double ix, double iy, double ih)
{
  x = ix;
  y = iy;
  h = ih;
}

////////////////////////////////////////////

UPose::UPose(UPose * source)
{
  *this = *source;
}

////////////////////////////////////////////

UPose::UPose(UPosition * pos, URotation * rot)
{
  x = pos->x;
  y = pos->y;
  h = rot->Kappa;
}

////////////////////////////////////////////

void UPose::set(UPosition * pos, URotation * rot)
{
  x = pos->x;
  y = pos->y;
  h = rot->Kappa;
}

/////////////////////////////////////////////////

void UPose::set(double ix, double iy, double ih)
{
  x = ix;
  y = iy;
  h = ih;
}

/////////////////////////////////////////////////

void UPose::add(double ix, double iy, double ih)
{
  x += ix;
  y += iy;
  h  = limitToPi(h + ih);
}

/////////////////////////////////////////////////

void UPose::fprint(FILE * fd, const char * prestring)
{ // prints the 2D position and heading only
  // assumes indoor coordinate system in meters
  fprintf(fd, "%s (x,y)=(%6.2f,%6.2f) meter %6.1f deg\n",
      prestring, x, y, getHeadingDeg());
}

/////////////////////////////////////////////////

// void UPose::print(char * s, const char * prestring, int bufLng)
// { // prints the 2D position and heading only
//   // assumes indoor sized coordinate system in meters
//   snprintf(s, bufLng, "%s (x,y)=(%6.2f,%6.2f) meter %6.1f deg",
//       prestring, x, y, getHeadingDeg());
// }

/////////////////////////////////////////////////

void UPose::snprint(const char * prestring, char * s, int bufLng)
{ // prints the 2D position and heading only
  // assumes indoor sized coordinate system in meters
  snprintf(s, bufLng, "%s (x,y)=(%.2f,%.2f) meter %.1f deg",
           prestring, x, y, getHeadingDeg());
}

/////////////////////////////////////////////////

double UPose::getHeadingDeg()
{ // returns heading in 0-360 degrees range
  double result;
  //
  result = limitToPi(h);
  result *= 180.0 / M_PI;
  if (result < 0.0)
    result += 360.0;
  //
  return result;
}

/////////////////////////////////////////////////

// bool UPose::save(Uxml3D * fxmap, const char * name /* = NULL */)
// {
//   bool result;
//   //FILE * fmap;
//   //
//   result = fxmap->saveStartTag("pose", name, true);
//   if (result)
//   {
//     fxmap->saveDoubleAttribute("x", x);
//     fxmap->saveDoubleAttribute("y", y);
//     fxmap->saveDoubleAttribute("rz", h);
//     fxmap->saveCloseTag(true);
//   }
//   //
//   return result;
// }

/////////////////////////////////////////////////

// bool UPose::load(Uxml3D * fxmap, char * name /* = NULL */)
// {
//   bool result = (fxmap != NULL);
//   char * data;
// //  double value;
//   char attName[UxmlFile::XML_NAME_LENGTH];
//   //
//   if (result)
//   {
//     if (name != NULL)
//       name[0] = '\0';
//     data = fxmap->getNextTag();
//     result = (data != NULL);
//   }
//   if (result)
//     result = (fxmap->isTagA("pose") and fxmap->isAFullTag());
//   while (result)
//   {
//     if (fxmap->getNextAttribute(&data, attName, NULL, 0))
//     { // read attributes in this full tag
//       if (strcmp(attName, "x") == 0)
//         fxmap->getDoubleAttribute(&data, NULL, &x);
//       else if (strcmp(attName, "y") == 0)
//         fxmap->getDoubleAttribute(&data, NULL, &y);
//       else if (strcmp(attName, "rz") == 0)
//         fxmap->getDoubleAttribute(&data, NULL, &h);
//       else if (strcmp(attName, "name") == 0)
//       { // if no use for name - just read to advance pointer
//         if (name != NULL)
//           fxmap->getNextAttribute(&data, NULL,
//                        name, UxmlFile::XML_NAME_LENGTH);
//         else
//           fxmap->getNextAttribute(&data, NULL,
//                        attName, UxmlFile::XML_NAME_LENGTH);
//       }
//       else
//         fxmap->errRemark("Unknown attribute in pose", name);
//     }
//     else
//       // no more attributes
//       break;
//   }
//   if (not result)
//     fxmap->errRemark("Error reading pose", data);
//   //
//   return result;
// }

/////////////////////////////////////////////

char * UPose::getAsSml(const char * name, char * buff, int buffCnt)
{
  if (name != NULL)
    snprintf(buff, buffCnt, "<pose name=\"%s\" x=\"%.3f\" y=\"%.3f\" h=\"%.5f\"/>",
         name, x, y, h);
  else
    snprintf(buff, buffCnt, "<pose x=\"%.3f\" y=\"%.3f\" h=\"%.5f\"/>", x, y, h);
  return buff;
}

/////////////////////////////////////////////////////

const char * UPose::codeXml(char * buf, const int bufCnt, const char * extraAtt)
{
  const char * es = "";
  //
  if (extraAtt != NULL)
    es = extraAtt;
  snprintf(buf, bufCnt, "<pose x=\"%.3f\" y=\"%.3f\" h=\"%.5f\" %s/>\n", x, y, h, es);
  return buf;
}

///////////////////////////////////////////////////////

const char * UPose::codeXml(const char * name, char * buff, const int buffCnt,
                               const char * extra)
{
  bool andName = false;
  const int MSL = 40000;
  char ns[MSL] = "";
  char * p1 = ns;
  int n;
  //
  if (name != NULL)
    andName = (strlen(name) > 0);
  if (andName)
    snprintf(p1, MSL, " name=\"%s\"", name);
  if (extra != NULL)
  {
    if (strlen(extra) > 0)
    {
      n = strlen(p1);
      p1 = &ns[n];
      snprintf(p1, MSL - n, " %s", extra);
    }
  }
  return codeXml(buff, buffCnt, ns);
}

//////////////////////////////////////



/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

UPoseQ::UPoseQ()
{
}

/////////////////////////////////////////////////

UPoseQ::UPoseQ(UPoseQ * source)
{
  *this = *source;
}

/////////////////////////////////////////////////

UPoseQ::~UPoseQ()
{
}

/////////////////////////////////////////////////

UMatrix4 * UPoseQ::makeQi()
{
  QiValid = false;
  d = Q.det();
  Qi = Q.inversed();
  QiValid = (Qi.err == 0);
  return &Qi;
}

/////////////////////////////////////////////////

// bool UPoseQ::save(Uxml3D * fxmap, const char * name /* = NULL */)
// {
//   bool result = (fxmap != NULL);
//   const char tag[] = "poseQ";
//   //
//   if (result)
//   {
//     fxmap->saveStartTag(tag, name, false);
//     // save pose
//     pose.save(fxmap);
//     // debug
//     // save pose covariance matrix
//     //fxmap->saveMatrix(&Q, "Qbefore connect");
//     //Q.connect();
//     // debug end
//     fxmap->saveMatrix(&Q, NULL);
//     // end qualified pose
//     fxmap->saveEndTag(tag);
//   }
//   //
//   return result;
// }

/////////////////////////////////////////////////

// bool UPoseQ::load(Uxml3D * fxmap, char * name /* = NULL */)
// { // save to file
//   bool result = (fxmap != NULL);
//   //int i;
//   char * data;
//   const char tag[] = "poseQ";
//   bool inPoseQ = false;
// //  char tagName[UxmlFile::XML_NAME_LENGTH];
//   //
//   if (result)
//   // read data
//   while (result)
//   {
//     data = fxmap->getNextTag();
//     if (data == NULL)
//     { // no more data - missing end tag?
//       result = false;
//       fxmap->errRemark("End tag not found", tag);
//       break;
//     }
//     if (fxmap->isTagA(tag))
//       inPoseQ = true;
//     else if (fxmap->isTagAnEnd(tag))
//     { // found a </poseQ>
//       inPoseQ = false;
//       break;
//     }
//     else if (fxmap->isTagA("pose"))
//     { // load pose estimate
//       fxmap->setBufferUnused();
//       result = pose.load(fxmap);
//     }
//     else if (fxmap->isTagA("matrix"))
//     { // load covariance matrix
//       fxmap->setBufferUnused();
//       result = fxmap->loadMatrix(&Q);
//       QiValid = false;
//     }
//   }
//   //
//   return result;
// }

///////////////////////////////////
/*  Va = [xa, ya, ha] Vbase = [xb, yb , hb] D = [x, y, h].
    xa = xb + x cos(hb) - y sin(hb);
    ya = yb + x sin(hb) + y cos(hb);
    ha = hb + h;
    Returns the new pose Va.
    (from Lu and Milios (1997)) */
void UPose::asAddC(UPose Vbase, UPose D)
{
  double shb, chb;
  //
  shb = sin(Vbase.h);
  chb = cos(Vbase.h);
  x = Vbase.x + D.x * chb - D.y * shb;
  y = Vbase.y + D.x * shb + D.y * chb;
  h = limitToPi(Vbase.h + D.h);
}

///////////////////////////////////////
/*
void UPose::addC(UPose D)
{
  double shb, chb;
  //
  shb = sin(h);
  chb = cos(h);
  x += D.x * chb - D.y * shb;
  y += D.x * shb + D.y * chb;
  h  = limitToPi(h + D.h);
}
*/
///////////////////////////////////////

UPose UPose::addCed(UPose * D) 
{
  UPose result;
  result.asAddC(*this, *D);
  return result;
}

///////////////////////////////////////

void UPose::add(double dist, double headingChange)
{
  x += dist * cos(h + headingChange / 2.0);
  y += dist * sin(h + headingChange / 2.0);
  h = limitToPi(h + headingChange);
}

///////////////////////////////////////
/*
  Inverse compounding operation calculating the
  pose change from Vbase to Va, i.e. D = Va o- Vb.
  Va = [xa, ya, ha] Vbase = [xb, yb , hb] D = [x, y, h].
  x = (xa - xb)cos(hb) + (ya - yb)sin(hb);
  y = -(xa - xb)sin(hb) + (ya - yb)cos(hb);
  h = ha - hb;
  Returns the inverse compound pose.
  (from Lu and Milios (1997)) */

void UPose::asSubC(UPose Va, UPose Vbase)
{
  double shb, chb;
  //
  shb = sin(Vbase.h);
  chb = cos(Vbase.h);
  x =   (Va.x - Vbase.x) * chb + (Va.y - Vbase.y) * shb;
  y = - (Va.x - Vbase.x) * shb + (Va.y - Vbase.y) * chb;
  h = limitToPi(Va.h - Vbase.h);
}

//////////////////////////////////////////////

void UPose::subC(UPose * Vbase)
{
  asSubC(*this, Vbase);
  /*
  double shb, chb;
  //
  shb = sin(Vbase.h);
  chb = cos(Vbase.h);
  x =  (x - Vbase.x) * chb + (y - Vbase.y) * shb;
  y = - (x - Vbase.x) * shb + (y - Vbase.y) * chb;
  h = limitToPi(h - Vbase.h);
  */
}

//////////////////////////////////////////////

UPose UPose::subCed(UPose * Vbase)
{
  UPose result;
  result.asSubC(*this, *Vbase);
  return result;
}

//////////////////////////////////////////////

void UPose::asNegC(UPose D)
{
  UPose V0(0.0, 0.0, 0.0);
  /*
  double shb, chb;
  //
  shb = sin(D.h);
  chb = cos(D.h);
  x = - D.x * chb - D.y * shb;
  y =   D.x * shb - D.y * chb;
  h = - D.h;
  */
  asSubC(V0, D);
}

//////////////////////////////////////////////

UPose UPose::neg()
{
  UPose result;
  result.asNegC(*this);
  return result;
}

//////////////////////////////////////////////

UPosition UPose::getMapToPose(UPosition mapPos)
{
   UPosition result;
   double ch, sh;
   double lx, ly; // local coordinates
   //
   ch = cos(h);
   sh = sin(h);
   lx = mapPos.x - x;
   ly = mapPos.y - y;
   //
   result.x =  ch * lx + sh * ly;
   result.y = -sh * lx + ch * ly;
   result.z = mapPos.z;
   //
   return result;
}

//////////////////////////////////////////////

U2Dpos UPose::getMapToPose(U2Dpos mapPos)
{
  U2Dpos result;
  double ch, sh;
  double lx, ly; // local coordinates
  //
  ch = cos(h);
  sh = sin(h);
  lx = mapPos.x - x;
  ly = mapPos.y - y;
  //
  result.x =  ch * lx + sh * ly;
  result.y = -sh * lx + ch * ly;
  //
  return result;
}

//////////////////////////////////////////////

UPosition UPose::getMapToPose(UPose * mapPos)
{
   UPosition result;
   double ch, sh;
   double lx, ly; // local coordinates
   //
   ch = cos(h);
   sh = sin(h);
   lx = mapPos->x - x;
   ly = mapPos->y - y;
   //
   result.x =  ch * lx + sh * ly;
   result.y = -sh * lx + ch * ly;
   result.z = 0.0;
   //
   return result;
}

//////////////////////////////////////////////

CvPoint UPose::getMapToPose(CvPoint mapPos)
{
   CvPoint result;
   double ch, sh;
   double lx, ly; // local coordinates
   //
   ch = cos(h);
   sh = sin(h);
   lx = double(mapPos.x) - x;
   ly = double(mapPos.y) - y;
   //
   result.x =  roundi(ch * lx + sh * ly);
   result.y = roundi(-sh * lx + ch * ly);
   //
   return result;
}

//////////////////////////////////////////////

UPosition UPose::getPoseToMap(UPosition posePos)
{
  UPosition result;
  double ch, sh;
  //
  ch = cos(h);
  sh = sin(h);
  //
  result.x =  ch * posePos.x - sh * posePos.y + x;
  result.y =  sh * posePos.x + ch * posePos.y + y;
  result.z = posePos.z;
  //
  return result;
}

//////////////////////////////////////////////

U2Dpos UPose::getPoseToMap(U2Dpos posePos)
{
   U2Dpos result;
   double ch, sh;
   //
   ch = cos(h);
   sh = sin(h);
   //
   result.x =  ch * posePos.x - sh * posePos.y + x;
   result.y =  sh * posePos.x + ch * posePos.y + y;
   //
   return result;
}

//////////////////////////////////////////////


UPosition UPose::getPoseToMap(UPose posePos)
{
   UPosition result;
   double ch, sh;
   //
   ch = cos(h);
   sh = sin(h);
   //
   result.x =  ch * posePos.x - sh * posePos.y + x;
   result.y =  sh * posePos.x + ch * posePos.y + y;
   result.z = 0.0;
   //
   return result;
}

//////////////////////////////////////////////

UPosition UPose::getPoseToMap(double localX, double localY)
{
  UPosition result;
  double ch, sh;
  //
  ch = cos(h);
  sh = sin(h);
  //
  result.x =  ch * localX - sh * localY + x;
  result.y =  sh * localX + ch * localY + y;
  result.z = 0.0;
  //
  return result;
}

//////////////////////////////////////////////

CvPoint UPose::getPoseToMap(CvPoint posePos)
{
   CvPoint result;
   double ch, sh;
   //
   ch = cos(h);
   sh = sin(h);
   //
   result.x =  roundi(ch * posePos.x - sh * posePos.y + double(x));
   result.y =  roundi(sh * posePos.x + ch * posePos.y + double(y));
   //
   return result;
}

///////////////////////////////////////////

UPose UPose::getMapToPosePose(UPose * mapPose)
{
  UPosition pos;
  UPose result;
  pos = getMapToPose(mapPose);
  result.x = pos.x;
  result.y = pos.y;
  result.h = limitToPi(mapPose->h - h);
  return result;
}

///////////////////////////////////////////

UPose UPose::getPoseToMapPose(UPose poseLocal)
{
  UPose result;
  double ch, sh;
  //
  ch = cos(h);
  sh = sin(h);
  //
  result.x =  ch * poseLocal.x - sh * poseLocal.y + x;
  result.y =  sh * poseLocal.x + ch * poseLocal.y + y;
  result.h = limitToPi(poseLocal.h + h);
  return result;
}

///////////////////////////////////////////

UPose UPose::operator= (UPoseTime source)
{
  x = source.x; y = source.y; h = source.h;
  return *this;
}

//////////////////////////////////////////////

UPose UPose::operator= (UPoseV source)
{
  x = source.x; y = source.y; h = source.h;
  return *this;
}

/////////////////////////////////////////////////

UPose UPose::operator= (UPoseTVQ source)
{
  x = source.x; y = source.y; h = source.h;
  return *this;
}

/////////////////////////////////////////////////

double UPose::getDistToPoseLineSigned(const double Px, const double Py)
{
  double A = -sin(h);
  double B = cos(h);
  double C = -A * x - B * y;
  double d = hypot(A,B);
  //
  if (d > 1e-50)
  { // normalize line
    A /= d;
    B /= d;
    C /= d;
  }
  // evaluete distance
  d = A * Px + B * Py + C; // sqrt(sqr(lA) + sqr(lB)) == 1;
  return d;
}

/////////////////////////////////////////////////

UMatrix4 UPose::asMatrix4x4PtoM()
{
//   cos(h)   -sin(h)   0   x
//   sin(h)   cos(h)   0   y
//      0        0     1   h
//      0        0     0    1

  UMatrix4 m(4, 4);
  double ch = cos(h);
  double sh = sin(h);
  m.setRow(0, ch, -sh,  0.0, x);
  m.setRow(1, sh,  ch,  0.0, y);
  m.setRow(2, 0.0, 0.0, 1.0, h);
  m.setRow(3, 0.0, 0.0,  0.0, 1.0);
  return m;
}

/////////////////////////////////////////////////

UMatrix4 UPose::asMatrix4x4PtoMPos()
{
//   cos(h)  -sin(h)   0    x
//   sin(h)   cos(h)   0    y
//      0        0     1    0
//      0        0     0    1

  UMatrix4 m(4, 4);
  double ch = cos(h);
  double sh = sin(h);
  m.setRow(0, ch, -sh,  0.0, x);
  m.setRow(1, sh,  ch,  0.0, y);
  m.setRow(2, 0.0, 0.0, 1.0, 0.0);
  m.setRow(3, 0.0, 0.0,  0.0, 1.0);
  return m;
}

/////////////////////////////////////////

UMatrix4 UPose::asMatrix4x4MtoP()
{
//   cos(h)   sin(h)   0  -cos(h)*x-sin(h)*y
//   -sin(h)   cos(h)   0   sin(h)*x-cos(h)*y
//      0        0     1      -h
//      0        0     0      1

  UMatrix4 m(4, 4);
  double ch = cos(h);
  double sh = sin(h);
  m.setRow(0,  ch,  sh, 0.0, -ch * x - sh * y);
  m.setRow(1, -sh,  ch, 0.0,  sh * x - ch * y);
  m.setRow(2, 0.0, 0.0, 1.0,  -h);
  m.setRow(3, 0.0, 0.0, 0.0,  1.0);
  return m;
}

/////////////////////////////////////////////////

UMatrix4 UPose::asMatrix3x3PtoM()
{
//   cos(h) -sin(h) x
//   sin(h) cos(h)  y
//     0       0     1

  UMatrix4 m(3, 3);
  double ch = cos(h);
  double sh = sin(h);
  m.setRow(0, ch,  -sh, x);
  m.setRow(1, sh,   ch, y);
  m.setRow(2, 0.0, 0.0, 1.0);
  return m;
}

/////////////////////////////////////////

UMatrix4 UPose::asMatrix3x3MtoP()
{
//   cos(h)   sin(h)  cos(h)*x+sin(h)*y
//  -sin(h)   cos(h)  -sin(h)*x+cos(h)*y
//      0        0           1

  UMatrix4 m(3, 3);
  double ch = cos(h);
  double sh = sin(h);
  m.setRow(0,  ch,   sh,  -ch * x - sh * y);
  m.setRow(1,  -sh,  ch,   sh * x - ch * y);
  m.setRow(2, 0.0, 0.0,   1.0);
  return m;
}

/////////////////////////////////////////

UMatrix4 UPose::asMatrix2x2PtoM()
{
//   cos(h) -sin(h)
//   sin(h) cos(h)

  UMatrix4 m(2,2);
  double ch = cos(h);
  double sh = sin(h);
  m.setRow(0, ch,   -sh);
  m.setRow(1, sh,   ch);
  return m;
}

UMatrix4 UPose::asMatrix2x2MtoP()
{
//   cos(h)  sin(h)
//   -sin(h) cos(h)

  UMatrix4 m(2,2);
  double ch = cos(h);
  double sh = sin(h);
  m.setRow(0, ch,   sh);
  m.setRow(1, -sh,   ch);
  return m;
}

/////////////////

UMatrix4 UPose::asRow4()
{
  UMatrix4 m(1,4);
  m.set(x, y, h, 1.0);
  return m;
}

/////////////////

UMatrix4 UPose::asCol4()
{
  UMatrix4 m(4, 1);
  m.set(x, y, h, 1.0);
  return m;
}

/////////////////

UMatrix4 UPose::asRow3()
{
  UMatrix4 m(1,3);
  m.set(x, y, h);
  return m;
}

/////////////////

UMatrix4 UPose::asCol3()
{
  UMatrix4 m(3, 1);
  m.set(x, y, h);
  return m;
}

///////////////////

UPose UPose::set(UMatrix * mat)
{
  if (mat->size() == 4)
  {
    x = mat->get(0) / mat->get(3);
    y = mat->get(1) / mat->get(3);
    h = mat->get(2) / mat->get(3);
  }
  else
  {
    x = mat->get(0);
    y = mat->get(1);
    h = mat->get(2);
  }
  return *this;
}

////////////////////////////////

UPose UPose::operator-(UPoseV pRef)
{
  return subCed(&pRef);
}

////////////////////////////////

UPose UPose::operator+(UPoseV pDelta)
{
  return addCed(&pDelta);
}

////////////////////////////////

UPose UPose::operator-(UPoseTime pRef)
{
  return subCed(&pRef);
}

////////////////////////////////

UPose UPose::operator+(UPoseTime pDelta)
{
  return addCed(&pDelta);
}

////////////////////////////////

UPose UPose::operator-(UPoseTVQ pRef)
{
  return subCed(&pRef);
}

////////////////////////////////

UPose UPose::operator+(UPoseTVQ pDelta)
{
  return addCed(&pDelta);
}


/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

UPoseTime::UPoseTime()
{
}

///////////////////////////////////////////////////////////////

UPose UPoseTime::getPoseAtTime(UPoseTime otherPose, UTime atTime)
{
  UPose result;
  UPose pv;
  double dt;
  double part, dh, dx, dy, ds;
  bool isOK;
  //
  dt = (otherPose.t - t);
  isOK = absf(dt) > 1e-6;
  if (isOK)
  {
    part = (atTime - t)/dt;
    dh = limitToPi(otherPose.h - h);
    dx = otherPose.x - x;
    dy = otherPose.y - y;
    ds = hypot(dx, dy);
    // direct position interpolation (as if same heading or
    // heading change only)
    result.x = x + part * dx;
    result.y = y + part * dy;
    result.h = limitToPi(h + part * dh);
    // if both movement and turn, so do a little more
    if ((absf(dh) > (1.0 * M_PI / 180.0)) and (ds > 0.01))
    { // combined move - follow curve
      // get end position if followed start heading
      pv.x = x + part * ds * cos(h);
      pv.y = y + part * ds * sin(h);
      // interpolate between this and other
      result.x = pv.x + part * (result.x - pv.x);
      result.y = pv.y + part * (result.y - pv.y);
    }
  }
  else
    // poses is assumed to be identical, so return this pose
    result = this->getPose();
  return result;
}

/////////////////////////////////////////////////////

void UPoseTime::fprint(FILE * fd, const char * preString)
{
  const int MSL = 100;
  char s[MSL];
  const int MTL = 30;
  char st[MTL];
  //
  t.getTimeAsString(st, true);
  snprintf(s, MSL, "%s %s", preString, st);
  UPose::fprint(stdout, s);
}

/////////////////////////////////////////////////////

void UPoseTime::snprint(const char * preString, char * buff, const int buffCnt)
{
  const int MSL = 100;
  char s[MSL];
  const int MTL = 30;
  char st[MTL];
  //
  t.getTimeAsString(st, true);
  snprintf(s, MSL, "%s %s", preString, st);
  UPose::snprint(s, buff, buffCnt);
}

/////////////////////////////////////////////////////

const char * UPoseTime::codeXml(char * buf, const int bufCnt, const char * extraAtt)
{
  const int MAL = 500;
  char as[MAL];
  const char * es = "";
  //
  if (extraAtt != NULL)
    es = extraAtt;
  // make longer extra attribute string
  snprintf(as, MAL, "%s tod=\"%ld.%06ld\"", es, t.getSec(), t.getMicrosec());
  // and add this to the ordinary pose
  return UPose::codeXml(buf, bufCnt, as);
}

///////////////////////////////////////////////////////

UPoseTime UPoseTime::operator= (UPoseTVQ source)
{
  x = source.x; y=source.y; h=source.h; t=source.t;
  return *this;
}


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

UPoseTVQ::UPoseTVQ()
  : UPoseTime()
{
  q = -1.0;
}

/////////////////////////////////////

UPoseTVQ::UPoseTVQ(double x, double y, double h, UTime t, double v, double qual)
  : UPoseTime(x, y, h, t)
{
  vel = v;
  q = qual;
}

/////////////////////////////////////

UPoseTVQ::UPoseTVQ(UPose pose, UTime t, double v, double qual)
  : UPoseTime(pose, t)
{
  vel = v;
  q = qual;
}

////////////////////////////////////

void UPoseTVQ::snprint(const char * prestring, char * buff, const int buffCnt)
{ // prints the 2D position and heading only
  // assumes indoor coordinate system in meters
  snprintf(buff, buffCnt, "%s (x,y)=(%.2f,%.2f)m, %.3frad, %.1fdeg, %lu.%06lu %.2fm/s %gq\n",
           prestring, x, y, h, getHeadingDeg(), t.getSec(), t.getMicrosec(), getVel(), q);
}

//////////////////////////////////////

UPoseTVQ UPoseTVQ::operator-(UPoseV pRef)
{
  UPoseTVQ result;
  //
  result = subCed(pRef);
  result.setVel(vel - pRef.getVel());
  //
  return result;
}

//////////////////////////////////////

UPoseTVQ UPoseTVQ::operator+(UPoseV delta)
{
  UPoseTVQ result;
  //
  result = addCed(delta);
  result.setVel(vel + delta.getVel());
  //
  return result;
}

//////////////////////////////////

UPoseTVQ UPoseTVQ::operator+(UPose delta)
{
  UPoseTVQ result;
  result = addCed(delta);
  return result;
}

////////////////////////////////////

void UPoseTVQ::set(UPoseV * source)
{
  x = source->x;
  y = source->y;
  h = source->h;
  vel = source->getVel();
  q = 0.0;
}

/////////////////////////////////////

UPoseV UPoseTVQ::getPoseV()
{
  UPoseV result(x, y, h, vel);
  return result;
}


