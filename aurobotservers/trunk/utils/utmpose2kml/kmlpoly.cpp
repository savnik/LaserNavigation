/***************************************************************************
 *   Copyright (C) 2008 by Christian Andersen   *
 *   chrand@mail.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <ugen4/ucommon.h>
#include <expat.h>
#include <ctype.h>

#include "kmlpoly.h"

KmlPoly::KmlPoly()
{
  isKml = false;
  isName = false;
  placemark = false;
  coordinates = false;
  pktCnt = 0;
  bufCnt = 0;
  strncpy(kmlname, "noname-mission", kmlnameMaxCnt);
  posesCnt = 0;
}


KmlPoly::~KmlPoly()
{
}

/*
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://earth.google.com/kml/2.2">
<Document>
        <name>KmlFile</name>
        <Style id="sn_ylw-pushpin">
                <IconStyle>
                        <scale>1.1</scale>
                        <Icon>
                                <href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>
                        </Icon>
                        <hotSpot x="20" y="2" xunits="pixels" yunits="pixels"/>
                </IconStyle>
        </Style>
        <StyleMap id="msn_ylw-pushpin">
                <Pair>
                        <key>normal</key>
                        <styleUrl>#sn_ylw-pushpin</styleUrl>
                </Pair>
                <Pair>
                        <key>highlight</key>
                        <styleUrl>#sh_ylw-pushpin</styleUrl>
                </Pair>
        </StyleMap>
        <Style id="sh_ylw-pushpin">
                <IconStyle>
                        <scale>1.3</scale>
                        <Icon>
                                <href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>
                        </Icon>
                        <hotSpot x="20" y="2" xunits="pixels" yunits="pixels"/>
                </IconStyle>
        </Style>
        <Placemark>
                <name>to_pometet</name>
                <styleUrl>#msn_ylw-pushpin</styleUrl>
                <LineString>
                        <tessellate>1</tessellate>
                        <coordinates>
12.30559350071359,55.67000671081631,0 12.30568972342508,55.6700621350543,0 12.30574909689494,55.67006790934568,0 12.30582075742493,55.67002980782293,0 12.30583922175561,55.66940861776587,0 12.30587198233156,55.66935319616968,0 12.30587198609368,55.66928738219438,0 12.3059190763098,55.66925851732702,0 12.30698368626549,55.6692643056329,0 12.30875921075941,55.66928367190939,0 12.3092821879941,55.66928721770374,0 12.30939683983797,55.66932185546397,0 12.30935999762601,55.66968902837714,0 12.30926787743723,55.67005735635058,0 12.30917575059915,55.67029692179166,0 12.30904472386334,55.67054632279171,0 12.30903039343349,55.67060174523601,0 12.30909591089514,55.6706236829107,0 12.30932522105184,55.67064100097441,0 12.30956610043045,55.67066019004234,0 12.30977084059255,55.67066249701771,0 </coordinates>
                </LineString>
        </Placemark>
</Document>
</kml>

*/


///////////////////////////////////////////////////////

void KmlPoly::startTag(void *userData, const XML_Char *el, const XML_Char **attr)
{
  KmlPoly * kp = (KmlPoly *) userData;
  //
  if(strcmp(el, "kml") == 0)
  {
    kp->isKml = true;
  }
  else if (strcmp(el, "Placemark") == 0)
  {
    kp->placemark = true;
  }
  else if (strcmp(el, "coordinates") == 0)
  {
    if (kp->isKml and kp->placemark)
      kp->coordinates = true;
  }
  else if (strcmp(el, "name") == 0)
  {
    if (kp->isKml)
      kp->isName = true;
  }
}

/////////////////////////////////////////////

void KmlPoly::endTag(void *userData, const XML_Char *el)
{
  KmlPoly * kp = (KmlPoly *) userData;

  if (strcmp(el, "kml") == 0)
  {
    kp->isKml = false;
  }
  else if (strcmp(el, "Placemark") == 0)
  {
    kp->placemark = false;
  }
  else if (strcmp(el, "coordinates") == 0)
  {
    kp->coordinates = false;
  }
  else if (strcmp(el, "name") == 0)
  {
    kp->isName = false;
  }
}

/////////////////////////////////////////////

void KmlPoly::tagText(void *userData, const XML_Char *s, int len)
{
  KmlPoly * kp = (KmlPoly *) userData;
  char * p1, *p2, *p3;
  p2 = (char*)s;
  int i;
  double d;
  //
  if (kp->isName and len > 2)
  { // find first usable part of name defined in kml file
    p3 = p2;
    while (isspace(*p2) and (p2 - p3 < len))
      p2++;
    p1 = strsep(&p2, " ");
    p2 = kp->kmlname;
    i = 0;
    while ((isalnum(*p1) or (*p1 == '_')) and (p2 - p3 < len))
    {
      *p2++ = *p1++;
      i++;
      if (i > kp->kmlnameMaxCnt - 2)
        break;
    }
    *p2 = '\0';
  }
  else if (kp->coordinates)
  {
    p1 = p2;
    while (p2-p1 < len and p2 != NULL)
    {
      p3 = p2;
      d = strtod(p3, &p2);
      if (p2 != p3)
        kp->pkt[kp->pktCnt++] = d;
      else
        // unusable character - skip
        p2++;
      // long,lat,height long,lat,height long,lat,height
      if (*p2 == ',')
        p2++;
    }
  }
}

////////////////////////////////////////////

bool KmlPoly::load(FILE * fptr)
{
  const int BUFFSIZE = 200000;
  char * buffer = NULL;
  int done;
  int len;
  bool result;
  char * p1;
  //
  buffer = (char*) malloc(BUFFSIZE);
  XML_Parser p = XML_ParserCreate(NULL);
  if (! p or buffer == 0)
  {
    fprintf(stderr, "Couldn't allocate memory for parser and buffer\n");
    return false;
  }
  // register handlers
  XML_SetUserData(p, this);
  XML_SetElementHandler(p, KmlPoly::startTag, KmlPoly::endTag);
  XML_SetCharacterDataHandler(p, KmlPoly::tagText);
  //
  do
  {
    p1 = fgets(buffer, BUFFSIZE, fptr);
    //printf("Read :%s", buffer);

    if (ferror(fptr))
    {
      fprintf(stderr, "Read error\n");
      return false;
    }
    done = feof(fptr);
    len = strlen(buffer);
    if (not done)
    { // ignore last line
      if (! XML_Parse(p, buffer, len, done))
      {
        fprintf(stderr, "Parse error at line %d:\n%s\n", (int) XML_GetCurrentLineNumber(p), XML_ErrorString(XML_GetErrorCode(p)));
        result = false;
        break;
      }
    }
  } while(!done);

  XML_ParserFree(p);
  free(buffer);
  return result;
}

//////////////////////////////////////////////////////

bool KmlPoly::makeRouteRule(FILE * fd, double offsetX, double offsetY, const char * name, bool inUTM)
{
  bool isOK = false;
  FILE *dst = fd;
  PoseQ po0, po1, po2;
  double h, d01, d12, lat = 0.0, lon = 0.0;
  int i;
  const int zone = 32;
  const int wgs84 = 23;
  double dy01, dx01, dy12, dx12;
  const double minRowLength = 20.0; // meter
  double hd, h1d = 0.0; //, h0d; // angles in compas degrees
  bool holdLine;
  const char * p1 = name;
  //
  if (dst != NULL and (pktCnt >= 4 or posesCnt > 0))
  { // write rule prolog
    if (p1 == NULL)
      p1 = kmlname;
    fprintf(dst, "<rule name=\"%s\" run=\"false\">\n", p1);
    fprintf(dst, "<description>\n");
    fprintf(dst, "  Converted from google earth plan - check that name is legal\n");
    fprintf(dst, "  Heading is estimated from line length, and may be inappropriate\n");
    fprintf(dst, "  (rev2)\n");
    fprintf(dst, "  # converted from KML - offset by %.2fE, %.2fN\n", offsetX, offsetY);
    fprintf(dst, "  #           map part - offset by %.2fE, %.2fN\n", offsetX - OX, offsetY - OY);
    fprintf(dst, "</description>\n");
    fprintf(dst, "<init>\n");
    fprintf(dst, "  global.drive.mission='%s'\n", name);
    fprintf(dst, "  global.drive.compasHeading=true\n");
    fprintf(dst, "  # global.drive.compasHeading=false # means radians\n");
    fprintf(dst, "  global.drive.continueDist = 1.5 # meter, relaxed driving\n");
    fprintf(dst, "  # global.drive.continueDist = 0 # meter, tight driving\n");
    fprintf(dst, "</init>\n");
    fprintf(dst, "# smr.speed = 0.7 # desired drive speed m/s (0.7m/s~2.5km/h)\n");
    fprintf(dst, "smr.do('set \"hakoliftinggearstateref\" 100')\n");
    fprintf(dst, "smr.do('set \"hakopowertakeoffstateref\" 0')\n");
    fprintf(dst, "smr.do('set \"hakoenginespeed\" 1900') # RPM\n");
    fprintf(dst, "smr.speed = 0.6\n");
    fprintf(dst, "global.drive.directOrWait = false\n");
    fprintf(dst, "#\n");
    po2.clear();
    po1.clear();
    dx12 = 0.0;
    dy12 = 0.0;
    d12 = 0.0;
    if (posesCnt == 0)
    {
      for (i = 0; i < pktCnt; i += 3)
      {
        po0 = po1;
        po1 = po2;
        dx01 = dx12;
        dy01 = dy12;
        d01 = d12;
        lon = pkt[i];
        lat = pkt[i+1];
        holdLine = false;
        if (i >= 3)
          fprintf(dst, "# to lat %.6f deg and long %.6f deg\n", lat, lon);
        LLtoUTM(wgs84, lat, lon, &po2.y, &po2.x, zone);
        // keep the old heading
        if (i >= 3)
        { // guess a good heading for the po1 node, based on
          // distance to next node.
          dy12 = po2.y - po1.y;
          dx12 = po2.x - po1.x;
          d12 = hypot(dy12, dx12);
          if ((d01 < minRowLength and d12 > minRowLength) or i == 3)
          { // use comming long (row) heading as heading
            po1.h = atan2(dy12, dx12);
            holdLine = true;
          }
          else if ((d01 < minRowLength and d12 < minRowLength) or
                    (d01 >= minRowLength and d12 >= minRowLength))
          { // if last distance was manoeuvre too, then
            // make this heading the average of the previous and next line
            h = atan2(dy01/d01 + dy12/d12, dx01/d01 + dx12/d12);
            //hd = limitTo2Pi(M_PI/2.0 - h) * 180.0 / M_PI;
            po1.h = h;
          }
          else
          { // keep last heading from the long streach
            po1.h = po0.h;
          }
          // last heading in compas degrees
          //h0d = limitTo2Pi(M_PI/2.0 - po0.h) * 180.0 / M_PI;
          // current heading in compas degrees
          h1d = limitTo2Pi(M_PI/2.0 - po1.h) * 180.0 / M_PI;
          // heading of this streach
          h = atan2(dy01, dx01);
          hd = limitTo2Pi(M_PI/2.0 - h) * 180.0 / M_PI;
          if (fabs(d01) > 0.1)
            fprintf(dst, "# distance is %.2fm long at compas heading %.1f\n", d01, hd);
          if (inUTM)
            fprintf(dst, "driveUTM(%.2f, %.2f, %.1f)\n", po1.x + offsetX, po1.y + offsetY, h1d);
          else
            fprintf(dst, "driveMap(%.2f, %.2f, %.1f)\n", po1.x + offsetX - OX, po1.y + offsetY - OY, h1d);
          fprintf(dst, "#\n");
          if (holdLine)
            fprintf(dst, "global.drive.holdLine=true\n");
        }
      }
    }
    else
    { // already in map coordinates
      for (i = 0; i < posesCnt; i++)
      {
        hd = limitTo2Pi(M_PI/2.0 - poses[i].h) * 180.0 / M_PI;
        if (inUTM)
          fprintf(dst, "driveUTM(%.2f, %.2f, %.1f)\n", poses[i].x + offsetX, poses[i].y + offsetY, hd);
        else
          fprintf(dst, "driveMap(%.2f, %.2f, %.1f)\n", poses[i].x + offsetX, poses[i].y + offsetY, hd);
      }
    }
    // last point too
    if (posesCnt == 0)
    {
      fprintf(dst, "# ends at lat %.6f deg and long %.6f deg\n", lat, lon);
      h = atan2(dy12, dx12);
      hd = limitTo2Pi(M_PI/2.0 - h) * 180.0 / M_PI;
      fprintf(dst, "# last distance is %.2fm long at compas heading %.1f\n", d12, hd);
      if (inUTM)
        fprintf(dst, "driveUTM(%.2f, %.2f, %.1f)\n", po2.x + offsetX, po2.y + offsetY, hd);
      else
        fprintf(dst, "driveMap(%.2f, %.2f, %.1f)\n", po2.x + offsetX - OX, po2.y + offsetY - OY, h1d);
    }
    fprintf(dst, "driveStop()\n");
    fprintf(dst, "# wait a 4 seconds before saving the MRC log\n");
    fprintf(dst, "smr.do('set \"hakoenginespeed\" 800') # RPM\n");
    fprintf(dst, "wait(4) : false\n");
    fprintf(dst, "smr.saveMrcLog()\n");
    fprintf(dst, "print('finished %s')\n", name);
    // prolog
    fprintf(dst, "</rule>\n");
    isOK = true;
  }
  else
    printf("Destnation file is not open! - no plan written\n");

  //
  return isOK;
}

//////////////////////////////////////////////

bool KmlPoly::makeRouteMRC(FILE * fd, double offsetX, double offsetY, const char * name)
{
  bool isOK = false;
  FILE *dst = fd;
  PoseQ po0, po1, po2;
  double h, d01, d12, lat = 0.0, lon = 0.0;
  int i;
  const int zone = 32;
  const int wgs84 = 23;
  double dy01, dx01, dy12 = 0.0, dx12 = 0.0;
  const double minRowLength = 20.0; // meter
  double hd, h1d = 0.0; //, h0d; // angles in compas degrees
  bool holdLine;
  const char * p1 = name;
  //
  if (dst != NULL and (pktCnt >= 4 or posesCnt > 0))
  { // write rule prolog
    if (p1 == NULL)
      p1 = kmlname;
    fprintf(dst, "gyrooffset = 12.340615\n");
    fprintf(dst, "gyrogain = %g\n", -1.0/3600.0*3.1415926/180.0);
    fprintf(dst, "set \"gyrotype\" 1\n");
    fprintf(dst, "routeinput \"mrc\" \"odoconnect\" \"dth\" \"$fogPhdZ\"\n");
    fprintf(dst, "set \"$gyro1gain\" gyrogain\n");
    fprintf(dst, "set \"$gyro1off\" gyrooffset\n");
    fprintf(dst, "set \"odocontrol\" 1\n");
    fprintf(dst, "control \"removelogvars\"\n");
    fprintf(dst, "log \"$time\" \"$odox\" \"$odoy\" \"$odoth\" \"$gpseasting\" \"$gpsnorthing\"\n");
    fprintf(dst, "log \"$xkalman\" \"$ykalman\" \"$thkalman\" \"$hakosteeringangleref\" \"$hakosteeringangle\" \"$hakospeedref\"\n");
    fprintf(dst, "log \"$kalmanmode\" \"$kalmanstatus\" \"$gpsquality\" \"$gpsdop\" \"$gpssatellites\" \"$odovelocity\"\n");
    fprintf(dst, "control \"startlog\"\n");
    fprintf(dst, "set \"kalmanon\" 1\n");
    fprintf(dst, "smr stream=1\n");
    fprintf(dst, "set \"hakoliftinggearstateref\" 130\n");
    // initialize kalman filter
    fprintf(dst, "set \"usekalmanodo\" 1\n");
    fprintf(dst, "fwd 10 @v0.4\n");
    fprintf(dst, "set \"hakoenginespeed\" 1900\n");
    if (posesCnt == 0)
    {
      // ready to run
      po2.clear();
      po1.clear();
      dx12 = 0.0;
      dy12 = 0.0;
      d12 = 0.0;
      for (i = 0; i < pktCnt; i += 3)
      {
        po0 = po1;
        po1 = po2;
        dx01 = dx12;
        dy01 = dy12;
        d01 = d12;
        lon = pkt[i];
        lat = pkt[i+1];
        holdLine = false;
        //if (i >= 3)
        //  fprintf(dst, "# to lat %.6f deg and long %.6f deg\n", lat, lon);
        LLtoUTM(wgs84, lat, lon, &po2.y, &po2.x, zone);
        // keep the old heading
        if (i >= 3)
        { // guess a good heading for the po1 node, based on
          // distance to next node.
          dy12 = po2.y - po1.y;
          dx12 = po2.x - po1.x;
          d12 = hypot(dy12, dx12);
          if ((d01 < minRowLength and d12 > minRowLength) or i == 3)
          { // use comming long (row) heading as heading
            po1.h = atan2(dy12, dx12);
            holdLine = true;
          }
          else if ((d01 < minRowLength and d12 < minRowLength) or
                    (d01 >= minRowLength and d12 >= minRowLength))
          { // if last distance was manoeuvre too, then
            // make this heading the average of the previous and next line
            h = atan2(dy01/d01 + dy12/d12, dx01/d01 + dx12/d12);
            //hd = limitTo2Pi(M_PI/2.0 - h) * 180.0 / M_PI;
            po1.h = h;
          }
          else
          { // keep last heading from the long streach
            po1.h = po0.h;
          }
          // last heading in compas degrees
          //h0d = limitTo2Pi(M_PI/2.0 - po0.h) * 180.0 / M_PI;
          // current heading in compas degrees
          h1d = limitTo2Pi(M_PI/2.0 - po1.h) * 180.0 / M_PI;
          // heading of this streach
          h = atan2(dy01, dx01);
          hd = limitTo2Pi(M_PI/2.0 - h) * 180.0 / M_PI;
          //if (fabs(d01) > 0.1)
          //  fprintf(dst, "# distance is %.2fm long at compas heading %.1f\n", d01, hd);
          fprintf(dst, "driveon %.2f %.2f %.1f @v0.7 : ($targetdist < 1.0)\n", po1.x + offsetX, po1.y + offsetY, po1.h * 180.0 / M_PI);
        }
      }
    }
    else
    {
      for (i = 0; i < posesCnt; i++)
      {
        hd = poses[i].h * 180.0 / M_PI;
        fprintf(dst, "driveon %.2f %.2f %.1f @v0.7 : ($targetdist < 1.0)\n", poses[i].x + offsetX, poses[i].y + offsetY, hd);
      }
    }
    // last point too
    if (posesCnt == 0)
    {
      h = atan2(dy12, dx12);
      hd = limitTo2Pi(M_PI/2.0 - h) * 180.0 / M_PI;
      fprintf(dst, "driveon %.2f, %.2f, %.1f @v0.5 : ($targetdist < 0.0)\n", po2.x + offsetX, po2.y + offsetY, h * 180.0 / M_PI);
    }
    // stop and save log
    fprintf(dst, "set \"hakoenginespeed\" 800\n");
    fprintf(dst, "fwd 1 @v0.2 : ($targetdist < 0.0) | ($odovelocity < 0.3)\n");
    fprintf(dst, "idle\n");
    fprintf(dst, "control \"savelog\" \"mrc_%s.logg\"\n", p1);
    isOK = true;
  }
  else
    printf("Destnation file is not open! - no plan written\n");

  //
  return isOK;
}

////////////////////////////////////////

bool KmlPoly::makeRoutePlanRule(FILE * fd, double offsetX, double offsetY, bool isClosed, const char * name, bool inUTM)
{
  bool isOK = false;
  FILE *dst = fd;
  PoseQ po0, po1, po2;
  double lat = 0.0, lon = 0.0;
  int i;
  const int zone = 32;
  const int wgs84 = 23;
  bool holdLine;
  const char * polyName = kmlname;
  //
  if (polyName == NULL)
    polyName = kmlname;
  if (dst != NULL)
  { // write rule prolog
    fprintf(dst, "<rule name=\"%sPoly\" run=\"true\">\n", polyName);
    // offset remark
    fprintf(dst, "# converted from KML - offset by %.2fE, %.2fN\n", offsetX, offsetY);
    fprintf(dst, "#           map part - offset by %.2fE, %.2fN\n", offsetX - OX, offsetY - OY);
    // delete old plan
    fprintf(dst, "poly.del(name())\n");
    // add route points
    if (posesCnt == 0)
    {
      for (i = 0; i < pktCnt; i += 3)
      {
        lon = pkt[i];
        lat = pkt[i+1];
        holdLine = false;
        //fprintf(dst, "# to lat %.6f deg and long %.6f deg:\n", lat, lon);
        LLtoUTM(wgs84, lat, lon, &po2.y, &po2.x, zone);
        fprintf(dst, "# poly.addPoint(name(), %.2f, %.2f)\n", po2.x + offsetX, po2.y + offsetY);
        if (not inUTM)
        {
          po2.x += offsetX - OX;
          po2.y += offsetY - OY;
        }
        fprintf(dst, "poly.addPoint(name(), %.2f, %.2f)\n", po2.x, po2.y);
        if (i > 0)
          // add to map
          fprintf(dst, "mapbase.addMapLine('%s_%d', %.2f, %.2f, %.2f, %.2f, %.2f, %d)\n", polyName, i, po1.x, po1.y, po2.x, po2.y, 0.5, true);
        else
          // save first point
          po0 = po2;
        po1 = po2;
      }
      if (isClosed and i > 2*3)
        // add the closing line to map too
        fprintf(dst, "mapbase.addMapLine('%s_%d', %.2f, %.2f, %.2f, %.2f, %.2f, %d)\n", polyName, i, po1.x, po1.y, po0.x, po0.y, 0.5, true);
    }
    else
    { // Poses already in map coordinates
      for (i = 0; i < posesCnt; i++)
      {
        fprintf(dst, "poly.addPoint(name(), %.2f, %.2f)\n", poses[i].x, poses[i].y);
      }
    }
    //
    // last coordinate system and closed or not
    fprintf(dst, "poly.setRefCoord(name(), 2) # map coordinate system\n");
    fprintf(dst, "# poly.setRefCoord(name(), 1) # UTM coordinate system\n");
    if (isClosed)
      fprintf(dst, "poly.setClosed(name()) # polygon\n");
    else
      fprintf(dst, "poly.setOpen(name()) # polyline\n");
    fprintf(dst, "# ends at lat %.6f deg and long %.6f deg\n", lat, lon);
    // rule end
    fprintf(dst, "</rule>\n");
    fprintf(dst, "\n");
    isOK = true;
  }
  else
    printf("Destination file is not open! - no plan written\n");
  //
  return isOK;
}

////////////////////////////////////////////////////////////

bool KmlPoly::modifyPath(int rowCnt, double rowSep, double minTurnDiam,
                         double hDist, bool useUTM)
{
  PoseQ pa, pb, pc, pd, pe, pf;
  double lat, lon;
  const int zone = 32;
  const int wgs84 = 23;
  const int MLL = 80;
  int lBest[MLL];
  double h, h2, d, dd, dBest = 1e100;
  int lii[MLL];
  int lr[MLL];
  bool lu[MLL];
  int i, j, m, n = 0;
  bool finished = false;
  bool found = false, found1 = false, found2 = false;
  //
  // start with first available line
  for (i = 0; i < rowCnt; i++)
    lii[i] = 0;
  // try all combinations
  while (not finished)
  { // associate available line number in lii with line index
    for (i = 0; i < rowCnt; i++)
      lu[i] = false;
    lu[0] = true;
    lr[0] = 0;  // first line is always 0
    for (j = 1; j < rowCnt; j++)
    {
      m = 0;
      for (i = 0; i < rowCnt; i++)
      { // find the available row number
        if (not lu[i])
        { // line is not traversed yet
          if (lii[j] == m)
          { // line index found for this srquence
            lr[j] = i;
            lu[i] = true;
            break;
          }
          else
            m++;
        }
      }
    }
    // calculate score
    d = 0.0;
    found = true;
    for (i = 0; i < rowCnt - 1; i++)
    {
      dd = absi(lr[i+1] - lr[i]);
      if (dd * rowSep < minTurnDiam)
      { // not usefull
        found = false;
        break;
      }
      // add headland distance plus extra for turn (2*90 deg with min turn diameter)
      d += dd * rowSep + (minTurnDiam * M_PI / 2.0 - minTurnDiam);
    }
    if (found and d < dBest)
    {
      dBest = d;
      for (j = 0; j < rowCnt; j++)
        lBest[j] = lr[j];
      found2 = true;
    }
    // advance
    finished = dBest < 1e99;
    for (i = rowCnt - 2; i > 0; i--)
    {
      if (lii[i] < rowCnt - i - 1)
      {
        lii[i]++;
        finished = false;
        break;
      }
      else
      {
        lii[i] = 0;
      }
    }
    n++;
    if (found)
      found1 = true;
    if (n % 10000 == 0)
    {
      if (found2)
        printf("B");
      else if (found1)
        printf("*");
      else
        printf(".");
      found1 = false;
      found2 = false;
    }
    if (n % 1000000 == 0)
      printf("\nTried %d paths:", n);
  }
  printf("\nTried %d paths in total:", n);
  for (i = 0; i < rowCnt; i++)
    printf("%d ", lBest[i]);
  printf("\n");
  //
  posesCnt = 0;
  if (finished)
  {
    lon = pkt[0];
    lat = pkt[1];
      //fprintf(dst, "# to lat %.6f deg and long %.6f deg:\n", lat, lon);
    LLtoUTM(wgs84, lat, lon, &pa.y, &pa.x, zone);
    if (not useUTM)
    {
      pa.x -= OX;
      pa.y -= OY;
    }
    // next
    lon = pkt[3];
    lat = pkt[4];
      //fprintf(dst, "# to lat %.6f deg and long %.6f deg:\n", lat, lon);
    LLtoUTM(wgs84, lat, lon, &pb.y, &pb.x, zone);
    if (not useUTM)
    {
      pb.x -= OX;
      pb.y -= OY;
    }
    // and heading
    h = atan2(pb.y - pa.y, pb.x - pa.x);
    pa.h = h + M_PI / 2.0;
    if (pa.h > M_PI)
      pa.h -= 2.0 * M_PI;
    pb.h = pa.h;
    h2 = h + M_PI;
    if (h2 > M_PI)
      h2 -= 2.0 * M_PI;
    //
    pc.clear();
    for (i = 0; i < rowCnt; i++)
    {
      pc.x = lBest[i] * rowSep;
      pc.y = 0.0;
      if (i % 2 == 0)
      {
        pd = pa.getPoseToMap(pc.x, pc.y);
        pd.h = h;
        pe = pb.getPoseToMap(pc.x, pc.y);
        pe.h = h;
        if (i < rowCnt - 1)
        {
          pc.x = (lBest[i+1] + lBest[i]) / 2.0 * rowSep;
          pc.y = -hDist;
          pf = pb.getPoseToMap(pc.x, pc.y);
        }
      }
      else
      {
        pd = pb.getPoseToMap(pc.x, pc.y);
        pd.h = h2;
        pe = pa.getPoseToMap(pc.x, pc.y);
        pe.h = h2;
        if (i < rowCnt - 1)
        {
          pc.x = (lBest[i+1] + lBest[i]) / 2.0 * rowSep;
          pc.y = hDist;
          pf = pa.getPoseToMap(pc.x, pc.y);
        }
      }
      if (i < rowCnt - 1)
      {
        if (lBest[i+1] > lBest[i])
          // moving away from first (left)
          pf.h = pa.h;
        else
        { // moving right 
          pf.h = pa.h + M_PI;
          if (pf.h > M_PI)
            pf.h -= M_PI;
        }
      }
      poses[posesCnt++] = pd;
      poses[posesCnt++] = pe;
      if (i < rowCnt - 1)
        poses[posesCnt++] = pf;
    }
  }
  return finished;
}


