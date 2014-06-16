/***************************************************************************
 *   Copyright (C) 2008 by Christian Andersen                              *
 *   chrand@mail.dk                                                        *
 * converted to c++ from Søren Hansen, s021751 and Peter Tjell,  s032041   *
 * matlab files                                                            *
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


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <iostream>
#include <cstdlib>
#include <ugen4/ucommon.h>
#include <ugen4/utime.h>
#include <math.h>
#include <string.h>
#include <expat.h>

#include "graph.h"
#include "kmlpoly.h"

using namespace std;



// convert function
/**
 * Open source and destination files */
bool openSourceFile(const char * src, FILE ** fs);
/**
 * Open source and destination files */
bool openDestFile(const char * dst, FILE ** fd);
/**
 * make kml file format and process all lines in source file */
bool processFiles(const char * name,
                  FILE * fs, FILE * fd,
                  bool doLine, double headingTick,
                  int eastCol, int northCol, int modeCol, int headingCol, PoseQ ref);
/**
 * Coordinate conversion */
/*bool utm2latlon(double easting, double northing, int zone,
                double * latitude, double * longitude);*/
/*bool getCmdLineOptions(int argc, char *argv[],
                       char * sourceName,  char * destName,
                       int * northCol, int * eastCol, int * modeCol);*/
/**
 * find a usefull utmPose to use as reference */
PoseQ getRefPose(const char * ref);


/** poses or line points loaded from XML-map file */
const int MPC = 1000;
PoseQ poses[MPC]; // map positions and poses
int posesCnt = 0; // count of map positions and poses
int lineNum = 100; // line number
double lineW; // line width from map perimeter value
int zone = 32;

/**
 * Load map to to the global variable poses[] */
void mapload(FILE * fptr);
/**
 * call back function used by expat */
void startMapTag(void *userData, const XML_Char *el, const XML_Char **attr);

Graph * graph = NULL;
KmlPoly * kmlpoly = NULL;
double offsetX = 0.0;
double offsetY = 0.0;

//LatLong- UTM conversion..h
//definitions for lat/long to UTM and UTM to lat/lng conversions


void printHelp()
{ // print help
  printf("----------------------- utmpose2kml, version 685 ------------------\n");
  printf("Usage: utmpose2kml [--help] [sourcefile [destinationFile [utmPoseFile]]] [-eN] [-nN] [-mN]\n");
  printf("Converts from (to) UTM data, assumed to be in zone 32\n");
  printf("Converts to (from) a lat-long based Google-earth map overlay file\n");
  printf("Default sourcefile      is utmPose.log\n");
  printf("Default destinationFile is utmPose.kml\n");
  printf("Default utmPoseFile is none, if included, then the first fix pose is used as reference (intended for odoPose)\n");
  printf("Option -eN          use column N as easting (default is 2 - second column)\n");
  printf("Option -nN          use column N as northing (default is 3)\n");
  printf("Option -hN          use column N as heading (default is 4) - for heading ticks only)\n");
  printf("Option -mN          use column N as fix mode (assumes 0=old (gray), \n"
      "                    1=no fix (red), 2=float (magenta), 3=fix (blue), 4=dgps (cyan))\n");
  printf("Option -cN          (not with option m) line colour 0=gray, \n"
         "                    1=red, 2=magenta, 3=blue, 4=cyan\n");
  printf("   NB!  max number of columns is 16\n");
  printf("Option -vL          paint heading tick vector every L meters)\n");
  printf("--\n");
  printf("Option -a           source file is an XML map: file with lines and points - to kml-file\n");
  printf("Option -g           source file is an XML graph: file with nodes and edges - edges are used only - to kml file\n");
  printf("Option -d [-c]      source file is an KML graph: converts to UTM path and drive rule (closed if -c option)\n");
  printf("Option -xX          Destination is offset X (easting) meters (with option -d only)\n");
  printf("Option -yY          Destination is offset Y (northing) meters (with option -d only)\n");
  printf("Option -rN -sM [-u] [-U]  with -d only, from first 2 points,\n"
         "                    -rN make N (>= 5) parallel drive lines\n"
         "                    -sM lines are to be spaced M meters,\n"
         "                    -u for UTM drive rather than map drive,\n"
         "                    -U make as MRC script rather than rule\n");
  printf("Option              assumes minimum turn diameter of 5.5m and extra drives are to the left\n");
  printf("Option -Nname       give rule or KML-shape this 'name' (in the destination file), else the name from the kml-file,\n");
  printf("Option -zZone       uses Zone for conversion to LL (default is %d)\n", zone);
  printf("--\n");
  printf("If easting is less than 20.000, then %f is added (map, graph and odometry)\n", OX);
  printf("If northing is less than 20.000, then %f is added (map, graph and odometry)\n", OY);
  printf("Option --help    prints this help text\n\n");
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////



// main
int main(int argc, char *argv[])
{
  int i;
  const char * src = NULL;
  const char * dst = NULL;
  const char * ref = NULL;
  bool isHelp = false;
  bool doLine = true;
  bool isXml = false;
  bool isGraph = false;
  bool isKmlToUtmDrive = false;
  int eastCol = 2; // first column is 0
  int northCol = 3;
  int headingCol = 4;
  int modeCol = 6;
  double headingTick = -1.0;
  char * p1, *p2;
  FILE *fs = NULL, *fd = NULL;
  bool isOK = false;
  PoseQ refPose;
  bool isClosed = false;
  double rowSep = 3.5;
  int rowCnt = 0;
  const char * name = NULL;
  bool inUTM = false;
  bool asMRC = false;
  //
  for (i = 1; i < argc; i++)
  {
    if (argv[i][0] > ' ' and argv[i][0] != '-')
    { // assume a legal argument
      if (src == NULL)
      { // first argument is source
        src = argv[i];
      }
      else if (dst == NULL)
      { // destination filename
        dst = argv[i];
      }
      else
        ref = argv[i];
    }
    else if (argv[i][0] > ' ')
    {
      p1 = &argv[i][1];
      if (*p1 == 'e')
        eastCol = strtol(++p1, NULL, 0);
      else if (*p1 == 'z')
        zone = strtol(++p1, NULL, 0);
      else if (*p1 == 'n')
        northCol = strtol(++p1, NULL, 0);
      else if (*p1 == 'm')
        modeCol = strtol(++p1, NULL, 0);
      else if (*p1 == 'c')
        modeCol = -strtol(++p1, NULL, 0);
      else if (*p1 == 'h')
        headingCol = strtol(++p1, NULL, 0);
      else if (*p1 == 'v')
      {
        headingTick = strtof(++p1, &p2);
        if (p2 == p1)
          // default value
          headingTick = 1.0;
      }
      else if (*p1 == 'a') // XML map file
        isXml = true;
      else if (*p1 == 'g') // XML graph file
        isGraph = true;
      else if (*p1 == 'd')
        isKmlToUtmDrive = true;
      else if (*p1 == 'x')
        offsetX = strtof(++p1, NULL);
      else if (*p1 == 'y')
        offsetY = strtof(++p1, NULL);
      else if (*p1 == 'r')
        rowCnt = strtol(++p1, NULL, 0);
      else if (*p1 == 's')
        rowSep = strtof(++p1, NULL);
      else if (*p1 == 'N')
        name = p1 + 1;
      else if (*p1 == 'c')
        isClosed = true; 
      else if (*p1 == 'u')
        inUTM = true;
      else if (*p1 == 'U')
        asMRC = true;
      else if (*p1 == '-')
        isHelp = true;
      else
      {
        isHelp = true;
        printf("Unknown option %s\n\n", p1);
      }
    }
  }
  if (isHelp)
  { /* print help */
    printHelp();
  }
  else
  { // do some conversion
    isOK = openSourceFile(src, &fs);
    if (isOK)
    { // source and destination file is OK
      if (isGraph)
      { // is a graph, load it as new source
        graph = new Graph();
        isOK = graph->graphload(fs);
        fclose(fs);
        fs = NULL;
      }
      else
      { // may need a reference pose from a logfile
        if (ref != NULL)
          refPose = getRefPose(ref);
        if (isXml)
        {
          mapload(fs);
          fclose(fs);
          fs = NULL;
        }
      }
      if (isKmlToUtmDrive)
      { // destination is not a kml file, but a text-file
        kmlpoly = new KmlPoly();
        kmlpoly->load(fs);
        if (rowCnt >= 5)
        {
          kmlpoly->modifyPath(rowCnt, rowSep, 5.56, 5.0, inUTM);
        }
        if (dst != NULL)
          openDestFile(dst, &fd);
        if (dst != NULL)
        {
          if (asMRC)
            // make MRC script in kalman UTM coordinates
            isOK = kmlpoly->makeRouteMRC(fd, offsetX, offsetY, name);
          else
          {
            fprintf(fd, "%s\n", "<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
            isOK = kmlpoly->makeRoutePlanRule(fd, offsetX, offsetY, isClosed, name, inUTM);
            if (not isClosed)
              isOK = kmlpoly->makeRouteRule(fd, offsetX, offsetY, name, inUTM);
          }
        }
      }
      else
      { // make a kml file
        isOK = openDestFile(dst, &fd);
        if (isOK)
        {
          /* bool processFiles(const char * name,
          FILE * fs, FILE * fd,
          bool doLine, double headingTick,
          int eastCol, int northCol, int modeCol, int headingCol, PoseQ ref)
          */
          if (name == NULL)
            name = dst;
          if (name == NULL)
            name = "mobotware path";
          isOK = processFiles(name, fs, fd, doLine, headingTick, eastCol, northCol, modeCol, headingCol, refPose);
        }
      }
    }
    if (fs != NULL)
      fclose(fs);
    if (fd != NULL)
      fclose(fd);
  }
  if (isOK)
    printf("Finished OK\n");
  else
    printf("Failed\n");
  //
  if (graph != NULL)
    delete graph;
  if (kmlpoly != NULL)
    delete kmlpoly;
  
  return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////

bool openSourceFile(const char * src, FILE ** fs)
{
  const char * dsrc = "utmPose.log";
  const char * sfn = src;
  bool result = false;
  //
  if (sfn == NULL)
    sfn = dsrc;
  *fs = fopen(sfn, "r");
  if (*fs == NULL)
    printf("sourcefile not found: %s\n", src);
  else
  {
    result = true;
    printf("Reading from file '%s'\n", sfn);
  }
  return result;
}

/////////////////////////////////////////////////

bool openDestFile(const char * dst, FILE ** fd)
{
  const char * ddest = "utmPose.kml";
  const char * dfn = dst;
  bool result = false;
  //
  if (dfn == NULL)
    dfn = ddest;
  *fd = fopen(dfn, "w");
  if (*fd == NULL)
    printf("Could not create destination file: %s\n", dfn);
  else
  {
    result = true;
    printf("Created destination file '%s'\n", dfn);
  }
  return result;
}

/////////////////////////////////////////////////

Edge * nextEdge(Node ** node, Edge * edge)
{
  Edge * result = NULL;
  Node * nn = *node;
  //
  if (edge != NULL)
    result = edge->next;
  if (result == NULL)
  { // we need next node - child, sister or oncle
    do
    {
      if (nn->child != NULL)
      {
        nn = nn->child;
        break;
      }
      if (nn->sister != NULL)
      {
        nn = nn->sister;
        break;
      }
      while (nn->parent != NULL)
      {
        if (nn->parent->sister != NULL)
        {
          nn = nn->parent->sister;
          break;
        }
        nn = nn->parent;
      }
    } while (nn->parent != NULL);
    result = nn->edge;
    if (nn->parent != NULL and result == NULL)
      result = nextEdge(&nn, result);
    *node = nn;
  }
  return result;
}

///////////////////////////////////////////////////////////////////////////

bool processFiles(const char * name,
                  FILE * fs, FILE * fd,
                  bool doLine, double headingTick,
                  int eastCol, int northCol, int modeCol, int headingCol, PoseQ ref)
{
  //const int zone = 32;
  const char * header = // "<Document><name>utmPose.kml</name>\n"
      "<Style id=\"sn_donut\">" "<IconStyle><color>ff1effdd</color><scale>0.4</scale>\n"
      "<Icon><href>http://maps.google.com/mapfiles/kml/shapes/donut.png</href></Icon></IconStyle></Style>\n" 
      "<Style id=\"sh_donut\"><IconStyle><color>ff1effdd</color><scale>0.52</scale>\n"
      "<Icon><href>http://maps.google.com/mapfiles/kml/shapes/donut.png</href></Icon></IconStyle></Style>\n" 
      "<Style id=\"utmLine0\"><LineStyle>\n"
      "<color>7f1f1f1f</color><width>3</width>"
      "</LineStyle></Style>\n" 
      "<Style id=\"utmLine1\"><LineStyle>\n"
      "<color>7f0000ff</color><width>4</width>"
      "</LineStyle></Style>\n"
      "<Style id=\"utmLine2\"><LineStyle>\n"
      "<color>7f9f009f</color><width>4</width>"
      "</LineStyle></Style>\n" 
      "<Style id=\"utmLine3\"><LineStyle>\n"
      "<color>7fff0000</color><width>4</width>"
      "</LineStyle></Style>\n" 
      "<Style id=\"utmLine4\"><LineStyle>\n"
      "<color>7f7f7f00</color><width>4</width>"
      "</LineStyle></Style>\n"
      "<StyleMap id=\"msn_donut\"><Pair><key>normal</key><styleUrl>#sn_donut</styleUrl></Pair>"
      "<Pair><key>highlight</key><styleUrl>#sh_donut</styleUrl></Pair></StyleMap>\n";
  const char * footer="</Document>";
  const int MSL = 200;
  char s[MSL];
  int m = 0, l = 0;
  double lat = 0.0, lon = 0.0, lat2 = 0.0, lon2 = 0.0;
  double q2 = 0;
  const int MCL = 17;
  double c[MCL];
  int minCol;
//  bool isOK;
  const char * lin0Hdr = "<Placemark><name>RTK-GPS old</name>\n"
      "<styleUrl>#utmLine0</styleUrl>\n"
      "<LineString><coordinates>\n";
  const char * lin1Hdr = "<Placemark><name>RTK-GPS autonomus</name>\n"
      "<styleUrl>#utmLine1</styleUrl>\n"
      "<LineString><coordinates>\n";
  const char * lin2Hdr = "<Placemark><name>RTK-GPS float</name>\n"
      "<styleUrl>#utmLine2</styleUrl>\n"
      "<LineString><coordinates>\n";
  const char * lin3Hdr = "<Placemark><name>RTK-GPS rtk-fix</name>\n"
      "<styleUrl>#utmLine3</styleUrl>\n"
      "<LineString><coordinates>\n";
  const char * lin4Hdr = "<Placemark><name>RTK-GPS dgps</name>\n"
      "<styleUrl>#utmLine4</styleUrl>\n"
      "<LineString><coordinates>\n";
  const char * lin100Hdr = "<Placemark><name>row-line</name>\n"
      "<styleUrl>#utmLine1</styleUrl>\n"
      "<LineString><coordinates>\n";
  const char * lin200Hdr = "<Placemark><name>edge</name>\n"
      "<styleUrl>#utmLine2</styleUrl>\n"
      "<LineString><coordinates>\n";
  const char * linEnd = "</coordinates></LineString></Placemark>\n";
  PoseQ pose, lastPose;
  double latH, lonH, d;
  bool poseValid;
  Node * node = NULL;
  Edge * edge = NULL, *edgeLast = NULL;
  int edgeCnt = 0;
  char * p1, *p2;
  bool result = false;
  bool headingTicNow = false;
  bool firstPoint = true;
  //
  // use destination file as name
  fprintf(fd, "<Document><name>%s</name>\n", name);
  // remaining header
  fprintf(fd, "%s\n", header);
  if (false and doLine)
    fprintf(fd, "<Placemark><name>RTK-GPS HAKO Pometet</name>\n"
        "<styleUrl>#utmLine1</styleUrl>\n"
        "<LineString><coordinates>\n");
  if (eastCol >= MCL or eastCol < 1)
    eastCol = 2;
  if (northCol >= MCL or northCol < 1)
    northCol = 3;
  if (modeCol >= MCL)
    modeCol = 6;
  minCol = maxi(maxi(modeCol, northCol), eastCol);
  if (graph != NULL)
  {
    node = &graph->rootNode;
    edge = node->edge;
    if (edge == NULL)
      edge=nextEdge(&node, edge);
  }
  // default if no quality to monitor
  if (modeCol < 1)
    pose.q = -modeCol;
  while (true)
  {
    poseValid = false;
    if (fs != NULL)
    {
      if (feof(fs))
        // no more data
        break;
      p1 = fgets(s, MSL, fs);
      if (isdigit(s[0]))
      {
        m = sscanf(s, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                  &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12], &c[13], &c[14], &c[15], &c[16]);
        if (m > minCol - 2)
        {
          lat2 = lat;
          lon2 = lon;
          pose.set(c[eastCol], c[northCol], c[headingCol]);
          poseValid = true;
          if (modeCol > 0)
            pose.q = c[modeCol];
        }
      }
      else if (p1 != NULL)
      { // assumed to be a rule - defining a polyline
        // look for: poly.addPoint(namepp, 707884.99, 6174119.18)
        p2 = strstr(p1, "poly.addPoint");
        if (p2 != NULL)
          p2 = strchr(p2, ',');
        if (p2 != NULL)
        {
          p2++;
          pose.x = strtof(p2, &p2);
          p2++;
          pose.y = strtof(p2, &p2);
          pose.h = 0;
          pose.q = 4;
          poseValid = true;
        }
      }
    }
    else if (l < posesCnt)
    { // this is a map
      pose = poses[l];
      poseValid = true;
    }
    else if (node != NULL)
    { // this is a graph with edges
      if (edge != NULL)
      { // there is an unpainted edge
        if (edge != edgeLast)
        { // get edge start position
          pose = edge->node1->getMapPose(edge->conn1);
          edgeLast = edge;
          pose.q = edgeCnt + 100;
        }
        else
        { // get edge end position
          pose = edge->node2->getMapPose(edge->conn2);
          edge=nextEdge(&node, edge);
          pose.q = edgeCnt + 100;
          edgeCnt++;
        }
      }
      else
        // finished
        break;
      poseValid = (edge != NULL);
    }
    else
      break;
    if (poseValid)
    {
      result = true;
      if (ref.y > 20000)
      { // move relative to ref pose
        pose = ref.getPoseToMapPose(pose);
      }
      else if (pose.y < 20000.0)
      {
        pose.y += OY;
        pose.x += OX;
      }
      // convert to lat-long
      UTMtoLL(23, pose.y, pose.x, zone, &lat,  &lon);
      //
      if (firstPoint)
      {
        lat2 = lat;
        lon2 = lon;
      }
      if (headingTick > 0)
      {
        d = pose.getDistance(lastPose);
        if (d > headingTick)
        { // skip big jumps
          if (d < 10.0 * headingTick)
          { // advance in heading direction
            lastPose.x = pose.x + cos(pose.h) * headingTick * 0.5;
            lastPose.y = pose.y + sin(pose.h) * headingTick * 0.5;
            // to lat - long
            UTMtoLL(23, lastPose.y, lastPose.x, zone, &latH,  &lonH);
            headingTicNow = true;
          }
          lastPose = pose;
        }
      }
      //
      if ((doLine and (l > 0 or fs == NULL or edge != NULL)) or firstPoint)
      {
        if ( pose.q != q2 or firstPoint)
        { // change line stype
          if (pose.q < q2 and pose.q < 100.0 and not firstPoint)
            // improved quality - add new point to lower quality line
            fprintf(fd, "%.7f,%.7f,0\n", lon, lat);
          //
          if (not firstPoint)
            fprintf(fd, "%s", linEnd);
          if (pose.q <= 0.5)
            // old position
            fprintf(fd, "%s", lin0Hdr);
          else if (pose.q <= 1.01)
            // autonomous
            fprintf(fd, "%s", lin1Hdr);
          else if (pose.q <= 2.01)
            // floating
            fprintf(fd, "%s", lin2Hdr);
          else if (pose.q <= 3.01)
            // FIX
            fprintf(fd, "%s", lin3Hdr);
          else if (pose.q <= 4.1)
            // DGPS
            fprintf(fd, "%s", lin4Hdr);
          else if (edge == NULL)
            // map line
            fprintf(fd, "%s", lin100Hdr);
          else
            // graph edge
            fprintf(fd, "%s", lin200Hdr);
          // repeat first point
          if ((pose.q > q2 and pose.q < 100) or firstPoint)
            // worse quality - add last good point to worse line
            fprintf(fd, "%.7f,%.7f,0\n", lon2, lat2);
        }
        firstPoint = false;
      }
      q2 = pose.q;
      //bool utm2latlon(double easting, double northing, int zone,
      //                double * latitude, double * longitude)
      //isOK = utm2latlon(e, n, zone, &lat, &lon);
      if (doLine)
      {
        fprintf(fd, "%.7f,%.7f,0\n", lon, lat);
        if (headingTicNow)
        { // add at heading direction line (in current colour)
          fprintf(fd, "%.7f,%.7f,0\n", lonH, latH);
          fprintf(fd, "%.7f,%.7f,0\n", lon, lat);
          headingTicNow = false;
        }
      }
      else
      {
        fprintf(fd, "<Placemark><styleUrl>#msn_donut</styleUrl><Point>"
            "<coordinates>%.7f,%.7f,0.0</coordinates>"
                "</Point></Placemark>\n", lon, lat);
      }
    }
    l++;
  }
  if (doLine)
    fprintf(fd, "%s", linEnd);
  fprintf(fd, "%s\n", footer);
  printf("finished %d lines\n", l);
  return result;
}

//////////////////////////////////////////////

// bool utm2latlon(double easting, double northing, int zone,
//                 double * latitude, double * longitude)
// {
//   double x = easting - 500000.0; // signed in meters from central meridian
//   double y = northing;
//   // center meridian in radians
//   double zone_CM = (6.0 * zone - 183.0) * M_PI / 180.0;
//    //Datum constants
//   // NAD83/WGS84  6,378,137  6,356,752.3142  1/298.257223563  Global
//   double a = 6378137.0;  //equatorial radius
//   double b = 6356752.3142;// polar radius
//   // GRS 80  6,378,137  6,356,752.3141  1/298.257222101  US
// // US  double a = 6378137.0;  // WGS84 equatorial radius
// // US  double b = 6356752.314;// polar radius
//   // Airy 1830  6,377,563.4 6,356,256.9 1/299.32 Great Britain
// //  double a = 6377563.4;  //equatorial radius
// //  double b = 6356256.9;// polar radius
//   double k0 = 0.9996;    //Scale factor - along longitude 0
//   double e = sqrt(1.0 - sqr(b) / sqr(a)); // excentricity ~ 0.08
//   // meditorial arc
//   double M = y/k0;
//   // footprint latitude
//   double mu = M/(a*(1.0 - sqr(e)/4.0 - 3.0 * pow(e,4) / 64.0 -
//         5.0 * pow(e,6) / 256.0));
//   double e1 = (1.0 - sqrt(1.0 - sqr(e)))/(1.0 + sqrt(1.0 - sqr(e)));
//   double j1 = (3.0 * e1 / 2.0 - 27.0 * pow(e1, 3) / 32.0);
//   double j2 = (21.0 * sqr(e1) / 16.0 - 55.0 * pow(e, 4) / 32.0);
//   double j3 = (151.0 * pow(e1, 3) / 96.0);
//   double j4 = (1097.0 * pow(e1, 4) / 512.0);
//   double fp = mu + j1 * sin(2.0 * mu) + j2 * sin(4.0 * mu) +
//         j3 * sin(6.0 * mu) + j4 * sin(8.0 * mu);
//   // and now lat-long
//   double e2 = sqr(e * a / b);
//   double c1 = e2 * sqr(cos(fp));
//   double t1 = sqr(tan(fp));
//   double r1 = a * (1.0 - sqr(e))/pow(1.0 - sqr(e) * sqr(sin(fp)), 3.0/2.0);
//   double n1 = a / sqrt(1.0 - sqr(e) * sqr(sin(fp)));
//   double d  = x / (n1 * k0);
//   double q1 = n1 * tan(fp) / r1;
//   double q2 = sqr(d)/2.0;
//   double q3 = (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * sqr(c1) - 9.0 * e2) *
//         pow(d, 4) / 24.0;
//   double q4 = (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * sqr(t1) -
//         3.0 * sqr(c1) - 252.0 * e2) * pow(d, 6) / 720.0;
//   double q5 = d;
//   double q6 = (1.0 + 2.0 * t1 + c1) * pow(d, 3) / 6.0;
//   double q7 = (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * sqr(c1) +
//         8.0 * e2 + 24.0 * sqr(t1)) * pow(d, 5) / 120.0;
//   double lat = fp - q1*(q2 - q3 + q4);
//   double lon = zone_CM + (q5 - q6 + q7) / cos(fp);
//   //
//   *latitude = lat * 180.0 / M_PI;
//   *longitude = lon * 180.0 / M_PI;
//   //
//   return true;
// }







//LatLong- UTM conversion.cpp
//Lat Long - UTM, UTM - Lat Long conversions

// #include <math.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include "constants.h"
// #include "LatLong-UTMconversion.h"


/*Reference ellipsoids derived from Peter H. Dana's website- 
             http://www.utexas.edu/depts/grg/gcraft/notes/datum/elist.html
             Department of Geography, University of Texas at Austin
             Internet: pdana@mail.utexas.edu
             3/22/95

             Source
             Defense Mapping Agency. 1987b. DMA Technical Report: Supplement to Department of Defense World Geodetic System
             1984 Technical Report. Part I and II. Washington, DC: Defense Mapping Agency
*/




//////////////////////////////////////////////////

PoseQ getRefPose(const char * ref)
{
  PoseQ result;
  FILE * rf;
  const int MSL = 500;
  char s[MSL];
  double t,x,y,h,v,q;
  int n, i = 0;
  char * p1;
  //
  rf = fopen(ref, "r");
  if (rf == NULL)
    printf("Failed to open reference file\n");
  else
  {
    while (not feof(rf))
    {
      p1 = fgets(s, MSL, rf);
      if (p1 != NULL)
      {
        n = sscanf(s, "%lf %lf %lf %lf %lf %lf", &t, &x, &y, &h, &v, &q);
        if (n==6 and q==3 and v > 0.15)
        {
          i++;
          if (i > 30)
          {
            result.set(x,y,h);
            break;
          }
        }
      }
    }
    if (result.x < 20000)
      printf("failed to find a valid pose, found (%.1fx,%.1fy,%.1fh)\n", result.x, result.y, result.h);
    else
      printf("Ref = (%.1fx,%.1fy,%.1fh)\n", result.x, result.y, result.h);
    fclose(rf);
  }
  return result;
}

////////////////////////////////////////////////

void mapResave()
{
  FILE * fd;
  int i;
  PoseQ * p = poses;
  double ofE = 120.0;
  double ofN = 249.4;
  int q2 = 0;
  
  fd = fopen("ku_life_offset.xml", "w");
  if (fd != NULL)
  {
    fprintf(fd, "<?xml version=\"1.0\" ?>\n");
    fprintf(fd, "<!-- offset by a constant using utmpose2kml -x -->>\n");
    fprintf(fd, "<map nodename=\"taastrup-offset%.3fE and %.3fN\">\n", ofE, ofN);
    for (i = 0; i< posesCnt; i++)
    {
      p->x += ofE;
      p->y += ofN;
      if (int(p->q + 0.5) != q2)
      {
        q2 = int(p->q + 0.5);
        fprintf(fd, "  <line number=\"%d\" perimeter=\"%g\">\n", q2, p->w);
        fprintf(fd, "    <start x=\"%.3f\" y=\"%.3f\"/>\n", p->x, p->y);
      }
      else
      {
        fprintf(fd, "    <end x=\"%.3f\" y=\"%.3f\"/>\n", p->x, p->y);
        fprintf(fd, "  </line>\n");
      }
      q2 = int(p->q + 0.5);
      //
      p++;
    }
    fprintf(fd, "</map>\n");
    fclose(fd);
  }
}

////////////////////////////////////////////////////////////////

void mapload(FILE * fptr)
{
  const int BUFFSIZE = 2000;
  char buffer[BUFFSIZE];
  int done;
  int len;
  XML_Parser p;

  /* Reset mapcounters: */
  posesCnt = 0;
  p = XML_ParserCreate(NULL);
  if (! p)
  {
    fprintf(stderr, "Couldn't allocate memory for parser\n");
    return;
  }
  XML_SetUserData(p, NULL);
  XML_SetStartElementHandler(p, startMapTag);
  //
  do
  {
    len = fread(buffer, 1, BUFFSIZE, fptr);
    printf("Read length = %d\n", len);

    if (ferror(fptr))
    {
      fprintf(stderr, "Read error\n");
      return;
    }
    done = feof(fptr);

    if (! XML_Parse(p, buffer, len, done))
    {
      fprintf(stderr, "Parse error at line %d:\n%s\n", (int) XML_GetCurrentLineNumber(p), XML_ErrorString(XML_GetErrorCode(p)));
      fclose(fptr);
      return;
    }
  } while(!done);

  XML_ParserFree(p);
  printf("loaded %i lines or points successfully.\n", posesCnt);
  //
  // debug
  //mapResave();
}

////////////////////////////////////////////////////////


void startMapTag(void *userData, const XML_Char *el, const XML_Char **attr)
{
  int i;

  if(strcmp(el, "line") == 0)
  {
    for(i = 0; attr[i] != NULL; i+= 2)
    { // get max width of a tree - from center out
      if(strcmp(attr[i], "perimeter") == 0)
        lineW = strtod(attr[i+1], NULL);
    }
    lineNum++;
  }
  else if ((strcmp(el, "start") == 0) or (strcmp(el, "end") == 0))
  {
    for(i = 0; attr[i]; i += 2)
    {
      if(strcmp(attr[i], "x") == 0)
        poses[posesCnt].x = atof(attr[i+1]);
      else if(strcmp(attr[i], "y") == 0)
        poses[posesCnt].y = atof(attr[i+1]);
      else if(strcmp(attr[i], "h") == 0)
        poses[posesCnt].h = atof(attr[i+1]);
    }
    poses[posesCnt].w = lineW;
    poses[posesCnt].q = lineNum;
    if (posesCnt < MPC)
      posesCnt++;
  }
  else if(strcmp(el, "point") == 0)
  {
    for(i = 0; attr[i] != NULL; i+= 2)
    { // get max width of a tree - from center out
      if(strcmp(attr[i], "perimeter") == 0)
        lineW = strtod(attr[i+1], NULL);
    }
    lineNum++;
  }
}

