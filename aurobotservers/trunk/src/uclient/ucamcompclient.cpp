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


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <cstdlib>
#include <stdio.h>
#include <readline/readline.h>
#include <readline/history.h>
#include <string.h>
#include <locale.h>

#ifdef OPENCV2
#include <highgui/highgui_c.h>
#else
#include <opencv/highgui.h>
#endif
#include <ugen4/ucommon.h>
#include <urob4/uclienthandler.h>
#include <urob4/uclientfuncbase.h>
#include <urob4/uclientfuncimage.h>

#include "uclientfuncpath.h"
#include "uclientfuncimggui.h"
#include "uclientfunclasergui.h"
#include "uclientfuncobstgrp.h"
#include "uclientfuncsf.h"

#define __CLIENT_VERSION__ "2.344"
using namespace std;

void printHelp();
void clientCmdLine();
void sendPing(UClientHandler * client);


int main(int argc, char *argv[])
{
  bool ask4help;
  const int MSL = 500;
  char s[MSL];
  bool asDaemon;
  //
  // init openCV highgui (sets locale to native default on PC)
  cvInitSystem(0, NULL );
  // change locale back to standard C - i.e. use '.' as decimal sign.
  setlocale(LC_ALL, "C");
  // alanyze command line parameters
  ask4help = setCommonPathAndOtherOptions(argc, argv, s, MSL,  &asDaemon, 24920, "client");
  if (ask4help)
    printHelp();
  else
  {
    printf("simple client " __CLIENT_VERSION__ ", compiled: " __DATE__ " " __TIME__ "\n");
    //
    clientCmdLine();
    //
    printf(" - Terminated!\n");
  }
  //
  return EXIT_SUCCESS;
}

/////////////////////////////////////////////

void printHelp()
{
  printf("This RAC client can display images and stuff. \n");
  printf("\n");
  printf("Command line options are:\n");
  printf(" -h --help This help message\n");
  printf(" -i --imagepath\n");
  printf("    Sets path where images are expected to be found and written\n");
  printf(" -d --datapath\n");
  printf("    Sets path where data (other than images) are expected to be found and written\n");
  printf(" -c --config <file>\n");
  printf("    Set path and filename for configuration file (default './robcam.conf')\n");
  printf(" -s --server <server host name>\n");
  printf("    sets the default server name (default 'localhost')\n");
  printf(" -p --port <port>\n");
  printf("    sets the default port number, (default 24920)\n");
  printf("\n");
}

/////////////////////////////////////////////

void clientCmdLine()
{
  UClientHandler * client;
  //UClientFuncBase * fBase;
  UClientFuncImgGui * fimage;
  UClientFuncLaserGui * flaser;
  UClientFuncObstGrp * fobst;
  UClientFuncSF * fsf;
//  UClientFuncPath * fpath;
  const int MLL = 500;
  char lineBuff[MLL];
  const int LLSIZE = 1;
  char lineLast[LLSIZE][MLL];
  int i, lineLastNext;
  bool lineLastMatch;
  char capt[MLL];
  char toHost[MLL] = "localhost"; // from command line parameters
  int toPort = serverPort;    // from command line parameters
  int n;
  char * line = NULL;
  const bool useReadline = true; // if false use fgets() for commandline
  bool res;
  double d1, d2;
  const int MRL = 30;
  char name[MRL];
  char val[MRL];
  int focalLine = -1;
  int fuzzyWidth = -1;
  int fuzzyClasses = 3; // number of classes 2 (mouse) 3: 2xmouse + background
  int fuzzyIter = 0; // fuzzy iterations (0 = seed area only)
  int fuzzyLine = -1;
  int fuzzyCol = 100;
  char * p1;
//  UImage * img;
  UPosition pos;
  URotation rot;
  //printf("clientCmdLine cvInitSystem\n");
  cvInitSystem(0, NULL);
  //
  //printf("clientCmdLine client handler\n");
  // create core client service
  client = new UClientHandler();
  client->setVerbose(false);
  client->setNamespace("uclient", "name=\"ucamcompclient\" version=\""
                 __CLIENT_VERSION__ "\"");
  // create image client function
  //printf("clientCmdLine client img gui\n");
  fimage = new UClientFuncImgGui();
  fimage->setVerbose(false);
  // create a path client function
  //printf("clientCmdLine path\n");
  //fpath = new UClientFuncPath();
  //fpath->setVerbose(false);
  // laser client
  //printf("clientCmdLine laser GUI\n");
  flaser = new UClientFuncLaserGui();
  flaser->setVerbose(true);
  flaser->setSaveImages(false);
  // obstacle group decoder
  fobst = new UClientFuncObstGrp();
  // led laser gui know about obstacles
  flaser->setObstList(fobst->getObstHistData());
  // obstacle group decoder
  fsf = new UClientFuncSF();
  // led laser gui know about scan features
  flaser->setSfPool(fsf->getSfPool());
  // add function to service
  //printf("clientCmdLine add-functions\n");
  client->addFunction(fimage);
  //client->addFunction(fpath);
  client->addFunction(flaser);
  client->addFunction(fobst);
  client->addFunction(fsf);
  // start handler of messages - connected or not
  //printf("clientCmdLine client start\n");
  client->start();
  // connect to server
  client->tryHoldConnection = true;
/*  client->tryConnect(toHost, toPort);
  if (not client->isConnected())
    printf("... failed to connect to '%s' port %d\n",
        client->getHost(), client->getPort());*/
  // this call may not be needed
  Wait(0.3);
  printf("Type help for available functions\n");
  // empty lineLast
  for (i = 0; i < LLSIZE; i++)
    lineLast[i][0] = '\0';
  lineLastNext = 0;
  while (true)
  {
    if (useReadline)
    {
      line = readline("## ");
      if (line == NULL)
        continue;
      if (strlen(line) == 0)
        continue;
      // add to history buffer if not used
      // the last LLSIZE times
      for (i = 0; i < LLSIZE; i++)
      { // linelastLast
        lineLastMatch = strcmp(lineLast[i], line) == 0;
        if (lineLastMatch)
          break;
      }
      if (not lineLastMatch)
      {
        add_history(line);
        strncpy(lineLast[lineLastNext], line, MLL);
        lineLastNext = (lineLastNext + 1) % LLSIZE;
      }
      // add e new-line
      snprintf(lineBuff, MLL, "%s\n", line);
      // free buffer allocated by readline()
      delete line;
      line=lineBuff;
    }
    else
    {
      line = lineBuff;
      printf("## ");
      // get line
      p1 = fgets(line, MLL, stdin);
      if (p1 == NULL)
        break;
    }
    if (strncasecmp(line, "quit", 4) == 0)
      break;
    if (strncasecmp(line, "exit", 4) == 0)
      break;
    else if (strncasecmp(line, "print", 5) == 0)
    {
      client->print("UClient");
      printf(" Image path '%s'\n", imagePath);
    }
    else if (strncasecmp(line, "help", 4) == 0)
    {
      printf("Functions in this UCLIENT application:\n");
      printf(" Client management:\n");
      printf(" - print                  Print client status\n");
      printf(" - quit or exit           Terminate\n");
      printf(" - silent, verbose        Message level to console\n");
      printf(" - ping                   Send a time request to server\n");
      printf(" - namespaceUse [true/false] Use namespace openings (default) or not\n");
      printf(" - help                   This\n");
      printf(" For (camera) image display only:\n");
      printf(" - clear                  Remove all windows\n");
      printf(" - UV <caption part>      Make UV analysis of next image\n");
      printf(" - CROMA <caption part>   Make CROMA anlysis of next image\n");
      printf(" - save <caption part> [noshow] Save image (caption.png) and optionally no show\n");
      printf(" - MAX                    Show images in size 800x600\n");
      printf(" - focus <line>           Show focus assist curve (-1 = off)\n");
      printf(" - fuzzy <mask>           Classify. mouse mask radius (-1=off)\n");
      printf(" - fuzzyc true/false      Make class of background too (else just 2)\n");
      printf(" - fuzzyLine -1/line col  Seed classes from line, left<-col->right\n");
      printf(" For Laser scanner image only:\n");
      printf(" - mmrhist N              Display scan history N laser scans\n");
      printf(" - mmrScale hgt [mmrPos]  Height in meter of MMR display (is %gm %gm)\n",
             flaser->getHeight(), flaser->getRobPos());
      printf(" - mmrNosave              Do not save each laserscan image\n");
      printf(" - paint <type> <value>   Type: Bold, Curves, GPS, Road, Segments, Obst\n");
      printf("                          Var, Speed, Path, PathAll, PathLinesAll\n");
      printf("                          visPoly, odoGrid, raw, all (true/false)\n");
      printf("                          mmr smr (selects robot size - approx)\n");
      printf("                          pathHistCnt visPolyCnt scanHistCnt gridSize (value)\n");
      //printf(" - paintpis <caption>     Paint last set of passable lines in this image\n");
      printf(" - save                   Save MMR top view image, and hereafter, saving %s\n",
             bool2str(flaser->getSaveImages()));
      printf(" Paint laserdata in photo - see UClientFuncLaserGui::doImage().\n");
      printf(" - mmrImg                 Paint newest scan camera image (hard coded params)\n");
      printf(" - mmrPosImg imgBMPFile x y z O P K  Paint newest scan in image taken at\n");
      printf("                          this pos (m), rot (deg) (img max size=800x600)\n");
      printf(" Server related commands:\n");
      printf(" - connect [host [port]]  Try connect\n");
      printf(" - <line>\\n               Send 'line' to server\n");
      printf(" - sHelp                  Send help request to server\n");
      printf(" - HUP                    Hang up (disconnect)\n");
      printf("\n");
    }
    else if (strncasecmp(line, "connect", 7) == 0)
    {
      n = sscanf(&line[7], "%s %d", toHost, &toPort);
      client->tryHoldConnection = false;
      if ((toPort >= 31000) and (toPort < 31100))
        // MRC ports
        client->setNamespaceUse(false);
      if ((strcmp(toHost, client->getHost()) == 0) and
           (toPort == client->getPort()) and client->isConnected())
        printf("Already connected to %s:%d\n",
               client->getHost(), client->getPort());
      else
      {
        if (client->isConnected())
        { // close old connection first
          client->tryHoldConnection = false;
          client->closeConnection();
        }
        client->tryHoldConnection = false;
        client->setPort(toPort);
        client->setHost(toHost);
        client->openConnection();
        Wait(0.5);
        if (not client->isConnected())
          printf("Failed to connect to '%s' port %d\n", toHost, toPort);
        Wait(1.0);
        client->tryHoldConnection = true;
      }
    }
    else if (strncasecmp(line, "hup", 3) == 0)
    {
      client->tryHoldConnection = false;
      client->closeConnection();
    }
    else if (strncasecmp(line, "ping", 4) == 0)
      client->sendPing();
    else if (strncasecmp(line, "max", 3) == 0)
    {
      fimage->setToMaxRes(not (strstr(line, "false") != NULL));
      if (not (strstr(line, "false") != NULL))
        printf("All images will now be shown as 800x600 images\n");
      else
        printf("All images will be shown in original size\n");
    }
    else if (strncmp(line, "focus", 5) == 0)
    {
      sscanf(&line[5], "%d", &focalLine);
      fimage->setFocalLine(focalLine);
      if (focalLine >= 0)
        printf("Line %d will be shown on next image\n", focalLine);
      else
        printf("Line off\n");
    }
    else if (strncasecmp(line, "fuzzyLine", 9) == 0)
    {
      fuzzyLine = -1;
      n = sscanf(&line[9], "%d %d", &fuzzyLine, &fuzzyCol);
      if (n > 0)
      {
        fimage->setFuzzyLine(fuzzyLine, fuzzyCol);
        printf("Set fuzzy classifier line to %d and col to %d\n",
              fuzzyLine, fuzzyCol);
      }
      else
      {
        fimage->setFuzzyLine(fuzzyLine, fuzzyCol);
        printf("Set fuzzy to OFF\n");
      }
    }
    else if (strncmp(line, "fuzzyc", 6) == 0)
      {
        n = sscanf(&line[6], "%d %d", &fuzzyClasses, &fuzzyIter);
        if (n > 0)
      {
        fimage->setFuzzyClasses(fuzzyClasses);
        printf("Set fuzzy classes value to %d (2,3 are legal)\n", fuzzyClasses);
      }
      if (n > 1)
      {
        fimage->setFuzzyIter(fuzzyIter);
        printf("Set fuzzy number of iterations to %d (0=seed area only)\n", fuzzyIter);
      }
      if (n <= 0)
        printf("Not legal, usage: fuzzyc <classes> <iterations>\n");
    }
    else if (strncmp(line, "fuzzy", 5) == 0)
    {
      sscanf(&line[5], "%d", &fuzzyWidth);
      fimage->setFuzzyWidth(fuzzyWidth);
      if (fuzzyWidth > 0)
        printf("classify into 2-3 clausters, mouse & shift mouse, (+global if 3)\n");
      else
        printf("Fuzzy classify off\n");
    }
    else if (strncasecmp(line, "clear", 5) == 0)
    {
      cvDestroyAllWindows();
      flaser->clear();
      fimage->clear();
    }
    else if (strncmp(line, "UV", 2) == 0)
    {
      capt[0] = 0;
      sscanf(&line[2], "%20s", capt);
      fimage->setUVSource(capt, UClientFuncImgGui::CDT_YUV);
    }
    else if (strncasecmp(line, "CROMA", 5) == 0)
    {
      capt[0] = 0;
      sscanf(&line[5], "%20s", capt);
      fimage->setUVSource(capt, UClientFuncImgGui::CDT_CROMA);
    }
    else if (strncasecmp(line, "save", 4) == 0)
    {
      capt[0] = 0;
      val[0] = '\0';
      sscanf(&line[4], "%s %s", capt, val);
      if (not (fimage->saveImage(capt, val) or
               flaser->saveImage()))
        printf("No image to save\n");
      if (not flaser->getSaveImages())
        flaser->setSaveImages(true);
    }
/*    else if (strncasecmp(line, "paintpis ", 8) == 0)
    {
      capt[0] = 0;
      sscanf(&line[8], "%20s", capt);
      img = fimage->getImage(capt);
      flaser->(img);
      fimage->showImage(img);
    }*/
    else if (strncasecmp(line, "mmrHist", 7) == 0)
    {
      n = strtol(&line[7], NULL, 0);
      flaser->setHistDisplay(n);
      flaser->repaint();
    }
    else if (strncasecmp(line, "mmrScale", 8) == 0)
    {
      n = sscanf(&line[8], "%lf %lf", &d1, &d2);
      if (n == 1)
        flaser->setScale(d1);
      if (n == 2)
        flaser->setScale(d1, d2);
      else
        printf("mmrScale usage 'mmrScale [image height in meter]  [position of MMR in meter]'\n");
      flaser->repaint();
    }
    else if (strncasecmp(line, "mmrImg", 6) == 0)
    {
      capt[0] = 0;
      sscanf(&line[7], "%20s", capt);
      flaser->doImage(fimage->getImageSource(capt), NULL, NULL, NULL);
    }
    else if (strncasecmp(line, "mmrPosImg", 9) == 0)
    {
      capt[0] = 0;
      n = sscanf(&line[10], "%s %lg %lg %lg %lg %lg %lg", capt, &pos.x, &pos.y, &pos.z, &rot.Omega, &rot.Phi, &rot.Kappa);
      printf("image '%s'\n -- seen from (%gx,%gy,%gz) at (%go, %gp, %gk) (degree)\n", capt,
             pos.x, pos.y, pos.z, rot.Omega, rot.Phi, rot.Kappa);
      rot.Omega *= M_PI/180.0;
      rot.Phi *= M_PI/180.0;
      rot.Kappa *= M_PI/180.0;
      flaser->doImage(NULL, capt, &pos, &rot);
    }
    else if (strncasecmp(line, "mmrNosave", 9) == 0)
      flaser->setSaveImages(false);
    else if (strncasecmp(line, "paint", 5) == 0)
    {
      n = sscanf(&line[6], "%s %s", name, val);
      if (n == 1)
        snprintf(val, MRL, "true");
      if (not flaser->setPaintVar(name, val))
        printf("unknown variable '%s' or value '%s'\n", name, val);
      else
        flaser->repaint();
    }
    else if (strncasecmp(line, "silent", 6) == 0)
    {
      client->setVerbose(false);
/*      fimage->setVerbose(false);
      fpath->setVerbose(false);
      flaser->setVerbose(false);*/
    }
    else if (strncasecmp(line, "verbose", 7) == 0)
    { // let client set all loaded functions
      client->setVerbose(true);
/*      fimage->setVerbose(true);
      fpath->setVerbose(true);
      flaser->setVerbose(true);*/
    }
    // namespaceUse
    else if (strncasecmp(line, "namespaceUse", 12) == 0)
    {
      capt[0] = 0;
      sscanf(&line[12], "%12s", capt);
      res = str2bool(capt);
      client->setNamespaceUse(res);
    }
    else
    { // then it must be for server - send to the connected server
      if (client->isConnected())
        res = client->blockSend(line, strlen(line));
      else
        printf("not connected - try help\n");
    }
  }
  // stop handler - if running
  client->stop(true);
  // close connection to server
  client->closeConnection();
  // terminate
  //printf("SML-CLIENT test terminated\n");
}

//////////////////////////////////////////////////////////


