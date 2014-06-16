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

#include <iostream>
#include <cstdlib>

#include <stdio.h>
#include <stdlib.h>
//#include <termios.h> // termio stuff

//#include "uresvarpool.h"
#include <ugen4/utime.h>
#include <ugen4/ucommon.h>
//#include "urobformat.h"
//#include "urobcamformat.h"
//#include "usockclient.h"

#include "usmltag.h"
#include "usmrcl.h"
//#include "uimgsockserv.h"
//#include "uimgpushtest.h"
#include "userverport.h"
#include "ufunctionbase.h"
#include "ufunctionimgpool.h"
//#include "ucmdexe.h"
#include "uclienthandler.h"
#include "uclientfuncimage.h"
#include "uclientport.h"
#include "upush.h"
//#include "uresposehist.h"
#include "ufunctionposehist.h"
#include "uvarcalc.h"
#include "ulogfile.h"
#include "ufuncplugbase.h"
#include "ufunctionvarpool.h"

using namespace std;

void printHelp();
void testSockClient();
void testsmrcl(int argc, char *argv[]);
void testServerPort();
void testSmlClient();
void testSmlEncoding();
void testPush();
void testServerGen();
void testLocale();
void testSystem();
void testCalc();
void testLog();
void testPlugBase();
void testImgPool();
void testVar();

////////////////////////////////////////////

int main(int argc, char *argv[])
{
  bool ask4help;
  // set default path
  appName = strrchr(argv[0], '/');
  appName++;
  //
  ask4help = setCommonPathAndOtherOptions(argc, argv, "localhost", 24920);
  if (ask4help)
    printHelp();
  else
  {
    printf("This is a test appliction to test the URob4 common library\n");
    printf("And do not perform any function on its own\n");
    //
    // testing
    //testSockClient();
    //testsmrcl(argc, argv);
    //testServerPort();
    //testServerGen();
    //testSmlClient();
    //testSmlEncoding();
    //testPush();
    //testLocale();
    //testSystem();
    //testCalc();
    //testLog();
    //testPlugBase();
    //
    //testImgPool();
    testVar();
    printf("- Program is terminated!\n");
  }
  //
  return EXIT_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////

void printHelp()
{
  printf("This program includes just common library classes \n");
  printf("(and some testfunctions in main.c for the library)\n");
  printf("- socket server and client support function \n");
  printf("- Component base for \"plugable\" functions\n");
  printf("- common classes for transfer of common data over socket\n");
  printf("  connections using SML (XML-like) format.\n");
  printf("\n");
  printf("options for library tests:\n");
  printf(" -h --help This help message\n");
  printf(" -i --imagepath\n");
  printf("    Sets path where images are expected to be found and written\n");
  printf(" -d --datapath\n");
  printf("    Sets path where data other than images are expected to be found and written\n");
  printf(" -c --config <file>\n");
  printf("    Set path and filename for configuration file (default './robcam.conf')\n");
  printf(" -s --server <server host name>\n");
  printf("    sets the default server name, usable by a socket client applications\n");
  printf(" -p --port <port>\n");
  printf("    sets the default port number, usable by a socket server application\n");
  printf("\n");
}

/////////////////////////////////////////////////////////////////////////

// void testSockClient()
// {
//   USockClient sc;
//   int i;
//   //
//   sc.setHost("localhost");
//   sc.setPort(24912);
//   //
//   sc.tryConnect();
//   //
//   if (sc.isConnected())
//   {
//     sc.blockSend("hej\n", 5);
//     for (i = 0; i < 100; i++)
//     {
//       Wait(0.1);
//     }
//     sc.doDisconnect();
//   }
// }

//////////////////////////////////////////

void testsmrcl(int argc, char *argv[])
{
  USmrCl smr;
  bool result;
  bool handled;
  const int MSL = 200;
  char s[MSL];
  int n;
  double x, y;
  int id = 0;
  //
  printf("setting odometer logging to 'odo_<time>.log'\n");
  smr.logIO.setLogName("smrcl");
  printf("Testing smr drive.\n");
  smr.setHost("localhost");
  smr.setPort(31001);
  result = smr.tryConnect();
  //
  if (not result)
    printf("Could not connect to MRC"
           " on host %s port %d\n",
           smr.getHost(), smr.getPort());
  else
  {
    //Wait(1.3);
    snprintf(s, MSL, "set \"odocontrol\" 0\n");
    handled = smr.sendString(s);
    printf("send: %s ... got (send %s)\n", s, bool2str(handled));
    n = 0;
    while (smr.isConnected())
    {
      n ++;
      printf("# loop %d\n", n);
      //Wait(0.3);
      //result = smr.handleGetevents(30);
      smr.start();
/*      result = smr.sendSMR("getevent", 1500, &reply, &handled);
      printf("send: getevent ... got (handled %s) %s\n", bool2str(handled), reply);
      result = smr.listen(1000, &reply, &handled);
      if (result)
        printf("listen                handled %s  %s\n", bool2str(handled), reply);
      else
        printf("listen timeout\n");*/
      printf("queued %d, started %d finished %d\n",
             smr.cmdLineQueued, smr.cmdLineStarted, smr.cmdLineFinished);
      //if (smr.cmdLineFinished == smr.cmdLineQueued)
      //  break;
      if (n > 20)
        break;
      if (n == 5)
        smr.startOdoStream(1);
      if (n %2 == 1)
      {
        printf("Sending '%s'\n", s);
        //result = smr.sendSMRgetId(s, 1500, &id, true);
        result = smr.sendString(s);
        printf("send command line '%s', got queued as ID%d (%s)\n", s, id, bool2str(result));
      }
      if (n %3 == 1)
      {
        Wait(0.015);
        printf("Sending 'eval $odox $odoy'\n");
        result = smr.sendSMReval("$odox", 2500, &x);
        printf("send: 'eval $odox' got %f (%s)\n", x, bool2str(result));
        result = smr.sendSMReval("$odoy", 1500, &y);
        printf("send: 'eval $odoy' got %f (%s)\n", x, bool2str(result));
      }
      smr.print("--");
    }
    if (smr.isConnected())
      smr.doDisconnect();
  }
  //smr.setOdoLogClosed();
}


/////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////

void testServerPort()
{
  UServerPort server;
  const int SL = 30;
  char c[SL] = "", *pc;
  bool isOK;
  int n, cnt= 1;
  UServerInMsg * msg;
  //
  printf("ServerPort test started\n");
  server.setVerbose(true);
  server.setServerNamespace("sestServer");
  server.setServerNamespaceAttribute("name=\"urob4\" version=\"4.1\"");
  server.setPort(24923);
  isOK = server.start();

  printf("Server started %s\n", bool2str(isOK));
  Wait(0.2);
  while (c[0] != 'q')
  {
    printf(">>");
    pc = fgets(c, SL, stdin);
    if (pc == NULL)
      break;
    switch (c[0])
    {
      case 'p':
        server.print("Server ");
        break;
      case 'n':
        sscanf(&c[1], "%d", &cnt);
        for (n = 0; n < cnt; n++)
        {
          if (server.getRxQueue()->isEmpty())
          {
            printf("Queue is empty\n");
            break;
          }
          msg = server.getRxQueue()->skipToNextMessage(true);
          msg->print("got");
        }
        break;
      default:
        printf("Press 'q' (quit), 'p' (print) or 'n <cnt>' (get from queue)\n");
        break;
    }
  }
  printf("Stopping clients and server ...\n");
  server.stop(true);
  printf("ServerPort test ended\n");
}

//////////////////////////////////////////


//////////////////////////////////////////////

void testServerGen()
{
  UServerPort * servPort;
  UCmdExe * cmdExe;
  UFunctionBase * fbase;
  UFunctionImgPool * fimgPool;
  UFunctionPoseHist * fposeHist;
  UFunctionVarPool * fvarpool;
  const int SL = 230;
  char c[SL] = "";
  int client;
  char * p1;
  //
  // configure base function
  fbase = new UFunctionBase();
  fbase->setVerbose(true);
  // image pool function
  fimgPool = new UFunctionImgPool();
  fimgPool->setVerbose(true);
  // pose history ressource
  fposeHist = new UFunctionPoseHist();
  fposeHist->setAliasName("map");
  fposeHist->setVerbose(true);
  // varpool
  fvarpool = new UFunctionVarPool();
  fvarpool->setVerbose(true);
  // Connect components in command execute object
  cmdExe = new UCmdExe();
  cmdExe->setName("rob4server");
  servPort = new UServerPort();
  cmdExe->setVerbose(false);
  cmdExe->setServer(servPort); // socket server
  cmdExe->addFunction(fbase);
  cmdExe->addFunction(fimgPool);
  cmdExe->addFunction(fposeHist);
  cmdExe->addFunction(fvarpool);
  // start socket server
  servPort->setVerbose(true);
  servPort->setServerNamespace("rob4");
  servPort->setPort(serverPort);
  servPort->start();
  // start read thread
  cmdExe->handleMessagesThreadStart();

  // test remove of modules
  //   cmdExe->deleteFunction(fcamPool);
  //   cmdExe->deleteFunction(fimage);
  //   // and add
  //   cmdExe->addFunction(fimage);
  //   cmdExe->addFunction(fcamPool);

  // keyboard loop
  while (c[0] != 'q')
  {
    printf(">>");
    p1 = fgets(c, SL, stdin);
    client = strtol(c, &p1, 10);
    if (c[1] < ' ')
    switch (c[0])
    {
      case 'p':
        printf("Path settings:\n");
        printf(" - image path: '%s'\n", imagePath);
        printf(" - data  path: '%s'\n", dataPath);
        servPort->print("Server ");
        cmdExe->print("CmdExe ");
        break;
      case 'q':
        break;
      default: // help
        printf("Press 'q' (quit), 'p' (print) or [N:]command (e.g.: help)\n");
        break;
    }
    else if (*p1 == ':')
    { // simulate command is from another client
      client = strtol(c, NULL, 0);
      p1++;
      cmdExe->postCommand( -1, p1);
    }
    else
    { // assume it is a command from cient -1 i.e. console.
      cmdExe->postCommand( -1, c);
    }
  }
  printf("Stopping clients and server ...\n");
  cmdExe->stop(true);
  printf("ServerPort test ended\n");
}

/////////////////////////////////////////////////////////////////////////////


void testSmlClient()
{
  UClientHandler * client;
  UClientFuncBase * fBase;
  UClientFuncImage * fimage;
  const int MLL = 100;
  char line[MLL], *p1;
  // create core client service
  client = new UClientHandler();
  client->setVerbose(true);
  // create a client function handler
  fBase = new UClientFuncBase();
  fBase->setVerbose(true);
  // create image client function
  fimage = new UClientFuncImage();
  fimage->setVerbose(true);
  // add function to service
  client->addFunction(fBase);
  client->addFunction(fimage);
  client->setNamespace("smlClient", "name=\"urob4\" version=\"4.1\"");
  // connect to server
  client->setPort(24920);
  client->setHost("localhost");
  client->openConnection();
  // do some test
  printf("testSmlClient: connected %s to %s port %d\n",
      bool2str(client->isConnected()),
      client->getHost(), client->getPort());
  //
  if (client->isConnected())
    // start handler of messages
    client->start();
  //
  while (client->isConnected())
  {
    printf("## ");
    p1 = fgets(line, MLL, stdin);
    if (strncasecmp(line, "quit", 4) == 0)
      break;
    else if (strncasecmp(line, "print", 5) == 0)
      client->print("print");
    else if (strncasecmp(line, "help", 4) == 0)
    {
      printf("Functions\n");
      printf(" quit      Terminate test\n");
      printf(" print     Print client status\n");
      printf(" help      This\n");
      printf(" <line>\\n  Line sendt to connected server\n");
      printf("\n");
    }
    else
      client->blockSend(line, strlen(line));
  }
  // stop handlre - if running
  client->stop(true);
  // close connection to server
  client->closeConnection();
  // terminate
  printf("SML-CLIENT test terminated\n");
}

/////////////////////////////////////////////////

void testSmlEncoding()
{
  UPosition p1;
  UPosition p2;
  const int MSL = 1000;
  char s[MSL];
  bool res;
  USmlTag nTag;
  ULineSegment line;
  //
  p1.set(10.0, 200.0, 0.05);
  p2.set(0.03334, 1234567890.0123456789, 0.012);
  res = nTag.codePosition(&p1, s, MSL, "testP1");
  printf("Test encoding of '10.0, 200.0, 0.00005' is (%s) '%s'\n", bool2str(res), s);
  res = nTag.codePosition(&p2, s, MSL, "testP2");
  printf("Test encoding of '0.03334, 1234567890.0123456789, 0.0'  is (%s) '%s'\n", bool2str(res), s);

  line.setFromPoints(p1, p2);
  res = nTag.codeLineSegment(&line, s, MSL, "testLine");
  printf("Test encoding of p1->p2 line is (%s):\n%s\n", bool2str(res), s);
}

////////////////////////////////////////////////

void testPush()
{
  UPush ps;
  ps.print("");
}

/////////////////////////////////////////////////

void testLocale()
{
  struct lconv * a;
  const char * s1 = "10,123.456";
  double d1, d2;
  const char * lp1, *lp2;

  a = localeconv();
  printf("LOCALE decimal point = '%s'\n", a->decimal_point);
  d1 = strtod(s1, NULL);
  lp1 = setlocale(LC_ALL, "");
  a = localeconv();
  d2 = strtod(s1, NULL);
  // back to C std
  lp2 = setlocale(LC_ALL, "C");
  a = localeconv();
  printf("value of %s is %f (%s) or %f (%s)\n", s1, d1, lp1, d2, lp2);
  printf("value 2\n");
}

////////////////////////////////////////////////

void testSystem()
{
  FILE * pf;
  const int MSL = 500;
  char s[MSL], *p1;
  int i = 0;
  //
  pf = popen("ls -l", "r");
  if (pf != NULL)
  {
    while (not feof(pf))
    {
      p1 = fgets(s, MSL, pf);
      if (strlen(s) > 0)
        printf("#%d, %s", i, s);
      i++;
    }
    printf("Finished");
    pclose(pf);
  }
}

////////////////////////////////////////////////////

void testCalc()
{
  UVarCalc calc;
  const char * a = "a";  // ok
  int m = MAX_VAR_NAME_SIZE;
  const char * b = "b1234567890_1234567890_1234567890"; // too long (33) (by one)
  const char * c = "AaT7$4"; // $ not allowed
  const char * d = "_H2_"; // not legal first char
  const char * e = "2E";  // not legal first char
  const char * f = "E.e"; // '.' not allowed
  bool result;
  const char * p2;
  //
  result = calc.getIdentifier(a, 10, "_$.", &p2);
  printf("A '%s' is a legal identifier (%s) til end (%s)\n", a, bool2str(result), bool2str(*p2 == '\0'));
  result = calc.getIdentifier(b, m, "_", &p2);
  printf("A '%s' is a legal identifier (%s) til end (%s)\n", b, bool2str(result), bool2str(*p2 == '\0'));
  result = calc.getIdentifier(c, m, "_", &p2);
  printf("A '%s' is a legal identifier (%s) til end (%s)\n", c, bool2str(result), bool2str(*p2 == '\0'));
  result = calc.getIdentifier(d, m, "_", &p2);
  printf("A '%s' is a legal identifier (%s) til end (%s)\n", d, bool2str(result), bool2str(*p2 == '\0'));
  result = calc.getIdentifier(e, m, "_", &p2);
  printf("A '%s' is a legal identifier (%s) til end (%s)\n", e, bool2str(result), bool2str(*p2 == '\0'));
  result = calc.getIdentifier(f, m, "_", &p2);
  printf("A '%s' is a legal identifier (%s) til end (%s)\n", f, bool2str(result), bool2str(*p2 == '\0'));
  //
}

//////////////////////////////////////////////////

void testLog()
{
  ULogFile l;
  const int MRL = 500;
  char res[MRL];
  //
  l.setLogName("testLog");
  l.openLog();
  l.toLog("Opened");
  snprintf(res, MRL, "Opened a '%s' logfile with file name '%s'", l.getLogName(), l.getLogFileName());
  l.toLog(res);
  printf("%s\n", res);
  l.toLog("closing");
  l.closeLog();
}

////////////////////////////////////////////////

void testPlugBase()
{
  UFuncPlugBase * fpb;
  //
  fpb = new UFuncPlugBase();
  printf("Default UFuncPlugBase name is: %s\n",
         fpb->name());
  delete fpb;
  printf("Finished UFuncPlugBase test\n");
}

void testImgPool()
{
  UImagePool * imgPool;
  UImage * img;
  int i;
  //
  imgPool = new UImagePool();
  for (i = 0; i < 100; i++)
  {
    img = imgPool->getImage(i, true, 120,160,1,8);
    img->clear(100);
  }
  for (i = 0; i < 100; i++)
  {
    img = imgPool->getImage(i, true, 240,320,1,8);
    img->clear(120);
  }
  for (i = 0; i < 100; i++)
  {
    img = imgPool->getImage(i, true, 640,480,3,8);
    img->clear(130);
  }
  delete imgPool;
}

///////////////////////////////////////////////////

void testVar()
{
  UVariable a,b,c;
  const int MSL = 500;
  char s[MSL];
  const int MAL = 700;
  double da[MAL];
  UMatrixBig mm(100, 7, da, MAL);
  //
  a.setValued("1,2,34,5,6", 0, true);
  b.setValued("false , 77, true ; 66, 55, false; true 12 true", 0, true);
  printf("a (%dx%d): %s\n", a.rows(), a.cols(), a.getValuesdAsString(s, MSL, 0));
  printf("b (%dx%d): %s\n", b.rows(), b.cols(), b.getValuesdAsString(s, MSL, 0));
  b.setValued("1;3;4;5;6", 0, true);
  printf("b (%dx%d): %s\n", b.rows(), b.cols(), b.getValuesdAsString(s, MSL, 0));
  c.setValued("false , 77, true 66, 55, false; true 12", 0, true);
  printf("c (%dx%d): %s\n", c.rows(), c.cols(), c.getValuesdAsString(s, MSL, 0));
  a.setValued("0.1, 0.02, 0.003, 0.0004, 0.00005, 0.000006", 0, false);
  printf("a (%dx%d): %s\n", a.rows(), a.cols(), a.getValuesdAsString(s, MSL, 0));
  a.makeHist(7, a.getElementCnt(), 50.0);
  a.openVarLog(true);
  UTime t;
  t.now();
  a.setValued(123.456, 4, false, &t);
  t += 0.1;
  a.setValued(765.432, 3, false, &t);
  t += 0.1;
  a.setValued("1.1 2.2 3; 4 5 6", 0, false, &t);
  printf("a (%dx%d): %s\n", a.rows(), a.cols(), a.getValuesdAsString(s, MSL, 0));
  t += 0.1;
  a.setValued("61.2 62.2 63; 64 65 66", 0, false, &t);
  printf("a (%dx%d): %s\n", a.rows(), a.cols(), a.getValuesdAsString(s, MSL, 0));
  t += 0.1;
  a.setValued("71.3 72.7 73; 74 75 76", 0, false, &t);
  printf("a (%dx%d): %s\n", a.rows(), a.cols(), a.getValuesdAsString(s, MSL, 0));
  t += 0.1;
  a.setValued("11.4 22.2 33 44" , 0, false, &t);
  printf("a (%dx%d): %s\n", a.rows(), a.cols(), a.getValuesdAsString(s, MSL, 0));
  t += 0.1;
  a.setValued("22.4 26.3 33 44" , 0, false, &t);
  printf("a (%dx%d): %s\n", a.rows(), a.cols(), a.getValuesdAsString(s, MSL, 0));
  t += 0.1;
  a.setValued("33.4 27 33 44" , 0, false, &t);
  printf("a (%dx%d): %s\n", a.rows(), a.cols(), a.getValuesdAsString(s, MSL, 0));
  a.getTimeSeries(&mm, 4);
  mm.print("mm");
}

