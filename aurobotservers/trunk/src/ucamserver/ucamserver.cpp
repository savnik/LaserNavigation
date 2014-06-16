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
#include <getopt.h>
// for dynamic load functions 
#include <dlfcn.h>
#include <signal.h>

#define USE_READLINE_LIB
#ifdef USE_READLINE_LIB
#include <readline/readline.h>
#include <readline/history.h>
#endif

#include <ugen4/utime.h>
#include <ugen4/ucommon.h>
#include <urob4/userverport.h>
#include <urob4/ufunctionbase.h>
#include <ucam4/ufunctioncambase.h>
#include <urob4/ufunctionimgpool.h>
#include <ucam4/ufunctionimage.h>
#include <urob4/ucmdexe.h>
#include <ucam4/ufunctioncam.h>
#include <ucam4/ucampool.h>
#include <urob4/ufunctionposehist.h>
#include <urob4/uclientfunccam.h>
#include <urob4/uclientfuncimage.h>
#include <ugen4/uimg3dpoint.h>

// #include <ugen4/ufuzzypixel.h>
// #include <ugen4/ufuzzysplit.h>

#include "userverstatic.h"

#include "ufunctioncampath.h"
#include "ufunctioncamgmk.h"
#include "../../libs/ugen4/ucommon.h"

#include <urob4/uclienthandler.h>
#include <urob4/uresclientifvar.h>
#include <urob4/usmrcl.h>
#include <urob4/ulogfile.h>
#include <urob4/usmlfile.h>
#include <urob4/usmlstring.h>
#include <urob4/ufuncplugbase.h>

#define __SERVER_VERSION__ "2.1966"

using namespace std;

void shutDownHandler(int signal );
bool runCameraServerComponent(bool deamon, const char * iniScript);
// bool getCmdLineOptions(int argc, char *argv[],
//                        int defaultServerPort,
// //                       char * moduleList, int moduleListCnt,
//                        bool * allStatic,
//                        bool * deamon,
//                        char * iniScript, int iniScriptCnt);
//     void printHelp();
void testFuzzySplit();
bool addCmdHist(const char * line, char * lineHist[],
                const int MHL, const int MLL,
                int * lineHistCnt, int * lineHistNewest);
//void getIniFiles();
void dummyCall();

/** pointer to server object */
UServerStatic * server = NULL;

/////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
  bool ask4help;
  bool asDaemon = false;
  const int MIL = 260;
  char iniScript[MIL];
  struct sigaction nact;
  //
  if (false)
  { // test subfunction
    testFuzzySplit();
  }
  else
  { // Registering the interrupt handler, catching
    /* Set up the structure to specify the signal action. */
    nact.sa_handler = shutDownHandler;
    sigemptyset (&nact.sa_mask);
    nact.sa_flags = 0;
  // catch the following actions
    sigaction (SIGINT, &nact, NULL);
    sigaction (SIGHUP, &nact, NULL);
    sigaction (SIGTERM, &nact, NULL);
    // set default initialisation script filename
    if (strrchr(argv[0], '/') == NULL)
      // application found in $PATH
      appName = argv[0];
    else
    { // application started useing absolute or relative path
      appName = strrchr(argv[0], '/');
      appName++;
    }
    snprintf(iniScript, MIL, "./%s.ini",  appName);
    ask4help = setCommonPathAndOtherOptions(argc, argv, iniScript, MIL, &asDaemon, 24920, "Mobotware camera server");
    //
    if (not ask4help)
    {
      getIniFiles(iniScript);
      runCameraServerComponent( asDaemon, iniScript);
    }
  }
  //
  return EXIT_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////

// void printHelp()
// {
//   printf("\n");
//   printf("Cameraserver " __SERVER_VERSION__ " command line options:\n");
//   printf(" -h --help :  This help message\n");
//   printf(" -a --deamon  Run server as deamon (no console input)\n");
//   printf(" -i --imagepath <path>\n");
//   printf("    Sets path where images are expected to be found and written\n");
//   printf(" -d --datapath <path>\n");
//   printf("    Sets path where data other than images are expected to be found and written\n");
//   printf(" -s --script <file>\n");
//   printf("    Load this script file after startup (command script)\n");
// /*  printf(" -c --confcam <file>\n");
//   printf("    Set path and filename for camera configuration file\n");*/
//   printf(" -m --modules <list of module filenames to load>\n");
//   printf("    List (space separated) of modules to load (e.g. -m './laser1.so.0')\n");
//   printf(" -p --port <port>\n");
//   printf("    sets the server port number\n");
//   printf(" -t --static\n");
//   printf("    load all available static modules\n");
//   printf("\n");
// }
// 
// //////////////////////////////////////////
// 
// bool getCmdLineOptions(int argc, char *argv[],
//                       int defaultServerPort,
// //                      char * moduleList, int moduleListCnt,
//                       bool * allStatic,
//                       bool * asDaemon,
//                       char * iniScript, int iniScriptCnt)
// {
//   static struct option long_options[] = {
//     {"imagepath", 1, 0, 'i'},
//     {"datapath", 1, 0, 'd'},
//     {"confcam", 1, 0, 'c'},
//     {"script", 1, 0, 's'},
//     {"help", 0, 0, 'h'},
//     {"deamon", 0, 0, 'a'},
// //    {"modules", 1, 0, 'm'},
//     {"port", 1, 0, 'p'},
//     {"static", 0, 0, 't'},
//     {0, 0, 0, 0}
//   };
//   bool ask4help = false;
//   int opt;
//   int option_index = 0;
// //   const int MSL = 500;
// //   char moduleDir[MSL] = "";
// //   char res[MSL];
// //   bool getMlist = false;
// //   FILE * mf;
// //   int n;
// //   char *p1, *p2;
//   // set default values
//   ask4help = setCommonPathAndOtherOptions(argc, argv, "", iniScript, iniScriptCnt, &asDaemon, defaultServerPort);
//   // take path from command line
//   while (not ask4help)
//   {
//     opt = getopt_long(argc, argv, "ahi:td:p:c:s:", long_options, &option_index);
//     if (opt == -1)
//       break;
//     switch (opt)
//     {
//       case -1: // no more options
//         break;
//       case 'i': // path setting for images
// //         if (optarg != NULL)
// //           strncpy(imagePath, optarg, MAX_PATH_LENGTH);
//         break;
//       case 'a': // run as a deamon
//         *deamon = true;
//         break;
//       case 'd': // path setting for other results
// //         if (optarg != NULL)
// //           strncpy(dataPath, optarg, MAX_PATH_LENGTH);
//         break;
// /*      case 'c': // config file for camera settings
//         if (optarg != NULL)
//           strncpy(configFileCam, optarg, MAX_PATH_LENGTH);
//         break;*/
//       case 's': // init script file
//         if (optarg != NULL)
//           strncpy(iniScript, optarg, iniScriptCnt);
//         break;
//       case 'p': // specific server port
//         if (optarg != NULL)
//           sscanf(optarg, "%d", &serverPort);
//         break;
// //       case 'm': // modulelist
// //         if (optarg != NULL)
// //           strncpy(moduleList, optarg, moduleListCnt);
// //         break;
//       case 't':
//         *allStatic = true;
//         break;
//       case 'h':
//         ask4help = true;
//         //printHelp();
//         break;
//       default:
//         printf("Unknown option '%c'\n", opt);
//         break;
//     }
//   }
//   // test imagePath and dataPath - if not writeable, then try current dir or home dir.
// //   testPathSettings();
// //   // get module list
// //   if (getMlist)
// //   {
// //     if (strlen(moduleDir) == 0)
// //       strcpy(moduleDir, ".");
// //     p1 = moduleDir;
// //     while (p1 != NULL)
// //     {
// //       if (*p1 == '\0')
// //         break;
// //       p2 = strchr(p1, ' ');
// //       if (p2 != NULL)
// //         *p2++ = '\0';
// //       snprintf(res, MSL, "ls %s/*.so* > modulelist.mod", p1);
// //       if (system(res) == 0)
// //       {
// //         mf = fopen("modulelist.mod", "r");
// //         if (mf != NULL)
// //         {
// //           while (not feof(mf))
// //           {
// //             if (fgets(res, MSL, mf) == NULL)
// //               break;
// //             n = strlen(res);
// //             if (--n > 0)
// //             { // cut off linefeed etc
// //               while ((res[n] <= ' ') and (n >= 0))
// //                 res[n--] = '\0';
// //               // add to module list
// //               if (strlen(moduleList) > 0)
// //                 strncat(moduleList, " ", moduleListCnt);
// //               strncat(moduleList, res, moduleListCnt);
// //             }
// //           }
// //         }
// //         fclose(mf);
// //       }
// //       else
// //         printf("Failed to get module list in '%s'\n", moduleDir);
// //       p1 = p2;
// //     }
// //   }
//   return ask4help;
// }

//////////////////////////////////////////

void shutDownHandler( int signal )
{ // print signal message - if from signal
  //
  if (signal > 0)
    // print signal message
    psignal( signal, "Signal");
  if (server != NULL)
  { //
    if (not server->isStopping())
    { // stop the server
      printf("Terminating server (%d) ...\n", signal);
      server->stop(true); // all client connection
      printf("Server terminated\n");
    }
  }
  if (server != NULL)
  {
    delete server;
    server = NULL;
  }
  if (signal > 0)
    // stop process now!
    exit(0);
}

///////////////////////////////////////////////////////

bool runCameraServerComponent(bool asDaemon, const char * iniScript)
{
  //const int SL = 30;
  //char c[SL] = "";
  const char * appNameV = "Camera_server " __SERVER_VERSION__ " (" __DATE__ " " __TIME__ " jca@oersted.dtu.dk)";
  char * p1;
  //char * p2 = moduleList;
  const int MLL = 200;
  char lineBuff[MLL];
  char * line = lineBuff;
  int client;
  //
  FILE * cmdf;
  char cmdfName[MLL];
  const int MLH = 100; // max line history
  char lhb[MLH][MLL];
  char *lineHist[MLH];
  int lineHistCnt = 0;
  int lineHistNewest = -1;
  int i;
  UServerPort * servPort;
  // server functions
  UFunctionCam * fcam;
  UFunctionImgPool * fpool;
  UFunctionImage * fimage;
  // "static" function pointers
  //UFunctionCamGmk * fgmk;
  //UFunctionCamPath * fpath;
  const bool verbose = false;
  //
  // create command execution object
  server = new UServerStatic();
  server->setVerbose(verbose);
  // create socket server object
  servPort = new UServerPort();
  servPort->setVerbose(verbose);
  servPort->setAllowConnections(false);
  servPort->setPort(serverPort);
  servPort->setServerNamespace("camServer");
  servPort->setServerNamespaceAttribute("name=\"ucamcomp\" version=\"" __SERVER_VERSION__ "\"");
  // combine server with port
  server->setServer(servPort); // socket server
  // configure camera function
  fcam = new UFunctionCam();
  fcam->setVerbose(verbose);
  // configure image from image pool function
  fpool = new UFunctionImgPool();
  fpool->setVerbose(verbose);
  // configure image function
  fimage = new UFunctionImage();
  fimage->setVerbose(verbose);
  // add basic camera server components to command execute object
  server->addFunction(fpool);
  server->addFunction(fcam);
  server->addFunction(fimage);
  // start message handling thread
  server->handleMessagesThreadStart();
  // give the server a name
  server->setName(appNameV);
  //
//   if (allStatic)
//   {
//     server->loadStaticModule( "var", "", NULL, 0);
//     server->loadStaticModule( "odoPose", "odoPose", NULL, 0);
//     server->loadStaticModule("path", "", NULL, 0);
//     server->loadStaticModule("gmk", "", NULL, 0);
//   }
  //
//   while (p2 != NULL)
//   { // get next module name in module list string
//     p1 = strsep(&p2, " ");
//     if (*p1 <= ' ')
//       break;
//     // load the module - prints errors to console if relevant
//     if (not server->loadStaticModule(p1, "", NULL, 0))
//       server->loadFunctionModule(p1, NULL, 0, "");
//   }
  // load ini script
  if (strlen(iniScript) > 0)
    server->executeScriptFile(iniScript, true, -1);

  // server is now assumed configured, and clients can be allowed
  servPort->setAllowConnections(true);
  //
  if (not asDaemon)
  { // reload command history
    snprintf(cmdfName, MLL, "%s.cmd", appName);
    cmdf = fopen(cmdfName, "r");
    // initialize cash history buffer
    for (i = 0; i < MLH; i++)
      lineHist[i] = lhb[i];
    if (cmdf != NULL)
    { // add old lines from last run
      while (fgets(line, MLL, cmdf))
      { // remove new-line
        line[strlen(line) - 1] = '\0';
        if (addCmdHist(line, lineHist, MLH, MLL, &lineHistCnt, &lineHistNewest))
          // add to line editor
          add_history(line);
      }
      fclose(cmdf);
    }
  }
  //
  Wait(0.2);
  if (asDaemon)
    printf("Running without console access - press ^C to kill\n");
  else
    printf("%s\n"
      " - on port %d\n"
      " - type h for help, q for quit\n\n",
      appNameV, serverPort);
  //
  // open the server port for connections
  servPort->start();
  // run small keyboard loop (to enable status print and quit)
  line[0] = '\0';
  while (true)
  {
    if (asDaemon)
      Wait(10.0);
    else
    {
#ifdef USE_READLINE_LIB
      // readline allocates a string and returns a pointer to it
      line = readline(">> ");
      if (line == NULL)
        continue;
      if (strlen(line) == 0)
        continue;
      // add to history buffer
      if (addCmdHist(line, lineHist, MLH, MLL, &lineHistCnt, &lineHistNewest))
        // add to readline buffer
        add_history(line);
      // add a new-line at the end
      snprintf(lineBuff, MLL, "%s\n", line);
      // free buffer allocated by readline()
      delete line;
      // let line point to just received line
      line=lineBuff;
#endif
#ifndef USE_READLINE_LIB
      // do not use readline library
      printf(">>");
      fgets(line, MLL, stdin);
#endif
      // get possible start number
      client = strtol(line, &p1, 10);
      // execute command
      if (line[1] < ' ')
      {
        if (line[0] == 'q')
          break;
        switch (line[0])
        {
          case 'p': // print status
            printf("Server console short status:\n");
//            printf(" - camera configuration file: '%s'\n", configFileCam);
            server->print("Server ");
            //cams->print("Camera pool:");
            //imgs->print("ImagePool");
            break;
          default: // help
            printf("Short help list:\n");
            printf("  q               Quit server (without use of signals)\n");
//            printf("  p               Print status for server (almost the same as 'module list')\n");
            printf("  [N:]command     Send 'command' to this server\n");
            printf("                  the N: may be used to fake a command as if from client N\n");
            printf("                  'server clients' gets a list of client numbers\n");
            printf("  help            Main (server core) help text\n");
//            printf("  static [module] Load one of the available static modules (try static help)\n\n");
          break;
        }
      }
      else if (*p1 == ':')
      { // send as from another client (see status for client numbers)
        p1++;
        server->postCommand(client, p1);
      }
      else
      { // else just send line as is from a console client
        server->postCommand(-1, line);
      }
    }
    // wait - a bit - for the reply to be printed on console
    Wait(0.015);
    // wait for server to finish queued commands - if not finished already
    //server->consoleLock.wait();
  }
  if (not asDaemon)
  { // save cmd history
    clear_history();
    cmdf = fopen(cmdfName, "w");
    if (cmdf != NULL)
    {
      for (i = 0; i < lineHistCnt; i++)
      {
        if (++lineHistNewest >= lineHistCnt)
          lineHistNewest = 0;
        fprintf(cmdf, "%s\n", lineHist[lineHistNewest]);
      }
      fclose(cmdf);
    }
  }
  //
  printf("%s shutdown\n", appName);
  shutDownHandler(0);
/*  printf("Stopping server ...\n");
  server->stop(true); // stop all clients and unload modules
  printf("Server terminated\n");*/
  //
  return true;
}

///////////////////////////////////////////////////


// void testFuzzySplit()
// {
//   const int MPC = 20;
//   UPixel pixx[MPC];
//   UPixel * pix;
//   UFuzzyPixel fp[MPC];
//   UFuzzyPixel * pfp;
//   UFuzzySplit split;
//   int i;
//   const int clustCnt = 4;
//   //
//   pix = pixx;
//   pix++->set(10, 200, 50);
//   pix++->set(11, 220, 51);
//   pix++->set(2, 210, 57);
//   pix++->set(100, 110, 54);
//   pix++->set(90,  130, 53);
//   pix++->set(110, 100, 52);
//   pix++->set(26, 210, 140);
//   pix++->set(17, 205, 155);
//   pix++->set(18, 230, 152);
//   pix++->set(19, 190, 151);
//   pix++->set(100, 0, 50);
//   pix++->set(51, 20, 51);
//   pix++->set(2, 210, 220);
//   pix++->set(10, 10, 14);
//   pix++->set(90,  130, 3);
//   pix++->set(110, 10, 52);
//   pix++->set(190, 210, 140);
//   pix++->set(17, 160, 15);
//   pix++->set(180, 23, 52);
//   pix++->set(190, 190, 51);
//   pfp = fp;
//   pix = pixx;
//   for (i = 0; i < MPC; i++)
//   {
//     pfp->setPixel(pix++);
//     split.addElement(pfp);
//     pfp++;
//   }
//   split.classify(clustCnt, 1e-5, 10, true);
//   pfp = fp;
//   split.countMembers();
//   for (i = 0; i < clustCnt; i++)
//     printf("Cluster %d has %d members\n", i, split.getMembCount(i));
//   printf("Finished\n");
// }

/////////////////////////////////////////////////////////////////

void dummyCall()
{ // just to define the classes needed by plugin modules
  UClientHandler dummy1;
  UResIfVar dummy2("dummy");
  USmrCl dummy3;
  UClientFuncCam dummy4;
  UDataDouble dummy5;
  UClientFuncImage dummy6;
  ULogFile dummy7;
  USmlFile dummy8;
  UFuncPlugBase dummy9;
  USmlString dummy10;
  UImg3Dpoints dummy11;
  dummy11.makePCLFile("aaa", false, false);
  //
  printf("This function should not be called\n");
}

/////////////////////////////////////////////////////

bool addCmdHist(const char * line, char * lineHist[], const int MHL, const int MLL,
                int * lineHistCnt, int * lineHistNewest)
{
  bool result;
  //
  if (strlen(line) <= 2)
    // do not save one or two character commands
    result = false;
  else if (*lineHistNewest < 0)
  { // first line
    *lineHistCnt = 0;
    result = true;
  }
  else
    // same as last?
    result = strcmp(line, lineHist[*lineHistNewest]) != 0;
  if (result)
  { // need to be saved
    if (*lineHistCnt < MHL)
      *lineHistNewest = (*lineHistCnt)++;
    else
    {
      (*lineHistNewest)++;
      if (*lineHistNewest >= MHL)
        *lineHistNewest = 0;
    }
    strncpy(lineHist[*lineHistNewest], line, MLL);
  }
  return result;
}

//////////////////////////////////////////////

// void getIniFiles()
// {
//   const int MCL = 260;
//   char cmd[MCL];
//   //
//   if (system("stat robcam.conf >/dev/null") != 0)
//   { // no local camera info file - so try to copy from /usr/local/smr/bin
//     //printf("robcam.conf not found\n");
//     if (system("cp /usr/local/smr/bin/robcam.conf . >/dev/null") == 0)
//       printf("Default camera configuration file copied from /usr/local/smr/bin/robcam.conf\n");
//     else
//       printf("No camera configuration file (robcam.conf) - an empty file may be created\n");
//   }
//   snprintf(cmd, MCL, "stat %s.ini >/dev/null", appName);
//   // printf("Dooing a '%s'\n", cmd);
//   if (system(cmd) != 0)
//   { // no local ini scriptfile - so try to copy from /usr/local/smr/bin
//     snprintf(cmd, MCL, "cp /usr/local/smr/bin/%s.ini . >/dev/null", appName);
//     if (system(cmd) == 0)
//       printf("Default ini script copied from /usr/local/smr/bin/%s.ini\n", appName);
//     else
//       printf("Default ini script not copied from /usr/local/smr/bin/%s.ini\n", appName);
//   }
// }
