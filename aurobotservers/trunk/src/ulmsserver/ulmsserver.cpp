/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                   *
 *   jca@elektro.dtu.dk                                                    *
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
// readline library
#define USE_READLINE_LIB
#ifdef USE_READLINE_LIB
  #include <readline/readline.h>
  #include <readline/history.h>
#endif

#include <ugen4/utime.h>
#include <ugen4/ucommon.h>
#include <umap4/upose.h>
#include <urob4/userverport.h>
#include <urob4/ufunctionbase.h>
#include <urob4/ucmdexe.h>
#include <urob4/ufunctionposehist.h>
#include <urob4/uclientfunccam.h>
#include <urob4/uclientfuncimage.h>
#include <urob4/ulogfile.h>

#include "ufunctionlaser.h"
#include "usick.h"
#include "uhokuyo.h"
#include "ulasersim.h"
#include "uresv360.h"
#include "ufuncv360.h"
#include "userverstatic.h"

#include <urob4/uclienthandler.h>
#include <urob4/uresclientifvar.h>
#include <urob4/usmrcl.h>
#include <urob4/ulogfile.h>
#include <urob4/usmlfile.h>
#include <urob4/usmlstring.h>
#include <urob4/ufuncplugbase.h>
#include <ugen4/uimg3dpoint.h>

#define __SERVER_VERSION__ "2.1966"

using namespace std;

// prototypes of functions defined in this file
void shutDownHandler(int signal);
bool runLaserServerComponent(bool daemon,
                             const char * iniScript);
void printHelp();
bool addCmdHist(const char * line, char * lineHist[],
                const int MHL, const int MLL,
                int * lineHistCnt, int * lineHistNewest);
void dummyCall();


UServerStatic * server = NULL;

///////////////////////////////////////////////

int main(int argc, char *argv[])
{
  bool ask4help;
  bool asDaemon = false;
  const int MIL = 260;
  char iniScript[MIL];
  //
  // Registering the interrupt handler, catching
  struct sigaction nact, oact;
  /* Set up the structure to specify the new action. */
  nact.sa_handler = shutDownHandler;
  sigemptyset (&nact.sa_mask);
  nact.sa_flags = 0;
  // catch the following actions
  sigaction (SIGINT, NULL, &oact);
  if (oact.sa_handler != SIG_IGN)
    sigaction (SIGINT, &nact, NULL);
  sigaction (SIGHUP, NULL, &oact);
  if (oact.sa_handler != SIG_IGN)
    sigaction (SIGHUP, &nact, NULL);
  sigaction (SIGTERM, NULL, &oact);
  if (oact.sa_handler != SIG_IGN)
    sigaction (SIGTERM, &nact, NULL);
  //
  // set default initialisation script filename
  if (strrchr(argv[0], '/') == NULL)
      // application found in $PATH
    appName = argv[0];
  else
  { // application started useing absolute or relative path
    appName = strrchr(argv[0], '/');
    appName++;
  }
  //
  ask4help = setCommonPathAndOtherOptions(argc, argv,
                              iniScript, MIL,
                              &asDaemon,
                              24919,
                              "Mobotware Laser scanner server"
                              );
  if (not ask4help)
  { // if no ini-file copy default ones from /usr/local/smr/bin/aursconf
    getIniFiles(iniScript); // defined in ugen4/ucommon.cpp
    // run server
    runLaserServerComponent(asDaemon /*, ttySick, ttyUrg, ttySim*/, iniScript);
  }
  //
  return EXIT_SUCCESS;
}

////////////////////////////////////////////////////

 
//////////////////////////////////////////

// bool getCmdLineOptions(int argc, char *argv[],
//                        int defaultServerPort,
// //                       char * moduleList, const int moduleListCnt,
// //                        char * ttySick,
// //                        char * ttyUrg,
// //                        char * ttySim,
//                        bool * asDaemon,
//                        char * iniScript, const int iniScriptCnt)
// {
// //   static struct option long_options[] = {
// //     {"imagepath", 1, 0, 'i'},
// //     {"datapath", 1, 0, 'd'},
// // //    {"confcam", 1, 0, 'f'},
// //     {"script", 1, 0, 's'},
// //     {"help", 0, 0, 'h'},
// //     {"deamon", 0, 0, 'a'},
// // //     {"modules", 1, 0, 'm'},
// // //     {"modDir", 1, 0, 'M'},
// //     {"port", 1, 0, 'p'},
// // //     {"sick", 1, 0, 's'},
// // //     {"urg", 1, 0, 'u'},
// // //     {"sim", 1, 0, 'v'},
// //     {0, 0, 0, 0}
// //   };
// //   bool ask4help = false;
// //   int opt;
// //   int option_index = 0;
// //   const int MTTYL = 50;
// //   const int MSL = 500;
// //   char moduleDir[MSL] = "";
// //   char res[MSL];
// //   bool getMlist = false;
// //   FILE * mf;
// //   int n;
// //   char *p1, *p2;
//   // set default name for (camera) cinfig file
// /*  snprintf(configFileCam, MAX_PATH_LENGTH, "%s/robcam.conf", getenv("HOME"));*/
//   // for path values
// //   imagePath[0] = 0;
// //   dataPath[0] = 0;
// //   // and server port
// //   serverPort=defaultServerPort;
//   ask4help = setCommonPathAndOtherOptions(argc, argv, iniScript, iniScriptCnt, &asDaemon, defaultServerPort);
//   // take path from command line
// //   while (not ask4help) 
// //   {
// //     opt = getopt_long(argc, argv, "ahi:d:p:s:f:",
// //                  long_options, &option_index);
// //     if (opt == -1)
// //       break;
// //     switch (opt)
// //     {
// //       case -1: // no more options
// //         break;
// //       case 'i': // path setting for images
// // //         if (optarg != NULL)
// // //           strncpy(imagePath, optarg, MAX_PATH_LENGTH);
// //         break;
// //       case 'a': // run without console input
// //         *deamon = true;
// //         break;
// //       case 'd': // path setting for data (results)
// // //         if (optarg != NULL)
// // //           strncpy(dataPath, optarg, MAX_PATH_LENGTH);
// //         break;
// // /*      case 'f': // (camera) config file
// //         if (optarg != NULL)
// //           strncpy(configFileCam, optarg, MAX_PATH_LENGTH);
// //         break;*/
// //       case 's': // init script filename
// //         if (optarg != NULL)
// //           strncpy(iniScript, optarg, iniScriptCnt);
// //         break;
// //       case 'p': // port number
// //         if (optarg != NULL)
// //           sscanf(optarg, "%d", &serverPort);
// //         break;
// // //       case 'm': // default servername
// // //         if (optarg != NULL)
// // //           strncpy(moduleList, optarg, moduleListCnt);
// // //         break;
// // //       case 'M': // modulelist directory
// // //         if (optarg != NULL)
// // //           strncpy(moduleDir, optarg, MSL);
// // //         getMlist = true;
// // //         break;
// // //       case 's': // serial device for SICK scanner
// // //         if (optarg != NULL)
// // //           strncpy(ttySick, optarg, MTTYL);
// // //         break;
// // //       case 'u': // serial device for URG (hokuyo) scanner
// // //         if (optarg != NULL)
// // //           strncpy(ttyUrg, optarg, MTTYL);
// // //         break;
// // //       case 'v': // host and port for simulated laserscanner
// // //           if (optarg != NULL)
// // //             strncpy(ttySim, optarg, MTTYL);
// // //           break;
// //       case 'h':
// //         ask4help = true;
// //         //printHelp();
// //         break;
// //       default:
// //         printf("Unknown option '%c'\n", opt);
// //         break;
// //     }
// //   }
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
// //                 strncat(moduleList, " ", moduleListCnt-1);
// //               strncat(moduleList, res, moduleListCnt-1);
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
//   //dummy.print("dummy");
//   return ask4help;
// }

///////////////////////////////////////////////////////

void shutDownHandler(int signal)
{ // print signal message - if from signal
  //
  if (signal > 0)
    // print signal message
    psignal(signal, "Signal");
  if (server != NULL)
  { //
    if (not server->isStopping())
    { // stop the server
      printf("Termination server (%d) ...\n", signal);
      server->stop(true); // all client connection
      printf("Deleting server (%s) ...\n", appName);
      delete server;
      server = NULL;
      printf("Server terminated\n");
    }
  }
  if (signal > 0)
    // stop process now!
    exit(0);
}



//////////////////////////////////////////

bool runLaserServerComponent(bool daemon,
//                             char * ttySick, char * ttyUrg, char * ttySim,
                            const char * iniScript)
{
  //UServerStatic * server;
  UServerPort * servPort;
  const char * appNameV = "Laser scanner server - " __SERVER_VERSION__ " (" __DATE__ " " __TIME__ " jca@oersted.dtu.dk)";
  char * p1;
//  UFunctionBase dummy;
//   const int MSL = 200;
//   char s[MSL];
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
  //
  // create command execution object
  server = new UServerStatic();
  server->setVerbose(false);
  // create socket server object
  servPort = new UServerPort();
  servPort->setVerbose(false);
  servPort->setServerNamespace("laserServer");
  servPort->setServerNamespaceAttribute(
      "name=\"ulmsserver\" version=\"" __SERVER_VERSION__ "\"");
  servPort->setAllowConnections(false);
  //
  // add server components to command execute object
  server->setServer(servPort); // socket server
  servPort->setPort(serverPort); // set port number
  // start message handling thread - no wait for modules to be in place
  server->handleMessagesThreadStart();

  // give the server a name
  server->setName(appNameV);
  // load initialization script
  if (strlen(iniScript) > 0)
    server->executeScriptFile(iniScript, true, -1);
  //
  if (not daemon)
  { // read old command history
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
  //
  if (daemon)
    printf("Running %s\n"
        " - on port %d\n"
        " - without console access - press ^C to kill\n", appName, servPort->getPort());
  else
    printf("Started laser scanner server\n"
        " - on port %d\n"
        " - type help for command list\n", serverPort);
  // open the server port
  // server is now assumed configured, and clients can be allowed
  servPort->setAllowConnections(true);
  servPort->start();
  // run small keyboard loop (to enable status print and quit)
  line[0] = '\0';
  while (line[0] != 'q')
  {
    if (daemon)
      Wait(10.0);
    else
    { // use
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
      free(line);
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
        switch (line[0])
        {
          case 'p': // print status
            server->print("Server ");
            break;
          default: // help
            printf("Short help list:\n");
            printf("  q               Quit server (without use of signals)\n");
//            printf("  p               Print status for server (almost the same as 'module list')\n");
            printf("  [N:]command     Send 'command' to this server\n");
            printf("                  the N: may be used to fake a command as if from client N\n");
            printf("                  'server clients' gets a list of client numbers\n");
            printf("  help            Main (server core) help text\n");
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
    // wait for server to finish queued commands
    // server->consoleLock.wait();
  }

  if (not daemon)
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
  printf("Normal shutdown\n");
  shutDownHandler(0);
  return true;
}


//////////////////////////////////////////////////////

void dummyCall()
{ // just to define the classes needed by plugin modules
  UClientHandler dummy1;
  UResIfVar dummy2("dummy");
  USmrCl dummy3;
  UDataDouble dummy4;
  UClientFuncCam dummy5;
  UClientFuncImage dummy6;
  ULogFile dummy7;
  USmlFile dummy8;
  UFuncPlugBase dummy9;
  USmlString dummy10;
  ULogFile dummy11;
  UImg3Dpoints dummy14;
  dummy14.makePCLFile("ccc", false, false);
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

