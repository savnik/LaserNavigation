/***************************************************************************
 *   Copyright (C) 2006 by Christian   *
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


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <locale.h>

#include <iostream>
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>

#define USE_READLINE_LIB
#ifdef USE_READLINE_LIB
#include <readline/readline.h>
#include <readline/history.h>
#endif

#ifdef USE_HIGHGUI
#ifdef OPENCV2
#include <highgui/highgui_c.h>
#else
#include <opencv/highgui.h>
#endif
#endif

#include <ugen4/utime.h>
#include <ugen4/ucommon.h>
#include <urob4/userverport.h>
#include <urob4/ufunctionbase.h>
#include <urob4/ucmdexe.h>
#include <urob4/uclienthandler.h>
#include <gst/gst.h>

#include "userverstatic.h"

#define __SERVER_VERSION__ "2.1965"


using namespace std;

// functins defined in this file
void shutDownHandler(int signal);
bool runServer(bool asDaemon,
               const char * iniScript,
               const char * cmdHist);
void printHelp(const char * appName, int defPort);
bool getCmdLineOptions(int argc, char *argv[],
                       int defaultServerPort,
//                       char * moduleList, int moduleListLength,
                       bool * deamon,
                       char * iniScript, const int iniScriptCnt);
void dummyCall();
bool addCmdHist(const char * line, char * lineHist[], const int MHL, const int MLL,
                int * lineHistCnt, int * lineHistNewest);

/**
 * Pointer to the main server instance */
UServerStatic * server = NULL;

////////////////////////////////////////////

int main(int argc, char *argv[])
{
  bool ask4help;
//   const int MFL = 1000;
//   char moduleList[MFL] = "";
  bool asDaemon = false;
  const int defaultServerPort = 24929;
  const int MIL = 260;
  char iniScript[MIL] = "";
  struct sigaction nact, oact;
  //
  // gstreamer init
  guint major, minor, micro, nano;
  gst_init (&argc, &argv);
  gst_version (&major, &minor, &micro, &nano);
  printf ("This program is linked against GStreamer %d.%d.%d %d\n",
            major, minor, micro, nano);  
    // end
#ifdef USE_HIGHGUI
  // init openCV highgui (sets locale to native default on PC)
  cvInitSystem(0, NULL );
#endif
  // crash and quit handler
    // Registering the interrupt handler, catching
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

  // change locale back to standard C - i.e. use '.' as decimal sign.
  // set default initialisation script filename
  // get command line options
  ask4help = setCommonPathAndOtherOptions(argc, argv, iniScript, MIL, &asDaemon, defaultServerPort, "Mobotware client");
  //ask4help = getCmdLineOptions(argc, argv, defaultServerPort, /*moduleList, MFL,*/ &deamon, iniScript, MIL);
  if (not ask4help)
  { // run the server
    getIniFiles(iniScript);
    runServer(asDaemon, iniScript, appName);
    printf("Server '%s' terminated.\n", appName);
  }
  //
  return EXIT_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////

// void printHelp(const char * appName, int defPort)
// {
//   printf("\n");
//   printf("Server command line options:\n");
//   printf(" -h --help :  This help message\n");
//   printf(" -a --deamon  Run server as deamon (else console is a server client console)\n");
//   printf(" -p --port <port>\n");
//   printf("    sets the server port number (default is %d)\n", defPort);
//   printf(" -s --script <filename>\n");
//   printf("    loads a command script for the server (default is ./%s.ini)\n", appName);
//   printf("\n");
// }

//////////////////////////////////////////

// bool getCmdLineOptions(int argc, char *argv[],
//                        int defaultServerPort,
// //                       char * moduleList, int moduleListLength,
//                        bool * deamon,
//                        char * iniScript, const int iniScriptCnt)
// {
//   static struct option long_options[] = {
//     {"imagepath", 1, 0, 'i'},
//     {"datapath", 1, 0, 'd'},
//     {"script", 1, 0, 's'},
//     {"help", 0, 0, 'h'},
//     {"deamon", 0, 0, 'a'},
// //     {"modules", 1, 0, 'm'},
// //     {"modDir", 1, 0, 'M'},
//     {"port", 1, 0, 'p'},
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
//   setCommonPathAndOtherOptions(argc, argv, "", defaultServerPort);
//   // take path from command line
//   while(true)
//   {
//     opt = getopt_long(argc, argv, "ahi:d:p:s:", long_options, &option_index);
//     if (opt == -1)
//       break;
//     switch (opt)
//     {
//       case 'i': // path setting for images
// //         if (optarg != NULL)
// //           strncpy(imagePath, optarg, MAX_PATH_LENGTH);
//         break;
//       case 'a': // path setting for images
//         *deamon = true;
//         break;
//       case 'd': // path setting for data
// //         if (optarg != NULL)
// //           strncpy(dataPath, optarg, MAX_PATH_LENGTH);
//         break;
//       case 's': // optional config file
//         if (optarg != NULL)
//           strncpy(iniScript, optarg, iniScriptCnt);
//         break;
//       case 'p': // specific server port
//         if (optarg != NULL)
//           sscanf(optarg, "%d", &serverPort);
//         break;
// //       case 'm': // modulelist
// //         if (optarg != NULL)
// //           strncpy(moduleList, optarg, moduleListLength);
// //         break;
// //       case 'M': // modulelist directory
// //         if (optarg != NULL)
// //           strncpy(moduleDir, optarg, MSL);
// //         getMlist = true;
// //         break;
//       case 'h':
//         ask4help = true;
//         //printHelp();
//         break;
//       default:
//         printf("Unknown option '%c'\n", opt);
//         break;
//     }
//   }
//   // test if imagePath and dataPath are valid - else try current or home dirctory
// //  testPathSettings();
//   //
//   // get module list from module directory
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
// //                 strncat(moduleList, " ", moduleListLength);
// //               strncat(moduleList, res, moduleListLength);
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

///////////////////////////////////////////////////////

bool runServer(bool asDaemon,
               const char * iniScript,
               const char * cmdHist)
{
  char * p1;
  const int MLL = 300;
  char lineBuff[MLL];
  char * line = lineBuff;
  int client, i;
  FILE * cmdf;
  char cmdfName[MLL];
  // server pointers
  //UServerStatic * server;
  UServerPort * servPort;
  const int MLH = 100; // max line history
  char lhb[MLH][MLL];
  char *lineHist[MLH];
  int lineHistCnt = 0;
  int lineHistNewest = -1;
  // UFunctionCam * fcam;
  //
  // create server core
  server = new UServerStatic();
  // give the server a name
  server->setName(appName);
  server->setVerbose(false);
  // create socket object
  servPort = new UServerPort();
  servPort->setVerbose(false);
  servPort->setPort(serverPort);
  servPort->setServerNamespace("auClient");
  servPort->setServerNamespaceAttribute("name=\"auClient\" version=\"" __SERVER_VERSION__ "\"");
  // add port to server
  server->setServer(servPort);
  //
  // start message handling thread
  server->handleMessagesThreadStart();
  //
  if (strlen(iniScript) > 0)
  { // execute any commands stored in initial scriptfile
    server->executeScriptFile(iniScript, true, -1);
  }
  //
  //
  if (not asDaemon)
  {
    snprintf(cmdfName, MLL, "%s.cmd", cmdHist);
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
  if (asDaemon)
    printf("Running %s\n"
           " - on port %d\n"
           " - without console access - press ^C to kill\n", appName, servPort->getPort());
  else
    printf("Starting %s\n"
        " - on port %d\n"
        " - type help for command list\n", appName, serverPort);

  // allow clients to connect to port
  servPort->start();
  //
  // run small keyboard loop (if not a deamon)
  line[0] = '\0';
  while (line[0] != 'q')
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
        // add to
        add_history(line);
      // add a new-line at the end
      snprintf(lineBuff, MLL, "%s\n", line);
      // free buffer allocated by readline()
//      if (line != lineBuff and line != NULL)
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
            printf(" - image path: '%s'\n", imagePath);
            printf(" - data  path: '%s'\n", dataPath);
            server->print("Status ");
            //cams->print("Camera pool:");
            //imgs->print("ImagePool");
            break;
            default: // help
              printf("Short help list:\n");
              printf("  q               Quit server\n");
              printf("  p               Print status for server (almost the same as 'module list')\n");
              printf("  cam command     Send command to camera server\n");
              printf("  laser command   Send command to laser scanner server\n");
              printf("  'if' command    Send command to another interface - see 'module reslist' for interface resources\n");
              printf("  command         Send 'command' to this server see module list for handled commands\n");
              printf("  [N:]command     the N: may be used to fake a command as if from client N\n");
              printf("                  try 'server clients' to get list of client numbers\n");
              printf("  help            Main help text\n");
//              printf("  static [module] Load one of the available static modules (try static help)\n\n");
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
  shutDownHandler(0);
  //
  return true;
}

/////////////////////////////////////////////

void dummyCall()
{ // just to define the classes neede by plugin modules
  UClientHandler dummy1;
}

////////////////////////////////////////////////////

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
////////////////////////////////////////

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
      delete server;
      server = NULL;
      printf("Server terminated\n");
    }
  }
  if (signal > 0)
    // stop process now!
    exit(EXIT_SUCCESS);
}

