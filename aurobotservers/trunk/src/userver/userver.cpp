/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *   $Rev: 59 $:                                                        *
 ***************************************************************************/
/************************** Version control information ***************************/
#define SERVER_VERSION   "$Rev:: 59  $:"
#define DATE             "$Date:: 2012-08-07 08:55:56 +0200 (Tue, 07 Aug 2012) $:"
/**********************************************************************************/


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>

#define USE_READLINE_LIB
#ifdef USE_READLINE_LIB
#include <readline/readline.h>
#include <readline/history.h>
#endif

#include "userverstatic.h"

#include <ugen4/utime.h>
#include <ugen4/ucommon.h>
#include <ugen4/udatabase.h>
#include <urob4/userverport.h>
#include <urob4/ufunctionbase.h>
#include <urob4/ucmdexe.h>
#include <urob4/uclientfuncimage.h>
#include <urob4/uvarcalc.h>

//#include <ucam4/ufunctionimage.h>
//#include <ucam4/ufunctioncam.h>
#include <urob4/ufunctionimgpool.h>
#include <urob4/ufunctionposehist.h>

#include <urob4/uclienthandler.h>
#include <urob4/uresclientifvar.h>
#include <urob4/usmrcl.h>
#include <urob4/ulogfile.h>
#include <urob4/usmlfile.h>
#include <urob4/ufuncplugbase.h>
#include <urob4/usmlstring.h>
#include <urob4/ulogfile.h>

#include <urob4/uclientfunccam.h>
#include <ugen4/uimg3dpoint.h>
//#include "userverstatic.h"

//#define __SERVER_VERSION__ "2.496"

/// pointer to server object
UServerStatic * server = NULL;

using namespace std;

// functins defined in this file
void shutDownHandler(int signal);
bool runServer(bool deamon, const char * iniScript);
void dummyCall();
bool addCmdHist(const char * line, char * lineHist[],
                const int MHL, const int MLL,
                int * lineHistCnt, int * lineHistNewest);

////////////////////////////////////////////

int main(int argc, char *argv[])
{
  bool ask4help;
  bool asDaemon = false;
  const int defaultServerPort = 24926;
  const int MIL = 250;
  char iniScript[MIL];
  struct sigaction nact;
  /* Set up the structure to specify the signal action. */
  nact.sa_handler = shutDownHandler;
  sigemptyset (&nact.sa_mask);
  nact.sa_flags = 0;
  // catch the following actions
  sigaction (SIGINT, &nact, NULL);
  sigaction (SIGHUP, &nact, NULL);
  sigaction (SIGTERM, &nact, NULL);
  // set default initialisation script filename
  s_argc = argc;
  s_argv = argv;
  // get command line options
  ask4help = setCommonPathAndOtherOptions(argc, argv, iniScript, MIL,
                                          &asDaemon,  defaultServerPort,
                                          "a Mobotware server"
                                         );
  if (not ask4help)
  {
    // run the server
    getIniFiles(iniScript);
    runServer(asDaemon, iniScript);
  }
  //
  return EXIT_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////

// void printHelp()
// {
//   printf("\n");
//   printf("Server version " SERVER_VERSION " command line options:\n");
//   printf(" -h --help :  This help message\n");
//   printf(" -a --deamon  Run server as deamon (no console input)\n");
// /*  printf(" -i --imagepath <path>\n");
//   printf("    Sets path where images are expected to be found and written\n");
//   printf(" -d --datapath <path>\n");
//   printf("    Sets path where data other than images are expected to be found and written\n");*/
//   printf(" -s --script <file>\n");
//   printf("    filename for initialization script (command script) default is ./%s.ini\n", appName);
//   printf(" -p --port <port>\n");
//   printf("    sets the server port number\n");
//   printf("\n");
// }

//////////////////////////////////////////

// bool getCmdLineOptions(int argc, char *argv[],
//                       int defaultServerPort,
//                       //char * moduleList, int moduleListLength,
//                       bool * deamon,
//                       char * iniScript, const int iniScriptCnt)
// {
//   static struct option long_options[] = {
//     {"script", 1, 0, 's'},
//     {"help", 0, 0, 'h'},
//     {"deamon", 0, 0, 'a'},
//     {"port", 1, 0, 'p'},
//     {0, 0, 0, 0}
//   };
//   bool ask4help;
//   int opt;
//   int option_index = 0;
//   // set default values
//   ask4help = setCommonPathAndOtherOptions(argc, argv, "", defaultServerPort);
//   // take path from command line
//   if (not ask4help)
//     while(true)
//     {
//       opt = getopt_long(argc, argv, "ahi:d:p:s:", long_options, &option_index);
//       if (opt == -1)
//         break;
//       switch (opt)
//       {
//         case -1: // no more options
//           break;
//         case 'a': // run as deamon
//           *deamon = true;
//           break;
//         case 's': // optional config file
//           if (optarg != NULL)
//             strncpy(iniScript, optarg, iniScriptCnt);
//           break;
//         case 'p': // specific server port
//           if (optarg != NULL)
//             sscanf(optarg, "%d", &serverPort);
//           break;
//         case 'i':
//         case 'd':
//         case 'h':
//           // already handled options
//           break;
//         default:
//           printf("Unknown option '%c'\n", opt);
//           break;
//       }
//     }
//   //
//   return ask4help;
// }

///////////////////////////////////////////////////////

void shutDownHandler(int signal)
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
    }
    delete server;
    server = NULL;
  }
  if (signal > 0)
  { // stop process now!
    printf("Server terminated, but try 'q' next time!\n");
    printf("(console reset may be needed)\n");
    exit(0);
  }
  else
    printf("Server terminated\n");
}

///////////////////////////////////////////////////////

bool runServer(bool asDaemon, const char * iniScript)
{
  const char * appNameV = "aurs-" SERVER_VERSION " (" __DATE__ " " __TIME__ " jca@oersted.dtu.dk)";
  char * p1;
  const int MLL = 300;
  char lineBuff[MLL];
  char * line = lineBuff;
  int client;
  // server pointers
  // UServerStatic * server;
  UServerPort * servPort;
  // UFunctionCam * fcam;
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
  // create server core
  server = new UServerStatic(); //UCmdExe();
  // give the server a name
  server->setName(appNameV);
  server->setVerbose(false);
  // create socket object
  servPort = new UServerPort();
  servPort->setVerbose(false);
  servPort->setPort(serverPort);
  servPort->setServerNamespace("uServer");
  servPort->setServerNamespaceAttribute("name=\"uServer\" version=\"" SERVER_VERSION "\"");
  // add port to server
  server->setServer(servPort);
  // start message handling thread
  server->handleMessagesThreadStart();
  //
  if (strlen(iniScript) > 0)
  { // execute any commands stored in initial scriptfile
    server->executeScriptFile(iniScript, true, -1);
  }
  //
  if (not asDaemon)
  {
    // read old command history
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
  // allow initial commands to finish
  Wait(0.2);
  //
  printf("Running %s\n"
      " - on port %d\n"
      " - type help for command list\n",  appName, servPort->getPort());
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
      {
        line = lineBuff;
        continue;
      }
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
      printf(">> ");
      fgets(line, MLL, stdin);
#endif
      // get possible start number
      client = strtol(line, &p1, 10);
      // execute command
      if (line[1] < ' ')
      {
        if (line[0] == 'q')
          // quit
          break;
        switch (line[0])
        {
          case 'p': // print status
            server->print("Server ");
            break;
          default: // help
            printf("Short help list:\n");
            printf("  q               Quit server\n");
            printf("  p               Print status for server (almost the same as 'module list')\n");
            printf("  [N:]command     Send 'command' to this server\n");
            printf("                  the N: may be used to fake a command as if from client N\n");
            printf("                  Print status (p) to get list of client numbers\n");
            printf("  h               This help text\n");
            printf("  help            Main help text\n");
            break;
        }
      }
      else if (*p1 == ':')
      { // send as from another client (see status for client numbers)
        p1++;
        server->postCommand(client, p1);
      }
      else
      { // else just send line as if from the server console
        server->postCommand(-1, line);
      }
    }
  }
  //
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
  printf("Normal shutdown.\n");
  shutDownHandler(0);
  //
/*  server->stop(true); // all client connection
  printf("Server terminated\n");*/
  //
  return true;
}

///////////////////////////////////////////////////

void dummyCall()
{ // just to define the classes needed by (some) plugin modules
  UClientHandler dummy1;
  UResIfVar dummy2("dummy");
  USmrCl dummy3;
  UClientFuncCam dummy4;
  UDataDouble dummy5;
  UClientFuncImage dummy6;
  ULogFile dummy7;
  USmlFile dummy8;
  UFuncPlugBase dummy9;
  //UFunctionCamBase dummy10; // nej
  USmlString dummy11;
  ULogFile dummy12;
  UVarCalc dummy13;
  UImg3Dpoints dummy14;
  dummy14.makePCLFile("bbb", false, false);
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

///////////////////////////////////////////////////


