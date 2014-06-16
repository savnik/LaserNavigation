/***************************************************************************
 *   Copyright (C) 2009 by Christian Andersen   *
 *   jca@elektro.dtu.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>

#include <ugen4/utime.h>
#include <ugen4/ucommon.h>
#include <ugen4/udatabase.h>
#include <urob4/userverport.h>

#include <urob4/ulogfile.h>


#include "urawserverport.h"

using namespace std;

#ifdef USE_PYTHON
//#include <python2.7/Python.h>
#define PY_MAJOR_VERSION 2
#define PY_MINOR_VERSION 7
#include <boost/python.hpp>
using namespace boost::python;
#endif

// functins defined in this file
void printHelp();
bool getCmdLineOptions(int argc, char *argv[],
                      int defaultServerPort, const char * xmlTag);
URawServerPort server;

////////////////////////////////////////////

int main(int argc, char *argv[])
{
  bool ask4help;
  const int defaultServerPort = 24921;
  struct sigaction nact;
  const char * xmlTag = NULL;
  /* Set up the structure to specify the signal action. */
  nact.sa_handler = shutDownHandler;
  sigemptyset (&nact.sa_mask);
  nact.sa_flags = 0;
  // catch the following actions
  sigaction (SIGINT, &nact, NULL);
  sigaction (SIGHUP, &nact, NULL);
  sigaction (SIGTERM, &nact, NULL);
  //
  // set default initialisation script filename
  if (strrchr(argv[0], '/') == NULL)
      // application found in $PATH
    appName = argv[0];
  else
  { // application started using absolute or relative path
    appName = strrchr(argv[0], '/');
    appName++;
  }
#ifdef USE_PYTHON
  printf("NB! Compiled with python test software!\n");

  Py_SetProgramName((char*)appName);  /* optional but recommended */
//  Py_Initialize();
//   try {
//     PyRun_SimpleString("result = 5 ** 2");
// 
//     object module(handle<>(borrowed(PyImport_AddModule("__main__"))));
//     object dictionary = module.attr("__dict__");
//     object result = dictionary["result"];
//     int result_value = extract<int>(result);
//     double result_d = extract<double>(result);
// 
//     printf("result is %d or as double %f\n", result_value, result_d);
// 
//     dictionary["result"] = 20;
// 
//     PyRun_SimpleString("print result");
//   } catch (error_already_set)
//   {
//     PyErr_Print();
//   }
#endif
  // get command line options
  ask4help = getCmdLineOptions(argc, argv, defaultServerPort, xmlTag);
  // do help or run server
  if (ask4help)
    printHelp();
  else
  { // run the server
    sprintf(dataPath,"log_%s", appName); // log to log_auservertest/
    sprintf(imagePath,"log_%s", appName); //
    testPathSettings();
    server.runServer(xmlTag);
  }
  //
#ifdef USE_PYTHON
  Py_Finalize();
#endif
  //
  return EXIT_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////

void printHelp()
{
  printf("\n");
  printf("%s Mobotware server stub " __SERVER_VERSION__ " a port server that echo all to console\n"
         " and send all you type to client\n", appName);
  printf(" command line options:\n");
  printf(" -p --port <port>\n");
  printf("    Sets the server port number\n");
  printf(" -n --namespace <tagname for XML connection>\n");
  printf(" -h --help :  This help message\n");
  printf("\n");
}

//////////////////////////////////////////

bool getCmdLineOptions(int argc, char *argv[],
                       int defaultServerPort, const char * xmlTag)
{
  static struct option long_options[] = {
    {"help", 0, 0, 'h'},
    {"namespace", 1, 0, 'n'},
    {"port", 1, 0, 'p'},
    {0, 0, 0, 0}
  };
  bool ask4help = false;
  int opt;
  int option_index = 0;
  serverPort = defaultServerPort;
  // take path from command line
  while(true)
  {
    opt = getopt_long(argc, argv, "hp:n:", long_options, &option_index);
    if (opt == -1)
      break;
    switch (opt)
    {
      case -1: // no more options
        break;
      case 'p': // specific server port
        if (optarg != NULL)
          sscanf(optarg, "%d", &serverPort);
        break;
      case 'n': // specific server port
        xmlTag = optarg;
        break;
      case 'h':
        ask4help = true;
        break;
      default:
        printf("Unknown option '%c'\n", opt);
        break;
    }
  }
  //
  return ask4help;
}

///////////////////////////////////////////////////////

void shutDownHandler(int signal)
{ // print signal message - if from signal
  //
  if (signal > 0)
    // print signal message
    psignal( signal, "Signal");
  if (not server.isStopping)
  { // stop the server
    printf("Terminating server (%d) ...\n", signal);
    server.terminate(); // all client connection
  }
  if (signal > 0)
  { // stop process now!
    printf("(console reset may be needed)\n");
    exit(0);
  }
  else
    printf("Server terminated\n");
}

///////////////////////////////////////////////////////

