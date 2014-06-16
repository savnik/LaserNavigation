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

#include <stdio.h>
#include <stdlib.h>


#define USE_READLINE_LIB
#ifdef USE_READLINE_LIB
#include <readline/readline.h>
#include <readline/history.h>
#endif

//#define USE_PYTHON
#ifdef USE_PYTHON
//#include <python2.7/Python.h>
#define PY_MAJOR_VERSION 2
#define PY_MINOR_VERSION 7
#include <boost/python.hpp>
using namespace boost::python;
#endif

#include "urawserverport.h"


URawServerPort::URawServerPort()
{
  isStopping = false;
  qeRunning = false;
  qeStop = false;
  startQueueEmpty();
}


URawServerPort::~URawServerPort()
{
  stop(true);
  stopQueueEmpty(true);
}


/////////////////////////////////////

#ifdef USE_PYTHON
#include <string>
class GV
{
  public:
    GV()
    {
      a=89;
      g=89.89;
      snprintf(buf, MSL, "eee                    a");
      s = buf;
    }
  double g;
  double h[2];
  int a;
  std::string s;
  static const int MSL = 45;
  char buf[MSL];
};

BOOST_PYTHON_MODULE(varGlobal) {
  boost::python::class_<GV>("globalVar")
    .def_readwrite("g", &GV::g)
//    .def_readwrite("h", &GV::h)
    .def_readwrite("a", &GV::a)
    .def_readwrite("s", &GV::s);
}
#endif

#ifdef USE_PYTHON
/// test of interface to python
static int numargs = 55;
GV globalvar;
object main_namespace;

extern "C"
{
  static  PyObject * emb_numargs(PyObject *self, PyObject *args)
  {
    int isOK;
      isOK = PyArg_ParseTuple(args, ":numargs");
      printf("This is a c-function called from python, called with %d arguments\n", isOK);
      if (not isOK)
        return NULL;
      return Py_BuildValue("i", numargs);
      // "i" means integer
  }

  static PyMethodDef EmbMethods[] = {
      {"numargs", emb_numargs, METH_VARARGS,
      "Return the number of arguments received by the process."},
      {NULL, NULL, 0, NULL}
  };

}

#endif


bool URawServerPort::runServer(const char * xmlTag)
{
  const char * appNameV = "uServer-" __SERVER_VERSION__ " (" __DATE__ " " __TIME__ " jca@oersted.dtu.dk)";
  char * p1;
  const int MLL = 300;
  char lineBuff[MLL];
  char * line = lineBuff;
  int client;
  //
  FILE * cmdf;
  char cmdfName[MLL];
  const int MLH = 10; // max line history
  char lhb[MLH][MLL];
  char *lineHist[MLH];
  int lineHistCnt = 0;
  int lineHistNewest = -1;
  int i;
  int defClient = 0;
  UServerClient * cnn;
  bool python = false;
  //
  // create socket object
  setVerbose(false);
  setPort(serverPort);
  if (xmlTag != NULL)
  {
    setServerNamespace(xmlTag);
    setServerNamespaceAttribute("name=\"auServerTest\" version=\"" __SERVER_VERSION__ "\"");
    setServerNamespaceUse(true);
  }
  else
    setServerNamespaceUse(false);
  // start message handling thread
  //
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
  //
  printf("%s\n"
      " - on port %d\n"
         " - h=help, q=quit, t=timestamped log, p=print client status\n",  appNameV, getPort());
  // allow clients to connect to port
  start();
  setServerNamespaceUse(false);
  setAllowConnections(true);
  logTimeStamp = true;
  //
  // python
#ifdef USE_PYTHON
  try
  {
    int isOK;
    PyObject * obj;
    Py_Initialize();
    obj = Py_InitModule("emb", EmbMethods); 
    if (obj == NULL)
      printf("no object\n");
    initvarGlobal();
    object main_module((
      handle<>(borrowed(PyImport_AddModule("__main__")))));
    main_namespace = main_module.attr("__dict__");
    isOK = PyRun_SimpleString("import varGlobal");
    if (not isOK)
      printf("import returned %d\n", isOK);
    main_namespace["var"] = ptr(&globalvar);
    isOK = PyRun_SimpleString("from math import *");
    if (not isOK)
      printf("import of math returned %d\n", isOK);
    isOK = PyRun_SimpleString("from numpy import *");
    if (not isOK)
      printf("array options with 'numpy' is not available - (try to install 'python-numpy')\n");
    PyRun_SimpleString("import os");
    PyRun_SimpleString("import emb");
    PyRun_SimpleString("print emb.numargs()");
  }
  catch (error_already_set &)
  {
    PyErr_Print();
  }
#endif
  //
  // run small keyboard loop (if not a deamon)
  line[0] = '\0';
  while (line[0] != 'q')
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
    printf(">> ");
    fgets(line, MLL, stdin);
#endif
    // get possible start number
    client = strtol(line, &p1, 10);
    //
    if (strlen(line) == 1 and line[0] == 'q')
      break;
    // execute command
    if (python)
    {
#ifdef USE_PYTHON
      // python
      try
      {
        PyRun_SimpleString(line);
      }
      catch (error_already_set &)
      {
        PyErr_Print();
      }
#else
      python = false;
#endif
    }
    else if (line[1] < ' ' and (line[0] == 'q' or line[0] == 'p' or line[0] == 'h' or line[0] == 't'))
    {
      if (line[0] == 'q')
        // quit
        break;
      switch (line[0])
      {
        case 'p': // print status
          printf("Default client is %d\n", defClient);
          print("status:");
          break;
        case 't': // togle log timestamp
          logTimeStamp = !logTimeStamp;
          if (logTimeStamp)
            printf("next connection will use timestamp in logfile\n");
          else
            printf("next connection will NOT use timestamp in logfile\n");
          break;
        default: // help
          printf("Short help list:\n");
          printf("  q               Quit server\n");
#ifdef USE_PYTHON
          printf("  y               python command\n");
#endif
          printf("  p               Print client list\n");
          printf("  [N:]command     Send 'command' to client N (default is 0)\n");
          printf("  t               Log communication with time stamp (else raw direct) is %s\n", bool2str(logTimeStamp));
          printf("  h               This help text\n");
          break;
      }
    }
    else if (strncmp(p1, "python", 6) == 0)
    {
      python = (strstr(p1, "end") == NULL);
      printf("Python mode %s\n", bool2str(python));
    }
    else if (*p1 == ':')
    { // send as from another client (see status for client numbers)
      p1++;
      cnn = getClient(client);
      if (cnn != NULL)
      {
        defClient = client;
        cnn->blockSend(p1, strlen(p1), 1000);
      }
      else
        printf("No client %d connected\n", defClient);
    }
    else
    { // else just send line as if from the server console
      cnn = getClient(defClient);
      p1 = line;
      if (cnn != NULL)
        cnn->blockSend(p1, strlen(p1), 1000);
      else
        printf("No client %d connected\n", defClient);
    }
  }
  //
  // save cmd history
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
  //
  printf("Normal shutdown.\n");
  shutDownHandler(0);
  //
/*  server->stop(true); // all client connection
  printf("Server terminated\n");*/
  //
  return true;
}

/////////////////////////////////////////////

///////////////////////////////////////////////

void * runQueueEmpty(void * server)
{ // this is a separate thread
  URawServerPort * obj = (URawServerPort *) server;
  //
  obj->run();
  //
  pthread_exit(NULL);
  return NULL;
}


///////////////////////////////////////////////

void URawServerPort::startQueueEmpty()
{
  pthread_attr_t  thConAttr;
  //
  // else use default port
  if (not qeRunning)
  { // Starts socket server thread 'runSockServer'
    pthread_attr_init(&thConAttr);
    // create socket server thread
    pthread_create(&thQE, &thConAttr, &runQueueEmpty, (void *)this);
    pthread_attr_destroy(&thConAttr);
  }
  else
  {
    fprintf(stderr, "UServerPort::start: Server port read loop is running already\n");
  }
}

//////////////////////////////////////////////////

void URawServerPort::stopQueueEmpty(bool andWait)
{
  // stop server thread.
  qeStop = true;
  // wait for server thread to terminate
  if (andWait and qeRunning)
    pthread_join(thQE, NULL);
}


void URawServerPort::run()
{
  bool isOK;
  int idle = 0;
  UTime t, t2;
  const double MAX_IDLE_WAIT_TIME = 0.007; // after executing commands without wait, wait a bit
  int idleLoopCnt = 0;
  //
  qeRunning = true;
  t.now();
  while (not qeStop)
  { // tell clients that server is alive
    isOK = handleOneMessageFromQueue();
    if (isOK)
    { // nothing to do - so wait
      idleLoopCnt++;
      idle++;
      t2.now();
    }
    else
    { // idle, so wait a bit
      t2.now();
      Wait(MAX_IDLE_WAIT_TIME);
    }
  }
  qeRunning = false;
}

//////////////////////////////////////////////

bool URawServerPort::handleOneMessageFromQueue()
{
  bool result = false;
  UServerInMsg * msg;
  const int MSL = 100;
  char s[MSL];
  const char * p1;
  //
  msg = getRxQueue()->skipToNextMessage(false);
  if (msg != NULL)
  { // ensure prompt is not printed to server console
    consoleLock.lock();
    //n = server->getRxQueue()->getNextOut();
    p1 = msg->message;
    if (strcasecmp(p1, "getevent") == 0)
      result = false;
    else
    {
      msg->rxTime.getTimeAsString(s);
      fprintf(stdout, "%lu.%06lu (%s) client %d:\n%s\n",
              msg->rxTime.getSec(), msg->rxTime.getMicrosec(),
                            s,    msg->client, msg->message);
    }
    consoleLock.unlock();
    result = true;
  }
  return result;
}

/////////////////////////////////////

void URawServerPort::terminate()
{
  stop(true);
  stopQueueEmpty(true);
}

///////////////////////////////////////////////////

bool URawServerPort::addCmdHist(const char * line, char * lineHist[],
                                const int MHL, const int MLL,
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

////////////////////////////////////////////////

void URawServerPort::gotNewClient(UServerClient * cnn)
{
  //
  printf("\nGot new client (%s)\n", cnn->getClientName());
  cnn->queueRawData = true;
  cnn->setLogTimestamp(logTimeStamp);
  bool isOpen = cnn->logOpen();
  if (isOpen)
  {
    if (logTimeStamp)
      printf("Logging raw data and time stamp to %s\n", cnn->getLogFilename());
    else
      printf("Logging raw received data to %s\n", cnn->getLogFilename());
  }
  else
    printf("failed to open logfile %s\n", cnn->getLogFilename());
  //
}

////////////////////////////////////////////////

