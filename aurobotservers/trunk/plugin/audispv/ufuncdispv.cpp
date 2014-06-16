/***************************************************************************
 *   Copyright (C) 2010 by DTU (Christian Andersen)                        *
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

#include <urob4/uvariable.h>
#include <urob4/uresposehist.h>
#include <urob4/uvarcalc.h>
#include <urob4/uimagepool.h>

#include "ufuncdispv.h"

//#if (CV_MAJOR_VERSION >= 1)

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object
#include <urob4/uvariable.h>

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncDispV' with your classname, as used in the headerfile */
  return new UFuncDispV();
}

#endif

UFuncDispV::~UFuncDispV()
{
}

///////////////////////////////////////////////////

bool UFuncDispV::handleCommand(UServerInMsg * msg, void * extra)
{ // message is unhandled
  bool result = false;
  //
  if (msg->tag.isTagA("dispv"))
    result = handleDispv(msg);
  else if (msg->tag.isTagA("dispvPush"))
    result = handlePush(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFuncDispV::handleDispv(UServerInMsg * msg)
{
  const int MRL = 500;
  char reply[MRL];
  // check for parameter 'help'
  if (msg->tag.getAttValue("help", NULL, 0))
  { // create the reply in XML-like (html - like) format
    sendHelpStart("DISPV");
    sendText("--- DISPV is an implementation of openCV dispv\n");
    snprintf(reply, MRL, "log=true|false      Opens or closes the logfile %s (open=%s)\n", logf.getLogFileName(), bool2str(logf.isOpen()));
    sendText(reply);
    sendText(            "do                  Show widget\n");
    sendText(            "silent              do not send a reply to this command\n");
    sendText("help       This message\n");
    sendText("----\n");
    sendText("see also: dispvPush for event handling of 3d cloud\n");
    sendText("see also: 'var dispv' for dispv generation parameters\n");
    sendHelpDone();
  }
  else
  { // get any command attributes (when not a help request)
    bool doLog = false;
    bool aLog = msg->tag.getAttBool("log", &doLog, true);
    bool anUpdate = msg->tag.getAttBool("do", NULL, false);
    bool silent = msg->tag.getAttBool("silent", NULL);
    //
    // implement command
    if (aLog)
    { // open or close the log
      aLog = logf.openLog(doLog);
      if (doLog)
        snprintf(reply, MRL, "Opened logfile %s (%s)", logf.getLogFileName(), bool2str(aLog));
      else
        snprintf(reply, MRL, "closed logfile %s", logf.getLogFileName());
      // send an information tag back to client
      if (not silent)
        sendInfo(reply);
    }
    if (anUpdate)
    { // default update call
      qmw->show();
      sendInfo("Widget should be shown");
    }
  }
  return true;
}

/////////////////////////////////////////////////////////////////

void UFuncDispV::createResources()
{
  // Create global variables - owned by this plug-in.
  // Returns a pointer to the the variable for easy access.
  varUpdateCnt = addVar("updCnt", 0.0, "d", "(r) Number of updates");
  varTime = addVar("time", 0.0, "t", "(r) Time at last update");
  start();  
}

///////////////////////////////////////////////////

void * startGUIThread(void * obj)
{ // call the hadling function in provided object
  UFuncDispV * ce = (UFuncDispV *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////

bool UFuncDispV::start()
{
  int err = 0;
  pthread_attr_t  thAttr;
  //
  if (not threadRunning)
  {
    pthread_attr_init(&thAttr);
    // create socket server thread
    err = (pthread_create(&threadHandle, &thAttr,
              &startGUIThread, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return err;
}

/////////////////////////////////////////////////

void UFuncDispV::stop(bool andWait)
{
  if (threadRunning)
  { // stop and join thread
    qapp->quit();
    if (andWait)
      pthread_join(threadHandle, NULL);
  }
}

/////////////////////////////////////////////////

void UFuncDispV::run()
{
  QApplication app(s_argc, s_argv);
  qapp = &app;
  //
  threadRunning = true;
  app.setOrganizationName("DTU");
  qmw = new QMainWindow();
  QLabel* l = new QLabel( qmw );
  l->setText( "Hello World!" );
  qmw->setCentralWidget( l );
  QAction* a = new QAction(qmw);
  a->setText( "Quit" );
  qmw->connect(a, SIGNAL(triggered()), SLOT(close()) );
  qmw->menuBar()->addMenu( "File" )->addAction( a );
  //show gui
  qmw->show();
  app.exec();
  threadRunning = false;
}

//#endif

