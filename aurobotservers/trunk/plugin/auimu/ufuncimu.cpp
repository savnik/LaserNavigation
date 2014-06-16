/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#include <ugen4/ucommon.h>
#include <urob4/uvariable.h>
#include <urob4/uresposehist.h>
#include <urob4/uvarcalc.h>

#include "ufuncimu.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object
#include <urob4/uvariable.h>

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncImu' with your classname, as used in the headerfile */
  return new UFuncImu();
}

#endif


void UFuncImu::init()
{
  varRot = NULL;
  varAcc = NULL;
  varGyro = NULL;
  varMag = NULL;
  varDevice = NULL;
  varOpen = NULL;
  varTime = NULL;
  varCnt = NULL;
  varErrCnt = NULL;
  varErrMsg = NULL;
  varUpdateRate = NULL;
  varBaud = NULL;
  varDataSample = NULL;
  threadRunning = false;
  threadStop = false;
}

///////////////////////////////////////////////////

bool UFuncImu::handleCommand(UServerInMsg * msg, void * extra)
{
  const int MRL = 500;
  char reply[MRL];
  bool aLog, doLog, aOpen, doOpen;
  // check for parameter 'help'
  if (msg->tag.getAttValue("help", NULL, 0))
  { // create the reply in XML-like (html - like) format
    sendHelpStart("IMU");
    sendText("--- IMU is an interface plugin for the IMU used in Paroll projekct\n");
    snprintf(reply, MRL, "log=true|false      Opens or closes the logfile %s (open=%s)\n", logf.getLogFileName(), bool2str(logf.isOpen()));
    sendText(reply);
    snprintf(reply, MRL, "open[=false]        Open connection to IMU (open=%s)\n", bool2str(varIsOpen->getBool()));
    sendText("help       This message\n");
    sendText("see also 'var imu' for status and opter options\n");
    sendHelpDone();
  }
  else
  { // get any command attributes (when not a help request)
    aLog = msg->tag.getAttBool("log", &doLog, true);
    aOpen = msg->tag.getAttBool("open", &doOpen, true);
    if (aOpen)
    {
      varOpen->setValued(doOpen);
      if (doOpen)
        sendInfo("requested port to open - see: var imu.isOpen");
      else
        sendInfo("port close requested - see: var imu.isOpen");
    }
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
      sendInfo(reply);
    }
  }
  return true;
}

/////////////////////////////////////////////////////////////////

void UFuncImu::createResources()
{
  varTime = addVar("time", 0.0, "t", "(r) Time of last update");
  varCnt = addVar("cnt", "1", "d", "(r) Update count");
  varRot = addVar("rot", "1 2 3", "3d", "(r) Device rotation roll, pitch and yaw - in degrees");
  varRot->makeTimeSeries(30000, 70.0);
  varAcc = addVar("acc", "1 2 3", "3d", "(r) Acceleration measurement [x, y, z] - in device units");
  varAcc->makeTimeSeries(10000, 10.0);
  varGyro = addVar("gyro", "1 2 3", "3d", "(r) Gyro measurement [x, y, z] in device units");
  varGyro->makeTimeSeries(10000, 10.0);
  varMag = addVar("mag", "1 2 3", "3d", "(r) Compas measurement [x, y, z] in device units");
  varMag->makeTimeSeries(10000, 10.0);
  varDevice = addVar("device", "/dev/ttyUSBS0", "s", "(rw) device name for the IMU");
  varBaud = addVar("baud", 57600.0, "d", "(rw) specified baudrate (set to -1 if not used)");
  varOpen = addVar("open", 0.0, "d", "(rw) is device to be open or closed");
  varIsOpen = addVar("isopen", 0.0, "d", "(r) is device open (0=closed, 1=open (no data), 2=receiving)");
  varErrCnt = addVar("errCnt", 0.0, "d", "(r) number of format errors");
  varErrMsg = addVar("errMsg", "no error", "s", "(r/w) last error message");
  varUpdateRate = addVar("updaterate", 0.0, "d", "(r) Update rate in Hz");
  varDataSample = addVar("dataSample", "none", "s", "(r) sample of inputdata every 10 sec");
// set as time stamped variable
//  varAcc->setHasTime(30.0, 30*100, false);
  // start the receive thread
  start();
}

////////////////////////////////////////////////////////////////

/*
Output format by Sparkfun's "9DOF Razor IMU" (rev 1.0 demo software)
!ANG:-0.48,0.37,23.70,AN:388,384,381,-4,-1,257,346,-191,744
!ANG:-0.47,0.37,23.76,AN:388,384,380,-4,-2,257,346,-191,744
!ANG:-0.51,0.36,23.83,AN:388,384,381,-6,-2,260,346,-191,744
!ANG:-0.52,0.41,23.89,AN:388,384,380,-2,0,258,342,-186,735
!ANG:-0.50,0.35,23.94,AN:388,384,381,-3,-1,258,342,-186,735
}*/

///////////////////////////////////////////////////////////////

void * startUFuncImuThread(void * obj)
{ // call the hadling function in provided object
  UFuncImu * ce = (UFuncImu *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////

bool UFuncImu::start()
{
  bool result = true;
  pthread_attr_t  thAttr;
  //
  if (not threadRunning)
  {
    pthread_attr_init(&thAttr);
    //
    threadStop = false;
    // create socket server thread
    result = (pthread_create(&threadHandle, &thAttr,
              &startUFuncImuThread, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return result;
}

///////////////////////////////////////////////////

void UFuncImu::stop(bool andWait)
{
  if (threadRunning and not threadStop)
  { // stop and join thread
    threadStop = true;
    pthread_join(threadHandle, NULL);
  }
}

/////////////////////////////////////////////////////

void UFuncImu::run()
{
  int updCnt = 0;
  UTime t;
  FILE * imuDev = NULL;
  const int MBL = 200;
  char buf[MBL];
  char * bp;
  int magx, magy, magz;
  //
  if (threadRunning)
    // prevent nested calls;
    return;
  t.now();
  while (not threadStop)
  {
    if (varOpen->getBool() and not varIsOpen->getBool())
    { // device is to be open - try
      if (varBaud->getInt() > 0)
      {
        setDeviceSpeed(varDevice->getValues(), varBaud->getInt());
        Wait(0.3);
      }
      imuDev = fopen(varDevice->getValues(), "r");
      if (imuDev == NULL)
      { // file not found
        varErrMsg->setValues("File not found (readable)", 0, true);
        printf("UFuncImu::run: IMU device %s could not be opened\n", varDevice->getValues());
        Wait(1.0);
      }
      else
        varIsOpen->setValued(true, 0);
    }
    else if (not varOpen->getBool() and varIsOpen->getBool())
    { // device is to be closed
      fclose(imuDev);
      imuDev = NULL;
      varIsOpen->setValued(false, 0);
    }
    // now do some decodeing
    if (varIsOpen->getBool())
    { // decode with locked device - to allow consistant data
      bp = fgets(buf, MBL, imuDev);
      if (bp != NULL)
      { // there is data
        if (strlen(bp) > 4 and strchr(bp, '\n') != NULL)
        { // there is a full line
          UTime updT;
          updT.now();
          lock();
          // make sure noone reads data while beeing updated
          getVarPool()->lock();
          if (strncmp(bp, "!ANG:", 5) == 0)
          { // angles
            UPosition p3;
            bp += 5;
            p3.y = strtod(bp, &bp); bp++; // roll
            p3.x = strtod(bp, &bp); bp++; // pitch
            p3.z = strtod(bp, &bp);
            if (*bp++ == ',')
              varRot->set3D(&p3, &updT);
          }
          // followed by raw sensor data
          if (strncmp(bp, "AN:", 3) == 0)
          { // raw sensor data
            UPosition p3;
            bp += 3;
            p3.y = strtod(bp, &bp); bp++;
            p3.x = strtod(bp, &bp); bp++;
            p3.z = strtod(bp, &bp);
            if (*bp++ == ',')
              varGyro->set3D(&p3, &updT);
            p3.y = strtod(bp, &bp); bp++;
            p3.x = strtod(bp, &bp); bp++;
            p3.z = strtod(bp, &bp);
            if (*bp++ == ',')
              varAcc->set3D(&p3, &updT);
            magy = strtol(bp, &bp, 10); bp++;
            magx = strtol(bp, &bp, 10); bp++;
            magz = strtol(bp, &bp, 10);
            if (*bp == '\n' or *bp == '\r')
            { // line has ended - do end bits
              if (magx != varMag->getInt(0) or magy != varMag->getInt(1) or magz != varMag->getInt(2))
              { // magnetometer is updated at a slower rate, so compare to last value
                varMag->setValued(magx, 0, false, &updT);
                varMag->setValued(magy, 1, false, &updT);
                varMag->setValued(magz, 2, false, &updT);
              }
              varCnt->add(1.0);
              updCnt++;
              varTime->setTime(updT);
              varIsOpen->setValued(2.0, 0, false, &updT);
            }
            else
            { // line end not found where expected, so issue error message
              varErrMsg->setValues("format error: ", 0, true, &updT);
              varErrMsg->setValues(buf, 14, true, &updT);
              varErrCnt->add(1.0);
              varIsOpen->setValued(1.0, 0, false, &updT);
            }
          }
          getVarPool()->unlock();
          unlock();
        }
      }
    }
    if (t.getTimePassed() > 10.0 and updCnt > 0)
    {
      varUpdateRate->setValued(updCnt / 10.0);
      updCnt = 0;
      t.now();
      varDataSample->setValues(buf, 0, true);
    }
    //
    if (not varIsOpen->getBool())
      Wait(1.0);
  }
}

