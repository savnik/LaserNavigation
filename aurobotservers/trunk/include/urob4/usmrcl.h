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

#ifndef USMRCL_H
#define USMRCL_H

#define MAX_REPLY_LENGTH 1000
#define MAX_SMR_CMD_LENGTH 200

#include <ugen4/u3d.h>
#include <ugen4/ulock.h>
#include <umap4/upose.h>

#include "ulogfile.h"
#include "usockclient.h" // @todo - replace USockClient by UClientPort

/**
Class to hold current odometry state from the robot
The class is protected by a lock, that should be used before
read and write of consistent data */

class USmrOdoState : public ULock
{
public:
  /**
  Constructor */
  USmrOdoState();
  /**
  Log state to logfile */
  void logState(FILE * logf);
  /**
  Print pose status to console (no locking) */
  void print(const char * prestring);

public:
  /**
  Pose and last update time */
  UPoseTime pose;
  /**
  Turn since last update */
  double dh; // turn since last update
  /**
  Time since last update */
  double dt; // time since last update
  /**
  Current velocity */
  double velocity;
  /**
  Total traveled distance (in m) */
  double dist;
  /**
  Flags for wich data is read from rs485 bus */
  unsigned int readFlags;
};


/**Connection to SMR-CL
  *@author Christian Andersen
  */

class USmrCl : public ULock
{
public:
  /**
  Constructor */
  USmrCl();
  /**
  Destructor */
  virtual ~USmrCl();
  /**
  Get stop condition on last finished command */
  inline int getStopCondition()
    { return cmdLineStopCnd; };
  /**
  Disconnect and stop command loop. */
  bool doDisconnect();
  /**
  Set socket port number */
  inline int setPort(int iPort) { return port = iPort;};
  /**
  Set socket port number */
  inline int getPort() { return port;};
  /**
  Set socket port number */
  int setHost(const char * iHost)
  {
    strncpy(host, iHost, MAX_HOST_LENGTH);
    return strlen(host);
  };
  /**
  Set socket port number */
  inline char * getHost() { return host;};
  /**
  Is connection established */
  inline bool isConnected() { return connected; };
  /**
  Is log open */
  inline bool isLogging() { return logIO.isOpen(); };
  /**
  Get logging mode */
  inline int getLogMode() { return logIOMode; };
  /**
  Set logging mode (
  0 = just errors.
  1=errors + output to smr,
  3=errors + output + info,
  4=errors + output + info + input from smr) (everything)  */
  inline void setLogMode(int value) { logIOMode = value; };
  /**
  Try connect to socket, NB! there is no loop that listens for events. */
  bool tryConnect();
  /**
  Set input - output log for the connection to MRC.
  if mode is 0, nothing is logged - except errors.
  if mode is 1, all output is logged, including ID events numbers
  if mode is 2, all input and output is logged. */
  bool setIOLog(const char * path, const char * name,
                      const int mode);
  /**
  Close logfile */
  void setIOLogClosed();
  /**
  Set odometry logfile.
  The filename is appended with timestamp and a '.log' extension.
  sample: path/name _yyyymmdd_hhmmss.ddd.log */
  bool setOdoLog(const char * path, const char * name)
  { return false; };
  /**
  Close odometry log */
  bool setOdoLogClosed()
  { return false; };
  /**
  A new pose state is available in the 'odoState' structure */
  virtual void eventPoseUpdated(bool streamSource);
  /**
  A new GPS position is received from MRC with these values.
  The function should be overwritten at a higher level
   * \param heading may be from odo-gps kalman filter
  to prevent printout of the received position. */
  virtual void eventGpsUpdate(UPoseTime odoState,
                              double easting, double northing, double heading,
                              double quality, double satellites,
                              double dop);
  /**
  A new INS data is received from MRC with these values.
  The function should be overwritten at a higher level
  to prevent printout of the received position.
  virtual void eventInsUpdate(UPoseTime odoState,
                              double accx, double accy, double accz,
                              double roll, double tilt,
                              double pan, double insTime); */
  virtual void eventInsUpdate(UPoseTime odoState,
                              double accx, double accy, double accz,
                              double roll, double tilt, double pan, double insTime);
  /**
   * the hako variables may be updated - tell possible users.
   * \param hakoManual manual or automatic enabled - probably useless
   * \param liftPos is the detected lift position - 100 is about center 80 is high and 120 is low
   * \param ptoSpeed is the RPM? of the power take off.
   * all from: "stream %d \"$hakomanual\" \"$hakoliftinggearpos\" \"$hakopowertakeoffspeed\"\n", */
  virtual void eventHakoVarUpdate(int hakoManual, int liftPos, int ptoSpeed)
  { printf("USmrCl::eventHakoVarUpdate: manual:%d, liftPos:%d, PTOSpeed:%d\n", hakoManual, liftPos, ptoSpeed); };
  /**
  Set odometry update time */
/*  double getOdoUpdateTimeInterval()
  { return odoEvery; };*/
  /**
  Start odometry in stream mode.
  Return true if true */
  bool startOdoStream(const int samplesInterval);
  /**
  Set verbose message level */
  inline void setVerbose(bool value)
  { verbose = value; };
  /**
  Get new simulated or replay data, as determined
  by this virtual function. It should be filled
  by an appropriate virtual function.
  Returns true if new data is filled into
  the odoState. */
  virtual bool getSimulatedPose();
  /**
  Send this buffer of data and
  wait for reply.
  Returns true if send and reply received.
  Reply 'eventtimeout' and 'streem' do not count as reply. */
/*  bool sendSMR(const char * cmd,
               double timeoutSec,
               const char ** lastReply);*/
  /**
  Send a drive command and wait for the queue number.
  If the queued reply is available within the timeout period, its
  number is returned at 'lineID', else last queued line is used, and
  the function returns false.
  If 'mustBeQueued', then a IDXX queued must be handled within timeout
  period to return true. */
/*  bool sendSMRgetId(const char * cmd,
                     double timeoutSec,
                     int * lineID,
                     bool mustBeQueued);*/
  /**
   * Get the value of a variable from the MRC (MRC), if the value is received within timeout
   * period, 'value' will hold the returned value.
   * \param varName is the variable name in the MRC (MRC), possibly including the initial '$' as needed.
   * \param timeoutSec Minimum timeout period is about 0.75 sec if message flow is low,
   * so a low 'timeoutSec' is replaced by this period.
   * \param value is a pointer to the place where the value is returned.
   * \param valStr is a buffer string, where the reply string from the MRC is placed.
   * \param valStrCnt is the length of the buffer
   * \returns true if a value is received. */
  bool sendSMReval(const char * varName, double timeoutSec,
                   double * value, char * valStr = NULL, const int valStrCnt = 0);
  /**
  Print smrcl connection status */
  void print(const char * prestring);
  /**
  Listen to the line from the smr and handle any stream data (and not handled getevents.
  Returns true if data is received. */
  bool handleLineData(const int timeoutMs);
  /**
   * Send string direct to smr, and do not wait for a replay
   * The connection is locked while sending to avoid conflict with service thread
   * and menoeuvre delivery.
   * \param s1 string to send
   * \param s2 any additional string to send, e.g. a terminaing line feed or extra stop condition
   * \param s3 any additional string to send, e.g. a terminaing line feed
   * \returns true if send within the timeout period */
  bool sendString(const char * s1, const char * s2 = NULL, const char * s3 = NULL);
  /**
   * Start read thread */
  bool start();
  /**
   * Stop read thread */
  void stop(bool andWait);
  /**
   * Run the listen thread - is started by a start() call and stopped by stop(true). Do NOT call this function directly. */
  void run();
  /**
   * Close and reset the mrc log, but maintain log variables
   * \param restart if true, the logging is restarted */
  void saveMrcLog(bool restart);

protected:
  /**
  There is info to report. this
  function may be overwritten by
  parent class. */
  virtual void info(const char * msg, int type = 0); //scInfo);
  /**
  Get a reply from MRC,
  wait no longer than timout for the reply.
  Reply may be split over more than one
  message block, or more than one line may be received
  Function returns when a '\n' is found in the reply, or a timeout
  has occured.
  Returns true if data is available.
  Setc connected flag if a connection error occured.
  Received data is stored in 'reply[]',
  with length in replyCnt */
  bool getLineFromSocket(int timeout_ms);
  /**
  Listen to the line for the timeout period of time and
  collect the reply to the reply buffer. The timeout
  should be about 10 ms, to ensure that the reply
  to the request is available.
  If 'waitFullTime' is false, then function returns after
  first data is received (or timeout).
  Reply and replyCnt holds the reply.
  Returns true if new data is available, and false
  if timeout. */
  bool getDataFromLine(int timeout, bool waitFullTime);
  /**
  Shut down the socket connection in a proper way */
  virtual void closeConnection();
  /**
  trach event to se if last 'drive' command is started or
  executed.
  Returns true if a hata in line were handled.
  The parameter flags is set according to line type. */
  bool setDriveState(bool * idEevent, bool * eventTimeout, bool * streamData);
  /**
  Get odometer measurements command from MRC (as eval)
  Returns true if valid reply is available within timeout (in seconds) */
  //bool measureOdo(double timeoutSec, UPoseTime * poseTime);
  /**
  Save this string in 'log' (communication log),
  if 'log' is open and logmode is
  higher or equal to this 'logLevel' */
  void toLog(const char * logString, const int logLevel);
  /**
  Test for avaiable message in reply buffer, that is
  not yet handled */
  bool isMessageInBuffer();
  /**
  Called when a getevent returnes a change in line status, i.e.
  a queued, finished or started */
  virtual void lineStateUpdated();
  /**
  A connection change has happened.
  The function is calle if connected or disconnected */
  virtual void connectionChange(bool connected);
  /**
   * \brief A watch event occured, update as needed
   * \param name is the name of the watch fired
   * \param atTime is the MRC time reported. */
  virtual void eventWatchFired(const char * name, double atTime)
  {};
  /**
   * An interface specification for handling of user events
   * \param eventstring is the string returned by MRC after the
   * keyword 'userevent' and stripped for whitespace. */
  virtual void gotUserEvent(const char * eventString)
  {};

private:
  /**
  Send a message to the MRC and return true
  if send.
  connected is set to false, if it is not possible to send. */
  bool sendOnly(const char * cmd);
  /**
  Listen for a line, and handle any known reply types
  these are IDXX queued, started, stopcond, flushed, syntaxErr,
  stream (assumed odometry), if handled then 'handled' is returned true.
  Returnes after first line, but more may be available.
  'gotReply' holds the received line.
  Returns true if a line were received.
  Function sets 'connected' flag as apprppriate.
  If simulated flag is set, then odometry data is updated as appropriate. */
  bool listen(int timeout, const char ** gotReply, bool * handled,
              bool * handledID,
              bool * handledEventTimeout,
              bool * handledStream);
  /**
  Handle all inut and empty 'event' queue, until an eventtimeout has
  occured. the 'Eventtimeout' could be short, as each receive should be handled
  in one MRC sampletime (10 ms) + transmission time.
  Returns true if an event timeout were received. */
  bool handleGetevents(const int timeoutMs);

public:
  /** robot position in own coordinates
      set when requested */
  USmrOdoState odoState;
  /** stop robot now! */
  bool cmdStop;
  /**
   * When true the control loop will try and reestables connection if
   * the connection fails. */
  bool tryHoldConnection;
  //
public:
  /**
  Last line reply */
  int  cmdLineQueued;
  /** command is started - not completed */
  int  cmdLineStarted;
  /** command is finished (robot has stopped) */
  int  cmdLineFinished;
  /** stop condition for last command */
  int  cmdLineStopCnd;
  /** Last line with syntaxerror */
  int cmdLineSyntaxError;
  /** got a user event with this number */
  int cmdLineUserEvent;
  /// number of doubles in gps streaming array (E,N,H, mode, sats, dof)
  static const int maxGpsVals = 6;
  /**
  Gps streamed values. The values are
  Easting, Northing, Satellites, dop, not used.
  DOP is Dillution Of Prediction 1 is perfect, 6 is OK, 50 is bad. */
  double gpsVals[maxGpsVals];
  /** number of stored INS values */
  static const int INSV = 7;
  /**
  INS values last received - beeing accx, accy, accz, roll, tilt, pan, time */
  double ins[INSV]; // ins streamed values

protected:
  /**
  Is connection to smrCl established */
  bool connected;
  /**
  Stop connection */
  bool stopRead;
  /**
  Is thread running */
  bool running;
  /**
  Name of host to connect to */
  char host[MAX_HOST_LENGTH];
  /**
  Port used when connecting to host */
  int port;
  /** reply from MRC, may be a line and a bit */
  char reply[MAX_REPLY_LENGTH];
  /**
  Bytes valid in reply */
  int replyCnt;
  /** last fully received line from MRC */
  char replyLine[MAX_REPLY_LENGTH];
  /**
  Number of reads (with >0 bytes returned). */
  unsigned int readCnt;
  /**
  Number of receive errors */
  unsigned int  errCnt;
  /**
  Number of transmitted messages */
  unsigned int txCnt;
  /**
  Number of connections tried */
  unsigned int connCnt;
  /**
  Make verbose messages (debug and info)
  - warnings and errors atr always shown */
  bool verbose;
  /**
  Log outgoing and incomming values, including
  Log getevent, eval, and all incomming.
  0 = no logging
  1 = log outgoing only
  2 = log all */
  int logIOMode;
  /**
   * Eval result string length */
  static const int MaxEvalLen = 500;
  /**
   * Eval string as received */
  char evalResult[MaxEvalLen];
  /**
   * semaphore when waiting for eval result, implemented
   * as request number and request and result number.
   * evalReq is incremented before eval is send, and
   * evalRes is set to evalReq on reply received.
   * This is to allow recovery when MRC goes down or do
   * not accept eval (e.g. syntax error) */
   int evalReq, evalRes;
   /**
    * Stream IMU data or hako tool details */
   bool streamImu;

public:
  /**
  Logfile handle for general communication (debug) logging */
  ULogFile logIO;
  /**
  Show evert N stream odometry update */
  int streamShowOdoEvery;
  /**
  Show evert N stream gps update */
  int streamShowGpsEvery;
  /**
  Show evert N stream ins update */
  int streamShowInsEvery;
  /**
  Count of odometry stream updates */
  int streamShowOdoCnt;
  /**
  Count of gps stream updates */
  int streamShowGpsCnt;
  /**
  Count of gps stream updates */
  int streamShowInsCnt;
  /**
  Count of gps stream updates */
  double streamInsTime;

private:
  /**
   * Thread handle for read loop. */
  pthread_t  thRead;
  /**
  Socket */
  int sock; // socket
  /** set true, whenever a valid event reply is received */
  bool gotGeteventReply;
};



#endif
