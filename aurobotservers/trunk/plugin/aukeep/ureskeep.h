/***************************************************************************
 *   Copyright (C) 2006 by DTU (by Christian Andersen, Lars m.fl.)         *
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

#ifndef URESKEEP_H
#define URESKEEP_H

#include <cstdlib>

#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>
#include <urob4/userverpush.h>
#include <urob4/ulogfile.h>
#include <urob4/uclienthandler.h>
#include <urob4/uvarcalc.h>

/**
 * Class to hold the needed items to montor the state of a process or a server */
class UKeepItem : public UVarCalc
{
public:
  /**
   * Constructor */
  UKeepItem();
  /**
   * DEstructor */
  ~UKeepItem();
  /**
   * Update the process monitor for this connection */
  bool tick();
  /**
   * Get a handle to a push object for process, or overall */
  UServerPush * getPushHandle(bool on)
  {
    if (on)
      return eventOn;
    else
      return eventOff;
  };
  /**
   * Set the (server) command to be issued, to start the (not running) process */
 // void setStartCmd(const char * startCmd);
  /**
   * Set the (server) command to be issued, to stop the (not running) process */
  //void setStopCmd(const char * startCmd);
  /**
   * Send the stop command now. If wait is true, then this will terminate the process.
   * If wait is false, then this will restart the process */
  //void sendStop();
  /**
   * Set the wait flag. If waiting, then the monitoring of this keep is off (and no action taken)
   * When the flag is switched on, then the monitoring is restarted.
   * \param value is the new value for the wait flag. */
  void setWait(bool value);
  /**
   * Set the receiver of the event commands (the implementor)
   * \param implementor implements the commands */
  void setCore(UCmdExe * pCore);
  /**
   * Set the name of the keep process - may also be the process to keep.
   * \param name is the symbol name of the keep process. */
  void setName(const char * name);
  /**
   * Set the expression that the keep function is to monitor.
   * \param expFor which of the three expressions are to be set -1=unknown, 0=off, 1=on
   * \param expr is the expression that should evaluate to true (exp > 0.5) when process is OK. */
  void setExpr(int expFor, const char * expr);
  /**
   * Set number of seconds to allow a process to start (typically after an off condition)
   * \param secs is number of seconds to wait before first monitoring action takes place. */
  void setAllow(double secs);
  /**
   * Set number of seconds to allow a process to start (typically after an off condition)
   * \param value allow off as a steady state, when value is true, otherwise an
   * off condition will change state to unknown, and thus retrigger an off event after
   * the allowed allow (to start) time. */
  void setAllowOff(bool value);
  /**
   * Create the items base vaiables in a var pool */
  bool createBaseVar();
  /**
   * Test if this this keep matches the provided name
   * \param name the proveded name to compare against */
  bool isA(const char * name);
  /**
   * Is this keep in wait mode
   * \returns true if waiting */
  inline bool isWait()
  { return varWait->getValueBool(); };
  /**
   * Update the monitoring process for this keep.
   * \param evaluated is set to false if a syntax error is detected in the expression.
   * \returns true if the expression is evaluated higher than 0.5, i.e. true. */
  bool monitorTest(int exprFor, bool * evaluated = NULL);
  /**
   * Print current status to this buffer string
   * \param preStr start by inserting this string into buffer
   * \param buff start of buffer
   * \param bufCnt length of buffer
   * \returns a pointer to the buffer */
  const char * print(const char * preStr, char * buff, const int buffCnt);
  /**
   * Set pointer to the resource logfile */
  void setLogFile(ULogFile * logf)
  { logfile = logf; };

protected:
  /**
   * Client connection to the other server */
  //UClientHandler * cnn;
  /**
   * number of iterations */
  int tickCnt;
  /**
   * Last time off was detected */
  UTime offTime;
  /**
   * Last tick time */
  UTime tickTime;
  /**
   * logfile to save events */
  ULogFile * logfile;
  /**
   * on process is on handler*/
  UServerPush * eventOn;
  /**
   * on process is on handler*/
  UServerPush * eventOff;
  /// pointer to name variable
  UVariable * varKeepName;
  /// pointer to expression to monitor
  UVariable * varOnExpr;
  /// pointer to expression to monitor
  UVariable * varOffExpr;
  /// pointer to expression to monitor
  UVariable * varUnknownExpr;
  /// pointer to wait variable for this keep
  UVariable * varWait;
  /// pointer to result of monitoring process
  UVariable * varRunningOK;
  /// number of monitorng actions
  UVariable * varMonitorCnt;
  /// number of monitorng actions
  UVariable * varOffCnt;
  /// allow this many seconds after start command is issued
  UVariable * varAllow;
  /// allow off state to be stable
  UVariable * varAllowOff;
  /**
   * interval between ticks */
  UVariable * varTickInterval;
  /// pointer to server core (to send start and stop commands using "do")
  UCmdExe * core;
};


/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendents) as shown.

@author Christian Andersen
*/
class UResKeep : public UResVarPool, public ULogFile, public UServerPush
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResKeep) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResKeep()
  { // set name and version
    setResID("keep", 410);
    UResKeepInit();
  };
  /**
  Destructor */
  virtual ~UResKeep();
  /**
   * Initialize resource */
  void UResKeepInit();
  /**
   * Create any resources that this modules needs
   * This method is called after the resource is registred by the server. */
  virtual void createResources();
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  The varPool has methods, and a call to one of these is needed.
  Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed.
  * If the returnStruct and returnStructCnt is not NULL, then
    a number (no more than initial value of returnStructCnt) of
    structures based on UDataBase may be returned into returnStruct array
    of pointers. The returnStructCnt should be modified to the actual
    number of used pointers (if needed).
  * If the call is allowed, but the result is invalid for some reason, the
    return value 'value' can be set to som agreed value, (e.g. 0.0 (false) and 1.0 for true). */
  virtual bool methodCall(const char * name, const char * paramOrder,
                  char ** strings, const double * pars,
                  double * value,
                  UDataBase ** returnStruct,
                  int * returnStructCnt);

// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  // Public functions that this resource provides for all users.
  /**
  Run the time loop to test for jobs to be activated.
  This call do not return until the threadStop flag is set true. */
  void run();
  /**
   * Get handle to relevant push handler
   * \param process if empty, the full resource handler is requested
   * else process is expected to be the name of the process.
   * \param eventOn if false, then the eventOff handler is returned.
   * \returns the requested event handler, or NULL if not found. */
  UServerPush * getPushHandle(const char * process, bool eventOn);
  /**
   * Get keep item pointer
   * \param name is the name of the keep
   * \returns NULL if a keep with this name is not found */
  UKeepItem * getKeepItem(const char * name);
  /**
   * Delete a keep, and stop any monitoring related to this keep
   * \param name is the name of the keep
   * \returns true if deleted keep, false if not existed. */
  bool deleteKeep(const char * name);
  /**
   * Add a new keep item */
  UKeepItem * add(const char * name);
  /**
   * Get number of current keep processes
   * \returns number of keeps, active or not */
  int getKeepsCnt()
  { return keepsCnt; };
  /**
   * Get list of current keeps
   * \param preStr is a short string, that is added at the start of the buffer.
   * \param buff is the buffer, where to write the list.
   * \param buffCnt is the size of the buffer
   * \returns a pointer to buff */
  const char * getKeepList(const char * preStr, char * buff, const int buffCnt);
  /**
   * Set the wait flag. If waiting, then the monitoring of all keep items are off (and no action taken)
   * When the flag is switched on, then the monitoring of (non waiting) keep items are restarted.
   * \param value is the new value for the wait flag. */
  void setWait(bool value);

protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();
  /**
  Start read thread
  \return Returns true if the read thread started. */
  bool start();
  /**
   * Stop read thread - and wait for thread join
   * \param andWait wait for thread to finish (ignored, waits always)
  */
  void stop(bool andWait);

public:
  /**
   * Is the resource to output verbose message to console */
  bool verbose;

protected:
  /**
  Thread handle for frame read thread. */
  pthread_t threadHandle;
  /**
  Is thread actually running */
  bool threadRunning;
  /**
  Should thread stop - terminate */
  bool threadStop;
  /// handle to push event handler, when any keep process goes off
  UServerPush * eventOff;
  /// maximum number of keeps
  static const int MAX_KEEPS_CNT = 100;
  /// array of keep handles
  UKeepItem * keeps[MAX_KEEPS_CNT];
  /// Number of keeps allocated
  int keepsCnt;
  //
  /// idle count
  UVariable * varIdleCnt;
  /// number of commands that returned error
  UVariable * varErrCnt;
  /// sample period time for keep thread
  UVariable * varKeepTime;
  /// pointer to wait for all keep items
  UVariable * varWait;
};

#endif

