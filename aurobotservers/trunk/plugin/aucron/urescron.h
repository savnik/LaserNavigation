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

#ifndef URESCRON_H
#define URESCRON_H

#include <cstdlib>

#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>
#include <urob4/userverpush.h>
#include <urob4/ulogfile.h>

/**
 * Class to hold one active bash job, that issues a command and saves result to logfile with timing info. */
class UCronJob
{

public:
  /**
   * Constructor */
  UCronJob()
  {
    jobLog = NULL;
    reuseCnt = 0;
    active = false;
    cmdStr = "";
    resInt = 0;
    pf = NULL;
    errCnt = 0;
  };
  /**
   * destructor */
  ~UCronJob();
  /**
   * Run the cron job */
  int runJob();
  /**
   * Get the reply from the job in this buffer
   * \param buff is the destination buffer
   * \param buffCnt is the length of the buffer
   * \returns a pointer to buff  */
  const char * getResultStr(char * buff, const int buffCnt);
  /**
   * Start a new thread with the job 
   * \returns true if the thread were started */
  bool start();

public:
  /// length of resultt buffer
  static const int MAX_RES_STR_SIZE = 500;
  /// result string (first part)
  char resStr[MAX_RES_STR_SIZE];
  /// start time of command
  UTime tStart;
  /// end time of command 
  double pipeTime;
  /// logfile for trsults
  ULogFile * jobLog;
  /// reuse of this job structure
  int reuseCnt;
  /// reuse of this job structure
  int errCnt;
  /// is a command active
  bool active;
  /// command string to be executed
  const char * cmdStr;
  /// result at close of pipe to command 
  int resInt;
  /// stop (a too long) job as soon as possible
  bool stopJob;

protected:
  /// The pipe file handle for the slot
  FILE * pf;
    /**
  Thread handle for job execution. */
  pthread_t threadHandle;

};


/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResCron : public UResVarPool, public ULogFile, public UServerPushImplement
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResCron) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResCron()
  { // set name and version
    setResID("cron", 203);
    UResCronInit();
  };
  /**
  Destructor */
  virtual ~UResCron();
  /**
   * Initialize resource */
  void UResCronInit();
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
   * Service function for push queue - to store owner of function */
  int findFunctionOwner(const char * tagName);
  /**
   * Get number of available finished jobs */
  int getResultStringsCnt();
  /**
   * Get result of all available job descriptors to this buffer.
   * \param last get result string for the 'last' jobs performed
   * \param buff is the destination buffer
   * \param buffCnt is the length of the buffer
   * \returns a pointer to buff  */
  const char * getResultStr(int last, char * buff, const int buffCnt);
  /**
  Ececute a push function with this index. The command is in msg, an XML formatted function.
  \param functionIndex the index returned by findFunctionOwner(tagName) - not used here 
  \param msg The XMK reading structure with the command.
  \param extra may be a pointer to a relevant object (not used here) 
  \returns true if the function were executed successfully - ie image found and 
   * processed - this is used to count good commands.*/
//  virtual bool executePushFunction(int functionIndex, UServerInMsg * msg, void * extra);
  /**
   * Get total number of jobs executed */
  int getDoneJobsCnt();
  /**
   * Get number of executed jobs that returned an error status */
  int getDoneJobsErrCnt();
  /**
   * Get number of active jobs (i.e. job started but not finished) */
  int getActiveCnt();

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
  /**
   * Get a job slot for execution of a command.
   * \returns NULL if no slot is available, else a not active slot. */
  UCronJob * getFreeJob();
  /**
   * Get next timed cron job and start it.
   * \returns false if no jobs are awating execution */
  bool handleOnePushCmd();


public:
  /**
   * list of pending cron jobs */
  UServerPush cmds;
  /**
   * Is the resource to output verbose message to console */
  bool verbose;

protected:
  /**
  Name of logfile */
  //char logGpsName[MAX_FILENAME_LENGTH];
  /**
  Thread handle for frame read thread. */
  pthread_t threadHandle;
  /**
  Is thread actually running */
  bool threadRunning;
  /**
  Should thread stop - terminate */
  bool threadStop;
  /**
   * Maximum number of active jobs at anyone time */
  static const int MAX_ACTIVE_CRON_JOBS = 10;
  /**
   * Active or finished jobs */
  UCronJob job[MAX_ACTIVE_CRON_JOBS];
  /**
   * Number of used entries in job list */
  int jobCnt;
  /**
   * Last used job */
  int jobLast;
  
  /// index to idle count 
  UVariable * varIdleCnt;
  /// index to number of commands executed
  UVariable * varJobCnt;
  /// number of commands that returned error
  UVariable * varErrCnt;
  /// time it took to execute last command
  UVariable * varLastTime;
  /// index to minimum time interval between cron jobs
  UVariable * varCronTime;
};

#endif

