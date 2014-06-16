//
// C++ Interface: userverpush
//
// Description:
//
//
// Author: Christian Andersen <jca@elektro.dtu.dk>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef USERVERPUSH_H
#define USERVERPUSH_H

#include <semaphore.h>
#include <ugen4/utime.h>

#include "uresbase.h"
#include "userverqueue.h"

class UServerPort;

#define MAX_SERVER_PUSH_CMDS   100
/**
Number of push watches that can be pending at any one time
(counting the threadless pushes only */
#define MAX_PUSH_WATCH_CNT     50

class UServerPush;
class UResVarPool;
class UVarPool;

/**
 * Base class for a push implement handler */
class UServerPushImplement
{
public:
  /**
   * Constructor */
  UServerPushImplement()
  {
    sem_init(&actionFlag, 0, 1);
    pushWatchCnt = 0;
  };
  /**
   * Constructor */
  virtual ~UServerPushImplement()
  {
  };
  /**
   * Service function for push queue - to find index of owner of function described in this tag */
  virtual int findFunctionOwner(const char * tagName)
  { return -1; };
  /**
  Find index for function, that handles this message, but taking the full XML tag as parameter */
  int getFunctionOwner(UServerInMsg * msg);
  /**
  Test to see if client is alive */
  virtual bool isClientAlive(int clientIdx, double holdOffTime)
  { return true; };
  /**
  Ececute a push function with this index. The command is in msg, an XML formatted function.
  \param functionIndex the index returned by findFunctionOwner(tagName)
  \param msg The XMK reading structure with the command.
  \param extra may be a pointer to a relevant object (laserscan, imega or other object usable by the function)
  \returns true if the function were executed successfully - ie image found and
   * processed - this is used to count good commands.*/
  virtual bool executePushFunction(int functionIndex, UServerInMsg * msg, void * extra)
  { return false; };
  /**
   * \brief Add an server push object to watch list.
   * This is called by a combined resource-push object
   * when the object is updated and thus a push command may
   * need triggering.
   * \param obj is the serverPush object that need to be consulted for pending commands
   * \return true if the object were added, and false if it existed already.  */
  bool addPushWatch(UServerPush * obj);
  /**
   * Get pointer to server Push implement structure (test - fix) */
//   UServerPushImplement * getServerPushImplement()
//   { return this; };
  /**
   * \brief get a pointer to a static resource.
   * \param resName The name of the resource to be acquired.
   * \param mayCreate if set to true, then the resource will be created if it do not exist.
   * \param staticOnly get only if a static resource (default). Getting non-static
   * resources may be dangerous of not within same plugin, if not compiled against the
   * same source (headerfilles).
   * \returns NULL if not found, else a bointer to an existing type */
  virtual UResBase * getStaticResource(const char * resName, bool mayCreate,
                               bool staticOnly = true)
  {
    printf("UServerPushImplement::getStaticResource: looking for resource\n");
    return NULL;
  };
  /**
   * Notifies server that an event has occurred, this may have triggered
   * on-event commands, a call will wake main thread (if idle) */
  inline void event()
  { // make semaphore binary, by emptying it, if not empty already
    sem_trywait(&actionFlag);
    // then post a flag, that there is something to do for main thread
    sem_post(&actionFlag);
  }

protected:
  /**
  Push watch object */
  UServerPush * pushWatch[MAX_PUSH_WATCH_CNT];
  /**
  Number of watches in store */
  int pushWatchCnt;
  /**
  Lock for watches - as there may be added watched from another thread */
  ULock pushWatchLock;
  /// flag to be posted, when an event has happend
  sem_t actionFlag;
};

/**
ServerPushQueue is a class of commands, that
awaits execution */
class UServerPushElement
{
  public:
  /*
    Constructor */
    UServerPushElement();
  /**
    Clear to unused value */
    void clear();
  /**
    Print content of queue element */
    void print(const char * preString);
  /**
   * Print content of queue element
   * \param preStr is to be put at the start of the buffer,
   * \param buff is the buffer to print to.
   * \param buffCnt is the length of the buffer
   * \returns pointer to buffer */
  const char * print(const char * preString, char * buff, int buffCnt);
  /**
   * Print call details to this buffer.
   * \param preStr is to be put at the start of the buffer,
   * \param buff is the buffer to print to.
   * \param buffCnt is the length of the buffer
   * \returns pointer to buffer */
  const char * printCall(const char * preStr,
                           char * buff,
                           const int buffCnt);
  /**
    Update the push event counter and the success counter.
    It further tests if the counts exceed the specified limit.
    \param success is the result of the execution of the push command.
    \Returns true if the function is to continue (to be active), or
    false if this were the last run. */
    bool update(bool success);
  /**
   * set this string as the call to be performed when the event has occured.
   * \param client is the client requested the push call.
   * \param callStr is the call string with all default parameters,
   * and with mark '%s', where event parameter is to be placed.
   * The string may be no longer than 500 chars long (MAX_MESSAGE_LENGTH_TO_CAM).
   * \returns true if call has valid syntax (do not check all, and not if call is implemented). */
  bool setCall(const int client, const char * callStr);
  /**
   * Unpack call string to name, parameter list and parameter prototypes.
   * \returns true if call syntax is OK */
  bool unpackCall();

  public:
  /**
  Is function active with a cmd */
  bool activeCmd;
  /**
  Is function active with a call */
  bool activeCall;
  /**
  Index to function to perform the stored function */
  int functionIndex;
  /**
  Next time the function is due for execution */
  UTime nextExeTime;
  /**
  Interval for execution in decimal seconds. */
  double interval;
  /**
  Command line to execute when time is right */
  UServerInMsg toDo;
  /**
  The number of function calls with a 'true' result
    to be executed before completion */
  int countGoodTarget;
  /**
  * The number of function calls with a 'true' result
  * obtained until now. */
  int countGood;
  /**
  The total number of function calls (regardless of result)
  to be executed before completion */
  int countTotalTarget;
  /**
  The total number of function calls (regardless of result)
  obtained until now. */
  int countTotal;
  /**
  Event count is used in event push functions, if
  not all events are to trigger a push. */
  unsigned int events;
  /**
    * qualified name of method to call - is a pointer to toDoMessage */
  const char * callName;
  /**
    * Maximum number of parameters allowed in call */
  static const int MAX_PAR = 10;
  /**
    * String parameters for call */
  const char * parString[MAX_PAR];
  /**
    * Double sized parameters */
  double parDouble[MAX_PAR];
  /**
    * parameter call type */
  char parOrder[MAX_PAR+1];
};


////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
Queue with server push commands, and parameters to start the
push event */

class UServerPushQueue
{
public:
  /**
    Constructor */
  UServerPushQueue();
  /**
    print the active content of the queue */
  void print(const char * preString);
  /**
    print the active content of the queue to a string buffer */
  void print(const char * preString, char * buff, int buffCnt);
  /**
    Find first free server push queue element.
    Returns NULL if no more space in queue */
  UServerPushElement * getFreeQueueElement();
  /**
    Get next queued push element that has to be activated.
    Returns pointer to element or NULL if
    none is ready for activation */
  UServerPushElement * getNextTimedPushElement();
  /**
  Get indexed queue element */
  inline UServerPushElement * get(int index)
  { return &pushCmd[index]; };
  /**
    Get number if elements in queue */
  inline int getPushCmdCnt()
  { return pushCmdCnt; };
  /**
    Get number if elements in queue that are active.
   * \param cmdCnt is the count of active push commands
   * \param callCnt is the count of active call commands
   * \returns count of active events - sum of call and cmd events */
  int getPushCmdActiveCnt(int * cmdCnt, int * callCnt);
  /**
    Flush active push commands from this client with this value.
    Returns number of push elements flushed. */
  int systemQueueflush(const int client, const char * attValue);
  /**
   * Get next time a job is scheduled to be initiated
   * \returns 10 seconds into the future if no timed jobs are active, else
   * the earliest due-time is returned (may be earlier than now) */
  UTime getNextExeTime();
  /**
   * do the push call called for by this push element.
   * Update the push element after the call.
   * \param vp is a var pool pointer, either the root or an element in the root tree, to implement the call.
   * \param pe the push element to implement call for.
   * \param value is the (optional) event value string to be used in the call
   * \return true if the calle returned 1.0 or 2.0 as value. */
  static bool doPushCall(UVarPool * vp, UServerPushElement * pe, const char * value);

private:
  /**
    Command queue for server push commands */
  UServerPushElement pushCmd[MAX_SERVER_PUSH_CMDS];
  /**
    Number of used server push commands */
  int pushCmdCnt;
  /**
    Next server push command to be tested for
    pending execution */
  int pushCmdNext;
};

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

class UServerPush
{
  public:
  /**
    Constructor */
    UServerPush();
  /**
    Destructor */
    virtual ~UServerPush();
  /**
    A new image (push element) is received.
    should be called, when new data is available.
    The new data (structure/class etc.) is
    transferred by the pData pointer.  */
    void gotNewData(void * pData);
  /**
    Called when a new resource is
    available (or is removed), local pointers to the resource should be
    updated as appropriate. */
    bool setResource(UResBase * resource, bool remove);
  /**
    A pretest when new data is available to request if it is needed,
    so that data extraction can be avoided if nothing is needed.
    \returns true if data is needed, and then expects that 'gotNewData(.)'
    is called with the appropriate pointer data.
    \returns false if noone needs data at this time, but update event counters
    as if 'gotNewData(.)' were called. */
    bool needNewData();
  /**
   * Set command executor reference for push functions
   * (same as setImplementor()) */
    inline void setCmdExe(UServerPushImplement * executor)
    { cmdImpl = executor; } ;
  /**
   * Set command executor reference for push functions
   * (same as setCmdExe()) */
    inline void setImplementor(UServerPushImplement * executor)
    { cmdImpl = executor; } ;
  /**
    Get the camera push queue */
    inline UServerPushQueue * getPushQueue()
    { return &push; } ;
  /**
    Add a push command.
    A push command is typically triggered when a new data is available.
    \Returns number of affected push commands.
    \Returns 0 if command failed or no commands affected by a flush command. */
    int addPushCommand(UServerInMsg * msg);
  /**
    Flush all pending commands from this client */
    void flushClientCmds(int clientIdx);
  /**
   * Debug print of camera status */
    void print(const char * preString);
  /**
   * Debug print of camera status to a string buffer. */
    void print(const char * preString, char * buff, int buffCnt);
  /**
   * Test for push (just a more appropriate name for the call) */
    inline void servicePendingPushCmds()
    { callGotNewDataWithObject(); };
  /**
   * \brief Add watch object if needed.
   * An image or other object is updated.
   * set object as updated, with an extra qualifier, that may be used as
   * a parameter in a push call.
   * this push list is then added to the to the push list examined by the server thread.
   * when server thread reach this push list it will call to get a possible associated data structure
   * by a call to needNewData() whish is expected to call gotNewData(void * data_structure) - but now in the server thread.
   * \returns true if the command were added and false if the command were there already. */
  bool setUpdated(const char * value);

  /**
    Called from push structure to get push object
    followed by a call to 'gotNewData(object)'.
    Should be overwritten by push object holder. */
    inline virtual void callGotNewDataWithObject()
    { // as there is no push data available in this
      // generic class, then call with a NULL pointer.
      // In 'real' use it should be called with a pointer, e.g.
      // to a laser scan or an image.
      gotNewData(NULL);
    };
  /**
    Has the push function got a cmdExe pointer
   * \returns true if a core pointer is available */
    inline bool gotCmdExe()
    { return cmdImpl != NULL; };
  /**
   * get server core pointer (cmdExe)
   * \returns the core pointer available in this object - may be NULL */
  inline UServerPushImplement * getImplementor()
  { return cmdImpl; };
  /**
    * Get number of active push commands
    * \param cmdCnt is set to number of command-based commands
    * \param callCnt is set to number of call-based commands.
    * \return total number of active event commands */
  inline int getPushCmdCnt(int * cmdCnt, int * callCnt)
  { return push.getPushCmdActiveCnt(cmdCnt, callCnt); };
  /**
    * test if the object has had an update since last check
    * \param lastCnt is the last saved count.
    * \param newCnt is where the new count is saved (if not NULL)
    * returns true if the update count (updateCnt) is different from lastCnt */
  inline bool isUpdated(int lastCnt, int * newCnt)
  {
    bool result = lastCnt != updateCnt;
    if (result and newCnt != NULL)
      *newCnt = updateCnt;
    return result;
  }
  /**
   * do the push call called for by this push element.
   * Update the push element after the call.
   * \param pe the push element to implement call for.
   * \param value is the (optional) event value string to be used in the call
   * \return true if the call returned 1.0 or 2.0 as value. */
  bool doPushCall(UServerPushElement * pe, const char * value);


  private:
  /**
  Pointer to push server queue */
  UServerPushImplement * cmdImpl;
  /**
  push queue when new push data gets available */
  UServerPushQueue push;
  /**
    * this variable is incremented on every call to setUpdated(),
    * and can be used to test for potential new values by
    * others than the push command handler. */
  int updateCnt;
  /**
   * pointer to a varpool resource for push-calls */
  UResVarPool * callImpl;
  /**
   * Lock, to ensure that an update call triggers this
   * update again - that is endless loop detection.
   * This is done by a lock, if not lockable, then
   * no calls are performed. */
  ULock callLock;
};

#endif
