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
#ifndef UCMDEXE_H
#define UCMDEXE_H

#include <ugen4/ulock.h>

//#include "userverport.h"
#include "ufunctionbase.h"
#include "urespool.h"
#include "uresvarpool.h"
#include "userverpush.h"

/**
Number of function modules (plug-ins) that can be loaded at one time */
#define MAX_FUNCTION_COUNT     100

/**
Reads receive queue and asks loaded functions to handle command.

@author Christian Andersen
*/
class UCmdExe : public UResVarPool, public UServerPushImplement
{
public:
  /**
  Constructor */
  UCmdExe()
  { // set name and version number
    setResID(getResClassID(), 202);
    UCmdExeInit();
  };
  /**
  Destructor */
  virtual ~UCmdExe();
  /**
   * Initialization of server core */
  void UCmdExeInit();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "core"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 168; };*/
  /**
  Set the name of the server */
  void setName(const char * serverName);
  /**
  Add one function object to server capabilities
  \Returns true if added (false if no more space). */
  bool addFunction(UFunctionBase * function);
  /**
   * Add one resource into the resource structure with this owner module.
   * \param res resource that should be added into resource pool
   * \param owner the owner module that just created the resource
   * \Returns true if added (false if no more space). */
  bool addResource(UResBase * res, UFunctionBase * owner);
  /**
  Add any new ressources this function has to offer */
  bool addNewRessources(UFunctionBase * function);
  /**
  Remove function from server function list.
  (This does not delete the function object).
  Returns true if function is found. */
  bool deleteFunction(UFunctionBase * function);
  /**
  Handle next message in queue, and return true
  if a message were handled. if no message is available
  then return false. */
  bool handleOneMessageFromQueue();
  /**
  Test the server push queue for pending
  executions. Start with the next after last test. */
  bool handleOneServerPushMessage();
  /**
  Send a message to a client or to console
  with fized length - message may include a zero character.
  Returns true if send.
  NB! this call is depricated, as it do not allow
  lock of client connection during af lengthy transmission!!
  Use sendMsg(UServerInMsg * msg, const char * message, int size) instead.
  * @todo make this function private */
  bool sendMsg(int clientIdx, const char * message, int size);
  /**
  * Send a message to a client or to console
    with fized length - message may include a zero character.
    Returns true if send.
  * @todo Should allow connection lock during lengthy connections, but this is not implemented yet. */
  bool sendMsg(UServerInMsg * msg, const char * message, int size);
  /**
  * Send zero terminated string.
  * Returns true if send.
  * NB! this call is depricated, as it do not allow
    lock of client connection during af lengthy transmission!!
  * Use sendMsg(UServerInMsg * msg, const char * message.
  * @todo make this function private */
  bool sendMsg(int clientIdx, const char * message);
  /**
  Send a message to a client or to console
  with fized length - message may include a zero character.
  Returns true if send.
   * @todo Should allow connection lock during lengthy connections, but this is not implemented yet. */
  bool sendMsg(UServerInMsg * msg, const char * message);
  /**
  Send a message to all active clients.
  The connection is locked during the transmission, except
  for the user 'lockedUser', as it is assumed that the connection
  to this user is locked already (if called as part of a user request).
  If this function is not called as a direct reply to a user (client) request
  the 'lockedUsed' should be set to -1.
  * Returns true if at least one client got the message. */
  bool sendMsgAll(const char * message, bool lockedUser);
  /**
  Shut down a connection to a client */
  void closeClient(int clientIdx);
  /**
  Print status to console */
  void print(const char * preStr);
  /**
  Print resource status to console */
  void printRess(const char * preStr)
  { resPool->print(preStr);};
  /**
  Set socket server, from where to get the commands */
  void setServer(UServerPort * socketServer);
  /**
  Set verbose messages - mostly for debug purpose */
  inline void setVerbose(bool value) { verboseMessages = value;};
  /**
  Send a warning to the client of with the tag type of the command, with a marking
  as a warning and the provided warning text. the text will be put in
  double quotes, and encoded with escape sequence as needed, s� (almost) all characters are legal. */
  bool sendWarning(UServerInMsg * msg, const char * warningText);
  /**
  Send an error to the client of with the tag type of the command, with a marking
  as an error and the provided error text. the text will be put in
  double quotes, and encoded with escape sequence as needed, s� (almost) all characters are legal. */
  bool sendError(UServerInMsg * msg, const char * errorText);
  /**
  Send a debug message to the client of with the tag type of the command, with a marking
  as a debug message and the provided warning text. the test will be put in
  double quotes, s� (almost) all characters are legal. */
  bool sendDebug(UServerInMsg * msg, const char * debugText);
  /**
  Send a info message to the client of with the tag type of the command, with a marking
  as an info message and the provided text. the test will be put in
  double quotes, and converted to XML-text (i.e. using &gt; for '>'). */
  bool sendInfo(UServerInMsg * msg, const char * infoText);
  /**
  Send a info message to the client of with tag type help.
  The test will be put in double quotes and converted to XML-text. */
  bool sendHelp(UServerInMsg * msg, const char * infoText);
  /**
  \brief Send the open help tag
  Send open tag with one 'info' attribute with the provided string
  as value. Must be finished with a call to sendHelpDone(msg).
  \param msg The message with the client information.
  \param infoText The string value to put into the info attribute.
  \returns true if send. */
  bool sendHelpStart(UServerInMsg * msg, const char * infoText);
  /**
  \brief Send a help close tag
  The help close tag </help> if followed by a full tag with the
  message 'msg' tag name and the text "info=done".*/
  bool sendHelpDone(UServerInMsg * msg);
  /**
  Start theeage handling thread */
  bool handleMessagesThreadStart();
  /**
  Thread function that handles the messages.
  Stopped by setting threadStop to true.
  Test if running by inspecting threadRunning. */
  void run();
  /**
   * wakes message thread regularly, when no activity
   * Claculates load percentage */
  void runIdle();
  /**
  Stop message handling thread, and wait for termination
  if the 'andWait' flag is true */
  void stop(bool andWait);
  /**
   * Is the server in the process of stopping all threads
   * \returns true if stopping */
  bool isStopping()
  { return stopping; };
  /**
  Execute one of the functions with this command message.
  The 'extra' parameter is intended for event triggered
  push-commands, where the object triggering the event
  can be transferred as the extra parameter.
  \param functionIndex the index returned by findFunctionOwner(tagName)
  \param msg The XMK reading structure with the command.
  \param extra may be a pointer to a relevant object (laserscan, imega or other object usable by the function)
  \param aPush set to true if this is a push command (to allow better logging etc)
  \Returns true if the functione were successfully executed. */
  bool executeFunction(int functionIndex, UServerInMsg * msg, void * extra, bool aPush);
  /**
  Ececute a push function, and lock for dynamic load/unload of function during operation.
  \param functionIndex the index returned by findFunctionOwner(tagName)
  \param msg The XMK reading structure with the command.
  \param extra may be a pointer to a relevant object (laserscan, imega or other object usable by the function)
  \returns true if the function were executed successfully - ie image found and
   * processed - this is used to count good commands.*/
  bool executePushFunction(int functionIndex, UServerInMsg * msg, void * extra);
  /**
  Find function owner (index into list) for this tag name (command keyword)
  Returns MAX_FUNCTION_COUNT if it is a system function keyword.
  Returns -1 if keyword is not found. */
  int findFunctionOwner(const char * tagName);
  /**
  Find the index of this function - start from the
  latest (highest number) to allocate new resources to the
  most likely owner.
  Returns -1 if no index were found, otherwise an index from 0 to MAX_FUNCTION_COUNT */
  int findFunctionIndex(UFunctionBase * function);
  /**
  Test to see if client is alive
  \param clientIdx is client number (between 0 and 19)
  \param holdOffTime is hold off time after connection.
  \returns true if client is active and has been active for at least the hold off time. */
  bool isClientAlive(int clientIdx, double holdOffTime);
  /**
  Get handle to loaded module (for further symbol references */
  void * getLoadedModuleRef(int functionIndex);
  /**
  Get number of server function modules.
  This excludes the server function itself. */
  inline int getFuncCnt()
  { return funcCnt; };
  /**
  Load a plugin module with this qualified filename.
  Filename is typically of the form './name.so.0'.
  Returns pointer to created function if loaded.
  Returns NULL if load failed.
  The loaded module is added to the function list.
  The function try to load the module static first, if this fails
  the module name is assumed to be a filename and attempts to load it as a plugin.
  The returned handle can be used to make some static initialization if needed.
  A number of error messages may be send to 'stderr' and stored in 'buff'
  if the load fails.
  \par [aliasName] is an alias name for modules that may be loaded more than once, i.e. interface module */
  UFunctionBase * loadFunctionModule(const char * moduleFileName, char * buff, int buffCnt, const char * aliasName);
  /**
  Unload a loadable module with this name.
  The name should be unique among loaded modules, and must at least hold
  three character.
  Module may be identified by an integer value (from function list) or the
  first part of the module name (at least 3 characters).
  If more modeles match the name, then the last module
  with a match is unloaded.
  Returns true if a module is unloaded, or false if no loaded module
  matches the name.
  Static modules should not be unloaded (but may be overloaded). */
  bool unloadFunctionModule(const char * value);
  /**
  Unload all modules - static as well as loaded - make an empty server */
  bool unloadAllModules();
  /**
  Is this function owner of this resource */
  bool isResourceOwner(const char * ressID, UFunctionBase * owner);
  /**
  Post command to command queue from string.
  \return true if command is posted, */
  bool postCommand(int client, const char * command);
  /**
  Is receive queue full, i.e. no more space to pose commands.
  \Returns true if queue is full. */
  bool isRxQueueFull();
  /**
  \Returns true if commandqueue is empty */
  bool isRxQueueEmpty();
  /**
  Execute a scriptfile of commands.
  The filename 'cfn' is assumed to be on the dataPath, unless the initial
  character of the name is '.' or '/'.
  Each command is put on the RX-queue if
  the queue is empty. If the queue is not empty the
  sequence is stalled until the queue is empty.
  If andWait is false, the commands are put on queue
  until no more commands or the queue is full.
  The command reply is returned to the client 'forClient'.
  If client is -1, then output to console (and push commands are not valid).
  \REturns true if all commands are put on queue. */
  bool executeScriptFile(const char * cfn, bool andWait, int forClient);
  /**
  Send text string, and ensure that special protected XML characters are converted
  to an escape sequence, e.g. '<' = &lt;.
  \Returns true if send.*/
  bool sendText(UServerInMsg * msg, const char * text);
  /**
  Get help text for available static modules */
  virtual bool getStaticHelpList(char * list, const int listCnt);
  /**
  Create a function at a higher level, this is intended
  to be static available modules in specific servers.
  The moduleName is a keyword to one of the available static modules.
  If no such module exist a NULL pointer is returned.
  If the module exist, it should be created and added to the function list.
  A pointer to the created function should be returned, to allow further processing.
   * \param[in] *moduleName name of module, euther a filename or a name of a stativ module
   * \param[in] *aliasName a specific (pre-) name to be used for command, resource and logfile.
   * \return true on succes, returns false if unknown module, 'why' is not modified (used if failed for a fatal reason) */
  virtual bool loadStaticModule(const char * moduleName, const char * aliasName, char * why, const int whyCnt);
  /**
  Create basic core variables and functions */
  void createBaseVar();
  /**
   * Function to implement a var-pool method call.
   * The function with 'name' and pameters in the order 'paramOrder'
  is to be called. The actual parameters are in 'strings' and 'doubles'.
   * A double sized result may be returned in 'value'.
   * May return struct values in the 'returnStruct' pointer list (when the
  pointer list is not NULL), and in this case the 'returnStructCnt' is set
  to the maximum number of pointers available and should be modified
  to the number of valid pointers returned.
   * Should return true if function was handled.
  (otherwise an invalid function specification is assumed). */
  virtual bool methodCall(const char * name, const char * paramOrder,
                          char ** strings, const double * doubles,
                          double * value,
                          UDataBase ** returnStruct = NULL,
                          int * returnStructCnt = NULL);
  /**
  Update local variables, e.g. clients, open, port */
  void updateLocalVar();
  /**
  Get load figure. Load is calculated as the number of idle waits in
  a fixed number of seconds.
  Result is in the range [1.0 to 0.0]. */
  inline double getLoad()
  { return wLoad; };
  /**
  \brief execute a command line command and return reply as help text.
  \param msg The source command
  \param cmdStr the command that should be send to the system
  The function uses popen to start the commands and continues to read
  from the stream until it terminates.
  \returns the value that the bash command returns (-2) if pipe can not be opened. */
  bool doBashCmd(UServerInMsg * msg, const char * cmdStr);
  /**
  \brief handles a command starting with the keyword 'do'.
  \param msg The source command
  \returns true if successfull */
  bool handleShellCmd(UServerInMsg * msg);
  /**
  \brief handles a command starting with the keyword 'alive'.
  \param msg The source command
  \returns true if successfull */
  bool handleAlive(UServerInMsg * msg);
  /**
   * \brief get a pointer to a static resource.
   * \param resName The name of the resource to be acquired.
   * \param mayCreate if set to true, then the resource will be created if it do not exist.
   * \param staticOnly get only if a static resource (default). Getting non-static
   * resources may be dangerous of not within same plugin, if not compiled against the
   * same source (headerfilles).
   * \returns NULL if not found, else a bointer to an existing type */
  UResBase * getStaticResource(const char * resName, bool mayCreate,
                               bool staticOnly = true);
  /**
   * \brief Test if the module with this index is loaded as static.
   * \returns true if module exist and is static, else false. */
  bool isModuleStatic(int moduleIndex);


protected: // internal functions
  /**
   * Name and version of the command executor */
  virtual const char * name();
  /**
   * \Returns a stringlist of tag-names handled by system.
   * e.g. "sys sysHigh system" that will handle all
   * tags of this type. */
  virtual const char * commandList();
  /**
  This is a system function command,
  deal with it.
  System functions are e.g. server pusk commands. */
  bool systemFunction(UServerInMsg * msg);
  /**
  This message is to be handled as a server push queue command */
  bool handleServerPushCommand(UServerInMsg * msg);
  /**
  System wide command */
  bool handleServerCommand(UServerInMsg * msg);
  /**
   * handle the quit command - from any client
   * \param msg is the reference to the client that send the command,
   * and possibly expects a reply
   * \returns true */
  bool handleQuit(UServerInMsg * msg);
  /**
  Send server general help back to client */
  bool sysServerHelp(UServerInMsg * msg);
  /**
  One or more resources are added or deleted - inform
  all other functions. */
  void resourcesUpdated();
  /**
  Update this function with the full resource list. */
  void resourcesUpdate(int funcIdx);
  /**
  Handling module load, unload and list commands */
  bool sysModuleCmd(UServerInMsg * msg);
  /**
  Unload the module with this index number and any ressources owned by this module.
  The index may be a static (compiled in) module or a loaded module.
  Returns true if the module is removed. */
  bool unloadThisModule(int funcIdx);
  /**
  Close log for all server commands */
  void closeLogServer();
  /**
   * Send a kill signal to server process.
   * \param msg optional reference to the client that ordered the kill */
  void killServer(UServerInMsg * msg);

private:
  /**
  Name of the server */
  const char * servName;
  /**
  Server, where to get the commands */
  UServerPort * server;
  /**
  Functions to handle the commands */
  UFunctionBase * func[MAX_FUNCTION_COUNT];
  /**
  Number of functions loaded */
  int funcCnt;
  /**
  Server push queue */
  UServerPushQueue push;
  /**
  If verboseMessages is true, then more messages are prionted to the console */
  bool verboseMessages;
  /**
  Thread running flag */
  bool threadRunning;
  /**
  Flag to stop thread - terminates the thread */
  bool threadStop;
  /**
  Thread handle for command read thrad. */
  pthread_t threadHandle;
  /**
  Pool of shared sresources */
  UResPool * resPool;
  /**
  Logfile for server */
  ULogFile logServer;
  /**
  Server log filename */
//  char logServerName[MAX_FILENAME_SIZE];
  /**
  Server load figure in the range [0..1]. Is calculated from the
  number of idle waits executed in a fixed number of seconds. */
  double wLoad;
  /**
  Max length of current handled message (saved part) (size 1000) */
  static const int MMS = MAX_MESSAGE_LENGTH_TO_CAM;
  /**
  Currentlt handling this message */
  char currentMessage[MMS];
  /**
  Command idle loop count */
  int idleLoopCnt;
  /**
   * stopping ther server */
  bool stopping;
  /// pointer to allowConnection plag
  UVariable * varAllowConnection;
  /// pointer to allowConnection plag
  UVariable * varTime;
  /// is server open for connections
  UVariable * varOpen4connections;
  /// number of active clients
  UVariable * varClients;
  /// number of last client connected
  UVariable * varLastClient;
  /// number of clients ever connected in this session
  UVariable * varLastClientSerial;
  /// port number for server
  UVariable * varPort;
  /// estimated load of server main thread
  UVariable * varIdle;
  /// time to wait before punking silent clients
  UVariable * varAlivePunkTime;
  /// should calls to plug-in functions be echo'ed to console (with time info)
  bool callNotice;
  /// flag is true, when main thread is idle
  bool isIdle;
  /// start time for idle
  UTime idleStart;
  /// seconds in idle state
  double idleTime;

public:
  /**
   * Lock to prohibit server console to print command prompt */
  //ULock consoleLock;
};


#endif
