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
#ifndef UFUNCTIONBASE_H
#define UFUNCTIONBASE_H

#include "userverqueue.h"
#include "uresbase.h"
#include "uclientfuncbase.h"
#include "uresvarpool.h"
#include "string.h"


/** forward declaration og class defined in ucmdexe.h */
class UCmdExe;

/**
Basic class for loadable functions for this component server.

@author Christian Andersen
*/
class UFunctionBase : public UCallBack, public UResVarPool
{
public:
  /**
  Constructor */
  UFunctionBase();
  /**
  Destructor */
  virtual ~UFunctionBase();
  /**
  Name of function
  The returned name is intended as informative to clients
  and should include a version number */
  virtual const char * name()
  { return pluginName; };
  /**
  Function, that shall return a string with all handled commands,
  i.e. should return "gmk gmk2d guidemark", if commands
  starting with any of these three keywords are handled
  by this function */
  virtual const char * commandList()
  { return commandsHandled; };
  /**
  List (space separated) of shared resources
  provided by this function.
  Must be an empty string if no resources are to be shared.
  Each resource ID must be no longer than 20 characters long. */
/*  virtual const char * resourceList()
  { return ""; };*/
  /**
  Get a pointer to the resource with this ID string.
  The match should be case sensitive.
  A pointer should be returned for each of the ISs in
  the resource ID list. */
/*  virtual UResBase * getResource(const char * resID)
  { return NULL; };*/
  /**
  Set a resource pointer as appropriate for this class.
  NB! this base class needs a pointer to the core for
  core functionality. */
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  Add one resource into the resource structure with this parent module
  \Returns true if added (false if no more space or core pointer is not valid). */
  bool addResource(UResBase * resource, UFunctionBase * owner);
  /**
  Create any resources that this modules needs and add these to the resource pool
  to be integrated in the global variable pool and method sharing. */
  virtual void createResources() {}
  /**
  This function is called if a resource for this function is
  updated - this could be used to verify that all resources
  are available and full functionality may be available. */
  virtual void resourceUpdated()
  { /* nothing at this level */};
  /**
  Send text reply mesage to client.
  Returns true if send.
  Returns false if client (or server) has gone.
  NB! call is depricated, as connection can not be locked during long transmissions.
  * @todo remove this function */
  bool sendMsgInt(int clientIdx, const char * message);
  /**
   * Send an help open tag with the provided subject, like
   * <help subject="subject"> 
   * \param msg reference to client connection
   * \param subject help subject
   * \returns true if send  */
  bool sendHelpStart(UServerInMsg * msg, const char * subject);
  /**
   * Send an help open tag with the provided subject, like
   * <help subject="subject">
   * \param subject help subject
   * \returns true if send  */
  bool sendHelpStart(const char * subject);
  /**
   * Send an help open tag with the provided subject, like
   * <help subject="subject">
   * where subject is the plug-in (alias) name
   * \returns true if send  */
  bool sendHelpStart();
  /**
   * Send an help close tag and an info message matching the message in msg
   * \param msg reference to client connection
   * \returns true if send  */
  bool sendHelpDone(UServerInMsg * msg);
  /**
   * Send an help close tag and an info message matching the message in msg
   * \returns true if send  */
  bool sendHelpDone();
  /**
  Send reply message to client with specific length
  - can be used for binary messages as well.
  Returns true if send.
  Returns false if client (or server) has gone. */
  bool sendMsg(UServerInMsg * msg, const char * message, int size);
  /**
  Send text reply mesage to client.
  Returns true if send.
  Returns false if client (or server) has gone. */
  bool sendMsg(const char * message);
  /**
  Send text reply mesage to client.
  Returns true if send.
  Returns false if client (or server) has gone. */
  bool sendMsg(UServerInMsg * msg, const char * message);
  /**
  Send reply message to client with specific length
  - can be used for binary messages as well.
  Returns true if send.
  Returns false if client (or server) has gone.
  NB! call is depricated, as connection can not be locked during long transmissions.
  * @todo remove this function */
  bool sendMsg(int clientIdx, const char * message, int size);
  /**
  Set verbose messages - mostly for debug purpose */
  inline void setVerbose(bool value)
    { verboseMessages = value;};
  /**
  Print information on this object to console. */
  void print(const char * preString);
  /**
  Open logfile - in default image directory */
  bool openLogfile(const char * name);
  /**
  Close logfile */
  void closeLogfile();
  /**
  Set loaded module and filename */
  void setLoadedModuleRef(void * module, const char * moduleFileName);
  /**
  Get loadable module pointer for further symbol ref */
  inline void * getLoadedModuleRef()
  { return ldModule; };
  /**
  Get loaded module name.
  Returns an empty string if not a dynamic loaded module */
  inline const char * getLoadedFileName()
  { return ldFileName; };
  /**
  Test if all resources are loaded as intended */
  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Send a text that may contain XML reserver characters '<"'&>'.
  The text will be converted, so that no reserver characters remain in the transmitted text.
  Returns true if send. */
  bool sendText(UServerInMsg * msg, const char * text);
  /**
  Send a text that may contain XML reserver characters '<"'&>'.
  The text will be converted, so that no reserver characters remain in the transmitted text.
  Returns true if send. */
  bool sendText(const char * text);
  /**
  Send a closing end-tag for this message.
  Returns true if the message was send. */
  bool sendEndTag(UServerInMsg * msg);
  /**
  Send a closing end-tag for this message.
  Returns true if the message was send. */
  bool sendEndTag();
  /**
   * send a tag start message with these attributes.
   * The tag name is the same as the command name (the command tag name)
   * I.e. like "<tagname attributes>\n"
   * The tag can be closed by a call to sendEndTag()
   * \param attributes is a string with all additional attributes to be placed inside the tag open message.
   * \returns true if send successfully. */
  bool sendStartTag(const char * attributes);
  /**
   * send a full tag message with these attributes. I.e. a "<tagname attributes/>\n"
   * The tag name is the same as the command name (the command tag name)
   * \param attributes is a string with all additional attributes to be placed inside the tag.
   * \returns true if send successfully. */
  bool sendFullTag(const char * attributes);
  /**
  Set alias name - default is empty
  \param aliasName is the new name for the plugin - this is used for the resource and the new single command handled. */
  virtual void setAliasName(const char * name);
  /**
  Set plugin name - the compile time will be added */
  void setName(const char * name)
  { // alias is not used by this module
    snprintf(pluginName, MAX_RESOURCE_LIST_SIZE, "%s (compiled %s %s)", name, __DATE__, __TIME__);
    //printf("NB! alias not implemented\n");
  };
  /**
   * Set command list and optional version information
   * e.g. setCommand("ball door", "ballfinder", "finds balls in images (by author)");
   * to set the commands "ball" and "door" to be handled by this plugin, and
   * add some author initial and a coompile date and time for the plugin.
   * \param cmdList is a space separated list of commands to be handled by this plugin
   * \param name is the resource name of the plugin, and the name of the base structure for local variables.
   * \param note an optional text string shown in the module list - could also hold author name (shown in variable list and other places).
  */
  virtual void setCommand(const char * cmdList, const char *  name, const char * note);
  /**
   * Get the alias name.
   * An alias name renames some generic plugins to give a new command keyword and a new resource and plugin name.
   * This is especially used for the generic server interface plugin, as this can handle plugins from more than one server, the base name is if, but the alias name could be "laser" for an interface to the laser server or "cam" for tnterface to the camera server.
   * Both plugins loaded into e.g. a behaviour server.
   * \returns the alias name (a single name, starting with a character and continue alfanumeric (no spaces), max length 32 chars).
  */
  const char * getAliasName()
  { return aliasName; };
  /**
   * \brief is the module loaded as static
   * A module is static if the ldFilename has a length of zero.
   * \returns true if the module is loaded as static */
  bool isStatic()
  { return (strlen(ldFileName) == 0); };
  /**
   * Set module load time, i.e. just befor call to open file
   * This is needed by the svs-library function, as image times are relative to start of module */
  virtual void setLoadTime(UTime loadTime)
  { ;};
  /**
   * New command to be handled by this module
   * \param msg is a pointer to the queue element with the command
   * \param extra is an optional void pointer to a data element acheived form a push command
   * \return true, if command is handled */
  bool newCommand(UServerInMsg * newMsg, void * extra);
  /**
   * New command to be handled by this module - NB!
   * @todo move all push functions and plugins to this type - rather than the void in newCommand
   * \param msg is a pointer to the queue element with the command
   * \param extra is an optional data element acheived form a push command casted to type UDataBase
   * \return true, if command is handled */
  bool newCmd(UServerInMsg * newMsg, UDataBase * extra);
  /**
   * if a command with this name mine.
   * \param cmdName is the searched command name.
   * \returns true if found.
   * The command name index is saved, and can be obtaind by thje function getCmdIndex(). */
  bool isMine(const char * cmdName);

protected:
  /**
  Handle command from client (or console).
  Return true if command is handles and alle needed
  actions are taken.
  Return false if command is not known (due
  to syntax error or that it belongs to another
  function) */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  Send a standard warning reply of the form:
  &lt;'tagname' warning="warning text"/&gt;,
  where 'tagname' is that tagname of the command and
  'warning text'; is the provided reply text (quotes are added by the function).
  Returns true if send. */
  bool sendWarning(UServerInMsg * msg, const char * warningText);
  /**
  Send a standard warning reply of the form:
  &lt;'tagname' warning="warning text"/&gt;,
  where 'tagname' is that tagname of the command and
  'warning text'; is the provided reply text (quotes are added by the function).
  \Returns true if send. */
  bool sendWarning(const char * warningText);
  /**
  Send a standard warning reply of the form:
  &lt;'tagname' info="warning text"/&gt;,
  where 'tagname' is that tagname of the command and
  'warning text' is the provided reply text (quotes are added by the function).
  \Returns true if send. */
  bool sendInfo(UServerInMsg * msg, const char * infoText);
  /**
  Send a standard warning reply of the form:
  &lt;'tagname' info="warning text"/&gt;,
  where 'tagname' is that tagname of the command and
  'warning text' is the provided reply text (quotes are added by the function).
  \Returns true if send. */
  bool sendInfo(const char * infoText);
  /**
  Send a standard warning reply of the form:
  &lt;'tagname' info="'warning text'"/&gt;,
  where 'tagname' is that tagname of the command and
  'warning text' is the provided reply text (quotes are added by the function).
  Returns true if send. */
  bool sendHelp(UServerInMsg * msg, const char * infoText);
  /**
  Send a standard warning reply of the form:
  &lt;'tagname' info="'warning text'"/&gt;,
  where 'tagname' is that tagname of the command and
  'warning text' is the provided reply text (quotes are added by the function).
  \Returns true if send. */
  bool sendHelp(const char * infoText);
  /**
  Send a standard warning reply of the form:
  &lt;'tagname' debug="'warning text'"/&gt;,
  where 'tagname' is that tagname of the command and
  'warning text' is the provided reply text (quotes are added by the function).
  Returns true if send. */
  bool sendDebug(UServerInMsg * msg, const char * debugText);
  /**
  Send a standard warning reply of the form:
  &lt;'tagname' debug="'warning text'"/&gt;,
  where 'tagname' is that tagname of the command and
  'warning text' is the provided reply text (quotes are added by the function).
  \Returns true if send. */
  bool sendDebug(const char * debugText);
  /**
  Send a standard error reply of the form:
  &lt; 'tagname' debug="warning text"/&gt;,
  where 'tagname' is that tagname of the command and
  'warning text' is the provided reply text (quotes are added by the function).
  \Returns true if send. */
  bool sendError(UServerInMsg * msg, const char * errorText);
  /**
  Send a standard error reply of the form:
  &lt; 'tagname' debug="warning text"/&gt;,
  where 'tagname' is that tagname of the command and
  'warning text' is the provided reply text (quotes are added by the function).
  \Returns true if send. */
  bool sendError(const char * errorText);
  /**
  Test to see if client is active.
  Especially for event push messages. */
  bool isClientAlive(int clientIdx, double holdOffTime);
  /**
  Test if the resource has this ID, and if so set the resource pointer 'currentResPtr'
  to the new resource and adjust the local flag as appropriate.
  Returns 'changed' true if the resource pointer is changed. */
  UResBase * setThisResource(const char * ID, UResBase * resource, bool remove, bool * changed,
                       UResBase * currentResPtr, bool * isLocal);
  /**
   * \brief get a pointer to a static resource.
   * \param resName The name of the resource to be acquired.
   * \param mayCreate if set to true, then the resource will be created if it do not exist.
   * \param staticOnly if true, then static resources - like 'odoPose' - are searched only.
   * \returns NULL if not found (or core not available), else a bointer to an existing type */
  UResBase * getStaticResource(const char * resName, bool mayCreate,
                               bool staticOnly = true);
  /**
   * Get the command index
   * \returns the command number in the command string list. first is 0. */
  int getCmdIndex()
  { return cmdIndex; }
  
public:
  /**
  Maximum length of resource list */
  static const int MAX_RESOURCE_LIST_SIZE = 250;
  /**
  Max size of one command ID string */
  static const int MAX_ID_LENGTH = 30;
  /// is reply to client and console to be limited
  bool silent;

protected:
  /**
  Print more status messages to console,
  when 'verboseMessages' are true.
  Mostly for debug. */
  bool verboseMessages;
  /**
  Alias name - if a module can be loaded multible times with different names */
  char aliasName[MAX_ID_LENGTH];
  /**
  Plugin name - if a module can be loaded multible times with different names */
  char pluginName[MAX_RESOURCE_LIST_SIZE];
  /**
  Space separated list of handled commands */
  char commandsHandled[MAX_RESOURCE_LIST_SIZE];
  /**
  Pointers to start of each command */
  char * cmdToks[20];
  /**
   * number of command tokens */
  int cmdToksCnt;
  /**
  Communication server to handle outgoing
  messages */
  UCmdExe * cmdHandler;
  /**
   * Command queue element to be handled */
  UServerInMsg * msg;

private:
  /**
  Handle for loaded module */
  void * ldModule;
  /**
  Filename for loaded module */
  char ldFileName[MAX_FILENAME_LENGTH];
  /**
   * function index - the command number in the command list */
  int cmdIndex;
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/** called when server makes a dlopen() (loads plugin into memory) */
void libraryOpen();

/** called when server makes a dlclose() (unloads plugin from memory) */
void libraryClose();

/**
Needed for correct loading and linking of library */
void __attribute__ ((constructor)) libraryOpen(void);
void __attribute__ ((destructor)) libraryClose(void);
/**
Allows server to create object(s) of this class */
extern "C" UFunctionBase * createFunc();
/**
... and to destroy the created object(s) */
extern "C" void deleteFunc(UFunctionBase * p);

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


#endif
