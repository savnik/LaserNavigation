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
#ifndef UFUNC_IF_H
#define UFUNC_IF_H

#include <cstdlib>

#include "ufunctionbase.h"
#include "usmltag.h"
#include "uresclientifvar.h"
#include "uresifbase.h"
#include "uresif.h"


/**
 * The base interface function for sending and receiving data to and from other servers.
 * This function is used to establish connection to another server and to add
 * data handlers, as these are available from loaded plugins.
 * @author Christian Andersen
*/
class UFunctionIf : public UFunctionBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunc or UFunction followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFunctionIf();
  /**
  Destructor */
  virtual ~UFunctionIf();
  /**
  Called by the server core. Should return the
  name of function. There should be a first short part separated
  by a space to some additional info (e.g. version and author).
  The returned name is intended as informative to clients
  and should include a version number */
  virtual const char * name();
  /**
  Create any resources that this modules needs and add these to the resource pool
  to be integrated in the global variable pool and method sharing. */
  virtual void createResources();
  /**
  Called by the server core when loaded, to get a list of
  keywords (commands) handled by this plugin.
  Return a list of handled functions in
  one string separated by a space.
  e.g. return "COG".
  The functions should be unique on the server. */
//  virtual const char * commandList();
  /**
  List (space separated) of shared resources
  provided by this function.
  Must be an empty string if no resources are to be shared.
  Each resource ID must be no longer than 20 characters long. */
//  virtual const char * resourceList();
  /**
  Called by the server core, when a new resource is
  available (or is removed), local pointers to the resource should be
  updated as appropriate. */
  //virtual bool setResource(UResBase * resource, bool remove);
  /**
  This function is called by the server core, when it needs a
  resource provided by this plugin.
  Return a pointer to a resource with an ID taht matches this 'resID' ID string.
  The string match should be case sensitive.
  Returns false if the resource faled to be created (e.g. no memory space). */
//  UResBase * getResource(const char * resID);
  /**
  return true if all ressources is available */
//  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Data that should be transferred to a client directly is received */
  bool dataTrap(USmlTag * tag);
  /**
  Set command list to an interface alias and an alias OnConnect name.
  This redefines the command keyword names of this interface */
  //virtual void setAliasName(const char * name);
protected:
  /**
  Add a data handler resource to the interface client */
  //bool addResource(const char * resName);

protected:
  static const int MAX_IF_HANDLERS = 100;
  /**
  Pointers to loaded interface data handlers */
  //UResIfBase * ifH[MAX_IF_HANDLERS];
  /**
  Are the interface resource added to this interface -
  NO, they are all added to all interfaces - should not be a problem. */
  // bool * ifHAdded[MAX_IF_HANDLERS];
  /**
  Number of used interface handlers */
  //int ifHCnt;
  /**
  Pointer to the interface resource */
  UResIf * resIf;
  /**
  Is the interface resource locally created */
  bool resIfLocal;
  /**
  Pointer to the handler of standard variables.
  NB! once created, it is added to the resource list of interface handlers,
  as if it were a plug-in interface data handler. */
  UResIfVar * ifVar;
  /**
  Is standard variable handler locally created */
  bool ifVarLocal;

private:
  /**
  Function to to handle interface management issues */
  bool handleIf(UServerInMsg * msg);
  /**
  Handle onnect push stack */
  bool handleIfPush(UServerInMsg * msg);
  /**
  Data trap serial number (0 is no data) */
  unsigned int dataTrapSerial;
  /**
  Data trap serial number (0 is no data) for trap of help messages */
  unsigned int dataTrapSerialHelp;
  /**
  Client number for trapped data */
  int dataTrapClient;
  /**
  Ressource list provided */
  //char resList[MAX_RESOURCE_LIST_SIZE];
  /**
  Interface alias name */
  char aliasOnConnectName[MAX_ID_LENGTH];
  /**
  Interface alias name */
  //char aliasCommandList[2 * MAX_ID_LENGTH];
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#endif

