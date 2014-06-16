/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@oersted.dtu.dk
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
#ifndef UFUNCTIONVARPOOL_H
#define UFUNCTIONVARPOOL_H

#include "ufunctionbase.h"
#include "uresvarpool.h"

/**
Interface function to handle local variables and var structures from other ressources.

	@author Christian <chrand@mail.dk>
*/
class UFunctionVarPool : public UFunctionBase
{
public:
  /**
  Constructor */
  UFunctionVarPool();
  /**
  Destructor */
  ~UFunctionVarPool();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs and add these to the resource pool
   * to be integrated in the global variable pool and method sharing.
   * Remember to add to resource pool by a call to addResource(UResBase * newResource, this).
   * This module must also ensure that any resources creted are deleted (in the destructor of this class)
   * Resource shut-down code should be handled in the resource destructor.
   * \return true if any resources are created. */
  virtual void createResources();
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  Set resource pointer - especially any resources acting af a var pool too */
  virtual bool setResource(UResBase * resource, bool remove);

protected:
  /**
  Handle command 'var' and all its options */
  bool handleVar(UServerInMsg * sg, void * extra);
  /**
  Handle this variable set or request.
  If a structure with no name is specified, then 'aStruct' will be set true.
   * \param andDesc if true, then the variable description is send too
   * \returns true if send */
  bool handleVarValues(UServerInMsg * msg, UVarCalc * vp, const char * attName,
                       const char * attValue, const char * type,
                      bool andDesc);
  /**
  Handle push commands */
  bool handleVarPush(UServerInMsg * msg);
  /**
  Send a copy of all values in this var pool.
  The structures in the var pool are recursed.
  All variables are returned as tags with the appropriate data type.
   * \param msg is the source message - for reply details
   * \param varPool is the vaiable list to send
   * \param structPreName is the name of the varpool
   * \param andDesc if true, then the variable description is send too. */
  void sendAllVar(UServerInMsg * msg, UVarCalc * varPool,
                  const char * structPreName, bool andDesc);
  /**
  * Send value information for a single variable in this var pool.
   * \param msg reference to the request and the return adress for the message
   * \param vv  pointer to the variable to be handled
   * \param structPreName global structure specification
   * \param andDesc if true, then the variable description is send too
   * \param extra is an optional extra attribute to send with the reply.
  * \Returns true if variable is found.
  * \returns varSize as the number of variables in the pseudo structure (1 to 3). */
  bool sendVar(UServerInMsg * msg, UVariable * vv,
                                 const char * structPreName,
                                 bool andDesc,
              const char * extra);
  /**
  Send the description part formatted with an open left margin and a ragged right margin */
  void sendDescription(UServerInMsg * msg, const char * prestring,
                       const char * desc);
  /**
  Decode this string as a method call, do the call and return the result */
  void makeAMethodCall(UServerInMsg * msg, char * aCallStr, char * returnStructType);
  /**
  Initialize this pointer array by structures if the type 'returnStructType'. */
  bool initCallReturnStructType(UDataBase ** rParOrg, const int MRP, char * returnStructType);
  /**
  Send a list of variables, either as XML tags (copy) or as help text
  \param msh is message for additional options.
  \param vp is the strucure to list.
  \param andDesc - send also the description.
  \param asSingle - send as XML tags rather than help text.
  \returns true if something is send. */
  bool sendVarList(UServerInMsg * msg, UVarPool * vp, bool andDesc, bool asSingle);

protected:
  /**
  pointer to UVarCalc resource */
  UResVarPool * varPool;
  /**
  Was the resource created locally */
  //bool varPoolLocal;

private:
  /**
  Margin for variable name and value when sending variable list */
  int leftMargin;
};

#endif
