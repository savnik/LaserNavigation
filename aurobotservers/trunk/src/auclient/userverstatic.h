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
#ifndef USERVERSTATIC_H
#define USERVERSTATIC_H

#include <urob4/ucmdexe.h>

#define STATIC_MODULE_TESTER

/**
Server core extension with static modules

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UServerStatic : public UCmdExe
{
public:
  /**
  Constructor */
  UServerStatic();
  /**
  Destructor */
  virtual ~UServerStatic();
  /**
  * Get help text for available static modules loadable by the loadStaticModule() function.
  * \Returns false if no module is available. */
  virtual bool getStaticHelpList(char * list, const int listCnt);
  /**
  * Create a function module that may be statically available in this server.
  * The moduleName is a keyword to one of the available static modules.
  * \Returns false if no such module exist.
  * If the module exist it is created and added to the function list. */
  inline virtual bool loadStaticModule(const char * moduleName)
  { return loadStaticModule(moduleName, NULL, NULL, 0); };
  /**
   * Create a function module that may be statically available in this server.
   * The moduleName is a keyword to one of the available static modules.
   * \Returns false if no such module exist.
   * If the module exist it is created and added to the function list.
  If alias name is provided (and the module allows) the module command keyword is redefined to the alias name
  (this is mostly (only) the generic interface module.
  \returns false if a module with an illegal name exist, in this case the reason is returned in 'why' (if not NULL) */
  virtual bool loadStaticModule(const char * moduleName, const char * aliasName, char * why, const int whyCnt);
};

#endif
