/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
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
  * Returns false if no module is available. */
  virtual bool getStaticHelpList(char * list, const int listCnt);
  /**
  * Create a function module that may be statically available in this server.
  * The moduleName is a keyword to one of the available static modules.
  * Returns false if no such module exist.
  * If the module exist it is created and added to the function list. */
  virtual bool loadStaticModule(const char * moduleName, const char * aliasName, char * why, const int whyCnt);
};

#endif
