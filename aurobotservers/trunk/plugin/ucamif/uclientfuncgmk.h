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
#ifndef UCLIENTFUNCGMK_H
#define UCLIENTFUNCGMK_H

#include <urob4/uresifbase.h>
#include <umap4/uprobpoly.h>

#include "ugmkpool.h"
//#include "ucalcmmr.h"
class UCalcMmr;

/**
Class to handle path data

@author Christian Andersen
*/

class UClientFuncGmk : public UResIfBase
{
public:
  /**
  Constructor */
  UClientFuncGmk();
  /**
  Destructor */
  ~UClientFuncGmk();
  /**
  Name of function
  The returned name is intended as informative to users
  and should include a version number */
  virtual const char * name();
  /**
  Function, that shall return a string with all handled commands,
  i.e. should return "gmk gmk2d guidemark", if commands
  starting with any of these three keywords are handled
  by this function */
  virtual const char * commandList();
  /**
  Got fresh data destined to this function. */
  virtual void handleNewData(USmlTag * tag);
  /**
  Set guidemark pool */
  inline UGmkPool * getGmkPool()
  { return gmkPool; }

protected:
  /**
  Unpack pathGet message */
  void handleGmkGet(USmlTag * tag);
  /**
  Unpack other Get messages like camget, and
  sets variables in calculator to reflect received value. */
  void handleGet(USmlTag * tag);
  /**
  This function is called, when new guidemark(s) is/are unpacked.
  It can be used to trigger other functions. */
  virtual void newDataAvailable(int updCnt, UTime updTime);

protected:
  /**
  Pointer to polygon data buffer */
  UGmkPool * gmkPool;
};

#endif
