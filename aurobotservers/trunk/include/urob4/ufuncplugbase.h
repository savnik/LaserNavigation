/***************************************************************************
 *   Copyright (C) 2008 by DTU Christian Andersen   *
 *   chrand@mail.dk   *
 *        vvvvvv                                                                 *
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
#ifndef UFUNCPLUGBASE_H
#define UFUNCPLUGBASE_H

#include "ufunctionbase.h"

/**
Plugin module base class.
This addes a few extra definitions that is needed when compiled as a plugin

	@author Christian Andersen <chrand@mail.dk>
*/
class UFuncPlugBase : public UFunctionBase
{
public:
  /**
   * Constructior */
  UFuncPlugBase();
  /**
   * destructor */
  virtual ~UFuncPlugBase();

};


/**
Plugin module base class, when the class can act as event generator.
This addes a few extra definitions that is needed when compiled as a plugin

        @author Christian Andersen <chrand@mail.dk>
*/
class UFuncPlugBasePush : public UFunctionBase, public UServerPush
{
public:
  /**
   * Constructior */
  UFuncPlugBasePush();
  /**
   * destructor */
  virtual ~UFuncPlugBasePush();
  /**
  Set (or remove) ressource (core pointer needed by event handling) */
  virtual bool setResource(UResBase * resource, bool remove);

protected:
  /**
  Handle stereo-push commands
  \param msg is the message with the push related command.
  \returns true if the command were handled successfully. */
  bool handlePush(UServerInMsg * msg);

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


#endif
