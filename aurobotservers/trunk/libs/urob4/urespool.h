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
#ifndef URESPOOL_H
#define URESPOOL_H

#include "uresbase.h"

/**
The number of shared resources a server can hold. */
#define MAX_RESOURCE_POINTERS 200

/**
Resource pool for server.
It holds the resource pointer, and the resource name and the function responcible for the resource.

@author Christian Andersen
*/
class UResPool
{
public:
  /**
  Constructor */
  UResPool();
  /**
  Destructor */
  ~UResPool();
  /**
  Add a resource.
  Returns true if resource is added.
  Returns false if no more space. */
  bool addResource(UResBase * resource);
  /**
  remove a resource by function index.
  Returns a pointer to the removed resource.
  One function index may own more than one resource,
  so call this function until a NULL is returned
  to remove all owned resources.
  Returns NULL is no resource with this index exist */
  UResBase * removeResourceFunc(int funcIndex);
  /**
  remove a resource by resource index.
  Returns a pointer to the removed resource.
  Returns NULL is no resource with index exist */
  UResBase * removeResourceIndex(int resIndex);
  /**
  Get resource with this index.
  Returns also a pointer to the resource ID string. */
  UResBase * getResource(int resIndex);
  /**
  Get resource based on ID string.
  Returns resource pointer if found.
  Returns NULL if not found.*/
  UResBase * getResource(const char * resID);
  /**
  Get highest number of items in resource pool */
  int getResCnt()
  { return resCnt; };
  /**
  Get owner index of this resource */
  int getResFunc(int ressIndex);
  /**
  Print short status */
  void print(const char * preString);
  /**
  Print status to string buffer.
  Prestring is added at the start and at max 'bufCnt' characters are written.*/
  void print(const char * preString, char * buff, int buffCnt);
  /**
  Tell all resources to save settings as needed */
  void saveSettings();
  /**
   * Tell all resources to stop all running threads */
  void stop(bool andWait);
  /**
   * Handle any pending replay update and allow other resources
   * to update to the same replay time.
   * \returns true if a replay were pending and serviced. */
  bool handleReplay();

protected:
  /**
  Array of resource pointers */
  UResBase * res[MAX_RESOURCE_POINTERS];
  /**
  Count of highest resource number in use */
  int resCnt;
  /**
  Array of function index numbers */
  //int ressFunc[MAX_RESOURCE_POINTERS];
};

#endif
