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
#ifndef UGMKPOOL_H
#define UGMKPOOL_H

#include <ugen4/ugmk.h>

/**
Holds all detected guidemarks

@author Christian Andersen
*/
class UGmkPool
{
public:
  /**
  Constructor */
  UGmkPool();
  /**
  Destructor */
  ~UGmkPool();
  /**
  Get handle for guidemark with this short code.
  If not available, then create if 'mayCreate' is true.
  Returns NULL if not 'mayCreate' and not found. */
  UGmk * getGmk(unsigned long shortCode, bool mayCreate);
  /**
  Get number of gmks with an update time newer than the parameter time */
  int getGmkNewCnt(UTime sinceTime);
  /**
  Get total number of guidemarks in pool */
  int getGmkCnt();
  /**
  Get a specific gmk among the new ones */
  UGmk * getGmkNew(int serial, UTime sinceTime);
  /**
  Get a specific gmk from the number in the pool */
  UGmk * getGmkNum(unsigned int serial);
  /**
  Get a specific gmk from the number its ID */
  UGmk * getGmkId(unsigned long ID);
  

protected:
  /**
  Max number of guidemarks */
  static const int MAX_GUIDEMARK_COUNT = 100;
  /**
  Guidemark storage list */
  UGmk * gmks[MAX_GUIDEMARK_COUNT];
  /**
  Used guidemarks */
  int gmksCnt;
};

#endif
