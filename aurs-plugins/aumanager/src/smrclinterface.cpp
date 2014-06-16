/** \file smrclginterface.h
 *  \ingroup Code generator
 *  \brief Interface class for SMR-CL Generator modules
 *
 *  This interface class shows the interface for any code-generator class
 *
 *  \author Anders Billesø Beck
 *  $Rev: 54 $
 *  $Date: 2009-03-30 17:15:21 +0200 (Mon, 30 Mar 2009) $
 */
/***************************************************************************
 *                  Copyright 2009 Anders Billesø Beck, DTU                *
 *                       anders.beck@get2net.dk                            *
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

#include <stdio.h>

#include "smrclinterface.h"

smrclInterface::smrclInterface() {
}

smrclInterface::smrclInterface(const smrclInterface& orig) {
}

smrclInterface::~smrclInterface() {
}

bool smrclInterface::initialize(void *nothing) {
    printf("SMR-CL Interface must be implemented in a code generator module\n");
    return false;
}

bool smrclInterface::setExecution(bool run, void *nothing){
    printf("SMR-CL Interface must be implemented in a code generator module\n");
    return false;
}

bool smrclInterface::processCommands(vector<robotCommand> *, bool waitForNew) {
    printf("SMR-CL Interface must be implemented in a code generator module\n");
    return false;
}
bool smrclInterface::processNextCommands(vector<robotCommand> *, bool waitForNew) {
    printf("SMR-CL Interface must be implemented in a code generator module\n");
    return false;
}

/** Set a command Id as active running */
bool smrclInterface::setActive(int id, bool value) {
    printf("SMR-CL Interface must be implemented in a code generator module\n");
    return false;
}

bool smrclInterface::setFailed(int id, bool value) {
    printf("SMR-CL Interface must be implemented in a code generator module\n");
    return false;
}

bool smrclInterface::getReload(int id) {
    printf("SMR-CL Interface must be implemented in a code generator module\n");
    return false;
}

bool smrclInterface::getDisable(int id){
    printf("SMR-CL Interface must be implemented in a code generator module\n");
    return false;
}
