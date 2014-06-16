/** \file smrclginterface.h
 *  \ingroup Code generator
 *  \brief Interface class for SMR-CL Generator modules
 *
 *  This interface class shows the interface for any code-generator class
 *
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

#include <string>
#include <string.h>
#include <vector>

using namespace std;

#ifndef _SMRCLINTERFACE_H
#define	_SMRCLINTERFACE_H

/** Container class for SMR-CL Command elements */
class robotCommand {
public:
    string  cmdName;
    string  preCommand;
    string  postCommand;
    string  command;
    string  failCond;
    string  reloadCond;
    string  successCond;
    string  initCommand;
    string  exitCommand;
    double  quality;
    bool    disable;
    bool    reload;
    int     id;

    /** Operator overload for performing an STL Sort */
    bool operator < ( const robotCommand& arg ) const {
        return quality < arg.quality;
    }
};


/** Interface class to be implemented by any SMR-CL Codegenerator module
 */
class smrclInterface {
public:
    smrclInterface();
    smrclInterface(const smrclInterface& orig);
    virtual ~smrclInterface();

    /** Initialization function
     * Pass any type of pointer you like through the void pointer
     */
    virtual bool    initialize(void *);

    /** Toggle the mission execution
     * This function informs the code-generator of execution status.
     * It can be used to load the command-set or load the mission.
     *
     * A void pointer is provided as input, if any informations regarding
     * the plan should be transferred
     */
    virtual bool    setExecution(bool, void *);

    /** Function to process the generated smr-cl code and place it into
     *  the robotCommand vector for the current route element.
     *
     * For better flexibility
     *
     * The input-param waitForNew indicates if the code-generator shuld wait
     * until a new set of smr-cl is generated. This will be set i.e. when
     * the robot changes into a new edge, to have fresh generated smr-cl.
     */
    virtual bool    processCommands(vector<robotCommand> *, bool waitForNew = false);
    /** Function to process the generated smr-cl code and place it into
     *  the robotCommand vector for the next route element.
     *
     * The input-param waitForNew indicates if the code-generator shuld wait
     * until a new set of smr-cl is generated. This will be set i.e. when
     * the robot changes into a new edge, to have fresh generated smr-cl.
     */
    virtual bool    processNextCommands(vector<robotCommand> *, bool waitForNew = false);
    /** Set a command Id as active running */
    virtual bool    setActive(int, bool);
    /** Set a command Id as failed */
    virtual bool    setFailed(int, bool);
    /** Check if the command has requested a reload */
    virtual bool    getReload(int);
    /** Check if the has been disabled */
    virtual bool    getDisable(int);
private:

};

#endif	/* _SMRCLINTERFACE_H */

