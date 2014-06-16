/** \file smrclgenerator.h
 *  \ingroup Planner
 *  \brief SMR-CL Generator liberary
 *
 *  This liberary generates SMR-CL plans for robot execution
 *
 *  Input is based on a route planning input, from the graphmap planner
 *
 *  \author Anders Billesø Beck
 *  $Rev: 59 $
 *  $Date: 2009-04-01 15:47:51 +0200 (Wed, 01 Apr 2009) $
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

using namespace std;

#include <string>
#include <vector>

# include <urob4/uresvarpool.h>
# include <urob4/uvarpool.h>

#include "mapcontainers.h"
#include "smrclinterface.h"
#include "rulesmrclgenerator.h" 

#ifndef _SMRCLGENERATOR_H
#define	_SMRCLGENERATOR_H






class smrclGenerator {
public:
    smrclGenerator();
    smrclGenerator(const smrclGenerator& orig);
    virtual ~smrclGenerator();

    //Management functions
    /** Load the root VarPool into the code generator module
     * This is used to extract smr-cl code from the global.codegen
     * varpool.
     **/
    bool    setVarpool(UVarPool *);
    /** Load a SMR-CL Generator module into the generator */
    bool    loadSmrclGenerator(smrclInterface *);
    /** Signals start of mission execution
     * When mission execution is running, the command-set should be fixed
     */
    bool setExecution(bool);
    /** Set a command as active
     */
    bool setActive(int);
    /** Generate a new switch statement from the planned route, from the current
     *  route element and the next
     *
     *  The bool parameter wait, determined if the code-generator should wait
     *  for a new loop of the code-generator or not.
     */
    string  generateSwitch(vector<route>,size_t, bool);
    /** Generates the initialization scripts to be executed before the
     *  first mission element
     */
    string  initializeMission(vector<route>);
private:
    //Stitching functions
    vector<robotCommand>    processCommands(vector<route>,size_t);
    string                  makeCaseCommand(vector<robotCommand>, size_t);
    bool                    clearCommands(void);

    //Code generator functions
    robotCommand* followLine    (vector<route>,int);
    robotCommand* shortOdoDrive (vector<route>,int);
    robotCommand* fillDrive     (vector<route>,int);
    robotCommand* stopCmd          (vector<route>, int);

    //Class objects and variables
public:
    vector<robotCommand>    oldCmds;
    vector<robotCommand>    cmds;
    vector<robotCommand>    nextCmds;

    int     evCounter;
    int     successId;
    int     crashId;

private :
    UVarPool    *rootPool;

    /** Pointer to code generator module (inheriting the interface)
     */
    smrclInterface  *codeGen;



};

#endif	/* _SMRCLGENERATOR_H */

