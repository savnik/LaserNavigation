/** \file rulesmrclgenerator.h
 *  \ingroup Code generator
 *  \brief SMR-CL Generator module based on the AuServer
 *
 *  This class implements the smrclInterface, to generate SMR-CL
 *  extracted from the AuServer variable pool
 *
 *  This allows for a completely code-independent code-generator
 *  module. Reccomended implementation is through the rule-based
 *  plug-in for AuRobotServers.
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

# include <urob4/uresvarpool.h>
# include <urob4/uvarpool.h>

#include "smrclinterface.h"

#ifndef _RULESMRCLGENERATOR_H
#define	_RULESMRCLGENERATOR_H

/** Class structure to hold pointers to a generator rule
 *
 */

class robotCommandUVar {
public:
    robotCommandUVar() { //Constructor
        cmdValid = false;
    }
    //Data containers
    string      cmdName;        /**< Name of the command */
    string      caseCmd;        /**< The processed command case statement */
    bool        cmdValid;       /**< Is the command valid loaded from rule */
    int         id;             /**< Id associated with the command */
    UVariable   *preCommand;    /**< Pre-command SMR-CL statements */
    UVariable   *postCommand;   /**< Post-command SMR-CL statements */
    UVariable   *command;       /**< The actual SMR-CL commands */
    UVariable   *failCond;      /**< The condition for the command to fail */
    UVariable   *reloadCond;    /**< The condition for the command to reload */
    UVariable   *successCond;   /**< The condition the having reached the destination connector */
    UVariable   *initCommand;   /**< Initialization statements issued at the start of each edge */
    UVariable   *exitCommand;   /**< Exit statements issued at end of each edge */
    UVariable   *quality;       /**< Quality factor for the command (for prioritizing) */
    UVariable   *failed;        /**< Variable indicating that this navigation mehod is currently failing*/
    UVariable   *disable;       /**< Variable to disable execution of the command */
    UVariable   *active;        /**< Variable to show if command is active in execution */
    UVariable   *reload;        /**< Variable reload the command if it is active in execution */
    robotCommandUVar *next;  /**< Structure of the commands for traversing the next edge */
};

/** Rule-based SMR-CL generator class */
class ruleSmrclgenerator: public smrclInterface  {
public:
    ruleSmrclgenerator();
    ruleSmrclgenerator(const ruleSmrclgenerator& orig);
    virtual ~ruleSmrclgenerator();

    //Functions implemented from smrclInterface
protected:
    /** Initialization function
     * Pass any type of data you like through the void pointer
     */
    bool    initialize(void *);
    /** Toggle the mission execution
     * This function informs the code-generator of execution status.
     * It can be used to load the command-set or load the mission.
     *
     * A void pointer is provided as input, if any informations regarding
     * the plan should be transferred
     */
    bool    setExecution(bool, void *);
    /** Set a command Id as active running */
    bool    setActive(int, bool);
    /** Set a command Id as failed */
    bool    setFailed(int, bool);
    /** Check if the command has requested a reload */
    bool    getReload(int);
    /** Check if the has been disabled */
    bool    getDisable(int);
    /** Function to process the generated smr-cl code and place it into
     *  the robotCommand vector for the current route element.
     *
     * The input-param waitForNew indicates if the code-generator shuld wait
     * until a new set of smr-cl is generated. This will be set i.e. when
     * the robot changes into a new edge, to have fresh generated smr-cl.
     */
    bool    processCommands(vector<robotCommand> *, bool waitForNew = false);
    /** Function to process the generated smr-cl code and place it into
     *  the robotCommand vector for the next route element.
     *
     * The input-param waitForNew indicates if the code-generator shuld wait
     * until a new set of smr-cl is generated. This will be set i.e. when
     * the robot changes into a new edge, to have fresh generated smr-cl.
     */
    bool    processNextCommands(vector<robotCommand> *, bool waitForNew = false);

private:
    /* Private variables */
    UVarPool    *plannerPool;
    UVarPool    *rootPool;
    UVariable   *running;
    UVariable   *ruleIter;

    int         generatorCount;
    int         lastRuleIter;
    int         ruleIterCmds;
    int         ruleIterNextcmds;

    /* Private functions */
    string                  ruleStringLoader(UVariable *);
    robotCommandUVar *      loadGeneratorRule(UVarPool *);
    /** Is any rules loaded into the code generator */
    size_t                  rulesLoaded(void);
    /** Load and/or refresh the list of code generator rules in the system */
    int                     updateRuleList(void);

    vector<robotCommandUVar> genRules;


};

#endif	/* _RULESMRCLGENERATOR_H */

