/** \file smrclgenerator.cpp
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

#include <vector>

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
/*********************** Version control information ***********************/
 #define VERSION  "1.0"
 #define REVISION "$Rev: 59 $:"
 #define DATE     "$Date: 2009-04-01 15:47:51 +0200 (Wed, 01 Apr 2009) $:"
/***************************************************************************/

#include <string>
#include <string.h>
#include <algorithm>
#include <map>

#include <urob4/uvarpool.h>

#include "smrclgenerator.h"
#include "smrclinterface.h"

smrclGenerator::smrclGenerator() {

    //Intialize variables
    evCounter = 0;
    codeGen = NULL;
}

smrclGenerator::smrclGenerator(const smrclGenerator& orig) {
}

smrclGenerator::~smrclGenerator() {
}

bool smrclGenerator::setVarpool(UVarPool *Varpool) {

    rootPool = Varpool;  

    if (rootPool == NULL) {
        printf("Failed getting root-varpool\n");
        return false;
    }

    //Create the rule-based SMR-CL Code generator module
    codeGen = new ruleSmrclgenerator;

    //Initialize the code-generator
    codeGen->initialize((void*)rootPool);

    return true;

}

//Set the mission execution status
bool  smrclGenerator::setExecution(bool run) {
    
    //Inform the code-generator that execution starts
    codeGen->setExecution(run,NULL);

    //Run the commands processor once, and set the success and crashId's
    codeGen->processCommands(&cmds,true);

    //Set the id's
    crashId     = cmds.size() + 1;
    successId   = cmds.size() + 2;

    return true;
}

//Set the active rule
bool smrclGenerator::setActive(int activeId) {

    bool activeFound = false;

    //If the active ID is the success Id, then the current active command
    //is the command of highest priority
    if (activeId == successId) activeId = nextCmds[0].id;
    printf("Setting %d active\n",activeId);

    for (size_t i = 0; i < cmds.size(); i++) {
        //Set the activeId true
        if (activeId == cmds[i].id) {
            codeGen->setActive(activeId, true);
            codeGen->setFailed(activeId,false);
            activeFound = true;
        }
        //Set the lower quality commands to !failed
        else if (activeFound) {
            codeGen->setActive(cmds[i].id, false);
            codeGen->setFailed(cmds[i].id,false);
        }
        //Any command with higher quality than activeId must have failed
        else {
            codeGen->setActive(cmds[i].id, false);
            codeGen->setFailed(cmds[i].id,true);
        }

    }

    return true;

}


/******************** ROBOT CODE ASSEMBLER *****************************/

/** Robot code assembler
 *
 * This function generates a switch statement for continous robot movement
 *
 */
string smrclGenerator::generateSwitch(vector<route> rPlan,size_t pIndex, bool wait) {

    string                  cmdStr;
    const int               STRLEN = 256;
    char                    tempStr[STRLEN];
    int                     caseCnt = 1;

    //Error check
    if (rPlan.size() <= (size_t)pIndex) {
        printf("Generator: Plan is has less elements than index");
        return cmdStr;
    }


    //Save the old commands (currently not used)
    oldCmds = cmds;

    //Clear the commands vector
    clearCommands();

    //Process the rules to generate command structure
    codeGen->processCommands(&cmds,wait);
    codeGen->processNextCommands(&nextCmds,wait);

    //Set the id's
    crashId = cmds.size() + 1;
    successId = cmds.size() + 2;
        
    //Sort the commands in highest quality first order
    sort(cmds.begin(),cmds.end());
    reverse(cmds.begin(),cmds.end());
    sort(nextCmds.begin(),nextCmds.end());
    reverse(nextCmds.begin(),nextCmds.end());
    //Generate commands output for the route element
    //cmds = processCommandsFromRule();//processCommands(rPlan,pIndex);//

    //Generate commands output for the next route element (forward planning)
    //(Only if the route has enough elements)
    // nextCmds = processCommands(rPlan,pIndex+1);
    
    //Reset trigger variables
    cmdStr += "success = 0\n";
    cmdStr += "reload  = 0\n";

    //Start the switch
    cmdStr += "switch (condVal)\n";

    //Make a case of the highest quality command for the following route element
    //to execute, if the previous command is finished successfully
    snprintf(tempStr,STRLEN,"  putevent \"ev%d\"\n",successId);
    cmdStr += tempStr;

    //Clear the fail-variables
    for(size_t i = 0; i < cmds.size(); i++) {
        snprintf(tempStr,STRLEN,"  fail%d = 0\n",cmds[i].id);
        cmdStr += tempStr;
    }
    //Load the init variables
    for (size_t i = 0; i < nextCmds.size(); i++)
        if ((nextCmds[i].quality > 0) && (!nextCmds[i].initCommand.empty())) {
            cmdStr += nextCmds[i].initCommand;
            cmdStr += "\n";
        }
    //Put the next command if there is any elements left in the mission, otherwise stop
    if ((pIndex+1) >= rPlan.size()) {
        cmdStr += "stop\n wait 2\n idle\n"; //Stop the robot
    } else {
       cmdStr += makeCaseCommand(nextCmds, 0); 
    }
    
    


    //Add the case-commands for the command, if the command is not empty
    for(size_t i = 0; i < cmds.size(); i++) {
        if (!cmds[i].command.empty()) {
            snprintf(tempStr,STRLEN,"  case %d\n",caseCnt);
            caseCnt++;
            cmdStr += tempStr;
            snprintf(tempStr,STRLEN,"  putevent \"ev%d\"\n",cmds[i].id);
            cmdStr += tempStr;

            //Add the command itself
            cmdStr += makeCaseCommand(cmds, i);
        }
    }

    //Make a "last resort case"
    snprintf(tempStr,STRLEN," case %d\n",caseCnt);
    caseCnt++;
    cmdStr += tempStr;
    snprintf(tempStr,STRLEN,"  putevent \"ev%d\"\n",crashId);
    cmdStr += tempStr;
    snprintf(tempStr,STRLEN,"  stop\n");
    cmdStr += tempStr;

    //Make a case of the highest quality command for the following route element
    //to execute, if the previous command is finished successfully
    snprintf(tempStr,STRLEN," case %d\n",successId);
    caseCnt++;
    cmdStr += tempStr;
    snprintf(tempStr,STRLEN,"  putevent \"ev%d\"\n",successId);
    cmdStr += tempStr;
    for(size_t i = 0; i < cmds.size(); i++) {
        snprintf(tempStr,STRLEN,"  fail%d = 0\n",cmds[i].id);
        cmdStr += tempStr;
    }
    for (size_t i = 0; i < nextCmds.size(); i++) {
         if ((cmds[i].quality > 0) && (!cmds[i].exitCommand.empty())) {
            cmdStr += cmds[i].exitCommand;
            cmdStr += "\n";
        }
        if ((nextCmds[i].quality > 0) && (!nextCmds[i].initCommand.empty())) {
            cmdStr += nextCmds[i].initCommand;
            cmdStr += "\n";
        }
    }
    //Put the next command if there is any elements left in the mission, otherwise stop
    if ((pIndex+1) >= rPlan.size()) {
        cmdStr += "stop\n wait 2\n idle\n"; //Stop the robot
    } else {
       cmdStr += makeCaseCommand(nextCmds, 0);
    }

    //Finally round up the switch
    cmdStr += "endswitch\n";

    return cmdStr;
}

/** Outputs the case string for a specific robotCommand output
 *
 **/

string smrclGenerator::makeCaseCommand(vector<robotCommand> inputCmds, size_t cmdIndex) {

    string cmdStr;
    const int   STRLEN = 256;
    char        tempStr[STRLEN];
    int         condCnt = 0;

    //Add a nametag
    snprintf(tempStr,STRLEN,"%% *** %s ***\n",inputCmds[cmdIndex].cmdName.c_str());
    cmdStr += tempStr;

    //Add pre-command
    if (!inputCmds[cmdIndex].preCommand.empty()) {
        cmdStr += inputCmds[cmdIndex].preCommand;
        cmdStr += "\n";
    }

    //Add command
    cmdStr += inputCmds[cmdIndex].command;
    cmdStr += " : "; //Start stop-criteria string;

    //First generate inverse-stop criterias
    bool found = false; //is any commands found
    for (size_t j = 0; j < cmdIndex; j++) {
        if ((inputCmds[j].quality > 0.0) && !inputCmds[j].command.empty()) {
            if (found) cmdStr += " | "; //Or-seperate the criterias (not the first)
            found = true;
            cmdStr += "((";
            snprintf(tempStr,STRLEN,"fail%d",inputCmds[j].id);
            cmdStr += tempStr;
            if (!inputCmds[j].failCond.empty()) {
                cmdStr += "|";
                cmdStr += inputCmds[j].failCond;
            }
            cmdStr += ")==0)";
            condCnt++;
        }
    }
    
    //Then put the reload-criterias for the function
    if (cmdIndex > 0) cmdStr += " | ";
    cmdStr += "(reload";
    if (!inputCmds[cmdIndex].reloadCond.empty()) {
        cmdStr += "|(";
        cmdStr += inputCmds[cmdIndex].reloadCond;
        cmdStr += ")";
    }
    cmdStr += ")";
    condCnt++;

    //Then put the failure-criteria for this function
    snprintf(tempStr,STRLEN," | (fail%d|",inputCmds[cmdIndex].id);
    cmdStr += tempStr;
    if (inputCmds[cmdIndex].failCond.empty()) cmdStr += "0";
    else                          cmdStr += inputCmds[cmdIndex].failCond;
    cmdStr += ")";
    condCnt++;

    //Put zero-conditions until the success criteria is matched
    for (; condCnt < (successId-1); condCnt++) {
        //if ((inputCmds[j].quality > 0.0) && !inputCmds[j].command.empty()) {
            cmdStr += " | (0)";
        //}
    }

    //Finally generate success string
    cmdStr += " | (success";
    for (size_t j = 0; j < inputCmds.size(); j++) {
        if (!inputCmds[j].successCond.empty())   {
            cmdStr += "|(";
            cmdStr += inputCmds[j].successCond;
            cmdStr += ")";
        }
    }
    cmdStr += ")\n";

    //Collect stop-condition
    cmdStr += "condVal = $condition\n";

    //Finally add post-command
    if (!inputCmds[cmdIndex].postCommand.empty()) cmdStr += inputCmds[cmdIndex].postCommand;

    //Return the output
    return cmdStr;


}


/** Function to clear the contends of the commands vectors
 */
bool    smrclGenerator::clearCommands(void) {

    for(size_t i = 0; i < cmds.size(); i++) {
        cmds[i].cmdName.clear();
        cmds[i].command.clear();
        cmds[i].exitCommand.clear();
        cmds[i].failCond.clear();
        cmds[i].id = 0;
        cmds[i].initCommand.clear();
        cmds[i].postCommand.clear();
        cmds[i].preCommand.clear();
        cmds[i].quality = 0.0;
        cmds[i].reloadCond.clear();
        cmds[i].successCond.clear();

    }

    for(size_t i = 0; i < nextCmds.size(); i++) {
        nextCmds[i].cmdName.clear();
        nextCmds[i].command.clear();
        nextCmds[i].exitCommand.clear();
        nextCmds[i].failCond.clear();
        nextCmds[i].id = 0;
        nextCmds[i].initCommand.clear();
        nextCmds[i].postCommand.clear();
        nextCmds[i].preCommand.clear();
        nextCmds[i].quality = 0.0;
        nextCmds[i].reloadCond.clear();
        nextCmds[i].successCond.clear();
    }

    return true;


}


/** Function to process all code-generator commands and return a
 *  quality ordered vector containing all generator output for
 *  the given route element
 */
vector<robotCommand> smrclGenerator::processCommands(vector<route> rPlan,size_t pIndex) {

    const int               nGenerators = 3;
    vector<robotCommand>    cmds;
    robotCommand            *newCmd = NULL;

    //Return just the stop commands if pIndex is out of scope
    if (pIndex >= rPlan.size()) {
        newCmd = stopCmd(rPlan,pIndex);
        cmds.push_back(*newCmd);
        delete newCmd;
    } else {
        //Run all code generator functions
        for (int i = 0; i < nGenerators; i++) {

            //Fix so far, switch beween generators
            switch (i) {
                case 0:
                    newCmd = followLine(rPlan,pIndex);
                    break;
                case 1:
                    newCmd = shortOdoDrive(rPlan,pIndex);
                    break;
                case 2:
                    newCmd = fillDrive(rPlan,pIndex);
                    break;
            };
            //Only use commands, if quality is better than zero..
            if (newCmd->quality > 0) cmds.push_back(*newCmd);
            delete newCmd;
        }
    }

    //Sort the commands by quality using the STL sorting algorithm
    //and reverse it to have largest quality first
    sort(cmds.begin(),cmds.end());
    reverse(cmds.begin(),cmds.end());

    return cmds;
}


string smrclGenerator::initializeMission(vector<route> rPlan) {

    string initStr;
    const int   STRLEN = 256;
    char        tempStr[STRLEN];

    //Process the rules to generate command structure
    clearCommands();
    codeGen->processCommands(&cmds,false);

    //Set condition value for first switch
    initStr = "condVal = 1\n";

    for(size_t i = 0; i < cmds.size(); i++) {
        snprintf(tempStr,STRLEN,"  fail%d = 0\n",cmds[i].id);
        initStr += tempStr;
    }

    //Put the next stage init commands in the success case
    for (size_t i = 0; i < cmds.size(); i++) {
        if ((cmds[i].quality > 0.0) && (!cmds[i].initCommand.empty())) {
            initStr += cmds[i].initCommand;
            initStr += "\n";
        }
    }

    return initStr;

}


/******************** ROBOT CODE GENERATOR FUNCTIONS ********************/



/** Follow line generates code to follow black tapelines
 *  if they are marked in the map
 *  Requires: linemarker.color = 0 (black) / 1 (white)
 *  Optional: linemarker.speed = xx (otherwise speed is limited to 0.30)
 *
 *  Quality is good (0.90) at speeds lower than 0.30 but drops above that
 *
 **/
robotCommand *smrclGenerator::followLine(vector<route> rPlan,int pIndex) {

    robotCommand *tempCmd = new robotCommand;
    char tempStr[128];

   printf("Plan size: %d\n",rPlan.size());

   printf("Routeedge: %s -> %s (along %f)\n",rPlan[pIndex].startNode.name,rPlan[pIndex].endNode.name,rPlan[pIndex].routeEdge.length);
   printf("Edge map size: %d\n",rPlan[pIndex].routeEdge.features.size());


   if (rPlan[pIndex].routeEdge.features.count("linemarker.color") == 0)
       printf("linemarker.color not found!!\n");

    //Detect if this command is going to be processed at all
    if ((rPlan[pIndex].routeEdge.features.count("linemarker.color") == 0) ||
        ((rPlan[pIndex].routeEdge.features["linemarker.color"] != 0) &&
        (rPlan[pIndex].routeEdge.features["linemarker.color"] != 1))) {
        tempCmd->quality = 0;
        printf("failing followline\n");
        return tempCmd;
        
    }
   

   

    //Setup quality
    if (rPlan[pIndex].routeEdge.features.count("linemarker.speed") > 0) {
        if (rPlan[pIndex].routeEdge.features["linemarker.speed"] < 0.30) {
            tempCmd->quality = 0.9;
        } else if (rPlan[pIndex].routeEdge.features["linemarker.speed"] < 0.70) {
            //Funky calculation of decreasing quality from speed going from 0.30->0.70
            tempCmd->quality = 0.9 * ((0.70-rPlan[pIndex].routeEdge.features["linemarker.speed"])/(0.40));
        } else {
            tempCmd->quality = 0;
        }
    } else {
        tempCmd->quality = 0.8; //Quality is not top at going 0.30
    }

    //If the first command goes between nodes, we must turn 180 (dirty hack)
    if ((pIndex == 0) &&
        (rPlan[pIndex].startNodeId != rPlan[pIndex].endNodeId)) {
        tempCmd->initCommand = "turn 180 @v0.10\n";
    }

    //Generate command code
    if (rPlan[pIndex].routeEdge.features["linemarker.color"] == 0) {

        //command code
        tempCmd->command.assign("followline \"bm\" ");
        if (rPlan[pIndex].routeEdge.features.count("linemarker.speed") > 0) {
            snprintf(tempStr,128,"@v%4.3f ",rPlan[pIndex].routeEdge.features["linemarker.speed"]);
            tempCmd->command.append(tempStr);
        }

        //Abort criterias
        tempCmd->failCond.assign("($nolineb)");

        //Success criterias
        //If a crossing black line is defined
        if ((rPlan[pIndex].endConn.features.count("linemarker.cross") > 0) &&
            (rPlan[pIndex].endConn.features["linemarker.cross"] == 1.0)) {
            tempCmd->successCond.assign("($crossingblackline)");
        } else { //Otherwise on distance
            snprintf(tempStr,128,"($drivendist > %f)",rPlan[pIndex].routeEdge.length);
            tempCmd->successCond.assign(tempStr);
        }

    } else {
        printf("Follow white line is not implemented yet");
        tempCmd->quality = 0;
        return tempCmd;

    }

    //Finally return the command
    return tempCmd;


}

/**
 * Plan a odometry-drive from connector to connector inside a node */
robotCommand *smrclGenerator::shortOdoDrive(vector<route> rPlan,int pIndex) {

    robotCommand *tempCmd = new robotCommand;
    double  angleIn  = rPlan[pIndex].startConn.features["th"];
    double  angleOut = rPlan[pIndex].endConn.features["th"];
    double  turnAngle;
    char    tempStr[128];

    //Detect if this command is going to be processed at all
    if ((rPlan[pIndex].routeEdge.features["properties.length"] > 0.30) ||
        (rPlan[pIndex].startNodeId != rPlan[pIndex].endNodeId))
    {
        tempCmd->quality = 0;
        return tempCmd;

    }
    
    //Calculate angle
    if (angleIn < angleOut) angleIn += 360.0;
    turnAngle = (angleOut - angleIn) + 180.0;

    //Generate command code
    tempCmd->quality = 0.8;
    //Decrease speed in initial drive
    tempCmd->preCommand.assign("drive @v0.20 :($drivendist < 0.05)\n");
    if ((turnAngle > -10.0) && (turnAngle < 10.0)) {
        tempCmd->command = "drive @v0.20 ";
        snprintf(tempStr,128,"($drivendist > %f)",rPlan[pIndex].endNode.features["dimensions.diameter"]-0.05);
        tempCmd->successCond.assign(tempStr);
    } else {
        snprintf(tempStr,128,"turnr %4.3f %4.3f ",rPlan[pIndex].endNode.features["dimensions.diameter"]-0.05,turnAngle);
        tempCmd->command.assign(tempStr);
        tempCmd->successCond.clear();
    }
    tempCmd->failCond.clear();
    


    return tempCmd;
}


/**
 * Simple drive to fill when control is lost in most other functions */
robotCommand *smrclGenerator::fillDrive(vector<route> rPlan,int pIndex) {

    robotCommand *tempCmd = new robotCommand;

    //Generate command code
    tempCmd->quality = 0.3;
    //Decrease speed in initial drive
    tempCmd->command.assign("drive ");
    tempCmd->failCond.assign("($drivendist > 0.30)");
    tempCmd->successCond.clear();

    return tempCmd;
}

robotCommand *smrclGenerator::stopCmd(vector<route> rPlan, int pIndex) {

    robotCommand *tempCmd = new robotCommand;

    //Generate command code
    tempCmd->quality = 0.1;
    //Decrease speed in initial drive
    tempCmd->command.assign("stop ");
    tempCmd->failCond.clear();
    tempCmd->successCond.clear();

    return tempCmd;
}

