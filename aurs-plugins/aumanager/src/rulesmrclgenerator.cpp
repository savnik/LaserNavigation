/* 
 * File:   rulesmrclgenerator.cpp
 * Author: andersbeck
 * 
 * Created on March 31, 2009, 2:14 PM
 */
#include <unistd.h>
#include "rulesmrclgenerator.h"

ruleSmrclgenerator::ruleSmrclgenerator() {

    generatorCount = 0;
    ruleIterCmds = 0;
    ruleIterNextcmds = 0;
}

ruleSmrclgenerator::ruleSmrclgenerator(const ruleSmrclgenerator& orig) {
}

ruleSmrclgenerator::~ruleSmrclgenerator() {
}

/* Initialization function */
bool ruleSmrclgenerator::initialize(void *rootVarpool) {

    rootPool = (UVarPool *)rootVarpool;
    UVarPool *rulestatePool;

    if (rootPool == NULL) {
        printf("Failed getting root-varpool\n");
        return false;
    }

    //Create global.planner pool
    plannerPool = rootPool->getStructDeep("global.planner.test",true,NULL,0);
    if (plannerPool != NULL) {
        printf("Created Planner Varpool successfully\n");
    } else {
        printf("Failed creating Planner Varpool\n");
        return false;
    }

    running = new UVariable;
    running->setValued(0);
    plannerPool->addVar("running",running,"Is the codegenerator running");

    //Get the rulestate.iterations variable
    rulestatePool = rootPool->getStructDeep("rulestate.iterations",false,NULL,0);
    if (rulestatePool == NULL) {
        printf("Failed getting the ruleState pool, no rule-refresh checks is performed\n");
        ruleIter = NULL;
    } else {
        ruleIter = rulestatePool->getLocalVariable("iterations",0);
        if (ruleIter == NULL) {
            printf("Error getting the iterations variable from rulestate-pool,no rule-refresh checks is performed\n ");
        } else {
            lastRuleIter = (int)ruleIter->getValued();
        }
    }
    return true;

}

bool ruleSmrclgenerator::setExecution(bool run, void *nothing){

    //Update the list of generator rules when execution is started.
    updateRuleList();

    //Toggle some stuff, when running is activated/deactivated
    if (run) {
        if (running != NULL)    running->setValued(1);  
    }else {
        if (running != NULL)    running->setValued(0);
    }

    return true;

}

/** Set a command Id as active running */
bool ruleSmrclgenerator::setActive(int id, bool value) {

    if ((size_t)id-1 >= genRules.size()) return false;
    if (genRules[id-1].active == NULL) return false;

    if (value) {
        printf("Setting %s active\n",genRules[id-1].cmdName.c_str());
        genRules[id-1].active->setValued(1);
    } else {
        genRules[id-1].active->setValued(0);
    }

    return true;
}

bool ruleSmrclgenerator::setFailed(int id, bool value) {

    if ((size_t)id-1 >= genRules.size()) return false;
    if (genRules[id-1].failed == NULL) return false;

    if (value) {
        printf("Setting %s failed\n",genRules[id-1].cmdName.c_str());
        genRules[id-1].failed->setValued(1);
    } else {
        genRules[id-1].failed->setValued(0);
    }

    return true;
}

bool ruleSmrclgenerator::getReload(int id) {

    if (genRules[id-1].reload->getValued() > 0.0) return true;
    else return false;
}

bool ruleSmrclgenerator::getDisable(int id) {
    if (genRules[id-1].disable->getValued() > 0.0) return true;
    else return false;
}

/* Function to process the generated smr-cl code and place it into
 *  the robotCommand vector for the current route element.
 */
bool ruleSmrclgenerator::processCommands(vector<robotCommand> *rCmds, bool waitForNew) {

    generatorCount++; //Increment counter

    //Update rules, if the rule list is empty
    if (!rulesLoaded()) updateRuleList();

    //Test if there is still no rules loaded
    if (!rulesLoaded()) return false;

    //update iteration counter
    ruleIterCmds = (int)ruleIter->getValued();
    
    //Wait until the rule-based planner has run another iteration since last
    //update, if it is required
    if (ruleIter != NULL) { //only if rule-iter has been loaded from rulebased plug-in
      while((waitForNew) && (ruleIterCmds == (int)ruleIter->getValued())) {
          usleep(10000); //Wait 10ms until ruleIter changes
      }
    }

    //Make sure that the size of rCmds is enough
    if (rCmds->size() != genRules.size()) rCmds->resize(genRules.size());

    //Loop through the generator rules
   for (size_t i = 0; i < genRules.size(); i++) {

       //Generate the command-set for the current commands
       if (genRules[i].cmdValid) {
            //Get the quality
            (*rCmds)[i].cmdName = genRules[i].cmdName;
            (*rCmds)[i].quality = genRules[i].quality->getValued(0);
            (*rCmds)[i].id = genRules[i].id;

            //Only extract the remaining stuff, if quality is higher than zero
            if ((*rCmds)[i].quality > 0.0) {
                //Extract the cmd strings
                if (genRules[i].command != NULL)
                     (*rCmds)[i].command = ruleStringLoader(genRules[i].command);
                if (genRules[i].preCommand != NULL)
                     (*rCmds)[i].preCommand = ruleStringLoader(genRules[i].preCommand);
                if (genRules[i].postCommand != NULL)
                     (*rCmds)[i].postCommand = ruleStringLoader(genRules[i].postCommand);
                if (genRules[i].failCond != NULL)
                     (*rCmds)[i].failCond = ruleStringLoader(genRules[i].failCond);
                if (genRules[i].reloadCond != NULL)
                     (*rCmds)[i].reloadCond = ruleStringLoader(genRules[i].reloadCond);
                if (genRules[i].successCond != NULL)
                    (*rCmds)[i].successCond = ruleStringLoader(genRules[i].successCond);
                if (genRules[i].initCommand != NULL)
                     (*rCmds)[i].initCommand = ruleStringLoader(genRules[i].initCommand);
                if (genRules[i].exitCommand != NULL)
                     (*rCmds)[i].exitCommand = ruleStringLoader(genRules[i].exitCommand);
                if (genRules[i].reload != NULL )
                    (*rCmds)[i].reload = (bool)genRules[i].reload->getValued();
                if (genRules[i].disable != NULL )
                    (*rCmds)[i].disable = (bool)genRules[i].disable->getValued();
            }
       }
   }

    return true;
}

/** Function to process the generated smr-cl code and place it into
 *  the robotCommand vector for the next route element.
 */
bool ruleSmrclgenerator::processNextCommands(vector<robotCommand> *rCmds, bool waitForNew) {


    //Test if there is still no rules loaded
    if (!rulesLoaded()) return false;
    //Wait until the rule-based planner has run another iteration since last
    //update, if it is required
    if (ruleIter != NULL) { //only if rule-iter has been loaded from rulebased plug-in
      while((waitForNew) && (ruleIterCmds == (int)ruleIter->getValued())) {
          usleep(10000); //Wait 10ms until ruleIter changes
      }
    }



    //Make sure that the size of rCmds is enough
    if (rCmds->size() != genRules.size()) rCmds->resize(genRules.size());

    //Loop through the generator rules
   for (size_t i = 0; i < genRules.size(); i++) {

       //Generate the commandset for the next commands
       //(a nextcommand can be valid, even the current command is ok)
       if ((genRules[i].next != NULL) && (genRules[i].next->cmdValid)) {

            (*rCmds)[i].cmdName = "next:";
            (*rCmds)[i].cmdName += genRules[i].cmdName;
            (*rCmds)[i].quality = genRules[i].next->quality->getValued(0);
            (*rCmds)[i].id = genRules[i].id;


            //Only extract the remaining stuff, if quality is higher than zero
            if ((*rCmds)[i].quality > 0.0) {
                //Extract the cmd strings
                if (genRules[i].next->command != NULL)
                    (*rCmds)[i].command = ruleStringLoader(genRules[i].next->command);
                if (genRules[i].next->preCommand != NULL)
                    (*rCmds)[i].preCommand = ruleStringLoader(genRules[i].next->preCommand);
                if (genRules[i].next->postCommand != NULL)
                    (*rCmds)[i].postCommand = ruleStringLoader(genRules[i].next->postCommand);
                if (genRules[i].next->failCond != NULL)
                    (*rCmds)[i].failCond = ruleStringLoader(genRules[i].next->failCond);
                if (genRules[i].next->reloadCond != NULL)
                    (*rCmds)[i].reloadCond = ruleStringLoader(genRules[i].next->reloadCond);
                if (genRules[i].next->successCond != NULL)
                    (*rCmds)[i].successCond = ruleStringLoader(genRules[i].next->successCond);
                if (genRules[i].next->initCommand != NULL)
                    (*rCmds)[i].initCommand = ruleStringLoader(genRules[i].next->initCommand);
                if (genRules[i].next->exitCommand != NULL)
                    (*rCmds)[i].exitCommand = ruleStringLoader(genRules[i].next->exitCommand);
                if (genRules[i].next->reload != NULL )
                    (*rCmds)[i].reload = (bool)genRules[i].reload->getValued();
                if (genRules[i].next->disable != NULL )
                    (*rCmds)[i].disable = (bool)genRules[i].next->disable->getValued();
            }
       }
   }
    return true;
}


/** Load og refresh the list of rule based smr-cl generator modules */
int ruleSmrclgenerator::updateRuleList(void) {

    UVarPool            *nextPool;
    UVarPool            *codebasePool;
    robotCommandUVar    *tmpCmd;
    bool                ruleFound;

    //Error check
    if (plannerPool == NULL) return -1;

    //Loop through all elements of the planner pool (global.planner)
    for (int i = 0; i < plannerPool->getStructCnt(); i++) {
        codebasePool = plannerPool->getStruct(i);

        //Extract the rule UVar elements
        tmpCmd = loadGeneratorRule(codebasePool);

        //Then extract the rule next variables
        nextPool = codebasePool->getStructDeep("next.test",false,NULL,0);
        if (nextPool != NULL) {
            tmpCmd->next = loadGeneratorRule(nextPool);
        } else tmpCmd->next = NULL;

        //Test if the rule is already created
        ruleFound = false;
        for (size_t j = 0; j < genRules.size(); j++) {
            if (!genRules[j].cmdName.compare(tmpCmd->cmdName)) {
                tmpCmd->id = genRules[j].id;
                genRules[j] = (*tmpCmd);
                ruleFound = true;
                break;
            }
        }
        //If the rule was not found, add a new one to the vector
        if (ruleFound == false) {
            tmpCmd->id = genRules.size()+1;
            genRules.push_back(*tmpCmd);
        }
    }
    //Cleanup (yes, i know there is still the tmpCmd.next alive :-(  )
    //delete tmpCmd;
    return genRules.size();
}

//Is any rules loaded
size_t ruleSmrclgenerator::rulesLoaded(void) {

    return genRules.size();
}



robotCommandUVar* ruleSmrclgenerator::loadGeneratorRule(UVarPool *rulePool) {

    robotCommandUVar *rCmd = new robotCommandUVar;
    UVariable *tmpUVar;

    //Extract the name
    rCmd->cmdName = rulePool->getPreName();

    //Extract the UVariables
    tmpUVar = rulePool->getLocalVariable("command",0);
    if (tmpUVar != NULL) rCmd->command = tmpUVar;
    else                 rCmd->command = NULL;
    tmpUVar = rulePool->getLocalVariable("quality",0);
    if (tmpUVar != NULL) rCmd->quality = tmpUVar;
    else                 rCmd->quality = NULL;
    tmpUVar = rulePool->getLocalVariable("precommand",0);
    if (tmpUVar != NULL) rCmd->preCommand = tmpUVar;
    else                 rCmd->preCommand = NULL;
    tmpUVar = rulePool->getLocalVariable("postcommand",0);
    if (tmpUVar != NULL) rCmd->postCommand = tmpUVar;
    else                 rCmd->postCommand = NULL;
    tmpUVar = rulePool->getLocalVariable("failcond",0);
    if (tmpUVar != NULL) rCmd->failCond = tmpUVar;
    else                 rCmd->failCond = NULL;
    tmpUVar = rulePool->getLocalVariable("reloadcond",0);
    if (tmpUVar != NULL) rCmd->reloadCond = tmpUVar;
    else                 rCmd->reloadCond = NULL;
    tmpUVar = rulePool->getLocalVariable("successcond",0);
    if (tmpUVar != NULL) rCmd->successCond = tmpUVar;
    else                 rCmd->successCond = NULL;
    tmpUVar = rulePool->getLocalVariable("initcommand",0);
    if (tmpUVar != NULL) rCmd->initCommand = tmpUVar;
    else                 rCmd->initCommand = NULL;
    tmpUVar = rulePool->getLocalVariable("exitcommand",0);
    if (tmpUVar != NULL) rCmd->exitCommand = tmpUVar;
    else                 rCmd->exitCommand = NULL;

    //Mark the command valid if quality and command is present
    if (rCmd->quality != NULL) {
        rCmd->cmdValid = true;
        //Create the status variables, if they aren't present
        tmpUVar = rulePool->getLocalVariable("failed",0);
        if (tmpUVar == NULL) {
            rCmd->failed  = rulePool->addVar("failed",0.0,"d","Has the command failed execution");
        } else rCmd->failed = tmpUVar;

        tmpUVar = rulePool->getLocalVariable("active",0);
        if (tmpUVar == NULL) {
            rCmd->active  = rulePool->addVar("active",0.0,"d","Is the command active in MRC");
        } else rCmd->active = tmpUVar;

        tmpUVar = rulePool->getLocalVariable("disable",0);
        if (tmpUVar == NULL){
            rCmd->disable = rulePool->addVar("disable",0.0,"d","Set to disable the command in MRC");
        } else rCmd->disable = tmpUVar;

        tmpUVar = rulePool->getLocalVariable("reload",0);
        if (tmpUVar == NULL) {
            rCmd->reload  = rulePool->addVar("reload",0.0,"d","Set to reload the command in MRC");
        } else rCmd->reload = tmpUVar;

    }

    return rCmd;

}

/** String loader from rule based
 *
 * This loader implements escape sequences for strings loaded from the rule
 * based code generator
 **/
string ruleSmrclgenerator::ruleStringLoader(UVariable *ruleVar) {

    string ruleStr;
    char tmpStr[2048];
    char *inStr;
    int n;

    //Error check
    if (ruleVar == NULL) return ruleStr;

    inStr  = (char*)ruleVar->getValues(0);

    //Loop through the string and search for backslash
    for (n = 0; (*inStr) != 0; n++, inStr++) {
        if (*inStr == '\\') { //Backslash detected
            if (*(inStr+1) == 'n') tmpStr[n] = '\n';
            inStr++;
            //more escape here if needed
        } else {
            tmpStr[n] = *inStr;
        }
    }
    tmpStr[n] = 0; //Terminate the string

    //Clear the string, if nothing has been written (otherwise transfer to string obj)
    if (n == 0)  ruleStr.clear();
    else    ruleStr = tmpStr;

    return ruleStr;

}




