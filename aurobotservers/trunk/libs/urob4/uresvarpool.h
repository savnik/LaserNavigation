/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@oersted.dtu.dk
 *                                                                         *
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
#ifndef URES_VARPOOL_H
#define URES_VARPOOL_H

#include <ugen4/utime.h>
#include <ugen4/conf.h>
#include <ugen4/udatabase.h>
#include <ugen4/uposrot.h>
#include <umap4/upose.h>

#include "uresbase.h"
#include "uvarmethod.h"
#include "ulogfile.h"
#include "uvarpool.h"

class UVarCalc;

/**
Shared ressource that adds a variable pool instance

	@author Christian <chrand@mail.dk>
*/
class UResVarPool : public UResBase, public UVarMethodImplement
{
public:
  /**
  Constructor */
  UResVarPool()
  { // set name and version number
    const int MSL = 100;
    char s[MSL];
    //
    setResID(getResClassID(), 1035);
    varPool = NULL;
    //logVar = NULL;
    snprintf(s, MSL, "%s.var", appName);
    logVar.setLogName(s);
    snprintf(note, MAX_NOTE_SIZE, "Variables and methods for %s", getResID());
    createVarSpace(0, 0, 0, note, false);
    //createBaseVar();
  };
  /**
  Destructor */
  virtual ~UResVarPool();
  /**
  Resource ID for this class */
  static const char * getResClassID()
  { return "varPool"; };
  /**
  Print status for this resource */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  * Create space in var pool.
  * This instance creates a variable pool with this initial pool size,
    it is expanded if more variables, structs or methods are added.
  * A not may be added to the created structure as a description of this var pool.
  * Returns true if space is created. */
  virtual bool createVarSpace(const int varCnt, const int structCnt, const int methCnt, const char * note, bool copy = false);
  /**
   * Add a symbol to to the symbol stack of type double.
   * The comment is the online help for this variable.
   * \param name is the name of the new variable (less than 30 chars)
   * \param initialValue is the start value of the variable (one element)
   * \param type is variable type, see UVariable::varType (d, s, d3d, d2d, d6d, dq, t, pose, rot, m2)
   * \param comment a description of the variable, for on-line help
   * \Returns the new variable (may be an existing with same name, or NULL if failed) */
  UVariable * addVar(const char * name, const double initialValue,
             const char * type,
             const char * comment);
  /**
   * Add an array symbol to to the symbol stack with an initial value from space separated value list.
   * the value vector may be a single value, a vector of values or a matrix.
   * The comment is the online help for this variable.
   * \param name is the name of the new variable (less than 30 chars)
   * \param initialValue is the start value as string e.g. "0.22 0.44 1.2" array of 3 elements or "1 2; 3 4" a 2x2 matrix
   * \param type is variable type, see UVariable::varType (d, s, d3d, d2d, d6d, dq, t, pose, rot, m2)
   * \param comment a description of the variable, for on-line help
   * \Returns the new variable (may be an existing with same name, or NULL if failed) */
  UVariable * addVar(const char * name,
                     const char * initialValue,
                     const char * type,
                     const char * comment);
  /**
   * Add an array symbol to to the symbol stack with an initial value from space separated value list.
   * The comment is the online help for this variable.
   * The same function as addVar(...) but to make naming the same as when adding a structo to UVarPool.
   * \param name is the name of the new variable (less than 30 chars)
   * \param initialValue is the start value as string e.g. "0.22 0.44 1.2" (array of 3 elements)
   * \param type is variable type, see UVariable::varType (d, s, d3d, d2d, d6d, dq, t, pose, rot, m2)
   * \param comment a description of the variable, for on-line help
   * \Returns the new variable (may be an existing with same name, or NULL if failed) */
  inline UVariable * addVarA(const char * name,
                     const char * initialValue,
                     const char * type,
                     const char * comment)
  { return addVar(name, initialValue, type, comment); };
  /**
   * Add a structure to the variable pool of this resource. The structure is a var-pool in its own right, and may
   * hold variables, structures and methods.
   * The comment is the online help for this variable.
   * \param name is the name of the new variable (less than 30 chars, start with an alfabet char (7-bit) followed by alfanumeric or underscore)
   * \param comment a description of the variable, for on-line help
   * \param copy should comment be copied, or is it a static string, that is available after return of function.
   * \Returns pointer to new structure (may be an existing with same name, or NULL if failed) */
  UVarPool * addStruct(const char * name,
                     const char * comment, bool copy = false);
  /**
   * Create the basic variables needed by the var pool */
  void createBaseVar();
  /**
   * Add a method to this local var pool.
   * \param formalName is the name of the method (max 32 characters, starting with an alpha ...)
   * \param paramTypes string with one char for each parameter d=double, s=string c=class object...)
   * \param comment is the help text displayed when the method is listed (on line help)
   * \returns -1 if the method could not be added (or exist already), else the index number of the added method. */
  int addMethod( const char * formalName,
                 const char * paramTypes,
                 const char * comment);
  /**
   * Add a method to this local var pool (implemented using UVariable parameters).
   * \param formalName is the name of the method (max 32 characters, starting with an alpha ...)
   * \param paramTypes string with one char for each parameter d=double, s=string c=class object...)
   * \param comment is the help text displayed when the method is listed (on line help)
   * \returns -1 if the method could not be added (or exist already), else the index number of the added method. */
  int addMethodV( const char * formalName,
                 const char * paramTypes,
                 const char * comment);
  /**
  Is this resource missing any base ressources */
  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Set ressource as needed (probably not used by this resource) */
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  Get the resource var pool */
  inline UVarCalc * getVarPool()
  { return varPool; };
  /**
  * Set an external var pool.
  * This is not the usual way to get a var-pool, but can be used to
    share a var pool with another resource (as the sequencer do). */
  inline void setVarPool(UVarCalc * sharedVarPool)
  { varPool = sharedVarPool; };
  /**
   * Get a (double) value relative to the global variable tree pool
   * \param naem name of variable, may contain . and index, like in 'svs.pos[2]'
  */
  bool getGlobalValue(const char * name, double * value);
  /**
   * Get global variable (string) value.
   * \param name is the global name of the variable, this may have an index, i.e. core.servername[2], the 
   * returned string pointer then points to the third character in the servername.
   * \param value should be the adress of a const char pointer.
   * \returns true if the variable exist and is a string variable. */
  bool getGlobalString(const char * name, const char ** value);
  inline bool getGlobalValue(const char * name, const char ** value)
  { return getGlobalString(name, value); };
  /**
   * \brief get a value in this variable tree
   * \param name name of variable that may include a structure, e.g. aaa.b
   * \param value pointer to a double value, where the value of the specified variable is returned
   * if the variable do not exist, then nothing is changed.
   * \return false if the variable do not exist. */
    bool getLocalValue(const char * name, double * value);
  /**
  Get a (bool) value relative to this variable tree */
  bool getLocalValue(const char * name, bool * value);
  /**
  Get a time value with this name relative to this variable tree */
  bool getLocalValue(const char * name, UTime * value);
  /**
  Get a (bool) value relative to the global variable tree */
  bool getGlobalValue(const char * name, bool * value);
  /**
  Get a time value with this name relative to the global variable tree */
  bool getGlobalValue(const char * name, UTime * value);
  /**
  Get the boolean value of this variable (default if no var-pool structure is available is fale. */
  bool getLocalValueBool(int idx);
  /**
  Get local value as time */
  UTime getLocalValueTime(int idx);
  /**
   * Get 3D position from 3 variables, starting with this index.
   * \param idx is the index of the first variable, assuming the order x,y,z
   * \returns result in a UPosition structure */
  UPosition getLocalValue3D(int idx);
  /**
   * Set 3D position to this variables.
   * \param idx is the index of the variable, index order x,y,z
   * \param value a position with x,y,z
   * \returns true if set */
  bool setLocalVar3D(int idx, UPosition * value);
  /**
   * Set pose in this variable.
   * \param idx is the index of the variable, index order x,y,h
   * \param value a position with x,y,h
   * \returns true if set */
  bool setLocalVarPose(int idx, UPose * value)
  {
    UPosition pos(value->x, value->y, value->h);
    return setLocalVar3D(idx, &pos);
  };
  /**
  Get 6D position from 6 variables, starting with this index.
   * \param idx is the index of the first of 6 variables, assuming the order x,y,z,Omage,Phi,Kappa.
   * \returns values in a UPosRot structure */
  UPosRot getLocalValue6D(int idx);
  /**
  Set 6D position to 6 consecutive variables starting with this index.
   * \param idx is the index of the first of 6 variables, assuming the order x,y,z,Omage,Phi,Kappa.
   * \param value the 6D value (x,y,z,o,p,k)
   * \returns true if there is 6 values with an index above idx (no name check) */
  bool setLocalVar6D(int idx, UPosRot * value);
  /**
  Get integer value of local variable with this index */
  inline int getLocalValueInt(int idx)
  { return roundi(getLocalValue(idx)); };
  /**
  Get the value of this variable (default if no var-pool structure is available is 0.0. */
  double getLocalValue(int idx);
  /**
   * Set the value of this variable from index
   * \param idx is the index to the variable in this var pool
   * \param value is double value to be assigned
   * \param element is the array index if variable is an array
  */
  bool setLocalVar(int idx, double value, const int element);
  /**
   * \brief Assigns a value to a variable.
   * The variable may be a part of a structure.
   * \param name is the name of the variable and may be a substructure, e.g. aa.bb.cc.d
   * \param value is the double value that should be assigned to the variable.
   * \param mayAdd If 'mayAdd' is true, then the variable is added if not found.
   * And if the structure (any of the sspecified structures) is missing, then this is added too.
   * \returns true if modified or added. */
  bool setLocalVar(const char * name, const double value, bool mayAdd);
  /**
   * \brief Assigns a value to a variable.
   * The variable may be a part of a structure.
   * \param name is the name of the variable and may be a substructure, e.g. aa.bb.cc.d
   * \param value is the double value that should be assigned to the variable.
   * \param mayAdd If 'mayAdd' is true, then the variable is added if not found.
   * And if the structure (any of the sspecified structures) is missing, then this is added too.
   * \returns true if modified or added. */
  bool setGlobalVar(const char * name, const double value, bool mayAdd);
  /**
   * Add the value of this variable from index
   * \param idx is the index to the variable in this var pool
   * \param value is double value to be added
   * \param element is the array index if variable is an array  */
  bool setLocalVarAdd(int idx, double value, const int element);
  /**
  * Function to implement a var-pool method call.
  * The function with 'name' and pameters in the order 'paramOrder'
    is to be called. The actual parameters are in 'strings' and 'doubles'.
  * A double sized result may be returned in 'value'.
  * May return struct values in the 'returnStruct' pointer list (when the
    pointer list is not NULL), and in this case the 'returnStructCnt' is set
    to the maximum number of pointers available and should be modified
    to the number of valid pointers returned.
  * Should return true if function was handled
    (otherwise an invalid function specification is assumed). */
  virtual bool methodCall(const char * name, const char * paramOrder,
                          char ** strings, const double * doubles,
                         double * value,
                         UDataBase ** returnStruct = NULL,
                         int * returnStructCnt = NULL);
  /**
  Make a call to a method in this or any sub-structure var-pool */
  bool callLocal(const char * name, const char * paramOrder,
                        char ** strings, const double * doubles,
                        double * value,
                        UDataBase ** returnStruct,
                        int * returnStructCnt);
  /**
  Make a call to a method in the global var-pool tree */
  bool callGlobal(const char * name, const char * paramOrder,
            char ** strings, const double * doubles,
            double * value,
            UDataBase ** returnStruct,
            int * returnStructCnt);
  /**
   *\brief Call a method from the root var pool scope
   * It may be in the structure direct (no '.') or in any sub-structure that the 'name.subname' indicate.
   * \param name is the name of the method, it may adress a sub structure, e.g. aa.bbbb
   * \param paramOrder the specific parameter order as a string, e.g "scd" for three parameters the
   * first 's' for string the second is a 'c' for class (eg a pose), and the last 'd's for double.
   * \param params is an array of UVariable pointers. with the actual parameters
   * \param returnStruct is an array of pointers to a datatype compatible with the UDataBase base type.
   * This array pointer may be NULL.
   * \param returnStructCnt is a pointer to the number of entries in the returnStruct array.
   * When the function returns this number will be modified to the actual number of values returned.
   * \return true if the method exist. */
  bool callGlobalV(const char * name, const char * paramOrder,
                   UVariable ** params,
                   UDataBase ** returnStruct,
                   int * returnStructCnt);
  /**
   * Call other plugin with a single string parameter.
   * \param function is full name of function to call.
   * \param stringParam is the single string parameter used in the call 
   * \returns true if function exist (plugin is loaded) */
  bool callVS(const char * function, const char * stringParam);
  /**
   * Call other plugin with a string, data-struct and integer parameter.
   * \param function is full name of function to call.
   * \param strPar is the single string parameter used in the call 
   * \param data   is the data needed by the call. 
   * \param coosys is an integer as the last parameter (often coordinate system of the data 0=odo,1=utm,2=map) 
   * \returns -1 if function does not exist (plugin not loaded), else the return value of the function (integer) */
  int callVSCD(const char * function, const char * strPar, UDataBase * data, int cooSys);
  /**
  Get description string for the variable with this index */
  const char * getVarDescription(int idx);
  /**
  Open logfile with default name (appName).var.log in the dataPath */
  bool logFileOpen();
  /**
  Close logfile for variable modifications */
  void logFileClose();
  /**
  Set logging of this structure. The structure may be empty to set
  logging for this variable pool. If 'open' is false, then logging is stopped. */
  void logFileStart(const char * structName, bool open);
  /**
  Get logfile name */
  inline const char * getLogFileName()
  { return logVar.getLogFileName(); };
  /**
  Is logfile open - the file itself, not for one specific structure */
  inline bool isLogFileOpen()
  { return logVar.isOpen(); };
  /**
  Is logfile open for this specific structure specific structure (empty string meens root). */
  bool isLogFileOpen(const char * structName);
    /**
   * test if the any variable in the local var pool (not sub-structures) is updated since last check
   * \param lastCnt is the last saved count.
   * \param newCnt is where the new count is saved (if not NULL)
     * returns true if the update count (updateCnt) is different from lastCnt */
  bool isVarPoolUpdated(int lastCnt, int * newCnt);


protected:
  /**
  Set the description note
  \param copy if note is not static, then copy should be set to true. */
  void setDescription(const char * note, bool copy = false);

private:
  /**
  The structure that actually holds the variables and possibly sub-structures and methosds */
  UVarCalc * varPool;
  /**
  Logfile for variable updates */
  ULogFile logVar;
  /**
  Filename for logfile */
  //char logVarName[MAX_FILENAME_LENGTH];
  /**
  Resource lock for variable logfile */
  ULock logVarLock;
  /**
  Size of not string */
  static const int MAX_NOTE_SIZE = 100;
  /**
  Note string */
  char note[MAX_NOTE_SIZE];
};

#endif
