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
#ifndef UVARPOOL_H
#define UVARPOOL_H

#include <ugen4/ulock.h>
#include <ugen4/uposrot.h>
#include <umap4/upose.h>
#include "uvarmethod.h"
#include "userverpush.h"
#include "uvariable.h"
#include "ulogfile.h"

/**
  This structure holds access to variables and functions.
  Variables can be either in this class or in a list of pointers to other instances of this class.
  In the end the value should be evaluated to a double.
* The lock is to protect pointers and values during access an var-space search.
* The Push class is to enable command execution on variable update.
* @author Christian <jca@oersted.dtu.dk> */

class UVarPool : public ULock, public UServerPush
{
public:
  /**
  Constructor */
  UVarPool();
  /**
  Destructor */
  virtual ~UVarPool();
  /**
  * Add a symbol to to the symbol stack.
  * The comment is intended as an online help for this variable.
   * \param name is the new name to be added
   * \param initialValue is the value to be assigned to the variable
   * \param type is a cahracter version of the data type
   * \param comment is a pointer to a static instance of a string describing the variable
   * \Returns the new variable */
  UVariable * addVar(const char * name, const double initialValue,
             const char *  type,
             const char * comment);
  /**
   * Add an array (double array or string) symbol to the symbol stack.
   * The comment is the online help for this variable.
   * \param name is the new name to be added
   * \param attValue is a string representation of the value(s)
   * \param type is a character version of the data type
   * \param comment is a pointer to a static instance of a string describing the variable
   * \Returns the new variable (may be an existing with the same name).
   * \returns NULL if failed */
   UVariable * addVarA(const char * name, const char * attValue,
             const char *  type,
             const char * comment);
/*   inline UVariable * addVar(const char * name, const char * attValue,
             const char *  type,
             const char * comment)
   { return addVarA(name, attValue, type, comment); };*/
  /**
   * Add a symbol to to the symbol stack (if not there already).
   * The comment is intended as an online help for this variable.
   * \param name is the new name to be added
   * \param initialValue is the (initial) value to be assigned to the variable
   * \param type is the data type - enumerated
   * \param comment is a pointer to a static instance of a string describing the variable
   * \param attValue is a string representation of the value(s), if NULL, then value is in initialValue
   * \Returns the new variable (may be an existing with the same name)
   * \returns NULL if failed */
  UVariable * addVar(const char * name, const double initialValue,
             UVariable::varType type,
             const char * comment, const char * attValue = NULL);
  /**
   * Add a symbol to to the symbol stack (if not there already).
   * The comment is the online help for this variable.
   * \param name is the new name to be added
   * \param source is the variable to be used as source
   * \param comment is a pointer to a static instance of a string describing the variable
   * \Returns the new variable (or an existing with the same name).
   * \returns NULL if failed */
  UVariable * addVar(const char * name,
              const UVariable * source,
              const char * comment);
  /**
   * Add a symbol to to the symbol stack (if not there already).
   * No value nor variable type is set.
   * The comment is the online help for this variable.
   * \param name is the new name to be added, if name has a '.' then a structure is created
   * \param comment is a pointer to a static instance of a string describing the variable
   * \Returns the variable with name and comment set, returns NULL is not created */
  UVariable * addVar(const char * name,
                     const char * comment);
  /**
  Add a set of variables as a new structure with this pre-name */
  int addStruct(const char * name, UVarPool * varStruct);
  /**
   * \brief Add a link to another place in the var tree
   * but do not change anything in the linked structure
   * \param name new name to call the structure
   * \param toVarStruct pointer to the existing var structure
   * \returns the index to the link */
  int addStructLink(const char * name, UVarPool * toVarStruct);
  /**
  Delete a structure from the list of available varPools. */
  void deleteStruct(const char * name);
  /**
   * Delete the method with this name and parameter list */
  void deleteMethod(const char * name, const char * paramStr);
  /**
   * Delete all methods in this structure
   * will be called if a resource that owns the variable structure is unloaded */
  void deleteMethods(UVarMethodImplement * owner);
  /**
   * Delete all variables, sub-structures and methods in this var pool */
  void deleteAll();
  /**
  Set the pre-name that (often) is placed in front of variables
  to indicate that the value belongs to this structure. */
  bool setPreName(const char * name);
  /**
  Return the prename for this group of variables */
  inline const char * getPreName()
  { return preName;};
  /**
  Return the prename for this group of variables down to the root */
  const char * getFullPreName(char * name, const int MNL);
  /**
  Return already compiled full prename */
  const char * getFullPreName()
  { return preNameFull; };
  /**
   * Assigns a value to a variable
   * \param index is the index of the variable in the local var-pool
   * \param value is the value to be assigned
   * \param element is the array index of the variable
   * \returns true if set.  */
  bool setLocalVar(const int index, const double value, const int element);
  /**
   * Assigns a value to a variable
   * \param index is the index of the variable in the local var-pool
   * \param value is the variable with the value to be assigned
   * \returns true if set.  */
  bool setLocalVar(const int index, UVariable * value);
  /**
  Add a value to a local variable */
  bool setLocalVarAdd(const int index, const double value, const int element);
  /**
   * \brief Assigns a value to a variable.
   * The variable may be a part of a structure.
   * \param name is the name of the variable and may be a substructure, e.g. aa.bb.cc.d
   * \param value is the double value that should be assigned to the variable (when attValue==NULL).
   * \param mayAdd If 'mayAdd' is true, then the variable is added if not found.
   * And if the structure (any of the sspecified structures) is missing, then this is added too.
   * \param varTyp the variable type.
   * \param attValue alternative source of values - set to NULL if no values
   * \returns true if modified or added. */
  bool setLocalVar(const char * name,
                   const double value,
                   bool mayAdd,
                   UVariable::varType vartyp, // = UVariable::d,
                   const char * attValue = NULL);
  /**
   * \brief Assigns a value to a variable.
   * The variable may be a part of a structure.
   * \param name is the name of the variable and may be a substructure and
   * optionally have an index, e.g. aa.bb.cc.d[3]
   * \param value is a string with values to be added, starting with index indicated by name.
   * \param mayAdd If 'mayAdd' is true, then the variable is added if not found.
   * And if the structure (any of the sspecified structures) is missing, then this is added too.
   * \param varTyp the variable type described as a string.
   * \param updTime is an optional update time (mostly relevant for time series variables)
   * \returns true is added  */
  bool setLocalVar(const char * name,
                   const char * value,
                   bool mayAdd,
                   const char * vartyp,
                   UTime * updTime = NULL);
  /**
   * \brief Assigns a value to a variable.
   * The variable may be a part of a structure.
   * \param name is the name of the variable and may be a substructure, e.g. aa.bb.cc.d
   * \param value is the double value that should be assigned to the variable.
   * \param mayAdd If 'mayAdd' is true, then the variable is added if not found.
   * And if the structure (any of the sspecified structures) is missing, then this is added too.
   * \returns true if modified or added. */
  inline bool setGlobalVar(const char * name, const double value, bool mayAdd)
  {
    return getRootVarPool()->setLocalVar(name, value, mayAdd, UVariable::d);
  };
  /**
   * Get index for variable of this name
   * \param name name of variable in this var-pool, may have array index
   * \param element if not NULL, then attary element index is returend here (0 is returned, if no index is specified).
   * \returns integer index if found, or -1 if not found in this var-pool.  */
  int getLocalVarIndex(const char * name, int * element);
  /**
  Get pointer to variable of this name.
  * returns NULL if not found. */
  //UVariable *  getVar(const char * name);
  /**
  * Get pointer to varPool structure of this name.
  * returns NULL if not found. */
  UVarPool *  getStruct(const char * name);
  /**
   * Get pointer to varPool structure with this index.
   * returns NULL if not found. */
  UVarPool *  getStruct(const int idx);
  /**
  * Get index for a structure for faster access and enumeration.
  * Returns -1 if no such variable. */
  int  getStructIdx(const char * name);
  /**
  * Get pointer to method in this varPool under this name.
  * returns NULL if not found. */
  UVarMethod *  getLocalMethod(const char * name, const char * paramList);
  /**
  * Get pointer to a method of this name, the method
    may be in a substructure.
  * If a '.' notation is used, the relevant structure is found first.
  * returns NULL if not found. */
  UVarPool *  getLocalMethod(const char * name,
                          const char * paramList,
                         UVarMethod ** method);
  /**
  Get pointer to method with this index.
  Returns NULL if index is out of range. */
  UVarMethod * getLocalMethod(const int index);
  /**
   * \brief Get index of this method in this var pool
   * \param name is then name (no structs allowed) of the method
   * \param paramList string with type specification of parameters d=double, s=string, c=class
   * \returns -1 if not found, else the index [0..] to method */
  int  getLocalMethodIdx(const char * name, const char * paramList);
  /**
   * Get value for this symbol.
   * \param idx is the index to the variable in this var-pool
   * \param element is the array index, if this is an array, default is first element
   * \returns value if the element as double
  */
  double getLocalValue(int idx, int element = 0);
  /**
  Get a boolean value */
  bool getLocalValueBool(int idx);
  /**
  Get local value and return as integer. */
  int getLocalValueInt(int idx);
  /**
  Get the value as time */
  UTime getLocalValueTime(int idx);
  /**
   * Get 3D position from this variables, assuming variable is an array with at least 3 elements
   * \param idx is the index of the first variable, assuming the order x,y,z
   * \returns result in a UPosition structure */
  UPosition getLocalValue3D(int idx);
  /**
   * Set 3D position to 3 to this variable as an array with 3 elements
   * \param idx is the index of the variable, assuming the order x,y,z
   * \param value a position with x,y,z
   * \returns true if space can be allocated */
  bool setLocalVar3D(int idx, UPosition * value);
  /**
   * Set 3D rotation to this variable as an array with 3 elements.
   * \param idx is the index of the variable, assuming the order x,y,z
   * \param value a rotation with rotation of axis x,y,z as Omega, Phi, Kappa
   * \returns true if space can be allocated */
  bool setLocalVarRot(int idx, URotation * value);
  /**
   * Get 3D rotation from this variables, assuming variable is an array with at least 3 elements.
   * \param idx is the index of the first variable, assuming the order x,y,z
   * \returns result in a UPosition structure */
  URotation getLocalValueRot(int idx);
  /**
   * Set 3D position to this variable as an array with 3 elements.
   * \param idx is the index of the first variable, assuming the order x,y,h
   * \param value a pose with x,y,h
   * \returns true if space can be allocated */
  bool setLocalVarPose(int idx, UPose * value)
  {
    UPosition pos(value->x, value->y, value->h);
    return setLocalVar3D(idx, &pos);
  };
  /**
   * Get 3D position to this variable as an array with 3 elements.
   * \param idx is the index of the first variable, assuming the order x,y,h
   * \param value a pose with x,y,h
   * \returns true if space can be allocated */
  UPose getLocalValuePose(int idx);
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
  Get value for the variable with this name
  Returns true if variable is found, and in this case the value in 'value' (if not NULL) */
  bool getGlobalValue(const char * name, double * value);
  /**
   * Get global variable (string) value.
   * \param name is the global name of the variable, this may have an index, i.e. core.servername[2], the 
   * returned string pointer then points to the third character in the servername.
   * \param value should be the adress of a const char pointer.
   * \returns true if the variable exist and is a string variable. */
  bool getGlobalString(const char * name, const char ** value);
  /**
   * Get value when the specified name is relative to the local
   * variable pool.
   * \param name is the variable name, e.g. aaa.bbb.c
   * \param value is where the value is retuirned. 'value' may be NULL.
   * if variable do not exist, then nothing is changed.
   * \returns true if the value exist.  */
  bool getLocalValue(const char * name, double * value);
  /**
  Get a boolean value for the variable with this name in the global variable tree
  Returns true if variable is found, and in this case the value in 'value' */
  bool getGlobalValue(const char * name, bool * value);
  /**
  Get a time value for the variable with this name in the global variable tree
  Returns true if variable is found, and in this case returns time in 'value' */
  bool getGlobalValue(const char * name, UTime * value);
  /**
  Get a boolean value for the variable with this name in the local (this) variable tree
  Returns true if variable is found, and in this case the value in 'value' */
  bool getLocalValue(const char * name, bool * value);
  /**
  Get a time value for the variable with this name in the local (this) variable tree
  Returns true if variable is found, and in this case returns time in 'value' */
  bool getLocalValue(const char * name, UTime * value);
  /**
  *  Get value for the variable or function call with this name
  *  Returns true if found, and value in 'value' (if not NULL).
  *  If not a variable, then a function return is tested, and
  the function parameters start at *params, this variable should
  be returned at end of parameters. */
  //bool getValue(const char * name, double * value, const char ** params, bool syntaxCheck);
  /**
  Get name of variable with this name */
  char * getVarName(int index);
  /**
  Get pointer to a specific local variable */
  inline UVariable * getLocalVar(int index)
  { return vars[index]; };
  /**
  Get pointer to first value in variable array. 'getVarsCnt()' returns size of array (the used part). */
  inline UVariable ** getVars()
  { return vars; };
  /**
  Get count of all variables and constants directly available, that is excluding structures.*/
  inline int getVarsCnt()
  { return varsCnt; };
  /**
  Get count of structures (pointers to other UVarPool objects) */
  inline int getStructCnt()
  { return varPoolsCnt; };
  /**
  Get count of available functions*/
  inline int getMethodCnt()
  { return methsCnt; };
  /**
  Set verbose flag */
  inline void setVerbose(bool value)
  { verbose = value;};
  /**
  Set verbose flag */
  inline bool isVarPoolVerbose()
  { return verbose;};
  /**
  Get maximum number of variables and constants - used or unused.*/
  inline int getVarMax()
  { return varsMaxCnt; };
  /**
  Get maximum number of structures (pointers to other UVarPool objects) - empty or used. */
  inline int getStructMax()
  { return varPoolsMaxCnt; };
  /**
  Get maximum number of functions - empty or used. */
  inline int getMethodMax()
  { return methsMaxCnt; };
  /**
  Create variable space for simple variables, structures and methods.
  A descriptive note may be added.
  \param varCnt is number of variables there should be allocated space fore (initially)
  \param structCnt is number of structures there should be allocated space fore initially.
  \param methCnt is number of methods there should be allocated space fore initially.
  \param note is a textual description of this structure.
  \param copy make copy of description, this should be set to true, if note is a local variable in the calling function.
  A local copy of the note string is then allocated. If note is a static string, then copy should be false.
  \Returns true if reqired space were created, otherwise false. */
  bool createVarSpace(const int varCnt, const int structCnt, const int methCnt, const char * note, bool copy);
  /**
  * Get a pointer to the structure thet (possibly) holds this variable or function.
    The full name may hold iteration through several structures, and the function will return the deepest level.
  * I.e.    road.leftSide.polyline should return a pointer to the
    'leftSide' structure and the name 'polyline' in the varName string buffer.
  * Returns a pointer to the relevant structure, and the remaining name in varName.
  * Returns NULL if the structure is not found. */
  UVarPool *  getStructDeep(const char * fullName, char * varName, const int varNameCnt);
  /**
  * Get a pointer to the structure that (possibly) holds this variable or function.
  * The full name may hold iteration through several structures, and the function will return the deepest level.
  * I.e.    road.leftSide.polyline should return a pointer to the
    'leftSide' structure and the name 'polyline' in the varName string buffer.
  * Returns a pointer to the relevant structure, and the remaining name in varName.
  * Returns NULL if the structure is not found and mayAdd==false.
  * Creates a new (empty) structure if mayAdd==true and the structure is not found. */
  UVarPool *  getStructDeep(const char * fullName,
                                      const bool mayAdd,
                                      char * varName, const int varNameCnt);
  /**
  * Add a new local (empty) structure.
  * A description may be added to this variable structure (including methods)
  \param name is the new name of the struct.
  \param note is a description of the structure
  \param copy - set to true if description is not available as static value - then pace for a copy of the string is allocated on heap.
  \returns a pointer to the new structure or NULL if no space */
  UVarPool * addStructLocal(const char * name, const char * note, bool copy);
//                            int vCnt, int sCnt, int mCnt);
  /**
  Add a method to this local var pool, and implemented by this implementor.
   * the function uses double array and string array to hold parameters (deprecated)
   * \param implementor is a pointer to a class that has a UVarMethodImplement ansestor
   * \param name of method to add - may be in a sub-structure using a '.' notation
   * \param paramTypes is a string with the parameter types (string (s) or double (d).
   * \param comment is intended as an online help description
   * \returns an index to the method */
  int addMethod(UVarMethodImplement * implementor,
                const char * formalName,
                const char * paramTypes,
                const char * comment);
  /**
   * Add a method to this local var pool, and implemented by this implementor.
   * The method is of the type that acceps parameters of the UVariable type.
   * \param implementor is a pointer to a class that has a UVarMethodImplement ansestor
   * \param name of method to add - may be in a sub-structure using a '.' notation
   * \param paramTypes is a string with the parameter types (string (s) or double (d).
   * \param comment is intended as an online help description
   * \returns an index to the method */
  int addMethodV(UVarMethodImplement * implementor,
                const char * formalName,
                const char * paramTypes,
                const char * comment);
  /**
  Get description for local variable with this index */
  const char * getVarDescription(int idx);
  /**
  Set parent var pool */
  void setParentVarPool(UVarPool * parent);
  /**
  Set parent var pool to be used when following variable scope in calculations */
  void setParentVarPoolScope(UVarPool * parent);
  /**
  Get parent var pool */
  UVarPool * getParentVarPool()
  { return parentVarPool;};
  /**
  Get parent var pool - following the variable scope parent route */
  UVarPool * getParentVarPoolScope()
  {
    if (parentVarPoolScope != NULL)
      return parentVarPoolScope;
    else
      return parentVarPool;
  };
  /**
  Get root var pool */
  UVarPool * getRootVarPool();
  /**
   *\brief Call a method in this var pool.
   * It may be in this structure or in any sub-structure that the 'name' indicate.
   * \param name is the name of the method, it may adress a sub structure, e.g. aa.bbbb
   * \param paramOrder the specific parameter order as a string, e.g "sdd" for three parameters the
   * first 's' for string and the two last 'd's for double.
   * \param strings is an array of string pointers. as many of these string pointrs as there
   * is 's' in the stringOrder should be valis and contain the string type parameters.
   * \param doubles is an array of doubles of at least the size of the number of 'd' in the paramOrder string.
   * \param value is where the direct return value are placed. It may be a value or a flag (true/false) of the
   * success of the function. This pointer may be NULL.
   * \param returnStruct is an array of pointers to a datatype compatible with the UDataBase base type.
   * This array pointer may be NULL.
   * \param returnStructCnt is a pointer to the number of entries in the returnStruct array.
   * When the function returns this number will be modified to the actual number of values returned.
   * \return true if the method exist. */
  bool callLocal(const char * name, const char * paramOrder,
                    char ** strings, const double * doubles,
                    double * value,
                    UDataBase ** returnStruct,
                    int * returnStructCnt);
  /**
   *\brief Call a method in this var pool (using UVariable parameters).
   * It may be in this structure or in any sub-structure that the 'name' indicate.
   * \param name is the name of the method, it may adress a sub structure, e.g. aa.bbbb
   * \param paramOrder the specific parameter order as a string, e.g "sdd" for three parameters the
   * first 's' for string and the two last 'd's for double.
   * \param params is an array of UVariable pointers - same number as paramOrder string length.
   * \param returnStruct is an array of pointers to a datatype compatible with the UDataBase base type.
   * This array pointer may be NULL. If NULL, then no return value is returned. Should be at least
   * one element to get a result, first return struccture should be a UVariable, the rest could be data.
   * \param returnStructCnt is a pointer to the number of entries in the returnStruct array.
   * When the function returns this number will be modified to the actual number of values returned.
   * \return true if the method exist. */
  bool callLocalV(const char * name, const char * paramOrder,
                  UVariable * params[],
                  UDataBase ** returnStruct,
                  int * returnStructCnt);
  /**
   *\brief Call a method in the root var pool.
   * It may be in this structure or in any sub-structure that the 'name' indicate.
   * \param name is the name of the method, it may adress a sub structure, e.g. aa.bbbb
   * \param paramOrder the specific parameter order as a string, e.g "sdd" for three parameters the
   * first 's' for string and the two last 'd's for double.
   * \param strings is an array of string pointers. as many of these string pointrs as there
   * is 's' in the stringOrder should be valis and contain the string type parameters.
   * \param doubles is an array of doubles of at least the size of the number of 'd' in the paramOrder string.
   * \param value is where the direct return value are placed. It may be a value or a flag (true/false) of the
   * success of the function. This pointer may be NULL.
   * \param returnStruct is an array of pointers to a datatype compatible with the UDataBase base type.
   * This array pointer may be NULL.
   * \param returnStructCnt is a pointer to the number of entries in the returnStruct array.
   * When the function returns this number will be modified to the actual number of values returned.
   * \return true if the method exist. */
  bool callGlobal(const char * name, const char * paramOrder,
                 char ** strings, const double * doubles,
                 double * value,
                 UDataBase ** returnStruct,
                 int * returnStructCnt);
  /**
   *\brief Call a method in this scope,
   * i.e. try if it fits a call in any of the parent var pools.
   * It may be in the structure direct (no '.') or in any sub-structure that the 'name.subname' indicate.
   * \param name is the name of the method, it may adress a sub structure, e.g. aa.bbbb
   * \param paramOrder the specific parameter order as a string, e.g "sdd" for three parameters the
   * first 's' for string and the two last 'd's for double.
   * \param strings is an array of string pointers. as many of these string pointrs as there
   * is 's' in the stringOrder should be valis and contain the string type parameters.
   * \param doubles is an array of doubles of at least the size of the number of 'd' in the paramOrder string.
   * \param value is where the direct return value are placed. It may be a value or a flag (true/false) of the
   * success of the function. This pointer may be NULL.
   * \param returnStruct is an array of pointers to a datatype compatible with the UDataBase base type.
   * This array pointer may be NULL.
   * \param returnStructCnt is a pointer to the number of entries in the returnStruct array.
   * When the function returns this number will be modified to the actual number of values returned.
   * \return true if the method exist. */
  bool callScope(const char * name, const char * paramOrder,
                  char ** strings, const double * doubles,
                  double * value,
                  UDataBase ** returnStruct,
                  int * returnStructCnt);
  /**
   *\brief Call a method in this scope - using UVariable parameters,
   * i.e. try if it fits a call in any of the parent var pools.
   * It may be in the structure direct (no '.') or in any sub-structure that the 'name.subname' indicate.
   * \param name is the name of the method, it may adress a sub structure, e.g. aa.bbbb
   * \param paramOrder the specific parameter order as a string, e.g "sdd" for three parameters the
   * first 's' for string and the two last 'd's for double.
   * \param params is an array of UVariable pointers. with the actual parameters
   * \param returnStruct is an array of pointers to a datatype compatible with the UDataBase base type.
   * This array pointer may be NULL.
   * \param returnStructCnt is a pointer to the number of entries in the returnStruct array.
   * When the function returns this number will be modified to the actual number of values returned.
   * \return true if the method exist. */
  bool callScopeV(const char * name, const char * paramOrder,
                  UVariable ** params,
                  UDataBase ** returnStruct,
                  int * returnStructCnt);
  /**
   *\brief Call a method from the root var pool scope
   * It may be in the structure direct (no '.') or in any sub-structure that the 'name.subname' indicate.
   * \param name is the name of the method, it may adress a sub structure, e.g. aa.bbbb
   * \param paramOrder the specific parameter order as a string, e.g "sdd" for three parameters the
   * first 's' for string and the two last 'd's for double.
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
  Get the description note */
  inline const char * getDescription()
  { return description; };
  /**
  Set the description note
  \param copy makes a copy of note, else the note pointer is used as is (i.e. pointes to a static string).
  */
  void setDescription(const char * note, bool copy);
  /**
  Set logfile for this structure.
  The logfile is a pointer to a ULogFile structure.
  An additional logfile lock is provided too. */
  void setLogfile(ULogFile * fileHandle, ULock * fileLock);
  /**
  Test if this structure is open for data change logging */
  inline bool isLogfileOpen()
  { return logVar != NULL; };
  /**
   * Get a variable value in this var-pool or in any of the parent var-pools (in scope path)
   * or in the math structure.
   * \param name is the name of the variable, and may include '.' notation for sub-variables
   * \param value is where the value is to be returned
   * \returns true if the variable were found. */
  bool getScopeValue(const char * name, double * value);
  /**
   * Set a variable in the scope to a value
   * Scope is in this var-pool or any parent var-pool.
   * \param name is the name of the variable - possibly with '.' notation.
   * \param value is the value to assign to the variable.
   * \param mayAdd if true the variable will be added to the local var-pool is not found in scope
   * \returns true if set. */
  bool setScopeVar(const char * name, const double value, bool mayAdd);
  /**
   * Set a variable in the scope to the value of source
   * Scope is in this var-pool or any parent var-pool.
   * \param name is the name of the variable - possibly with '.' notation.
   * \param source is the value to assign to the variable.
   * \param mayAdd if true the variable will be added to the local var-pool is not found in scope
   * \returns true if set. */
  bool setScopeVar(const char * name,
                   const UVariable * source, bool mayAdd);
  /**
   * List variable in this structure into this buffer.
   * One variable on each line, preceded by the preString.
   * \param preString is to go in front of all variable lines
   * \param buff is the buffer for the result
   * \param buffCnt is the length of the buffer
   * \param andStructs lists also any sub-structures variables - if true */
  void listVars(const char * preStr, char * buff, const int buffCnt, bool andStructs);
  /**
   * \brief get pointer to variable object in this variable tree
   * \param name is the name of the variable, may include structure name (e.g. aaa.bbb.c)
   * \param element is set to the array element indicated by name (0 if no index specified)
   * \returns NULL if variable is not found, otherwise a pointer to the variable object */
  UVariable * getLocalVariable(const char * name, int * element);
  /**
   * Get the pointer to the logfile if theis varpool structure needs logging
   * \returns pointer to logfile class, is NULL if no log is started (or intended) */
  ULogFile * getLogVar()
  { return logVar; };
  /**
   * Get the file lock to the log-file
   * \returns a poinmter to the lock */
  ULock * getLogVarLock()
  { return logVarLock; };
  /**
   * \brief get pointer to variable object
   * NB! the returned pointer may be ilegal if module that owns the variable is unloaded
   * so it must be used only while executing one command in main server core thread, if
   * needed in a thread - like the sequencer, then get the value only - getValue().
   * \param name is the name of the variable, may include structure name
   * \param element is set to the array element specified in name, or 0 if not array.
   * e.g. set to 3 in name is "imu.rot[3]". 
   * Element is set to -1, if name ends with '.sizeof'
   * \returns NULL if variable is not found, otherwise a pointer to the variable object */
  UVariable * getGlobalVariable(const char * name, int * element = NULL);
  
protected:
  /**
  Evaluate system specific functions - that need to know
  position of planner etc.
  The called function name is in 'name' the parameters in 'pars'
  and if the function returns a value it
  must be put in 'value' */
  virtual bool evaluateSystemFunction(const char * name,
                                      const double pars[], int parsCnt,
                                      double * value, bool syntaxCheck);
  /**
  * Create the stack of local variables.
  * Creates the maximum number of variables ever to be stored in this variable pool.*/
  bool createVarStack(const int maxCount);
  /**
  * Create the stack of references to other var-pools, that from this var-pool is seen as structures,
     as objects with variables and functions (and possibly other structures).
  * Creates the maximum number of variables ever to be stored in this variable pool. */
  bool createVarPoolStack(const int maxCount);
  /**
   * Create the stack of references to functions that can be called to evaluate double variables.
   * Returns true if space is allocated for 'maxCount' functions */
  bool createFunctionStack(const int maxCount);

protected:
  /**
  Print more to console if verbose is true */
  bool verbose;
  /**
  * Stack of structures with other variables subordinate to this structure. */
  UVarPool ** varPools;
  /**
   * array of boolean to indicate if the var pool is created locally. */
  bool * varPoolsLocal;
  /**
  Description for this structure (used when displaying global variables) */
  const char * description;
  /**
  Logfile for variable updates */
  ULogFile * logVar;
  /**
  Resource lock on logfile */
  ULock * logVarLock;

private:
  /**
  Stack of variable */
  UVariable ** vars;
  /**
  Number of used variables */
  int varsCnt;
  /**
  Maximum number of variables */
  int varsMaxCnt;

  /**
  Count of subordinat varPool structures actually populated. */
  int varPoolsCnt;
  /**
  Space available for subordinat varPool structures. */
  int varPoolsMaxCnt;

  /**
  Available function calls in this var pool */
  UVarMethod * meths;
  /**
  * Number of available function calls */
  int methsCnt;
  /**
   * Maximum number of available function calls */
  int methsMaxCnt;
  /**
  Variable pre-name, as in pre-name.variable */
  char preName[MAX_VAR_NAME_SIZE];
  /**
  Variable pre-name, as in pre-name.variable */
  char preNameFull[MAX_VAR_NAME_SIZE * MAX_STRUCT_NAMES];
  /**
  Parent var pool */
  UVarPool * parentVarPool;
  /**
  Parent var pool when evaluating variables */
  UVarPool * parentVarPoolScope;
  /**
  copy of description - if not literal */
  char * descriptionCopy;
};

#endif
