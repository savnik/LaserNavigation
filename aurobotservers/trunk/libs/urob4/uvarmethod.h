/***************************************************************************
 *   Copyright (C) 2008 by DTU Christian Andersen   *
 *   chrand@mail.dk   *
 *        vvvvvv                                                                 *
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
#ifndef UVARMETHOD_H
#define UVARMETHOD_H

#include <string.h>
#include <ugen4/udatabase.h>

class UVariable;

/**
Name length of a variable */
#define MAX_VAR_NAME_SIZE 32
/**
Max struct concatination variables */
#define MAX_STRUCT_NAMES 15
/**
Maximum number of parameters used
in dynamic functions implemented in C++ (or C).
Used in the UVarMethod structure. */
#define MAX_PARAM_COUNT 10


/**
 * This class implements a method call, that is, the interface definition of a method call */
class UVarMethodImplement
{
  public:
    
    virtual ~UVarMethodImplement() {};
  /**
     * Function to implement a var-pool method call.
     * \param name is the name of the called function 
     * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
     * \param strings is an array of string pointers for the string type parameters (may be NULL if not used) 
     * \param doubles is an array with double typed parameters (may be NULL if not used) 
     * \param value is the (direct) result of the class, either a double value, or 0.0 for false 1.0 for true 2.0 for implicit stop if a controll call.
     * \param returnStruct is an array of class object pointers that can be used as parameters or return objects (may be NULL)
     * \param returnStructCnt is the number of objects in the returnStruct buffer
   * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
    virtual bool methodCall(const char * name, const char * paramOrder,
                            char ** strings, const double * doubles,
                            double * value,
                            UDataBase ** returnStruct = NULL,
                            int * returnStructCnt = NULL);
  /**
     * Function to implement a var-pool method call.
     * \param name is the name of the called function
     * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
     * \param strings is an array of string pointers for the string type parameters (may be NULL if not used)
     * \param doubles is an array with double typed parameters (may be NULL if not used)
     * \param value is the (direct) result of the class, either a double value, or 0.0 for false 1.0 for true 2.0 for implicit stop if a controll call.
     * \param returnStruct is an array of class object pointers that can be used as parameters or return objects (may be NULL)
     * \param returnStructCnt is the number of objects in the returnStruct buffer
   * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
    virtual bool methodCallV(const char * name, const char * paramOrder,
                                          UVariable * params[],
                                          UDataBase ** returnStruct,
                                          int * returnStructCnt);
};

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

/**
A variable for the calculator */
class UVarMethod
{
  public:
  /**
    Constructor */
  UVarMethod();
  /**
   * Add new method call - using double array and string array parameters
   * \param implementor a pointer to the resource with an implementor
   * \param formalName name of the function
   * \param paramsList is a string with the parameter types e.g. "dds" for two double typed and a string.
   * \param comment a constant (static) string, where the method cam find some help or description */
  void setMethod(UVarMethodImplement * implementor,
                   const char * formalName,
                   const char * paramsList,
                   const char * comment);
  /**
   * Add new method call - using UVariable parameters
   * \param implementor a pointer to the resource with an implementor
   * \param formalName name of the function
   * \param paramsList is a string with the parameter types e.g. "dds" for two double typed and a string.
   * \param comment a constant (static) string, where the method cam find some help or description
   * */
  void setMethodV(UVarMethodImplement * implementor,
                const char * formalName,
                const char * paramsList,
                const char * comment);
  /**
    Call the specified method.
     * May return struct values in the 'returnStruct' pointer list (when the
    pointer list is not NULL), and in this case the 'returnStructCnt' is set
    to the maximum number of pointers available and should be modified
    to the number of valid pointers returned.
    The method should return true if the method exist (with this parameter list) */
  bool methodCall(const double * paramDouble,
                    char ** paramStr,
                    double * returnValue,
                    UDataBase ** returnStruct = NULL,
                    int * returnStructCnt = NULL);
  /**
   * Call a method using UVariable parameters and result */
  bool methodCallV(UVariable * params[],
                               UDataBase ** returnStruct,
                               int * returnStructCnt);
  /**
   * Mark this method as deleted (zero length name) */
  void deleteMethod();
  /**
   * Is the method valid.
   * \returns true if valid */
  bool valid()
    { return (strlen(name) > 0); };

  public:
  /**
   * Name of variable */
  char name[MAX_VAR_NAME_SIZE + 1];
  /**
   * Number and type of parameters, e.g. 'sdd' for one string then 2 times double */
  char paramOrder[MAX_PARAM_COUNT + 1];
  /**
   * Pointer to a class that evaluates the var-pool function */
  UVarMethodImplement * implementedBy;
  /**
   * Description of the function */
  const char * description;
  /**
   * Is UVariable parameter call implemented */
  //bool useVParams;
};

#endif
