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
#ifndef UVARIABLE_H
#define UVARIABLE_H

#include <ctype.h>

#include <ugen4/udatabase.h>
#include <ugen4/ulock.h>
#include <ugen4/uposrot.h>
#include <umap4/upose.h>
#include <iau_mat.h>

#include "uvarmethod.h"
#include "ulogfile.h"
#include "uvarlog.h"

class UVarPool;
class UMatrix;
/**
A variable for the calculator */
class UVariable : public UDataBase, public UVarLog
{
public:
  /**
   * Constructor */
    UVariable();
  /**
   * Copy constructior */
    UVariable(UVariable * source);
  /**
   * Destructor */
    virtual ~UVariable();
  /**
    Get (end) type of this structure */
    virtual const char * getDataType()
    {
      return "var";
    };
      /**
    Function to test if the class or one of its ancestors is of a specific type */
    virtual bool isAlsoA(const char * typeString);
    /**
    Print status for this structure */
    virtual void snprint(const char * preString, char * buff, const int buffCnt);
  /**
   * defined types of variables (d=double i=integer, b=boolean, t=time, s=string)
     all except string is double based (double or double array) */
    typedef enum {none, d, s, d3d, d2d, d6d, dq, t, pose, rot, m2, i, b} varType;
  /**
    Test for variable type by a simple string compare.
    Returns true is exact match - is case sensitive) */
    inline bool isTypeA(const varType testType)
    { return (dataType == testType); };
  /**
     * Get double type value of this variable
     * \param idx variable may be an array, so an index is provided
   * \returns the value as double */
    double getValued(const int idx);
    /**
    Supplementary call
    * \param idx variable may be an array, default is 0
    * \returns the value as double */
    inline double getDouble(const int idx = 0)
    { return getValued(idx); };
    /**
    Supplementary call
    * \param idx variable may be an array, default is 0
    * \returns the value as float */
    inline float getFloat(const int idx = 0)
    { return getValued(idx); };
    /**
   * Get integer type value of this variable
   * \param idx variable may be an array, so an index is provided
   * \returns the value as integer */
    inline int getValueInt(const int idx = 0)
    { return roundi(getValued(idx)); };
  /**
   * Get boolean type value of this variable
   * \param idx variable may be an array, so an index is provided
   * \returns the value as boolean */
  bool getValueBool(const int idx = 0);
  /**
   * Get boolean type value of this variable
   * \param idx variable may be an array, so an index is provided
   * \returns the value as boolean */
  inline bool getBool(const int idx = 0)
  {
    return getValueBool(idx);
  };
  /**
   * Get integer type value of this variable
   * \param idx variable may be an array, so an index is provided
   * \returns the value as int*/
  int getInt(const int idx = 0);
  /**
   * Get UTime type value from this variable
   * \param idx variable may be an array, so an index is provided
   * \returns the value as UTime */
  UTime getTime(const int idx = 0);
  /**
  * Set double type value of this variable.
  * If too little space allocated, then more is allocated
  * \param value is the value to be set for the index idx
  * \param ix may be an array, so an index is provided
  * \param mayAdd if true and the index is higher than element cnt, then
  *               more space is allocated.
  * \param updT is an optional pointer to the update time (relevant for time series only)
  * \returns true if set, returns false if index is negative or
  * if datatype is not double based */
  bool setValued(double value, const int ix = 0, bool mayAdd = false, UTime * updT = NULL);
  /**
  Supplementary call to set single value
  * \param value is the value to be set for the index idx
  * \param idx variable may be an array, default is 0.
  * \param mayAdd if true and the index is higher than element cnt, then
  *               more space is allocated.
  * \param updT is an optional pointer to the update time (relevant for time series only)
  * \returns true if set, returns false if index is negative or
  * if datatype is not double based */
  inline bool setDouble(double value, const int idx = 0, bool mayAdd = false, UTime * updT = NULL)
  { return setValued(value, idx, mayAdd, updT); };
  /**
  Call to set a series of (double) values.
  If variable is too small to hold the data, then it is extended.
  * \param values array of doubles to be inserted into the variable
  * \param valuesCnt number of values in array to set
  * \param updT is an optional pointer to the update time (relevant for time series only)
  * \returns true if set, returns false if index is negative or
  * if datatype is not double based */
  bool setDouble(double * values, const int valuesCnt, UTime * updT = NULL);
  /**
  Supplementary call to set single value
  * \param value is the value to be set for the index idx
  * \param idx variable may be an array, default is 0.
  * \param mayAdd if true and the index is higher than element cnt, then
  *               more space is allocated.
  * \param updT is an optional pointer to the update time (relevant for time series only)
  * \returns true if set, returns false if index is negative or
  * if datatype is not double based */
  inline bool setInt(int value, const int idx = 0, bool mayAdd = false, UTime * updT = NULL)
  { return setValued(double(value), idx, mayAdd, updT); };
  /**
  Supplementary call to set single value
  * \param value is the value to be set for the index idx
  * \param idx variable may be an array, default is 0.
  * \param mayAdd if true and the index is higher than element cnt, then
  *               more space is allocated.
  * \param updT is an optional pointer to the update time (relevant for time series only)
  * \returns true if set, returns false if index is negative or
  * if datatype is not double based */
  inline bool setBool(bool value, const int idx = 0, bool mayAdd = false, UTime * updT = NULL)
  { return setValued(double(value), idx, mayAdd, updT); };
  /**
   * Set value of this variable, assuming value can be converted to double.
   * If too little space allocated, then more is allocated.
   * If variable is a matrix or the attValue has ';' (as row seperator)
   * then the size of the matrix is set from the data, and 'idx' should be set to 0.
   * \param attValue is a string with the values for the array, at
   * maximum the number of variables in one element is used - e.g. 3 for a d3d type
   * \param idx may be an array, so an index is provided
   * \param mayAdd if true, then array is expanded as needed following idx
  * \param updT is an optional pointer to the update time (relevant for time series only)
   * \returns true if set, returns false if index is negative or
   * if datatype is not double based */
  bool setValued(const char* attValue, const int idx, bool mayAdd, UTime * updT = NULL);
  /**
   * Set value of this string variable.
   * If too little space allocated, then more is allocated
   * if idx is not 0, then idx is the first character to be set, values befroe this is unchanged, but the string
   * is not terminated after the value (if the original string is longer)
   * \param attValue is a string to be added
   * \param idx index, where the first character is to be set.
   * \param mayAdd if true, then array is expanded as needed following idx and string length
  * \param updT is an optional pointer to the update time (relevant for time series only)
   * \returns true if set, returns false if index is negative or
   * if datatype is not a string */
  bool setValues(const char * attValue, const int idx, bool mayAdd, UTime * updT = NULL);
  /**
   * Set value of this string variable.
   * If too little space allocated, then more is allocated
   * if idx is not 0, then idx is the first character to be set, values befroe this is unchanged, but the string
   * is not terminated after the value (if the original string is longer)
   * \param attValue is a string to be added
   * \param idx index, where the first character is to be set.
   * \param mayAdd if true, then array is expanded as needed following idx and string length
  * \param updT is an optional pointer to the update time (relevant for time series only)
   * \returns true if set, returns false if index is negative or
   * if datatype is not a string */
  inline bool setString(const char * attValue, const int idx, bool mayAdd, UTime * updT = NULL)
  { return setValues(attValue, idx, mayAdd, updT); }
  /**
   * Set value from this source.
   * Optinally set just element 'elem'
   * \param source is the new value
   * \param idx is the element in the destination to be set from the source
   * if elem > 0 and type is double, then just the element is set from the first value in the source.
   * if elem > 0 and type is string, then the source is treates as a new substring to be set at this position
   * if elem == 0, then the source is copied to this variable in total (incl type)
  * \param updT is an optional pointer to the update time (relevant for time series only)
   * \returns true if set. */
  bool setValue(const UVariable * source, const int idx, UTime * updT = NULL);
  /**
  * Set value from this source.
  * variable is resized to match the source existing variable has too few elements
  * \param mat is the new value for the matrix variable
  * \returns true if set. */
  bool setValueM(UMatrix * mat, UTime * updT = NULL);
  /**
  * Set value from this source.
  * variable is resized to match the source existing variable has too few elements
  * \param mat is the new value for the matrix variable
  * \returns true if set. */
  bool setValueM(matrix * mat, UTime * updT = NULL);
  /**
  * Set value from this source.
  \param rCnt is the number of rows in the source array.
  \param cCnt is the number of columns in each row.
  \param values is an array with rCnt*cCnt elements.
  \returns true if set. */
  bool setValueM(int rCnt, int cCnt, double * values, UTime * updT = NULL);
  /**
   * Assign values to a new (empty) variable.
   * \param newName The new name of the variable
   * \param attValue string with value for variable
   * \param type string indicationg how to interpet the value string
   * \param comment - new comment (static string - pointer used directly)
   * \returns true if values are set. */
  bool setValueNew(const char * newName, const char * attValue,
                   const char * type,
                   const char * comment);
  /** Set value of this variable to decimal reprecentation of current time.
  * \param idx index, where the first character is to be set (default is 0).
  * \returns true if value is set. */
  bool setTimeNow(const int idx = 0);
  /** Set value of this variable to decimal representation of current time (decimal seconds since 1 jan 1970).
  * \param t is time to set.
  * \param idx index, where the first character is to be set (default is 0).
  * \returns true if value is set. */
  bool setTime(UTime t, const int idx = 0);
  /**
     * Get double type value of this variable
   * \returns the value as double of first element */
  inline double getValued() const
  {
    if (dValue != NULL)
      return *dValue;
    else
      return 0.0;
  };
  /**
   * Test if the data is double based */
  inline bool isDouble() const
  { return isDoubleBased; }
  /**
   * Test if the data is a string */
    inline bool isString() const
    { return elementSize == 1 and not isDoubleBased; }
  /**
    Save this variable to logfile */
    void saveToLog(ULogFile * logFile, const char * prename, ULock * fileLock);

  /**
     * Set data type and element size
     * If the type were different before, then the element size is
     * changed and if the element reduced to be within the new buffer
     * range. element count is unchanged if space allows.
     * \param newType set the data type to this value
   * \returns false if unknown data type - should not be */
    bool setType(varType newType);
  /**
     * Get data type as a character string to this buffer
     * \param to a character buffer to store result
     * \param toCnt the size of the buffer
   * \returns a pointer to the buffer */
    const char * getTypeChar(char * to, const int toCnt);
  /**
     * Get data type as a character string to this buffer
     * \param to a character buffer to store result
     * \param toCnt the size of the buffer
   * \returns a pointer to the buffer */
    UVariable::varType getType() const
    { return dataType; };
  /**
     * Set data type from a character string
     * \param to a character description of data type
   * \returns a pointer to the buffer */
    bool setTypeChar(const char * to);
  /**
     * Add a value to this double sized variable
     * \param value to add
     * \param idx optional index to the value
     * \param updTime set to this update time if relevant (relevant if variable has history)
   * \returns new value */
    double add(double value, int idx = 0, UTime * updTime = NULL);
  /**
   * Get variable type from a character representattion */
    static varType typeFromChar(const char * typ);
  /**
     * Split name, assuming an index is embedded in the name.
     * The function finds first index only, no decimals allowed in index.
     * Can be called with no object
     * e.g.  'data[3]' returns true and index=3 length = 4
     * e.g.  'data[ 3 ]' returns true and index=3 length = 4
     * e.g.  'data[3][6]' returns true and index=3 length = 4
     * e.g.  'data[3,6]' returns true and index=3, col=6 length = 4
     * e.g.  'data[3 6]' returns true and index=3, col=6 length = 4
     * e.g.  'data' returns true and index=0 length = 4
     * e.g.  'data[]' returns true and index=0 length = 4
     * e.g.  'data[3.0]' returns false and index=3 length = 4
     * e.g.  'data[3' returns false and index=3 length = 4
     * \param nameWithIndex is the full name, e.g.  'data[3]'
     * \param index is where the index is returned e.g. 3
     * \param nameLng is where the length without index is returned e.g. 4
   * \returns true if format is as expected. */
    //static bool splitNameIndexStatic(const char * nameWithIndex, int * index, int * col int * nameLng);
  /**
     * Split name, assuming an index is embedded in the name.
     * The function finds also index for arrays - i.e. [1,4] is fifth element in row two, and
     * returns index 9 if matrix has 5 columns, i.e. index is row*columns+col from name[row,col].
     * e.g.  'data[3]' returns true and index=3 length = 4
     * e.g.  'data[ 3 ]' returns true and index=3 length = 4
     * e.g.  'data[3][6]' returns true and index=3 length = 4
     * e.g.  'data[3,6]' returns true and index=3, col=6 length = 4
     * e.g.  'data[3 6]' returns true and index=3, col=6 length = 4
     * e.g.  'data' returns true and index=0 length = 4
     * e.g.  'data[]' returns true and index=0 length = 4
     * e.g.  'data[3.0]' returns false and index=3 length = 4
     * e.g.  'data[3' returns false and index=3 length = 4
     * \param nameWithIndex is the full name, e.g.  'data[3]'
     * \param index is where the index is returned e.g. 3
     * \param col is set to column index, if specified as a matrix (and not NULL)
     * \param nameLng is where the length without index is returned e.g. 4
   * \returns true if format is as expected. */
    static bool splitNameIndex(const char * nameWithIndex, int * index, int * col, int * nameLng);
  /**
     * Set name of variable.
     * Strips any index from the name. If there is an index, this is returned in *idx.
     * \param name of variable - nay have index, e.g. foo[8]
     * \param idx optional pointer to integer, where index is to be returned
   * */
    void setName(const char * newName, int * idx = NULL);
  /**
     * Get value (double type) as string - for data transfer
     * \param idx is the index if variable is an array
     * \param preStr is optional characters to put in front of value string, e.g. a space
     * \param buff is the buffer for the result
     * \param buffCnt is the length of the provided buffer
   * \returns a pointer to the buffer */
    const char * getValuedAsString(int idx, const char* preStr, char* buff, const int buffCnt, int histIdx = 0);
  /**
   * Get all double values to this string buffer
   * \param buff is the buffer for the result
   * \param buffCnt is the length of the provided buffer
   * \param histIdx is 0 if current value, else time series index
   * \returns a pointer to the buffer */
  const char * getValuesdAsString(char * buff, const int buffCnt, int histIdx);
  /**
  * Get string with values of a matrix, where rows are on seperate lines, and values in %g format
  * \param buff is the buffer for the result
  * \param buffCnt is the length of the provided buffer
  * \returns a pointer to the buffer */
  const char * getValuesmAsString(char * buff, const int buffCnt);
  /**
  * Get string with values of a matrix, where rows are on seperate lines, and values in %g format
  * Limit the number of raws and columns
  * \param buff is the buffer for the result
  * \param buffCnt is the length of the provided buffer
  * \param maxRows is the max number of rows shown
  * \param maxCols is the maximum number of columns shown
  * \returns a pointer to the buffer */
  const char * getValuesmAsStringLimited(char * buff, const int buffCnt, int maxRows, int maxCols);
  /**
   * Get all values - string or double - to this string buffer
   * \param buff is the buffer for the result
   * \param buffCnt is the length of the provided buffer
   * \param histIdx is 0 if current value, else time series index
   * \returns a pointer to the buffer */
  const char * getValuesAsString(char * buff, const int buffCnt, int histIdx);
  /**
   * Get all values - string or double - to this string buffer and pack into
   * quotations marks or apostrophs.
   * If double based quotation marks are used, if string the first part of the string
   * is searched, and if a quotation mark is found (before a apostroph), then
   * string is packed in apostroph.
   * \param buff is the buffer for the result
   * \param buffCnt is the length of the provided buffer
   * \returns a pointer to the buffer */
  const char * getValuesAsStringQuoted(char * buff, const int buffCnt, int histIdx);
  /**
   * make an attribute string from the name and value of this variable
   * \param buff is the string buffer to hold the result
   * \param buffCnt is the length of the buffer
   * \returns a pointer to the buffer */
  const char * getGetAsXmlAttribute(char * buff, const int buffCnt);
  /**
   * Get number of elements, i.e. 3 for a 3d type or elements in a vector or an array.
     if a string, then the length of the string (as returned by strlen()) */
  inline int getElementCnt() const
    { return elementCnt; };
  /**
   * Get number of elements as getElementCnt() (used virtually from time series subclass) */
  virtual int getElementCnt2() const
    { return elementCnt; };
  /**
   * Get byte size of buffer */
  inline int getByteCnt() const
    { return byteCnt; };
  /**
   * Get the base of the variables in this variable.
   * \returns a pointer to the value buffer */
  inline const char * getValueBuffer() const
  { return (const char *)dValue; };
  /**
   * Get pointer to value of this variable (assuming it is a string).
   * \param idx=0 index to possible starting position of returned string
   * \returns a pointer to the value buffer (starting at position idx) */
  inline const char * getString(const int idx = 0) const
  { return getValues(idx); };
  /**
   * Get the base of the variables in this variable, assuming it is a string.
   * \param idx=0 index to possible starting position of returned string
   * \returns a pointer to the value buffer (starting at position idx) */
  inline const char * getValues(const int idx = 0) const
  {
    const char * p1;
    p1 = (const char *)dValue;
    if (idx < byteCnt and idx > 0)
      p1 += idx;
    return p1;
  };
  /**
   * Copy the content of the source variable to this variable
   * \param source - the data source. */
  void copy(const UVariable * source, bool alsoName);
  /**
   * Get first 3 values in a URotation structure */
  URotation getRot() const;
  /**
   * Get first 3 values in a UPosition structure */
  UPosition get3D() const;
  /**
   * Get first 6 values in a UPosRot structure */
  UPosRot get6D() const;
  /**
   * Get first 3 values in a UPose structure */
  UPose getPose() const;
  /**
   * Get first 3 values in a UPose structure */
  UPoseTime getPoseTime() const;
  /**
   * Set first 3 values from a URotation structure */
  bool setRot(URotation * value, UTime * updT = NULL);
  /**
   * Set first 3 values from a UPosition structure (order x,y,z) */
  bool set3D(UPosition * value, UTime * updT = NULL);
  /**
   * Set first 6 values from a UPosRot structure (order x,y,z,Omega,Phi,Kappa) */
  bool set6D(UPosRot * value, UTime * updT = NULL);
  /**
   * Set first 3 values from a UPose structure (order x, y, h) */
  bool setPose(UPose * value, UTime * updT = NULL);
  /**
   * Set first 4 values from a UPose structure (order x, y, h, tod) */
  bool setPose(UPoseTime * value, UTime * updT = NULL);
  /**
     \brief negate - element by element
     * Negate value of this variable, i.e. change sign of all values.
     * Works on double based variables only.
   */
  void negate();
  /**
     \brief inverse - element by element
     * inverse all values one by one, i.e. a = 1.0/a.
     * if |a| < 1e-100, then a is set to +1e100
     * Works on double based variables only.
   */
  void inverse();
  /**
   * \brief Greater than - element by element
   * Make this variable an array of same size as left and right, with the values
   * this[i]=1.0 if left[i] is > right[i], else 0.0.
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setGt(UVariable * left, UVariable * right);
  /**
   * \brief Greater or equal- element by element
   * Make this variable an array of same size as left and right, with the values
   * this[i]=1.0 if left[i] is >= right[i], else 0.0.
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setGe(UVariable * left, UVariable * right);
  /**
   * \brief Less than - element by element
   * Make this variable an array of same size as left and right, with the values
   * this[i]=1.0 if left[i] is \< right[i], else 0.0.
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setLt(UVariable * left, UVariable * right);
  /**
   * \brief Less than or equal - element by element
   * Make this variable an array of same size as left and right, with the values
   * this[i]=1.0 if left[i] is \<= right[i], else 0.0.
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setLe(UVariable * left, UVariable * right);
  /**
   * \brief Equal
   * Sets the value of this variable (size 1) to 1 if all the elements
   * in left is (exactly) equal to the one on the right.
   * The left and right size must be equal.
   * Else the value is 0.
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setEq(UVariable * left, UVariable * right);
  /**
   * \brief Not equal
   * Sets the value of this variable (size 1) to 1 if one or more of the elements
   * in left is not (exactly) equal to the one on the right.
   * The left and right size must be equal. Else the value is 0.
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setNe(UVariable * left, UVariable * right);
  /**
   * \brief element by element sum of values
   * Make this variable an array of same size as left and right,
   * with the values: this[i]= left[i] + right[i]
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setPlus(UVariable * left, UVariable * right);
  /**
   * \brief element by element difference of values
   * Make this variable an array of same size as left and right,
   * with the values: this[i]= left[i] - right[i]
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setDiff(UVariable * left, UVariable * right);
  /**
   * \brief element by element product of values
   * Make this variable an array of same size as left and right,
   * with the values: this[i]= left[i] * right[i]
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setProduct(UVariable * left, UVariable * right);
  /**
   * \brief element by element division of values
   * Make this variable an array of same size as left and right,
   * with the values: this[i]= left[i] / right[i]
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setDivide(UVariable * left, UVariable * right);
  /**
   * \brief element by element binary AND values
   * Make this variable an array of same size as left and right,
   * with the values: this[i]= long(left[i]) & long(right[i])
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setBinaryAnd(UVariable * left, UVariable * right);
  /**
   * \brief element by element binary OR values
   * Make this variable an array of same size as left and right,
   * with the values: this[i]= long(left[i]) & long(right[i])
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setBinaryOr(UVariable * left, UVariable * right);
  /**
   * \brief element by element binary XOR values
   * Make this variable an array of same size as left and right,
   * with the values: this[i]= long(left[i]) ^ long(right[i])
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setBinaryXor(UVariable * left, UVariable * right);
  /**
   * \brief element by element logical OR values
   * Make this variable an array of same size as left and right,
   * with the values: this[i]= left[i] or right[i]
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setLogicalOr(UVariable * left, UVariable * right);
  /**
   * \brief element by element logical And values
   * Make this variable an array of same size as left and right,
   * with the values: this[i]= left[i] and right[i]
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based and has same number of values */
  bool setLogicalAnd(UVariable * left, UVariable * right);
  /**
   * \brief element by element bit shift left
   * Make this variable an array of same size as left,and shift based on first element of right
   * with the values: this[i]= double(long(left[i]) << long(right[0]))
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based */
  bool setBinaryShiftLeft(UVariable * left, UVariable * right);
  /**
   * \brief element by element bit shift right
   * Make this variable an array of same size as left,and shift based on first element of right
   * with the values: this[i]= double(long(left[i]) << long(right[0]))
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based */
  bool setBinaryShiftRight(UVariable * left, UVariable * right);
  /**
   * \brief element by element modulo operation
   * Make this variable an array of same size as left,and operate based on first element of right
   * with the values: this[i]= double(long(left[i]) % long(right[0]))
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based */
  bool setModulo(UVariable * left, UVariable * right);
  /**
   * \brief element by element power operation
   * Make this variable an array of same size as left,and operate based on first element of right
   * with the values: this[i]= pow(left[i], right[0])
   * \param left is left operator value
   * \param right is right operator value
   * \returns true if both are double based */
  bool setPow(UVariable * left, UVariable * right);
  /**
   * \brief set all element to the negated value of the current value.
   * \returns true if variable is double based */
  bool setNot();
  /**
   * Set parent var pool to this variable. this is needed to
   * let updates trigger an update event
   * \param value pointer to the parent var-pool, may be NULL for disabling this notify event */
  inline void setParent(UVarPool * value)
  { parent = value; };
  /**
  Set size for a variable. Sets the type to a matrix.
  \param rows number of rows.
  \param cols number of columns.
  \return size in elements */
  int setSize(int rows, int cols);
  /**
  Set size of a double based or a string variable.
  \param rows number of rows.
  \param cols number of columns.
  \return size in elements */
  int setSize(int elements);
  /**
  Get number of rows for this variable.
  \returns 1 if not a matrix, else number of rows. */
  int rows();
  /**
  Get number of rows for this variable.
  \returns returns number of elements in variable, or in each row if a matrix. */
  int cols();
  /**
  \returns number of elements in this variable.
  NB! size returned for a matrix is rows*cols, but actually data element is 2 values larger to store rows and cols) */
  int getSize();
  /**
  Get array of values as doubles*/
  inline double * getValuesd()
  {
    if (dataType == m2)
      return &dValue[2];
    else
      return dValue;
  }
  /**
  Get values to a matrix structure.\n
  The mat matrix must be initialized, and the size of the matrix will not be changed.
  If the size of this matrix do not match mat, then no data is loaded into mat, and function returns false.
  \param mat is a IAU matrix, where the result will be placed.
  \returns true if the size match (and data is loaded to mat).
  \returns false if matrix size do not match  */
  bool getM(matrix * mat);
  /**
  Get values to a matrix structure.\n
  The mat matrix must be initialized, and the size of the matrix will not be changed.
  If the size of this matrix do not match mat, then no data is loaded into mat, and function returns false.
  \param mat is a openCV matrix (packed into the UMatrix class), where the result will be placed.
  \returns true if the size match (and data is loaded to mat).
  \returns false if matrix size do not match  */
  bool getM(UMatrix * mat);
  /**
  Open logfile for this variable.
  \param logFileOpen if true, then log is opened (with name of variable extended with '.log. if false, then log is closed. '
  \returns true if log is open. */
  bool openVarLog(bool logFileOpen);
  /**
  Make time series history for this variable
  \param varCnt is the size of the time series (number of entries) to be allocated for on-line history.
  \param maxUpdateRate maksimum update rate for the time series (in Hz) (must be > 1e-6 Hz)
  \returns true if seccessful. */
  inline bool makeTimeSeries(int histCnt, double maxUpdateRate)
  {
    return makeHist(histCnt, elementCnt, maxUpdateRate);
  };
  /**
  Set description of a variable
  \param desc is the description.
  \param makeCopy if true, then space iss allocated for description, if
  false then the value is assumed to be stationary in the lifetime of the variable, i.e. the
  description pointer is set directly to the source */
  const char * setDescription(const char * desc, bool makeCopy);
  /**
  Is the description buffer used */
  bool descriptionBufferUsed() const
  {
    return descriptionBuffer != NULL and description == descriptionBuffer;
  };
  /**
   * Set all elements in this (double based) variable to this value
   * \param value value to be set into all elements - typically 0.0 or 1.0.
   * \returns true if a double based variable */
  bool setAll(double value);
  /**
   * Set this m2 type variable to 1.0 in diagonal and 0.0 in other elements.
   * \returns true if a m2 variable */
  bool setUnit();


protected:
  /**
   * Set variable as updated, to notify for push events */
  //void setUpdated();
  /**
   * Set variable as updated, with timestamp - for local history only (if any)
   \param updTime time of this update. */
  void setUpdated(UTime * updTime = NULL);
  /**
  Decode this replay line, that is assumed to be valid.
  \param line is a pointer to the line.
  \returns true if the line is valid and used (step is successfull).
  \returns false if line is not a valid step. */
  virtual bool decodeReplayLine(char * line);

public:
  /**
    Name of variable */
    char name[MAX_VAR_NAME_SIZE + 1];
  /**
    Description */
    const char * description;

  protected:
  /**
   * Variable type - may be part of a semistructure (or a string)
   * p.t. the following types are defined
   * d     : the default double - or double array
   * dq    : an array of 2 values aaaX, aaaQ - a value with an associated quality
   * pose  : an array of 3 values aaaX aaaY aaaH - in this order
   * 2d    : an array of 2 values aaaX aaaY - in this order
   * 3d    : an array of 3 values aaaX aaaY aaaZ - in this order
   * 6d    : an array of 6 values aaaX aaaY aaaZ aaaO aaaP aaaK - in this order
   * rot   : an array of 3 values aaaO aaaP aaaK - in this order
   * s     : a string
   * m2    : 2-dimensional matrix (first 2 elements are rows and columns) */
    varType dataType;
  /**
    Value of variable */
    double * dValue;
  /**
   * size of data buffer in elements */
    int elementCnt;
  /**
   * size of data buffer in bytes */
    int byteCnt;
  /**
   * flag for double sized variables */
    bool isDoubleBased;
  /**
   * size of one data element - in bytes */
    int elementSize;
    /**
     * Pointer to parent var pool - especially for update notification */
    UVarPool * parent;
private:
  /**
  Description data buffer */
  char * descriptionBuffer;
};

#endif
