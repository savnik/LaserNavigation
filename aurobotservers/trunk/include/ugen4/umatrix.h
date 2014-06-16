/***************************************************************************
 *   Copyright (C) 2006 by DTU                                             *
 *   jca@oersted.dtu.dk                                                    *
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

#ifndef UMATRIX_H
#define UMATRIX_H

#define MATRIX_NO_RANGE_CHECK
#include <stdio.h>

//#define OPENCV2
#ifdef OPENCV2
#include <opencv2/imgproc/imgproc_c.h>
#else
#include <opencv/cv.h>
#endif
#include "udatabase.h"

// Size of vectors and
// side size for matrices
#define MAX_VECTOR4_SIZE 4
#define MAX_BIG_VECTOR_SIZE 40

class UPosition;
class UMatrix; // forward declaration
class UMatrixBig; // forward declaration

///////////////////////////////////////////////////////////

/**
Virtual class for matrix operations. This class do not include any
elements. decendent classes must define a data area and point m to
this area. All initialisation and constructors must be handled
by decendent classes. <br>
The matrix index is (row, col), and are 0-based. i.e. for a matrix
of size 4x3 the row number range is 0..3 and the column munber range is 0..2. <br>
This class offers most of the useual matrix operations - the rest is supported by openCV */
class UMatrix : public UDataBase
{
public:
  /**
  Initializes of a matrix to required size, with all elements
  left as is, that is: if column number is cahanged
  all data vontent (with exception of elements
  if first row) are reshuffeld. */
  bool init(const unsigned int rows,
          const unsigned int cols);
  /**
  Initializes of a matrix to required size, and with
  elements along the main diagonal set to i. Other elements
  are set to 0.0. */
  bool init(const unsigned int rows,
          const unsigned int cols,
          const double i);
  /**
  Initialize matrix with user provided data buffer
  \param iRows is (initial) matrix number of rows
  \param iCols is (initial) matrix number of columns
  \param data is array of doubles to hold at least the initial matrix,
  \param bufferSizeInDoubles is the size of the double buffer (may have extra space for
  reorganisation of matrix. */
  inline bool init(const unsigned int iRows, const unsigned int iCols,
                 double * data,
                 unsigned int bufferSizeInDoubles)
  { // connect pointer
    cvInitMatHeader( &mat,
                iRows, iCols,
                CV_64FC1, data, cols() * sizeof(double));
    matrixSize = bufferSizeInDoubles;
    err = 0;
    return true;
  };
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "matrix";  };
  /**
  Function to test if the class or one of its ancestors is of a specific type */
  virtual bool isAlsoA(const char * typeString);
  /**
  Set all values to zero.
  err must be 0 before call. */
  void clear();
  /**
  Resize this matrix, any new elements are un-initialized. */
  inline bool setSize(const unsigned int iRows,
                 const unsigned int iCols)
  {
      return init(iRows, iCols);
  };
  /**
  Expand matrix maintaining old values.
  The 'err' value in matrix will be set to -1 if
  there is no more space to increase the matrix. */
  bool expand(const unsigned int iRows,
                 const unsigned int iCols);
  /**
  Expand matrix maintaining old values.
  Fills the new elements with 0.0 except in
  the main diagonal, where the provided value
  will be inserted.
  The 'err' value in matrix will be set to -1 if
  there is no more space to increase the matrix. */
  bool expand(const unsigned int iRows,
                 const unsigned int iCols,
                 const double iVal);

  /**
  Normalize homogeneous vector, so that last element is 1. */
  bool normalize();
  /**
  Gets one value in a vector.
  Position is zero based, i.e. get(0) reads first element. */
  inline double get(const unsigned int at)
  {
    return mat.data.db[at];
  };
  /**
  Set one value in a vector.
  Position is zero based, i.e. set(0, 1.0) sets first element to 1.0. <br>
  returns 0 if within range */
  inline bool setAt(const unsigned int at, const double value)
  {
    bool result = (at < size());
    if (result)
      mat.data.db[at] = value;
    return result;
  };
  /**
  Set up to 4 values in a vector. <br>
  Sets values within vector size only. <br>
  Returns 0. */
  inline void set(const double value0, const double value1,
          const double value2 = 0.0, const double value3 = 0.0)
  { // there should always be at least 4 elements
      mat.data.db[0] = value0;
      mat.data.db[1] = value1;
      mat.data.db[2] = value2;
      mat.data.db[3] = value3;
  }
  /**
  Get the size of the actual vector.
  More space may be allocated - see maxSize(). */
  inline unsigned int size() { return mat.rows * mat.cols;};
  /**
  is vector a row or a column */
  inline bool isRow() { return cols() == 1;};
  /**
  is vector a row or a column */
  inline bool isCol() { return rows() == 1;};
  /**
  Resize this matrix as required, and if the size is increased
  the new elements are filled i along the main diagonal and
  other new elements are set to zero. */
  /*
  void resize(const unsigned int rows,
          const unsigned int cols,
          const double i);
  */
  /**
  Make this matrix the matrix product og A and B.
  this = A * B. */
  int mult(UMatrix * A, UMatrix * B);
  /**
  Scale all elements in this matrix with scale.
  this *= scale */
  void mult(const double scale);
  /**
  Make this matrix the sum of matrix A and B.
  this = A + B. */
  void add(UMatrix * A, UMatrix * B);
  /**
  Add the matrix A to this matrix */
  void add(UMatrix * A);
  /**
  Make this matrix the element difference of the matrix A and B.
  this = A - B. */
  void sub(UMatrix * A, UMatrix * B);
  /**
  Subtract the A matrix from this matrix. */
  void sub(UMatrix * A);
  /**
  Add val to all elements in this matrix. */
  void add(const double val);
  /**
  Make a mionor matrix as copy of 'mA' by removing
  row 'ar' and column 'ac'.
  Returns -1 if error in source or calculation. */
  int setMinor(UMatrix * mA,
               const unsigned int ar ,
               const unsigned int ac);
  /**
  Transpose. Make this matrix a transposed version of B. */
  void transpose(UMatrix * B = NULL);
  /**
  Transpose this matrix. */
  //void transpose();
  /**
  Inverse. Make this matrix an inversed copy of B.
  The method ishould be CV_SVD for normal matrix
  and CV_SYM_SVD for symmetric matrices.
  These methods do not use temporary dynmically allocated matricec.
  The method CV_LU use gaussian elimination is also supported */
  void inverse(UMatrix * B, int method = CV_SVD);
  /**
  Solve the set of equation in this matrix
  with the unknowns in result vector X beeing equal to
  the iB vector. this * X = iB.
  Returns true if result in X is valid (matrix not singular). */
  bool solve(UMatrix * iB, UMatrix * X);
  /**
  Copy a matrix. */
  void copy(UMatrix * source);
  /**
  Get a matrix element. */
  inline double get(int row, int col)
  { // get cell value
#ifndef MATRIX_NO_RANGE_CHECK
    if ((row < rows) and (col < cols) and (row >= 0) and (col >=0))
#endif
      return cvGetReal2D(&mat, row, col);
#ifndef MATRIX_NO_RANGE_CHECK
    else
    { // set error
      error(-1, "UMatrix::get: index error");
      return (0.0);
    }
#endif
  };
  /**
  Get forst element in a row, for dynamic access to column elements */
  inline double * getRow(int row)
     { return (double *)cvPtr2D(&mat, row, 0); };
  /**
  Set a matrix element. */
  inline int setRC(const unsigned int row,
                 const unsigned int col,
                 const double value)
  { // set cell value
#ifndef MATRIX_NO_RANGE_CHECK
    if ((row < rows) and (col < cols) and (row >= 0) and (col >=0))
      // index
#endif
      cvSetReal2D(&mat, row , col, value);
#ifndef MATRIX_NO_RANGE_CHECK
    else
      // set error
      error(-1, "UMatrix::set: index error");
#endif
    return err;
  };
  /**
  Get pointer to data area (first element) */
  inline double * getData() { return mat.data.db;};
  /**
  Is matrix result valid. same as (err == 0) */
  inline bool valid() { return (err == 0); };
  /**
  Make matrix as product of two vectors. */
  //int set(UVector * v1, UVector * v2);
  /**
  Make outher product of these vectors */
  //inline int set(UVector v1, UVector v2)
  //    { return set(&v1, &v2);};
  /**
  Set up to first 4 elements in a row. */
  int setRow(unsigned int row, const double value0, const double value1 = 0.0,
                   const double value2 = 0.0, const double value3 = 0.0);
  /**
  Set matrix row from array of doubles.
  \param row is the row to be set (0 based)
  \param valCnt is the number of values to be set (no more than number of columns)
  \param val is the array of values.
  \returns err=0 */
  int setRow(unsigned int row, unsigned int valCnt,  const double val[]);
  /**
  Set up to first 4 elements in a column. */
  int setCol(unsigned int col, const double value0, const double value1 = 0.0,
             const double value2 = 0.0, const double value3 = 0.0);
  /**
  Set matrix diagnal to these values.
  Other elements in matrix are untouched */
  int setDiag(double value0, double value1,
              double value2 = 0.0, double value3 = 0.0);
  /**
  Set matrix values from an array of double values.
  \param rCnt is the number of rows in matrix
  \param cCnt is number of columns.
  \param values is an array of size (rCnt,cCnt) with matrix values. first row first.
  \returns true if space for array values. */
  bool setMat(const int rCnt, const int cCnt, double * values);
  /**
  Get sum of diagonal elements */
  double trace();
  /**
  Get squared sum of all elements.
  Especially usefull for vectors, there the result
  is the vector distance squared, or the dot product by itself */
  double sqSum();
  /**
  Print matrix values to console in float format */
  void print(const char * prestring);
  /**
  Copy elements to string in human readable format */
  virtual void snprint(const char * prestring, char * buff, const int buffCnt);
  /**
  Copy elements to string s (Maple format). */
  virtual void snprintMaple(const char * prestring, char * s,
                        const int length);
  /**
  save to file in textfile format for MATLAB etc.
  with numbers in '-1.988e-12' type format with
  delimitor between columns as sepcified in DELIM, with
  default as space. <br>
  Returns 0 if successfull, else -1. */
  int save(char * filename, char delim = ' ');
  /**
  Save matrix in matlab format *.m with this variable name. */
  bool save(FILE * f, const char * name);
  /**
  Set err value and show error message to console. */
  void error(const int ierr, const char * message);
  /**
  Find determinant if matrix is size 1x1, 2x2, 3x3
  \param err set to == 0 if sucessfull or
  \param err set to = -1 if matrix is too big
   * \returns determinant */
  double det(int * error);
  /**
  Calculates singular value decomposition (SVD) of this matrix (MxN with M >= N).
  So that:
     source = U * W * V';
  The result is placed in W (diagonal if W is (NxN)
  or as vector if W is (Nx1). Also U and V can be calculated,
  columns in U (MxN) and rows in V (NxN) (or may be NULL if not needed).
  The matrix size of W, U and V must be set before call.
  The flag may be either 0 or sum of: <br>
    CV_SVD_MODIFY_A enables modification of matrix src1 during the operation. It speeds up the processing. <br>
    CV_SVD_U_T means that the tranposed matrix U is returned. Specifying the flag speeds up the processing. <br>
    CV_SVD_V_T means that the tranposed matrix V is returned. Specifying the flag speeds up the processing. <br>
  Preferred value is CV_SVD_MODIFY_A + CV_SVD_V_T. but
  default is just CV_SVD_V_T (a bit slower, but 'this' matrix is left as is). <br>
  Eigenvalue note:<br>
  If source matrix is symmetric the eigenvalues in W agree with matlab, otherwise it seems just to be some
  vector (diagonal) that can be used in back-transformation using the found U and V matrices.
  U and V are equal if source matrix is symmetric.
  */
  void eig(UMatrix * W, UMatrix * U, UMatrix * V, int flag = CV_SVD_V_T);
  /**
  Return number of rows in array */
  inline unsigned int rows() { return mat.rows;};
  /**
  Return number of rows in array */
  inline unsigned int cols() { return mat.cols;};
  /**
  Return maximum number of elements available in
  array data area (rows * columns).
  Type is always double. */
  inline int elements() { return matrixSize;} ;
  /**
  Return maximum number of elements available in
  array data area (rows x columns).
  Type is always double. */
  inline int maxSize() { return matrixSize;} ;
  /**
  Get handle to opencv array structure.
  This can be used for all CvArr type arguments. */
  inline CvArr * cvArr() { return &mat;};
  /**
  Get handle to opencv array structure.
  This can be used for all CvMat type arguments. */
  inline CvMat * cvMat() { return &mat;};
  //
public:
  /**
  Error, set to -1 on matrix operation error */
  int err;
protected:
  /**
  Number of elements in matrix buffer, i.e. may be larger than rows X columns. */
  unsigned int matrixSize; // in elements
  /**
  Actual matrix data area in opencv-format */
  CvMat mat;
};

/////////////////////////////////////////////////////////////


/**
Class for 3d matrices with up to 4 rows and 4 columns.
Actual size may be smaller but not bigger than this size.
The memory allocation is static in the class members, so no
heap space is used. */
class UMatrix4 : public UMatrix
{
public:
  /**
  Constructor of a matrix of this type with size (0,0). */
  UMatrix4();
  /**
  Constructor of a matrix of this size */
  UMatrix4(const unsigned int rows,
          const unsigned int cols);
  /**
  Constructor of a vector with one row of this size */
  UMatrix4(const unsigned int cols);
  /**
  Constructor of matrix with this size and initial value i
  along the main diagonal (and other elements zero).*/
  UMatrix4(const unsigned int rows,
          const unsigned int cols,
          const double i);
  /**
  Expand current matrix to new size maintaining
  current content in place.
  New positions will be zero filled except
  main diagonal, that will be filled with 'iVal'.
  The size can also be smaller than original size. */
  /*
  void expand(const unsigned int iRows,
                 const unsigned int iCols,
                 const double iVal);
  */
  /**
  Returns a copy of V as this, with pointers corrected.
  result (this) = V. */
  UMatrix4 operator= (UMatrix4 V);
  /**
  Returns a vector that is the produvt of this and the vextor V.
  result = this * V.*/
  //U4 operator* (UVector4 V);
  /**
  Returns a matrix that is the matrix product of this and B.
  result = this * B.*/
  UMatrix4 operator* (UMatrix4 B);
  /**
  Returns a matrix tith all elements are from this matrix multiplied with scale.
  result = this * scale. */
  UMatrix4 operator* (const double scale);
  /**
  Multiply a position with this matrix
  and obtain a new (rotated translated) position */
  UPosition operator* (UPosition pos);
  /**
  Returns a transposed copy of this. */
  UMatrix4 transposed();
  /**
  Returns a transposed copy of this. */
  UMatrix4 inversed();
  /**
  Returns the sum of this and B. */
  UMatrix4 operator+ (UMatrix4 B);
  /**
  Returns a this matrix with all elements added the value val.
  result = this + val */
  UMatrix4 operator+ (const double val);
  /**
  Returns the element difference of this and B.
  result = this - B. */
  UMatrix4 operator- (UMatrix4 B);
  /**
  Returns a this matrix with all elements subtracted the value val.
  result = this - val */
  UMatrix4 operator- (const double val);
  /**
  Solve a equation set.
  solve matrix (this) x vector (result) = vector (B) */
  //UMatrix4 solve(UMatrix4 * B);
  /**
  Find determinant of matrix */
  double det(int * error = NULL);
  /**
  Get row as vector */
  UMatrix4 row(int r);
  /**
  Get collumn as vector */
  UMatrix4 col(int c);
  /**
  Find density value for a vector position 'v', when
  this matrix is assumed to be a covariance matrix. <br>
  The mean is in vector vm.*/
  double density(UMatrix4 v, UMatrix4 vm, bool * isOK = NULL);
  /**
  Find density function at point 'v', for a distribution
  with 'this' as inverse covariance matrix and
  determinant of covariance matrix of 'det'.
  Mean value of distribution is assumed to be zero. <br>
  On return is 'isOK' false if function failed. */
  double densityQi(UMatrix4 vd, double det, bool * isOK = NULL);
  /**
  Find eigenvalues and eigenvectors for 2x2 matrix.
  If matrix A is larger than 2x2, then solution is found
  for the 2x2 formed by the first 2 rows and columns.
  If roots are complex, the 'complex' flag is set true and
  the returned eigenvalues and vectors are the real part only.
  The eigenvalues are returned in a vector (size 2) and
  the eigenvectors are returned in 'eigenvectors' as columns
  corresponding to the elements in this vector of eigenvalues. */
  UMatrix4 eig2x2(bool * complex, UMatrix4 * eigenvectors);

private:
  /**
  Reconnect matrix pointer to data area */
  void connect();


private:
  /**
  Data area for matrix of max size 4x4 */
  double d[MAX_VECTOR4_SIZE * MAX_VECTOR4_SIZE];
};

///////////////////////////////////////////////////////

/**
Big matrix with up to 40 rows and 40 columns.
Space for elements are in local, i.e. not on heap (memalloc()). */
class UMatrixBig : public UMatrix
{
//private:
  //double d[MAX_BIG_VECTOR_SIZE * MAX_BIG_VECTOR_SIZE];
public:
  /**
  Constructor of a matrix of this size */
  UMatrixBig(const unsigned int rows,
          const unsigned int cols,
          double * data = NULL,
          unsigned int elements = 0);
  /**
  Constructor of a vector with one row */
  UMatrixBig(const unsigned int cols,
              double * data = NULL,
              unsigned int elements = 0);
  /**
  Constructor of matrix with this size and initial value i
  along the main diagonal (and other elements zero).*/
  UMatrixBig(const unsigned int rows,
          const unsigned int cols,
          const double i,
          double * data = NULL,
          unsigned int elements = 0);
  /**
  Destructor deallocated allocated memory. */
  ~UMatrixBig();
  /**
  Returns a copy of V as this, with pointers corrected.
  result (this) = V. */
  UMatrixBig operator= (UMatrixBig V);
  /**
  Solve a equation set.
  solve matrix (this) x vector (result) = vector (B) */
//  bool solve(UVectorBig * B, UVectorBig * X);
  /**
  Calculate determinant .*/
  double det(int * error);
};

#endif
