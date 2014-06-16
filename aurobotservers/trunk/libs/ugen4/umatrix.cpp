/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
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

//#include <iostream.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "umatrix.h"
#include "ucommon.h"
#include "u3d.h"


/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////
// UMatrix

bool UMatrix::isAlsoA(const char * typeString)
{
  bool result;
  result = (strcmp(UMatrix::getDataType(), typeString) == 0);
  if (not result)
      // ask the ancestor if type is known
    result = UDataBase::isAlsoA(typeString);
  return result;
}

///////////////////////////////////////////////////////

bool UMatrix::init(const unsigned int iRows,
                 const unsigned int iCols)
{
  if ((iRows * iCols) <= matrixSize)
  {
    cvInitMatHeader(&mat, iRows, iCols,
                    CV_64FC1, mat.data.db);
    // mat.rows = iRows;
    // mat.cols = iCols;
    err = 0;
  }
  else
  {
//    mat.rows = 0;
//    mat.cols = 0;
    error(-1, "Matrix init error - size too big");
  }
  return (err == 0);
}

//////////////////////////////////////////////////

bool UMatrix::expand(const unsigned int iRows,
                 const unsigned int iCols)
{ // resize and fill increased new cells acording to iVal along
  // the main diagonal and other cells = 0.0.
  int r, rowLng, rLast;
  unsigned int fr = rows(); // old row count
  unsigned int fc = cols(); // old column count
  //UMatrix4 a;
  double * s;
  double * d;
  if ((iRows * iCols) > matrixSize)
    error(-1, "Matrix resize error - size too big");
  else
  { //change size
    setSize(iRows, iCols);
    if (fc != cols())
    { // new row length, so rows from
      // second to last has to be moved
      rowLng = mini(fc, cols()) * sizeof(double);
      // last row to move
      rLast = mini(fr, rows()) - 1;
      if (cols() > fc)
      { // more columns, so must be
        // moved from back end
        for (r = rLast; r > 0; r--)
        { // get old start of row
          s = &getData()[r * fc];
          // get new start of row
          d = getRow(r);
          // move row
          memmove(d, s, rowLng);
        }
      }
      else
      { // fewer columns, so moved in increasing order
        for (r = 1; r <= rLast; r++)
        { // get old start of row
          s = &getData()[r * fc];
          // get new start of row
          d = getRow(r);
          // move row
          memmove(d, s, rowLng);
        }
      }
    }
  }
  return (err == 0);
}

//////////////////////////////////////////////////

bool UMatrix::expand(const unsigned int iRows,
                 const unsigned int iCols,
                 const double iVal)
{ // resize and fill increased new cells acording to iVal along
  // the main diagonal and other cells = 0.0.
  unsigned int r,c, r0, c0;
  unsigned int fr = rows(); // old row count
  unsigned int fc = cols(); // old column count
  double * d;
  // expand and move present row elements
  expand(iRows, iCols);
  if (err == 0)
  { //fill new rows and column elements with new initial value
    if (fc < cols())
      // more columns, so start from row 0
      r0 = 0;
    else
      // fewer or same number of columns
      // so just fill new rows
      r0 = fr;
    for (r = r0; r < rows(); r++)
    {
      if (r < fr)
        c0 = fc;
      else
        c0 = 0;
      // get pointer to first element
      d = &getRow(r)[c0];
      for (c = c0; c < cols(); c++)
      { // insert values
        if (r == c)
          *d = iVal;
        else
          *d = 0.0;
        d++;
      }
    }
  }
  return (err == 0);
}

/////////////////////////////////////////////////////////

void UMatrix::clear()
{
  double * d;
  unsigned int i;
  //
  d = getData();
  for (i = 0; i < size(); i++)
    *d++ = 0.0;
  err = 0;
}

//////////////////////////////////////////////////

bool UMatrix::init(const unsigned int iRows,
                 const unsigned int iCols,
                 const double i)
{ // set this size and fill with initial value i
  // along the main diagonal. and zero the rest.
  //
  if ((iRows * iCols) <= matrixSize)
  {
    cvInitMatHeader(&mat, iRows, iCols,
                    CV_64FC1, mat.data.db);
    err = 0;
    cvSetIdentity(&mat, cvScalar(i));
  }
  else
    error(-1, "UMatrix::init(rows,cols,i): - size too big");
  return (err == 0);
}

//////////////////////////////////////////////////

int UMatrix::mult(UMatrix * A, UMatrix * B)
{ // multiplies A * B and puts result in this
  // this = A * B
  //
  bool result;
  //
  result = (A->cols() == B->rows());
  if (result)
    result = setSize(A->rows(), B->cols());
  else
    err = -1;
  if (result)
    cvMatMul(&A->mat, &B->mat, &mat);
  return result;
}

//////////////////////////////////////////////////

bool UMatrix::normalize()
{ // normalize a homogeneous coordinate
  // so that the last element is 1
  unsigned int i;
  double * d;
  double w;
  //
  if ((isRow() or isCol()) and (size() > 1))
  {
    d = getData();
    w = d[size() - 1];
    if (absd(w) > 1e-90)
    {
      for (i = 0; i < size() - 1; i++)
        d[i] = d[i] / w;
      d[size() - 1] = 1.0;
    }
    else
      error(-1, "UVector::normalize error w == 0");
  }
  else
    error(-1, "UVector::normalize: matrix size error");
  return (err == 0);
}

////////////////////////////////////////////////////

void UMatrix::add(UMatrix * A, UMatrix * B)
{ // make this matrix the the addition of hese:
  // this = A + B
  unsigned int r;
  double * sa;
  double * sb;
  double * d;
  if ((A->rows() == B->rows()) and
      (A->cols() == B->cols()))
  { // set result matrix
    setSize(A->rows(), A->cols());
    sa = A->getData();
    sb = B->getData();
    d = getData();
    for (r = 0; r < size(); r++)
      *d++ = *sa++ + *sb++;
  }
  else
    error(-1, "UMatrix::add(A,B) - matrix bad sized");
}

//////////////////////////////////////////////////

void UMatrix::add(UMatrix * A)
{ // add Matrix A to this matrix:
  // this += A
  unsigned int r;
  double * s; // source
  double * d; // destination
  if ((A->rows() == rows()) and
      (A->cols() == cols()))
  { // set result matrix
    d = getData();
    s = A->getData();
    for (r = 0; r < size(); r++)
      *d++ += *s++;
  }
  else
    error(-1, "UMatrix::add(A) - matrix bad sized");
}

//////////////////////////////////////////////////

void UMatrix::sub(UMatrix * A, UMatrix * B)
{ // subtract these two matrices and make this matrix the result
  // this = A - B
  unsigned int r;
  double * sa;
  double * sb;
  double * d;
  if ((A->rows() == B->rows()) and
      (A->cols() == B->cols()))
  { // set result matrix
    setSize(A->rows(), A->cols());
    sa = A->getData();
    sb = B->getData();
    d = getData();
    for (r = 0; r < size(); r++)
      *d++ = *sa++ - *sb++;
  }
  else
    error(-1, "UMatrix::sub(A,B) - matrix bad sized");
}

//////////////////////////////////////////////////

void UMatrix::sub(UMatrix * A)
{ // make this matrix the the subtraction of hese:
  // this -= A
  unsigned int r;
  double * s; // source
  double * d; // destination
  if ((A->rows() == rows()) and
      (A->cols() == cols()))
  { // set result matrix
    d = getData();
    s = A->getData();
    for (r = 0; r < size(); r++)
      *d++ -= *s++;
  }
  else
    error(-1, "UMatrix::add(A) - matrix bad sized");
}

//////////////////////////////////////////////////

void UMatrix::add(const double val)
{ // add a scalar to this matrix
  unsigned int r,c;
  double * v;
  //
  for (r=0; r < rows(); r++)
  {
    v = getRow(r);
    for (c=0; c < cols(); c++)
      *v++ += val;
      //set(r, c, get(r, c) + val);
  }
}


//////////////////////////////////////////////////

void UMatrix::transpose(UMatrix * B)
{ // transpose this matrix from B     d[0 * 4 + 2]
  unsigned int r,c;
  if ((B == this) or (B == NULL))
  { // need a buffer for rersult
    if (isRow() or isCol())
      setSize(cols(), rows());
    else
      error(-1, "UMatrix::transpose: non-vector, then source and destination must be different");
  }
  else
  {
    if (B->isRow() or B->isCol())
    {
      this->copy(B);
      setSize(B->cols(), B->rows());
    }
    else
    {
      setSize(B->cols(), B->rows());
      for (r = 0; r < B->cols(); r++)
        for (c = 0; c < B->rows(); c++)
          setRC(r, c, B->get(c, r));
    }
  }
}

//////////////////////////////////////////////////

void UMatrix::inverse(UMatrix * B, int method /*= CV_SVD*/)
{
  cvInvert(&B->mat, &mat, method);
}

//////////////////////////////////////////////////

bool UMatrix::solve(UMatrix * B, UMatrix * X)
{
  return cvSolve(cvArr(), B->cvArr(), X->cvArr(), CV_SVD );
}

//////////////////////////////////////////////////

void UMatrix::mult(const double scale)
{ // scale multiply
  unsigned int r;
  double * v;
  //
  v = getData();
  for (r = 0; r < size(); r++)
    *v++ *= scale;
}

//////////////////////////////////////////////////

void UMatrix::copy(UMatrix * source)
{
  unsigned int r,c;
  if (source->rows() * source->cols() <= matrixSize)
  {
    setSize(source->rows(), source->cols());
    err = source->err;
    for (r = 0; r < rows(); r++)
      for (c = 0; c < cols(); c++)
        setRC(r, c, source->get(r, c));
  }
  else
     error(-1, "UMatrix::copy:: index error");
}

//////////////////////////////////////////////////
/*
int UMatrix:: set(UVector * v1, UVector * v2)
{ // make matrix as product of these two vectors
  unsigned int r, c;
  double * row;
  // accumulate source errors
  err = v1->err + v2->err;
#ifndef MATRIX_NO_RANGE_CHECK
  if ((err == 0) and ((v1->size > MaxMatrixSize) or (v2->size > MaxMatrixSize)))
    err = -1;
#endif
  if (err == 0)
  { // set matrix size
    setSize(v1->size(), v2->size());
    for (r = 0; r < rows(); r++)
    { // get pointer to first element in row
      row = getRow(r);
      for (c = 0; c < cols(); c++)
        row[c] = v1->get(r) * v2->get(c);
    }
  }
  return err;
}
*/
//////////////////////////////////////////////////

int UMatrix::setMinor(UMatrix * mA,
                 const unsigned int ar ,
                 const unsigned int ac)
{ // make the minor matrix of source matrix 'mA' by deleting
  // row 'ar' and column 'ac'
  unsigned int r,c;
  double * rowS, *rowD;
  err = mA->err;
#ifndef MATRIX_NO_RANGE_CHECK
  if ((err == 0) and ((ar >= mA->rows) or (ac >= mA->cols) or
                      (mA->rows < 2) or (mA->cols < 2)))
    err = -1;
#endif
  if (err == 0)
  {
    setSize(mA->rows() - 1, mA->cols() - 1);
    for (r = 0; r < rows(); r++)
    {
      rowD = getRow(r);
      if (r < ar)
        rowS = mA->getRow(r);
      else
        rowS = mA->getRow(r+1);
      for (c = 0; c < cols(); c++)
        if (c < ac)
          rowD[c] = rowS[c];
        else
          rowD[c] = rowS[c+1];
    }
  }
  return err;
}

//////////////////////////////////////////////////

void UMatrix::eig(UMatrix * W, UMatrix * U, UMatrix * V, int flag /* = CV_SVD_V_T */)
{
  if ((cols() == W->cols()) or
      (cols() == W->rows() and W->cols() == 1))
    cvSVD(&mat, &W->mat, &U->mat, &V->mat, flag);
  else
    error(-1, "UMatrix(MxN)::eig(W(NxN or Nx1), U, V): matrix size error");
}

//////////////////////////////////////////////////

double UMatrix::det(int * error)
{ // calculate determinant
  double result = 0.0;
  result = cvDet(&mat);
  //
  return result;
}

//////////////////////////////////////////////////

int UMatrix::setRow(unsigned int row,  const double value0,  const double value1,
                    const double value2 ,  const double value3 /*= 0.0*/)
{ // set up to 4 elements in a row
  double * val;
  //
  val = getRow(row);
  if (row < rows())
  { // set all valid columns in this row
    if (cols() > 0)
      *val++ = value0;
    if (cols() > 1)
      *val++ = value1;
    if (cols() > 2)
      *val++ = value2;
    if (cols() > 3)
      *val++ = value3;
  }
  return err;
}

//////////////////////////////////////////////////

int UMatrix::setRow(unsigned int row, unsigned int valCnt,  const double val[])
{ // set up to 4 elements in a row
  double * dst;
  int m = mini(valCnt, cols());
  if (m > 0)
  {
    dst = getRow(row);
    memmove(dst, val, m * sizeof(double));
    err = 0;
  }
  return err;
}

//////////////////////////////////////////////////

int UMatrix::setCol(unsigned int col,  const double value0,  const double value1,
                    const double value2 ,  const double value3 /*= 0.0*/)
{ // set up to 4 elements in a row
  //
  if (col < cols())
  { // set all valid columns in this row
    if (rows() > 0)
      setRC(0, col, value0);
    if (rows() > 1)
      setRC(1, col, value1);
    if (rows() > 2)
      setRC(2, col, value2);
    if (rows() > 3)
      setRC(3, col, value3);
  }
  return err;
}

//////////////////////////////////////////////////

int UMatrix::setDiag(const double value0,
                     const double value1,
                     const double value2,
                     const double value3)
{
  setRC(0, 0, value0);
  if ((rows() > 1) and (cols() > 1))
    setRC(1, 1, value1);
  if ((rows() > 2) and (cols() > 2))
    setRC(2, 2, value2);
  if ((rows() > 3) and (cols() > 3))
    setRC(3, 3, value3);
  return true;
}

//////////////////////////////////////////////////

double UMatrix::trace()
{
  int rc = mini(rows(), cols());
  int i;
  double result = 0.0;
  for (i = 0; i < rc; i++)
    result += get(i,i);
  return result;
}

//////////////////////////////////////////////////

void UMatrix::print(const char * prestring)
{ // send matrix to console
  unsigned int r,c;
  printf("%s:\n", prestring);
  if (err != 0)
    printf("Error: %d != 0\n", err);
  for (r = 0; r < rows(); r++)
  {
    printf("Row %d: %f", r, get(r,0));
    for (c = 1; c < cols(); c++)
      printf(", %f", get(r, c));
    printf("\n");
  }
}

//////////////////////////////////////////////////

void UMatrix::snprint(const char * prestring, char * buff, const int buffCnt)
{ // send matrix to console
  unsigned int r,c;
  char * p1;
  int n;
  //
  snprintf(buff, buffCnt, "%s:\n", prestring);
  n = strlen(buff);
  p1 = &buff[n];
  if (err != 0)
    snprintf(p1, n, "Error: %d != 0\n", err);
  for (r = 0; r < rows(); r++)
  {
    n += strlen(p1);
    p1 = &buff[n];
    snprintf(p1, n, "Row %d: %f", r, get(r,0));
    for (c = 1; c < cols(); c++)
    {
      n += strlen(p1);
      p1 = &buff[n];
      snprintf(p1, n, ", %g", get(r, c));
    }
    n += strlen(p1);
    p1 = &buff[n];
    snprintf(p1, n, "\n");
  }
}

//////////////////////////////////////////////////

void UMatrix::snprintMaple(const char * prestring,
                      char * s,
                      const int length)
{ // send matrix to string
  // always in Maple LinearAlgebra format
  unsigned int r,c;
  char s2[20];
  //
  strncpy(s, prestring, length);
  for (r = 0; r < rows(); r++)
  {
    if (r == 0)
      strcat(s, "<<");
    for (c = 0; c < cols(); c++)
      {
      if (c == 0)
        snprintf(s2, 20, "%e", get(r, c));
      else
        snprintf(s2, 20, "|%e", get(r, c));
      if (int(strlen(s)) < length - 14)
        strcat(s, s2);
      else
        break;
      }
    if (int(strlen(s)) < length - 14)
    {
      if (r < (cols() - 1))
        strcat(s, ">,<");
      else
        strcat(s, ">>;");
    }
    else
      break;
  }
}

//////////////////////////////////////////////////

int UMatrix::save(char * filename, char delim /*= ' '*/)
{ // save to file in e format for MATLAB (etc)
  // delimited with character 'delim'
  FILE * f;
  unsigned int r = 0, c;
  int result = -1;
  //
  if (err == 0)
  {
    f = fopen(filename, "w");
    if (f != NULL)
    { // file is open, now write elements
      for (r = 0; r < rows(); r++)
      {
        for (c = 0; c < (cols() - 1); c++)
          fprintf(f, "%1.15e%c", get(r, c), delim);
        fprintf(f, "%1.15e\n", get(r, cols() - 1));
      }
      fclose(f);
      result = 0;
    }
  }
  return err + result;
}

////////////////////////////////////////////////////

bool UMatrix::save(FILE * f, const char * name)
{ // save in matlab format with this variable name
  bool result = (f != NULL) and (err == 0);
  unsigned int r, c;
  //
  fprintf(f, "\n%s = zeros(%d, %d);\n", name, rows(), cols());
  for (r = 0; r < rows(); r++)
  {
    fprintf(f, "%s(%d,:) = [", name, r+1);
    for (c = 0; c < cols(); c++)
    {
      fprintf(f,"%e ", get(r,c));
    }
    fprintf(f, "];\n");
  }
  //
  return result;
}


/////////////////////////////////////////////

void UMatrix::error(const int ierr = -1, const char * message = "")
{ // show error
   err = ierr;
   printf("%s\n", message);
}



//////////////////////////////////////////////////////

double UMatrix::sqSum()
{
  int i;
  int n = rows() * cols();
  double result = 0.0;
  double * v = getData();
  //
  for (i = 0; i < n; i++)
    result += sqr(*v);
  //
  return result;
}

////////////////////////////end

bool UMatrix::setMat(const int rCnt, const int cCnt, double * values)
{
  if (setSize(rCnt, cCnt))
    memcpy(getData(), values, rCnt * cCnt * sizeof(double));
  return err == 0;
}


//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
// UMatrix4

UMatrix4::UMatrix4()
{
  cvInitMatHeader( &mat, 3, 3, CV_64FC1, d );
  //m = d;
  err = 0;
  //rows = 0;
  //cols = 0;
  matrixSize = MAX_VECTOR4_SIZE * MAX_VECTOR4_SIZE;
}

/////////////////////////////////////////////

UMatrix4::UMatrix4(const unsigned int iRows,
                 const unsigned int iCols)
{ // matrix
  cvInitMatHeader( &mat, iRows, iCols, CV_64FC1, d );
  err = 0;
  matrixSize = MAX_VECTOR4_SIZE * MAX_VECTOR4_SIZE;
  //
  if ((rows() * cols()) > matrixSize)
    error(-1, "UMatrix4::UMatrix4: Matrix size too big");
}

/////////////////////////////////////////////

UMatrix4::UMatrix4(const unsigned int iCols)
{ // vector
  cvInitMatHeader( &mat, 1, iCols, CV_64FC1, d );
  err = 0;
  matrixSize = MAX_VECTOR4_SIZE * MAX_VECTOR4_SIZE;
  //
  if (cols() > matrixSize)
    error(-1, "UMatrix4::UMatrix4: Matrix size too big");
}

///////////////////////////////////////////////

UMatrix4::UMatrix4(const unsigned int iRows,
                 const unsigned int iCols,
                 const double i)
{ // connect pointers
  if (iRows * iCols > 0)
  {
    cvInitMatHeader( &mat, iRows, iCols, CV_64FC1, d );
    err = 0;
    matrixSize = MAX_VECTOR4_SIZE * MAX_VECTOR4_SIZE;
    // init the rest
    init(iRows, iCols, i);
  }
  else
  {
    printf("UMatrix4::UMatrix4: incalid size of matrix (%ud x %ud)\n", iRows, iCols);
  }
}

//////////////////////////////////////////////////

void UMatrix4::connect()
{ // reconnect pointer to matrix data area
  cvSetData(&mat, d, cols() * sizeof(double));
}

//////////////////////////////////////////////////


UMatrix4 UMatrix4::operator= (UMatrix4 source)
{ // reconnect pointers
  source.connect();
  memcpy(this, &source, sizeof(source));
  connect();
  // copy data
  // copy(&source);
  return *this;
}

//////////////////////////////////////////////////
/*
UVector4 UMatrix4::operator* (UVector4 V) {
  UVector4 result(rows());
  // reconnect pointers
  V.connect();
  // multiply
  result.mult(this, &V);
  return result;
}
*/
//////////////////////////////////////////////////

UMatrix4 UMatrix4::operator* (UMatrix4 B)
{ // make result matrix
  UMatrix4 result(rows(), B.cols());
  // reconnect pointers
  B.connect();
  connect();
  // add
  result.mult(this, &B);
  return result;
}

//////////////////////////////////////////////////

UMatrix4 UMatrix4::operator* (const double scale)
{ // copy this value
  UMatrix4 result(*this);
  // reconnect pointers
  result.connect();
  connect();
  // multiply
  result.mult(scale);
  return result;
}

//////////////////////////////////////////////////

UPosition UMatrix4::operator* (UPosition pos)
{ // copy this value
  UPosition result;
  // reconnect pointers
  connect();
  // multiply
  result = pos.transferred(this);
  //
  return result;
}

//////////////////////////////////////////////////

UMatrix4 UMatrix4::transposed()
{ // returns a transposed copy of this
  UMatrix4 result(cols(), rows());
  result.transpose(this);
  return result;
}

//////////////////////////////////////////////////

UMatrix4 UMatrix4::inversed()
{ // returns an inversed copy of this
  UMatrix4 result(rows(), cols());
  result.inverse(this);
  return result;
}

//////////////////////////////////////////////////

UMatrix4 UMatrix4::operator+ (UMatrix4 B)
{ // make result matrix
  UMatrix4 result(rows(), cols());
  // reconnect pointers
  result.connect();
  B.connect();
  connect();
  // add
  result.add(this, &B);
  return result;
}

//////////////////////////////////////////////////

UMatrix4 UMatrix4::operator+ (const double val)
{ // copy this value
  UMatrix4 result(*this);
  // reconnect pointers
  result.connect();
  // add
  result.add(val);
  return result;
}

//////////////////////////////////////////////////

UMatrix4 UMatrix4::operator- (UMatrix4 B)
{ // make result matrix
  UMatrix4 result(rows(), cols());
  // reconnect pointers
  connect();
  B.connect();
  // subtract
  result.sub(this, &B);
  return result;
}

//////////////////////////////////////////////////

UMatrix4 UMatrix4::operator- (const double val)
{ // copy this value
  UMatrix4 result(*this);
  // reconnect pointer
  result.connect();
  // add
  result.add(-val);
  return result;
}

//////////////////////////////////////////////////////
/*
UVector4 UMatrix4::solve(UVector4 * B)
{ // solve matrix (this) x vector (result) = vector (B)
  bool isOK;
  // make result vector
  UVector4 result(rows());
  //result.print("result pre  ");
  // make copy of B vector (is manipulated by solve)
  UVector4 equals(*B);
  equals.connect();
  //equals.print("equals ");
  // make copy of matrix (is manipulated by solve)
  // so that this matrix is not disordered
  UMatrix4 matrix(*this);
  matrix.connect();
  //matrix.print("matrix ");
  // solve on this matrix copy
  isOK = matrix.UMatrix::solve(&equals, &result);
  if (not isOK)
    result.err = -1;
  //result.print("result post ");
  return result;
}
*/
//////////////////////////////////////////////////////

double UMatrix4::det(int * error)
{
  UMatrix4 mm(3,3);
  double result = 0;
  double a0i;
  double detmm;
  unsigned int i;
  int e = 0;
  if (rows() != cols())
    e = -1 + err;
  if (e == 0)
  {
    if (rows() <= 3)
      result = UMatrix::det(error);
    else
      for (i = 0; i < cols(); i++)
      {
        mm.setMinor(this, 0, i);
        a0i = get(0,i);
        detmm = mm.det(error);
        if ((i % 2) == 0)
          result += a0i * detmm;
        else
          result -= a0i * detmm;
      }
  }
  if (error != NULL)
    *error = e;
  return result;
}

//////////////////////////////////////////////////////

UMatrix4 UMatrix4::row(int r)
{
  UMatrix4 result(1, cols());
  unsigned int c;
  //
  for (c = 0; c < cols(); c++)
    result.set(c, get(r, c));
  //
  return result;
}

//////////////////////////////////////////////////////

UMatrix4 UMatrix4::col(int c)
{
  UMatrix4 result(rows(), 1);
  unsigned int r;
  //
  for (r = 0; r < rows(); r++)
    result.set(r, get(r, c));
  //
  return result;
}

//////////////////////////////////////////////////////

double UMatrix4::density(UMatrix4 v, UMatrix4 vm, bool * isOK)
{
  double result = 0.0;
  bool OK;
  int failed;
  UMatrix4 Qi;
  UMatrix4 vd;
  double d = 0.0;
  //
  OK = (rows() == cols());
  if (OK)
  { // matrix is square, now find determinant
    d = det(&failed);
    // is determinant available and usable
    OK = (d > 1e-10) and (failed == 0);
  }
  if (OK)
  {
    Qi = inversed();
    OK = (Qi.err == 0);
  }
  if (OK)
  {
    vd = v - vm;
    result = Qi.densityQi(vd, d, isOK);
    /*
    e = -0.5 * (vd * Qi * vd);
    n = rows;
    result = exp(e) / (pow(sqrt(2 * PI), n) * sqrt(d));
    */
  }
  /*
  *isOK = OK;
  */
  return result;
}

//////////////////////////////////////////////////////

double UMatrix4::densityQi(UMatrix4 vd, double det, bool * isOK)
{
  double result = 0.0;
  bool OK;
  unsigned int n;
  double e;
  //
  OK = (rows() == cols());
  if (OK)
  {
    e = -0.5 * (vd * *this * vd.transposed()).get(0);
    n = rows();
    result = exp(e) / (pow(sqrt(2 * M_PI), n) * sqrt(det));
  }
  if (isOK != NULL)
    *isOK = OK;
  return result;
}

//////////////////////////////////////////////////////

UMatrix4 UMatrix4::eig2x2(bool * complex, UMatrix4 * eigenvectors)
{
  UMatrix4 result(2, 1, 0.0);
  bool doV = (eigenvectors != NULL); // do also eigenvectors
  double a,b,c,d;
  //
  if ((cols() < 2) or (rows() < 2))
    result.err = -1; // not big enough
  else
  { // find elements in root equation (second order)
    b = -(get(0,0) + get(1,1));
    c = get(0,0)*get(1,1) - get(0,1)*get(1,0);
    d = b*b - 4 * c;
    *complex = (d < 0.0);
    if (*complex)
    { // real part is b/2
      result.set(b/2.0, b/2.0);
    }
    else
    { // result is real so assign vales to result vector
      d = sqrt(d);
      result.set((-b+d)/2.0, (-b-d)/2.0);
    }
    if (doV and (d > 1e-10))
    { // find also eigenvectors
      // first root
      a = get(0,0) - result.get(0);
      b = get(0,1);
      if ((absf(a) < 1e-12) and (absf(b) < 1e-12))
      { // bad numerics - try other equation
        a = get(1,0);
        b = get(1,1) - result.get(0);
      }
      // printf("1st vec (d=%f), roots (a,b)=%f, %f\n", d, a, b);
      if (absf(a) > absf(b))
      { // avoid dividing by b if b is zero
        d = sqrt(1.0 + sqr(b/a));
        eigenvectors->setRC(0,0,-b/a/d);
        eigenvectors->setRC(1,0,1.0/d);
      }
      else
      { // avoid dividing by a if a is zero
        d = sqrt(1.0 + sqr(a/b));
        eigenvectors->setRC(0,0,1.0/d);
        eigenvectors->setRC(1,0,-a/b/d);
      }
      // second root
      a = get(0,0) - result.get(1);
      b = get(0,1);
      if ((absf(a) < 1e-12) and (absf(b) < 1e-12))
      { // bad numerics - try other equation
        a = get(1,0);
        b = get(1,1) - result.get(1);
      }
      // printf("2nd vec (a,b)=%f, %f\n", a, b);
      if (absf(a) > absf(b))
      {
        d = sqrt(1.0 + sqr(b/a));
        eigenvectors->setRC(0,1,-b/a/d);
        eigenvectors->setRC(1,1,1.0/d);
      }
      else
      {
        d = sqrt(1.0 + sqr(a/b));
        eigenvectors->setRC(0,1,1.0/d);
        eigenvectors->setRC(1,1,-a/b/d);
      }
    }
  }
  return result;
}
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
// UMatrixBig


UMatrixBig::UMatrixBig(const unsigned int iRows,
                 const unsigned int iCols,
                 double * data /* =NULL */,
                 unsigned int elements /*=0*/)
{ // connect pointer
  cvInitMatHeader( &mat,
               iRows, iCols,
               CV_64FC1 );
  if (data == NULL)
  { // allocate data area
    cvCreateData(&mat);
    matrixSize = iRows * iCols;
  }
  else
  {  // user data buffer
    cvSetData(&mat, data, cols() * sizeof(double));
    matrixSize = elements;
  }
  err = 0;
}

///////////////////////////////////////////////

UMatrixBig::UMatrixBig(const unsigned int iCols,
                 double * data /* =NULL */,
                 unsigned int elements /*=0*/)
{ // connect pointer
  cvInitMatHeader( &mat,
               1, iCols,
               CV_64FC1 );
  if (data == NULL)
  { // allocate data area
    cvCreateData(&mat);
    matrixSize = iCols;
  }
  else
  { // user data buffer
    cvSetData(&mat, data, cols() * sizeof(double));
    matrixSize = elements;
  }
  err = 0;
}

///////////////////////////////////////////////

UMatrixBig::UMatrixBig(const unsigned int iRows,
                 const unsigned int iCols,
                 const double i,
                 double * data /* =NULL */,
                 unsigned int elements /*=0*/)
{ // connect pointer
  cvInitMatHeader( &mat,
               iRows, iCols,
               CV_64FC1 );
  if (data == NULL)
  {  // allocate data area
    cvCreateData(&mat);
      // set local
    matrixSize = iRows * iCols;
  }
  else
  {  // user data buffer
    cvSetData(&mat, data, sizeof(double) * cols());
    matrixSize = elements;
  }
  err = 0;
  // init the rest
  init(iRows, iCols, i);
}

//////////////////////////////////////////////////////

UMatrixBig::~UMatrixBig()
{
  cvReleaseData(&mat);
}

//////////////////////////////////////////////////////
/*
bool UMatrixBig::solve(UVectorBig * B, UVectorBig * X)
{ // solve matrix (this) x vector (X) = vector (B)
  //
  // make copy of B vector (is manipulated by solve)
  UVectorBig equals(B->size());
  equals.copy(B);
  // make copy of matrix (is manipulated by solve)
  // so that this matrix is not disordered
  UMatrixBig matrix(rows(), cols());
  matrix.copy(this);
  // solve on this matrix copy
  matrix.UMatrix::solve(&equals, X);
  return X;
}
*/
//////////////////////////////////////////////////////

double UMatrixBig::det(int * error)
{
  UMatrixBig mm(3,3);
  double result = 0;
  double a0i;
  double detmm;
  unsigned int i;
  int e = 0;
  if (rows() != cols())
    e = -1 + err;
  if (e == 0)
  {
    if (rows() <= 3)
      result = UMatrix::det(error);
    else
      for (i = 0; i < cols(); i++)
      {
        mm.setMinor(this, 0, i);
        a0i = get(0,i);
        detmm = mm.det(error);
        if ((i % 2) == 0)
          result += a0i * detmm;
        else
          result -= a0i * detmm;
      }
  }
  if (error != NULL)
    *error = e;
  return result;
}



