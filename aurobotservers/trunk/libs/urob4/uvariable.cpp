/***************************************************************************
 *   Copyright (C) 2008 by DTU Christian Andersen   *
 *   chrand@mail.dk   *
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

#include <ugen4/umatrix.h>

#include "uvarpool.h"
#include "uvariable.h"
#include <iau_mat.h>

UVariable::UVariable()
{
  isDoubleBased = true;
  dValue = NULL;
  byteCnt = 0;
  elementCnt = 0;
  elementSize = sizeof(double);
  dataType = d;
  description = NULL;
  name[0] = '\0';
  parent = NULL;
  descriptionBuffer = NULL;
}

//////////////////////////////

UVariable::UVariable(UVariable * source)
{
  dValue = NULL;
  byteCnt = 0;
  elementCnt = 0;
  parent = NULL;
  descriptionBuffer = NULL;
  copy(source, true);
}

///////////////////////////////////////////

UVariable::~UVariable()
{
  if (dValue != NULL)
    free(dValue);
  dValue = NULL;
  if (descriptionBuffer != NULL)
    free(descriptionBuffer);
  descriptionBuffer = NULL;
  description = NULL;
  elementCnt = 0;
  byteCnt = 0;
}

//////////////////////////////////////////////

void UVariable::snprint(const char * preString, char * buff, const int buffCnt)
{
  char * p1 = buff;
  int n = 0;
  //
  snprintf(p1, buffCnt - n, "%s%s=\"", preString, name);
  n += strlen(p1);
  p1 = &buff[n];
  if (isString())
    snprintf(p1, buffCnt - n, "%s", (char*) dValue);
  else
    getValuesdAsString(p1, buffCnt - n, 0);
  strncat(p1, "\"\n", buffCnt - n);
}

//////////////////////////////////////////////

bool UVariable::isAlsoA(const char * typeString)
{
  bool result;
  result = (strcmp(UVariable::getDataType(), typeString) == 0);
  if (not result)
      // ask ansestor type
    result = UDataBase::isAlsoA(typeString);
  return result;
}

//////////////////////////////////////////////////

void UVariable::saveToLog(ULogFile * logFile, const char * prename, ULock * fileLock)
{
  UTime t;
  const int MTL = 5;
  char vt[MTL];
  const int MSL = 2000;
  char s[MSL];
  //
  if (logFile != NULL)
  {
    t.now();
    fileLock->lock();
    if (logFile->isOpen())
    {
      fprintf(logFile->getF(), "%lu.%06lu %s %s%s %s\n",
            t.getSec(), t.getMicrosec(), getTypeChar(vt, MTL),
            prename, name, getValuesAsStringQuoted(s, MSL, 0));
    }
    fileLock->unlock();
  }
}

/////////////////////////////////////////

double UVariable::getValued(const int idx)
{
  double result = 0.0;
  if (isDouble() and idx >= 0)
  {
    if (dataType == m2)
    {
      if (idx < elementCnt - 2)
        result = dValue[idx + 2];
    }
    else if (idx < elementCnt)
      result = dValue[idx];
  }
  return result;
}

/////////////////////////////////////////

bool UVariable::getValueBool(const int idx)
{
  if (isDouble())
    return fabs(getValued(idx)) >= 0.5;
  else
    return false;
}

/////////////////////////////////////////

int UVariable::getInt(const int idx)
{
  double d;
  if (idx == 0)
    d = getValued();
  else
    d = getValued(idx);
  return roundi(d);
}

/////////////////////////////////////////

UTime UVariable::getTime(const int idx)
{
  double d;
  UTime result;
  if (idx == 0)
    d = getValued();
  else
    d = getValued(idx);
  result.setTime(d);
  return result;
}

/////////////////////////////////////////

bool UVariable::setValued(double value, const int ix, bool mayAdd, UTime * updT)
{
  bool result;
  char * buf;
  int n, b0, idx;
  //
  if (dataType == m2)
    idx = ix + 2;
  else
    idx = ix;
  result = (idx >= 0) and isDouble();
  if (result)
  {
    if (byteCnt == 0 or (idx >= byteCnt/elementSize and mayAdd))
    {
      dValue = (double*) realloc(dValue, elementSize * (idx + 1));
      result = (dValue != NULL);
      if (result)
      {
        buf = (char*)dValue;
        n = (idx - elementCnt) * elementSize;
        if (n > 0)
        { // some elements from last used index to this new index start
          // starting at
          b0 = elementSize * elementCnt;
          // zero these elements
          bzero(&buf[b0 - 1], n);
        }
        elementCnt = idx + 1;
        byteCnt = elementSize * elementCnt;
      }
    }
    else if (idx >= elementCnt and idx < byteCnt/elementSize)
      // just a resize, there is plenty of space
      elementCnt = idx + 1;
    //
    if (result and idx < elementCnt)
    {
      dValue[idx] = value;
      setUpdated(updT);
    }
    else
    { // failed to set value
      const int MSL = 300;
      char s[MSL];
      printf("UVariable::setValued: failed to set value to element %d in %s\n",
             idx, getGetAsXmlAttribute(s, MSL));
    }
  }
  return result;
}

/////////////////////////////////////////

bool UVariable::setDouble(double * values, const int valuesCnt, UTime * updT)
{
  bool result = isDoubleBased;
  if (result)
  {
    if (valuesCnt > elementCnt)
      // make more spece if needed
      result = setValued(0.0, valuesCnt - 1, true);
    if (result)
    {
      memcpy(dValue, values, elementSize * valuesCnt);
      setUpdated(updT);
    }
  }
  return result;
}

/////////////////////////////////////////

bool UVariable::setTimeNow(const int idx)
{
  UTime t;
  t.now();
  return setValued(t.getDecSec(), idx, false, &t);
}

/////////////////////////////////////////

bool UVariable::setTime(UTime t, const int idx)
{
  return setValued(t.getDecSec(), idx, false, &t);
}

/////////////////////////////////////////

bool UVariable::setValued(const char * attValue, const int idx, bool mayAdd, UTime * updT)
{
  bool result;
  int n, row = 0, cls = -1;
  double val;
  const char * p2, *p1 = attValue, *pm = NULL;
  //
  result = (idx <= 0 or (idx > 0 and isDouble()));
  if (result and mayAdd)
  { // set type
    pm = strchr(attValue, ';');
    if (pm != NULL)
      setType(m2);
    else if (not isDouble())
      setType(d);
    // get (first) value
  }
  n = maxi(0, idx);
  if (result and p1 != NULL)
  { // potentially more than one value
    while (result and (n < elementCnt or mayAdd) and strlen(p1) > 0)
    {
      while (isspace(*p1) or *p1 == ',')
        p1++;
      if (*p1 == ';')
      {
        if (cls <= 0)
          cls = n;
        row++;
        p1++; // skip ';' and white space
        while (isspace(*p1))
          p1++;
      }
      val = strtod(p1, (char**)&p2);
      if (p1 == p2)
      { // it may be a boolean 'true' or 'false'
        val = str2bool3(p1, false, &p2);
        if (p1 == p2)
          // not a valid boolean nor a valid number - stop
          break;
      }
      setValued(val, n, mayAdd, updT);
      n++;
      if (p2 == NULL)
        break;
      p1 = p2;
    }
    if (dataType == m2 and mayAdd)
    { // set matrix size
      if (pm == NULL)
        cls = n;
      int rws = row + 1;
      // set new matrix size
      setSize(rws, cls);
      if ((rws * cls) > n)
      { // protect against too few values
        for (int j = n; j < rws * cls; j++)
          setValued(0.0, j, false, updT);
        printf("UVariable: Missing elements in matrix %s (%dx%d)! but found only %d values (padded zeros)\n",
               name, rws, cls, n);
      }
    }
  }
  return result;
}

//////////////////////////////////////////////

bool UVariable::setValues(const char * attValue, const int idx, bool mayAdd, UTime * updT)
{ // set as string
  bool result;
  int n, i, m;
  char * p1;
  bool terminate;
  //
  result = (idx == 0 or (idx > 0 and isString()));
  if (result)
  { // set first variable type to string
    setType(s);
    // test for space
    n = strlen(attValue);
    if (idx + n >= byteCnt)
    { // new size
      // debug
      // printf("UVariable::setValues: increased length of string to from %d to %d bytes\n", byteCnt, n + idx);
      // debug end
      m = elementSize * (idx + n + 1);
      dValue = (double*) realloc(dValue, m);
      result = (dValue != NULL);
      if (result)
      {
        p1 = (char*)dValue;
        p1 += elementCnt;
        for (i = elementCnt; i < idx; i++)
          // fill uninitialized part with space
          *p1++ = ' ';
        // and terminate
        *p1 = '\0';
        byteCnt = m;
      }
      // debug
      // printf("UVariable::setValues: increased length of string to %d bytes OK\n", n + idx);
      // debug end
    }
    if (result)
    { // set new value
      p1 = (char*)dValue;
      p1 += idx;
      terminate = ((int)strlen(p1) < n or idx == 0);
      strncpy(p1, attValue, n);
      if (terminate)
      {
        elementCnt = idx + n;
        p1[n] = '\0';
      }
      setUpdated();
    }
  }
  return result;
}

//////////////////////////////////////////////

bool UVariable::setValueM(UMatrix * mat, UTime * updT)
{
  return setValueM(mat->rows(), mat->cols(), mat->getData(), updT);
}

//////////////////////////////////////////////

bool UVariable::setValueM(matrix * mat, UTime * updT)
{
  return setValueM(getrows(mat), getcols(mat), *mat->mat, updT);
}

////////////////////////////////////////////

bool UVariable::setValueM(int rCnt, int cCnt, double * values, UTime * updT)
{
  int n;
  double * d1;
  //
  n = setSize(rCnt, cCnt);
  if (n >= rCnt * cCnt)
  {
    d1 = &dValue[2];
    memcpy(d1, values, n * sizeof(double));
    setUpdated(updT);
    return true;
  }
  else
    return false;
}


///////////////////////////////////////////////////////

bool UVariable::setValue(const UVariable * source, const int idx, UTime * updT)
{ // set as string
  bool result = true;
  const int MSL = 500;
  char s[MSL];
  //
  if (idx <= 0 and elementCnt <= source->getElementCnt())
    copy(source, false);
  else
  {
    if (isString())
    {
      if (source->isString())
        setValues(source->getValueBuffer(), idx, true, updT);
      else
      {
        ((UVariable*)source)->getValuesdAsString(s, MSL, 0);
        setValues(s, idx, true, updT);
      }
    }
    else
    {
      if (source->isString())
        setValued(source->getValueBuffer(), idx, true, updT);
      else
        setValued(source->getValued(), idx, true, updT);
    }
  }
  return result;
}

//////////////////////////////////////////////

bool UVariable::setType(varType newType)
{
  bool result = true;
  //
  switch (newType)
  {
    case s:
      elementSize = 1;
      isDoubleBased = false;
      break;
    case d:
    case d3d:
    case d2d:
    case rot:
    case pose:
    case dq:
    case t:
    case b:
    case i:
    case m2:
      elementSize = sizeof(double);
      isDoubleBased = true;
      break;
    default:
      elementSize = sizeof(double);
      isDoubleBased = true;
      result = false;
      break;
  }
  dataType = newType;
  //elementCnt = byteCnt / elementSize;
  return result;
}

/////////////////////////////////////////////////

bool UVariable::setTypeChar(const char * to)
{
  bool result = true;
  setType(typeFromChar(to));
  return result;
}

////////////////////////////////////////////

UVariable::varType UVariable::typeFromChar(const char * typ)
{
  varType result;
  //
  if (strcmp(typ, "d") == 0)
    result = d;
  else if (strcmp(typ, "6d") == 0)
    result = d6d;
  else if (strcmp(typ, "3d") == 0)
    result = d3d;
  else if (strcmp(typ, "2d") == 0)
    result = d2d;
  else if (strcmp(typ, "rot") == 0)
    result = rot;
  else if (strcmp(typ, "pose") == 0)
    result = pose;
  else if (strcmp(typ, "dq") == 0)
    result = dq;
  else if (strcmp(typ, "s") == 0)
    result = s;
  else if (strcmp(typ, "t") == 0)
    result = t;
  else if (strcmp(typ, "i") == 0)
    result = i;
  else if (strcmp(typ, "b") == 0)
    result = b;
  else if (strcmp(typ, "m") == 0)
    result = m2;
  else if (strcmp(typ, "m2") == 0)
    result = m2;
  else if (strcmp(typ, "matrix") == 0)
    result = m2;
  else
    result = none;
  //
  return result;
}

/////////////////////////////////////////////////

const char * UVariable::getTypeChar(char * to, const int toCnt)
{
  switch (dataType)
  {
    case d:   strncpy(to, "d", toCnt); break;
    case d6d: strncpy(to, "6d", toCnt); break;
    case d3d: strncpy(to, "3d", toCnt); break;
    case d2d: strncpy(to, "2d", toCnt); break;
    case rot: strncpy(to, "rot", toCnt); break;
    case pose:strncpy(to, "pose", toCnt); break;
    case dq:  strncpy(to, "dq", toCnt); break;
    case t:   strncpy(to, "t", toCnt); break;
    case s:   strncpy(to, "s", toCnt); break;
    case b:   strncpy(to, "b", toCnt); break;
    case i:   strncpy(to, "i", toCnt); break;
    case m2:   strncpy(to, "m2", toCnt); break;
    default:
      strncpy(to, "none", toCnt);
      break;
  }
  return to;
}

////////////////////////////////////////////////////

double UVariable::add(double value, int idx, UTime * updTime)
{
  double v;
  v = getValued(idx);
  v += value;
  setValued(v, idx, false, updTime);
  return v;
}

/////////////////////////////////////////////////////

// bool UVariable::splitNameIndexStati(const char * nameWithIndex, int * index, int * col, int * nameLng)
// {
//   int idx;
//   int n;
//   const char *p1;
//   char *p2;
//   bool result = true;
//   //
//   p1 = strchr(nameWithIndex, '[');
//   if (p1 != NULL)
//   {
//     n = p1 - nameWithIndex;
//     p1++;
//     idx = strtol(p1, &p2, 0);
//     while (isblank(*p2) or (*p2 == ','))
//       p2++;
//     if (*p2 != ']')
//       result = false;
//   }
//   else
//   {
//     idx = 0;
//     n = strlen(nameWithIndex);
//   }
//   if (index != NULL)
//     *index = idx;
//   if (nameLng != NULL)
//     *nameLng = n;
//   return result;
// }

/////////////////////////////////////////////////////

bool UVariable::splitNameIndex(const char * nameWithIndex, int * index, int * col, int * nameLng)
{
  int idx;
  int n;
  const char *p1;
  char *p2;
  bool result = true;
  //
  p1 = strchr(nameWithIndex, '[');
  if (p1 != NULL)
  {
    n = p1 - nameWithIndex;
    p1++;
    idx = strtol(p1, &p2, 0);
    while (isblank(*p2) or (*p2 == ','))
      p2++;
    if (isdigit(*p2))
    {
      int c = strtol(p2, &p2, 0);
      if (col != NULL)
        *col = c;
      while (isblank(*p2))
        p2++;
    }
    if (*p2 != ']')
      result = false;
  }
  else
  {
    idx = 0;
    n = strlen(nameWithIndex);
  }
  if (index != NULL)
    *index = idx;
  if (nameLng != NULL)
    *nameLng = n;
  return result;
}

////////////////////////////////////////////////////

void UVariable::setName(const char * newName, int * idx)
{
  int n, c = -1;
  splitNameIndex(newName, idx, &c, &n);
  if (idx != NULL)
  {
    if ((*idx == 0 or rows() > 1) and c >= 0)
      *idx = *idx * cols() + c;
  }
  strncpy(name, newName, MAX_VAR_NAME_SIZE);
  n = mini(n, MAX_VAR_NAME_SIZE);
  name[n] = '\0';
}

////////////////////////////////////////////////////

const char * UVariable::getGetAsXmlAttribute(char * buff, const int buffCnt)
{
  char * p1 = buff;
  int n = 0, i;
  const char * pres = "";
  //
  snprintf(p1, buffCnt - n - 1, "%s=\"", name);
  buff[buffCnt - 1] = 0;
  for (i = 0; i < elementCnt; i++)
  {
    n += strlen(p1);
    p1 = &buff[n];
    getValuedAsString(i, pres, p1, buffCnt - n);
    pres = " ";
  }
  n += strlen(p1);
  p1 = &buff[n];
  if (n < buffCnt - 2)
  { // terminate value
    buff[n++] = '"';
    buff[n] = '\0';
  }
  else
    buff[buffCnt-1]='\0';
  return buff;
}

//////////////////////////////////////////////////////

const char * UVariable::getValuedAsString(int idx, const char * preStr, char * buff, const int buffCnt, int histIdx)
{
  double d1;
  const double * dBase;
  char * p1;
  //
  if (histIdx > 0 and hasHist())
  {
    dBase = hist->getData(histIdx);
    if (dBase != NULL)
      d1 = dBase[idx];
    else
      d1 = 0.0;
  }
  else
    d1 = dValue[idx];
  if (dataType == d or dataType == m2)
    snprintf(buff, buffCnt, "%s%.13g", preStr, d1);
  else
  {
    snprintf(buff, buffCnt, "%s%.4f", preStr, d1);
    // remove trailing zeros
    p1 = &buff[strlen(buff) - 1];
    while (*p1 == '0')
      p1--;
    if (*p1 == '.')
      p1--;
    p1++;
    *p1 = '\0';
  }
  return buff;
}

////////////////////////////////////////////////////

const char * UVariable::getValuesdAsString(char * buff, const int buffCnt, int histIdx)
{
  int n = 0, m, c;
  char * p1 = buff;
  const char * pres = "";
  //
  *p1 = '\0';
  if (isDouble())
  {
    if (dataType == m2)
      m = 2;
    else
      m = 0;
    c = cols();
    for (int i = 0; i < rows(); i++)
    {
      for (int j = 0; j < c; j++)
      {
        n += strlen(p1);
        p1 = &buff[n];
        getValuedAsString(i*c + j + m, pres, p1, buffCnt - n, histIdx);
        pres = " ";
      }
      pres = "; ";
    }
  }
  else
    snprintf(buff, buffCnt, "pt unsupported format");
  return buff;
}

///////////////////////////////////////////////////////////

const char * UVariable::getValuesmAsString(char * buff, const int buffCnt)
{
  return getValuesmAsStringLimited(buff, buffCnt, 999999, 999999);
}

////////////////////////////////////////////////////////////


const char * UVariable::getValuesmAsStringLimited(char * buff, const int buffCnt, int maxRows, int maxCols)
{
  int n = 0, c, i, j = 0;
  char * p1 = buff;
  const char * pres = "\t";
  //
  *p1 = '\0';
  if (isDouble())
  {
    c = cols();
    for (i = 0; i < mini(rows(), maxRows); i++)
    {
      for (j = 0; j < mini(c, maxCols); j++)
      {
        n += strlen(p1);
        p1 = &buff[n];
        snprintf(p1, buffCnt - n, "%s%g", pres, getValued(i*c + j));
        pres = " \t";
      }
      if (j < c)
        pres = " ...;\n\t";
      else
        pres = " ;\n\t";
    }
    if (i < rows())
    { // finish with eclipse
      n += strlen(p1);
      p1 = &buff[n];
      if (j < c)
        snprintf(p1, buffCnt - n, " ...;\n\t...");
      else
        snprintf(p1, buffCnt - n, " ;\n\t...");
    }
  }
  else
    snprintf(buff, buffCnt, "pt unsupported format");
  return buff;
}

////////////////////////////////////////////////////

const char * UVariable::getValuesAsString(char * buff, const int buffCnt, int histIdx)
{
  if (isString())
  { // debug
    // printf("UVariable::getValuesAsString: getting copy of string length=%d to buffer of length=%d\n",
    //        strlen(getValueBuffer()), buffCnt);
    // debug end
    strncpy(buff, getValueBuffer(), buffCnt);
    // make sure it is zero terminated
    buff[buffCnt - 1] = '\0';
  }
  else
    getValuesdAsString(buff, buffCnt, histIdx);
  return buff;
}

////////////////////////////////////////////////

const char * UVariable::getValuesAsStringQuoted(char * buff, const int buffCnt, int histIdx)
{
  const char *pb, *p1, *p2;
  char q = '"';
  char *p3 = buff;
  int n = 0;
  //
  pb = (char*)dValue;
  if (pb != NULL)
  {
    if (isString())
    { // check for type of quotation marks already used.
      p1 = strchr(pb, '\'');
      p2 = strchr(pb, '"');
      if (p2 != NULL)
      {
        if (p2 < p1 or p1 == NULL)
          q = '\'';
      }
    }
    // no test for a later apostroph or quotation mark of the wrong type
    if (buffCnt > 2)
    {
      *p3++ = q;
      *p3 = '\0';
      n = 1;
    }
    getValuesAsString(p3, buffCnt - n, histIdx);
    n += strlen(p3);
    p3 = &buff[n];
    if (buffCnt - n > 2)
    {
      *p3++ = q;
      *p3 = '\0';
    }
  }
  else
  {
    snprintf(buff, buffCnt, "%c(null)%c", q, q);
  }
  return buff;
}

////////////////////////////////////////////////////

void UVariable::copy(const UVariable * source, bool alsoName)
{
  int i, n; // bytes to move
  //
  if (source->descriptionBuffer != NULL)
    setDescription(source->description, alsoName and source->descriptionBufferUsed());
  if (alsoName)
    strncpy(name, source->name, MAX_VAR_NAME_SIZE + 1);
  setType(source->getType());
  i = source->getElementCnt();
  if (isDouble())
  { // create variable space
    if (dataType == m2)
      i += 2;
    n = i * elementSize;
  }
  else
    n = source->getByteCnt();
  if (n > byteCnt)
  { // make more space if needed
    dValue = (double*)realloc(dValue, n);
    byteCnt = n;
  }
  // copy values
  if (n > 0)
    memcpy((char*)dValue, source->getValueBuffer(), n);
  // set destination size
  elementCnt = source->getElementCnt();
  if (source->hasHist())
  { // mark as updated
    UTime t = source->hist->getUpdTime();
    setUpdated(&t);
  }
  else
    setUpdated(NULL);
}


/////////////////////////////////////////////////////

bool UVariable::set3D(UPosition * value, UTime * updT)
{
  bool result = true;
  //
  result = setValued(value->z, 2, true, updT);
  if (result)
  { // space is OK
    setValued(value->y, 1, false, updT);
    setValued(value->x, 0, false, updT);
  }
  //
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setRot(URotation * value, UTime * updT)
{
  bool result = true;
  //
  result = setValued(value->Kappa, 2, true, updT);
  if (result)
  { // space is OK
    setValued(value->Phi, 1, false, updT);
    setValued(value->Omega, 0, false, updT);
  }
  //
  return result;
}

////////////////////////////////////////////////////

bool UVariable::set6D(UPosRot * value, UTime * updT)
{
  bool result = true;
  //
  result = setValued(value->Kappa, 5, true, updT);
  if (result)
  { // space is OK
    setValued(value->Phi, 4, false, updT);
    setValued(value->Omega, 3, false, updT);
    setValued(value->z, 2, false, updT);
    setValued(value->y, 1, false, updT);
    setValued(value->x, 0, false, updT);
  }
  //
  return result;
}

////////////////////////////////////////////////////

bool UVariable::setPose(UPose * value, UTime * updT)
{
  bool result = true;
  //
  result = setValued(value->h, 2, true, updT);
  if (result)
  { // space is OK
    setValued(value->y, 1, false, updT);
    setValued(value->x, 0, false, updT);
  }
  //
  return result;
}

////////////////////////////////////////////////////

bool UVariable::setPose(UPoseTime * value, UTime * updT)
{
  bool result = true;
  //
  result = setValued(value->t.getDecSec(), 3, true, updT);
  if (result)
  { // space is OK
    setValued(value->h, 2, false, updT);
    setValued(value->y, 1, false, updT);
    setValued(value->x, 0, false, updT);
  }
  //
  return result;
}

/////////////////////////////////////////////////////

UPosition UVariable::get3D() const
{
  UPosition result;
  //
  if (elementCnt >= 3)
    result.set(dValue[0], dValue[1], dValue[2]);
  else if (elementCnt >= 2)
    result.set(dValue[0], dValue[1], 0.0);
  else if (elementCnt >= 1)
    result.set(dValue[0], 0.0, 0.0);
  //
  return result;
}

/////////////////////////////////////////////////////

URotation UVariable::getRot() const
{
  URotation result;
  //
  if (elementCnt >= 3)
    result.set(dValue[0], dValue[1], dValue[2]);
  else if (elementCnt >= 2)
    result.set(dValue[0], dValue[1], 0.0);
  else if (elementCnt >= 1)
    result.set(dValue[0], 0.0, 0.0);
  //
  return result;
}

/////////////////////////////////////////////////////

UPosRot UVariable::get6D() const
{
  UPosRot result;
  //
  if (elementCnt >= 6)
    result.set(dValue[0], dValue[1], dValue[2], dValue[3], dValue[4], dValue[5]);
  else if (elementCnt >= 5)
    result.set(dValue[0], dValue[1], dValue[2], dValue[3], dValue[4], 0.0);
  else if (elementCnt >= 4)
    result.set(dValue[0], dValue[1], dValue[2], dValue[3], 0.0, 0.0);
  else if (elementCnt >= 3)
    result.set(dValue[0], dValue[1], dValue[2], 0.0, 0.0, 0.0);
  else if (elementCnt >= 2)
    result.set(dValue[0], dValue[1], 0.0, 0.0, 0.0, 0.0);
  else if (elementCnt >= 1)
    result.set(dValue[0], 0.0, 0.0, 0.0, 0.0, 0.0);
  //
  return result;
}

/////////////////////////////////////////////////////

UPose UVariable::getPose() const
{
  UPose result;
  //
  if (elementCnt >= 3)
    result.set(dValue[0], dValue[1], dValue[2]);
  else if (elementCnt >= 2)
    result.set(dValue[0], dValue[1], 0.0);
  else if (elementCnt >= 1)
    result.set(dValue[0], 0.0, 0.0);
  //
  return result;
}
/////////////////////////////////////////////////////

UPoseTime UVariable::getPoseTime() const
{
  UPoseTime result;
  //
  if (elementCnt >= 4)
    result.setPt(dValue[0], dValue[1], dValue[2], dValue[3]);
  else if (elementCnt >= 3)
    result.set(dValue[0], dValue[1], dValue[2]);
  else if (elementCnt >= 2)
    result.set(dValue[0], dValue[1], 0.0);
  else if (elementCnt >= 1)
    result.set(dValue[0], 0.0, 0.0);
  //
  return result;
}

/////////////////////////////////////////////////////

void UVariable::negate()
{
  double * v = dValue;
  int i;
  //
  if (isDoubleBased)
  {
    for (i = 0; i < elementCnt; i++)
    {
      *v++ *= -1;
    }
  }
}

/////////////////////////////////////////////////////

void UVariable::inverse()
{
  double * v = dValue;
  int i;
  //
  if (isDoubleBased)
  {
    for (i = 0; i < elementCnt; i++)
    {
      if (fabs(*v > 1e-100))
        *v = 1.0/(*v);
      else
        *v = 1e100;
      v++;
    }
  }
}

/////////////////////////////////////////////////////

bool UVariable::setGt(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      *v = left->getValued(i) > right->getValued(i);
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setGe(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      *v = left->getValued(i) >= right->getValued(i);
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setLt(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      *v = left->getValued(i) < right->getValued(i);
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setLe(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      *v = left->getValued(i) <= right->getValued(i);
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setEq(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int i;
  bool equal = true;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < m; i++)
    {
      equal &= left->getValued(i) == right->getValued(i);
    }
    *dValue = equal;
    setSize(1);
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setNe(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int i;
  bool equal = true;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < m; i++)
    {
      equal &= left->getValued(i) == right->getValued(i);
    }
    *dValue = not equal;
    setSize(1);
  }
  return result;
}
/////////////////////////////////////////////////////

bool UVariable::setPlus(UVariable * left, UVariable * right)
{
  bool result;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      *v = left->getValued(i) + right->getValued(i);
      v++;
    }
  }
  return result;
}
/////////////////////////////////////////////////////

bool UVariable::setDiff(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      *v = left->getValued(i) - right->getValued(i);
      v++;
    }
  }
  return result;
}
/////////////////////////////////////////////////////

bool UVariable::setProduct(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      *v = left->getValued(i) * right->getValued(i);
      v++;
    }
  }
  return result;
}
/////////////////////////////////////////////////////

bool UVariable::setDivide(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  double v2;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      v2 = right->getValued(i);
      if (fabs(v2) > 1e-100)
        *v = left->getValued(i) / right->getValued(i);
      else
        *v = left->getValued(i) * 1e100;
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setBinaryAnd(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      *v = double(long(left->getValued(i)) & long(right->getValued(i)));
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setBinaryOr(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      *v = double(long(left->getValued(i)) | long(right->getValued(i)));
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setBinaryXor(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      *v = double(long(left->getValued(i)) ^ long(right->getValued(i)));
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setLogicalAnd(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      *v = left->getValued(i) and right->getValued(i);
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setLogicalOr(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int m = right->getElementCnt();
  int k = mini(n, m);
  int i;
  double * v = dValue;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    result = (m == n);
    for (i = 0; i < k; i++)
    {
      *v = left->getValued(i) or right->getValued(i);
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setBinaryShiftLeft(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int i;
  double * v = dValue;
  long v2;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    v2 = roundi(right->getValued(0));
    for (i = 0; i < n; i++)
    {
      *v = long(left->getValued(i)) << v2;
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setBinaryShiftRight(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int i;
  double * v = dValue;
  long v2;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    v2 = roundi(right->getValued(0));
    for (i = 0; i < n; i++)
    {
      *v = long(left->getValued(i)) >> v2;
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setModulo(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int i;
  double * v = dValue;
  long v2;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    v2 = roundi(right->getValued(0));
    for (i = 0; i < n; i++)
    {
      *v = long(left->getValued(i)) % v2;
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setPow(UVariable * left, UVariable * right)
{
  bool result = true;
  int n = left->getElementCnt();
  int i;
  double * v = dValue;
  double v2;
  // must have same number of elements
  result = left->isDouble() and right->isDouble();
  if (result)
  { // should have same size too
    v2 = right->getValued(0);
    for (i = 0; i < n; i++)
    {
      *v = pow(left->getValued(i), v2);
      v++;
    }
  }
  return result;
}

/////////////////////////////////////////////////////

bool UVariable::setNot()
{
  int i;
  double * v = dValue;
  // must have same number of elements
  if (isDoubleBased)
  { // should have same size too
    for (i = 0; i < elementCnt; i++)
    {
      *v = not *v;
      v++;
    }
  }
  return isDoubleBased;
}

/////////////////////////////////////////////////////

bool UVariable::setValueNew(const char * newName, const char * attValue,
                            const char * type,
                            const char * comment)
{
  varType tp;
  char * p1;
  const char * p2;
  //double v = 0.0;
  bool result;
  // set name
  strncpy(name, newName, MAX_VAR_NAME_SIZE);
  // set type
  tp = typeFromChar(type);
  if (tp == UVariable::none)
  { // no type, so take type from value
    tp = d; // assume double
    if (strlen(attValue) > 0)
    {
      strtod(attValue, &p1);
      if (p1 == attValue or attValue[0] == ' ')
        tp = s;
    }
  }
  setType(tp);
  if (isString())
  { // skip first blank, as this may be a key to string
    p2 = attValue;
    if (*p2 == ' ')
      p2++;
    result = setValues(p2, 0, true);
  }
  else
    // is double based - just set value(s)
    result = setValued(attValue, 0 , true);
  description = comment;
  return result;
}

////////////////////////////////////////////////

// void UVariable::setUpdated()
// {
//   if (parent != NULL)
//   { // notify parent
//     parent->setUpdated(name);
//     // add to global log (if included)
//     if (parent->getLogVar() != NULL)
//       saveToLog(parent->getLogVar(), parent->getFullPreName(), parent->getLogVarLock());
//   }
//   if (hasHist())
//   { // variable has private history or log
//     changed(dValue, isDouble(), NULL);
//   }
// }

////////////////////////////////////////////////

void UVariable::setUpdated(UTime * updTime)
{
  if (parent != NULL)
  { // notify parent
    parent->setUpdated(name);
    // add to global log (if included)
    if (parent->getLogVar() != NULL)
      saveToLog(parent->getLogVar(), parent->getFullPreName(), parent->getLogVarLock());
  }
  if (hasHist())
  { // variable has private history or log
    changed(dValue, isDouble(), updTime);
  }
}

////////////////////////////////////////////////

int UVariable::rows()
{
  int result = 1;
  if (dataType == m2)
    result = roundi(dValue[0]);
  return result;
}

////////////////////////////////////////////////

int UVariable::cols()
{
  int result = elementCnt;
  if (dataType == m2)
    result = roundi(dValue[1]);
  return result;
}

////////////////////////////////////////////////////

int UVariable::setSize(int rows, int cols)
{
  if (dataType != m2)
    setType(m2);
  // size no less than 1;
  rows = maxi(1, rows);
  cols = maxi(1, cols);
  // need more space?
  if ((rows * cols + 2) > elementCnt)
    // allow more space and/or fill new elements with zero
    setValued(0.0, rows * cols - 1, true);
  else
    // just set count of elements to a lower value (including size)
    elementCnt = rows * cols + 2;
  dValue[0] = rows;
  dValue[1] = cols;
  if (hasHist())
    if (elementCnt > hist->colCnt)
      hist->setHistSize(hist->rowCnt, elementCnt);
  return getSize();
}

////////////////////////////////////////////////////

int UVariable::setSize(int elements)
{
  if (elements > elementCnt)
  {
    if (isString())
      setValues(" ", elements - 1, true);
    else
      setValued(0.0, elements - 1, true);
  }
  else
    elementCnt = elements;
  if (hasHist())
    if (elementCnt > hist->colCnt)
      hist->setHistSize(hist->rowCnt, elementCnt);
  return getSize();
}

////////////////////////////////////////////////////

int UVariable::getSize()
{
  if (dataType == m2)
    return elementCnt - 2;
  else
    return elementCnt;
}

////////////////////////////////////////////////////

bool UVariable::getM(matrix * mat)
{
  bool result =  dataType == m2;
  int n;
  if (result)
  {
    result = roundi(dValue[0]) == getrows(mat) and roundi(dValue[1]) == getcols(mat);
    if (result)
    {
      n = getrows(mat) * getcols(mat);
      array2mat(mat, &dValue[2], n);
    }
  }
  return result;
}

////////////////////////////////////////////////////

bool UVariable::getM(UMatrix * mat)
{
  bool result =  dataType == m2;
  if (result)
  {
    result = roundi(dValue[0]) == (int)mat->rows() and roundi(dValue[1]) == (int)mat->cols();
    if (result)
      result = mat->setMat(mat->rows(), mat->cols(), &dValue[2]);
  }
  return result;
}

/////////////////////////////////////////////////

bool UVariable::openVarLog(bool logFileOpen)
{
  bool result;
  if (not logFileOpen)
    // close log
    result = openLog(false);
  else if (parent == NULL)
    // open log needs a name
    result = openLog(true, "", name);
  else
    // parent has a name too
    result = openLog(true, parent->getFullPreName(), name);
  if (result and isDoubleBased and hist->rowMaxCnt == 0)
    // make space for buffer to determine timestamp
    hist->setHistSize(1, getElementCnt());
  return result;
}

/////////////////////////////////////////////////

bool UVariable::decodeReplayLine(char * line)
{
  UTime t;
  unsigned long ts, tus = 0;
  char *p2 = line;
  //
  ts = strtol(p2, &p2, 10);
  if (*p2 == '.')
  {
    p2++;
    tus = strtol(p2, &p2, 10);
  }
  t.setTimeU(ts, tus);
  if (t.valid and isDoubleBased and p2 != NULL)
  {
    int n;
    for (n = 0; n < elementCnt; n++)
    { // skip a char if komma or semicolon or space separated
      while (*p2 == ',' or *p2 == ';' or isspace(*p2))
        p2++;
      // get value
      dValue[n] = strtod(p2, &p2);
      if (p2 == NULL or *p2 == '\0')
        break;
    }
    if (n > 0)
      setUpdated(&t);
  }
  return t.valid;
}

////////////////////////////////////////////

const char * UVariable::setDescription(const char * desc, bool makeCopy)
{
  if (not makeCopy)
    description = desc;
  else
  { // both value and name, then also description
    int n = strlen(desc) + 1;
    descriptionBuffer = (char *) realloc(descriptionBuffer, n);
    memcpy(descriptionBuffer, desc, n);
    description = descriptionBuffer;
  }
  return description;
}

////////////////////////////////////////////

bool UVariable::setAll(double value)
{
  bool result = true;
  if (isDoubleBased)
  {
    int ix;
    if (dataType == m2)
      ix = 2;
    else
      ix = 0;
    for (int i = ix; i < elementCnt; i++)
      dValue[i] = value;
    setUpdated(NULL);
  }
  else
    result = false;
  return result;
}

/////////////////////////////////////////////////

bool UVariable::setUnit()
{
  bool result = true;
  if (dataType == m2)
  {
    for (int i = 0; i < rows(); i++)
      for (int j = 0; j < cols(); j++)
      {
        if (i == j)
          dValue[i*cols() + j + 2] = 1.0;
        else
          dValue[i*cols() + j + 2] = 0.0;
      }
    setUpdated(NULL);
  }
  else
    result = false;
  return result;
}
