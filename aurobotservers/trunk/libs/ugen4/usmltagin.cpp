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
#include <ctype.h>

#include "usmltagin.h"
#include "utime.h"

////////////////////////////////////////////////////////////////////////

char * str2xml(char * dest, int destLength,
               const char * source)
{
  const char * s;
  char * d;
  int n, i;
  //
  n = strlen(source);
  if (n < destLength)
  {
    if (source != dest)
      s = source;
    else
    { // source and dest is the same,
      //   so try to avoid collissions
      s = &dest[destLength - n];
      memmove((void *)s, source, n);
    }
    d = dest;
    for (i = 0; i < n; i++)
    {
      if (strchr("<>'\"&", *s) == NULL)
        *d++ = *s++;
      else
      {
        switch (*s)
        {
          case '\'':
            if ((d - dest + 6) >= destLength)
              break;
            strcpy(d, "&apos;");
            d += 6;
            break;
          case '"':
            if ((d - dest + 6) >= destLength)
              break;
            strcpy(d, "&quot;");
            d += 6;
            break;
          case '&':
            if ((d - dest + 5) >= destLength)
              break;
            strcpy(d, "&amp;");
            d += 5;
            break;
          case '<':
            if ((d - dest + 4) >= destLength)
              break;
            strcpy(d, "&lt;");
            d += 4;
            break;
          case '>':
            if ((d - dest + 4) >= destLength)
              break;
            strcpy(d, "&gt;");
            d += 4;
            break;
        }
        s++;
      }
      if ((d - dest) >= destLength)
        break;
    }
    *d = '\0';
  }
  //
  return dest;
}

///////////////////////////////////////////////////////////////

char * str2xmlMin(char * dest, int destLength,
               const char * source)
{
  const char * s;
  char * d;
  int n, i;
  //
  n = strlen(source);
  if (n < destLength)
  {
    if (source != dest)
      s = source;
    else
    { // source and dest is the same,
      //   so try to avoid collissions
      s = &dest[destLength - n];
      memmove((void *)s, source, n);
    }
    d = dest;
    for (i = 0; i < n; i++)
    {
      if (strchr("<&", *s) == NULL)
        *d++ = *s++;
      else
      {
        switch (*s)
        {
          case '&':
            if ((d - dest + 5) >= destLength)
              break;
            strcpy(d, "&amp;");
            d += 5;
            break;
          case '<':
            if ((d - dest + 4) >= destLength)
              break;
            strcpy(d, "&lt;");
            d += 4;
            break;
        }
        s++;
      }
      if ((d - dest) >= destLength)
        break;
    }
    *d = '\0';
  }
  //
  return dest;
}

///////////////////////////////////////////////////////////////


char * xml2str(char * dest, int destLength,
               const char * source, int sourceLength)
{
  const char * s, *sEnd;
  char * d, *dEnd;
  int n;
  //
  s = source;
  d = dest;
  if (sourceLength >= 0)
    n = sourceLength;
  else
    n = strlen(source);
  sEnd = &source[n];
  dEnd = &dest[destLength-1];
  while (s < sEnd)
  {
    if (*s == '\0')
      break;
    else if (*s != '&')
      *d++ = *s++;
    else
    {
      if (strncmp(s, "&lt;", 4) == 0)
      {
        *d++ = '<';
        s += 4;
      }
      else if (strncmp(s, "&gt;", 4) == 0)
      {
        *d++ = '>';
        s += 4;
      }
      else if (strncmp(s, "&amp;", 5) == 0)
      {
        *d++ = '&';
        s += 5;
      }
      else if (strncmp(s, "&quot;", 6) == 0)
      {
        *d++ = '"';
        s += 6;
      }
      else if (strncmp(s, "&apos;", 6) == 0)
      {
        *d++ = '\'';
        s += 6;
      }
      else
        *d++ = *s++;
    }
    if (d > dEnd)
      break;
  }
  *d = '\0';
  return dest;
}

/////////////////////////////////////////////////////////////////

const char * findStopChar(const char * source, char stop)
{
  const int MBL = 100;
  char brk[MBL];
  int brkIdx = 0;
//  const int MTL = 100000;
//  char ts[MTL]; // copy of source
  const char * result = NULL;
  const char * p1;
  int inStr = 0;
  //
  brk[0] = stop;
  p1 = source;
  while (*p1 != '\0')
  {
    if (*p1 == brk[brkIdx])
    {
      if (brkIdx == 0)
      {
        result = p1;
        break;
      }
      if (*p1 == '\'' or *p1 == '"')
        // decrease string nesting level
        inStr--;
      brkIdx--;
    }
    else if (strchr("[(\"'", *p1) != NULL)
    {
      if (inStr > 0 and (*p1 == '(' or *p1 == '['))
      { // brackets are allowed in strings in any flavor
        ;
      }
      else
      { // satrt of a substring
        brkIdx++;
        if (brkIdx >= MBL)
          // too many levels
          break;
        if (*p1 == '(')
          brk[brkIdx] = ')';
        else if (*p1 == '[')
          brk[brkIdx] = ']';
        else
        { // increase string nesting level
          brk[brkIdx] = *p1;
          inStr++;
        }
      }
    }
    p1++;
  }
  return result;
}


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

USmlTagIn::USmlTagIn()
{
  next = NULL;
  tagCnt = 0;
  tag = NULL;
  tagEndFound = false;
  valid = false;
  ignoreTagEndInString = true;
  tagName[0] = '\0';
}

///////////////////////////////////////////////////////

USmlTagIn::~USmlTagIn()
{
}

///////////////////////////////////////////////////////

bool USmlTagIn::setTag(const char * tagString)
{ //
  tag = tagString;
  tagCnt = maxi(0, strlen(tag) - 1);
  tagEndFound = (tag[tagCnt] == '>');
  if (tagEndFound)
    toEnd = &tag[tagCnt];
  else
  {
    tagCnt++;
    toEnd = NULL;
  }
  reset();
  return valid;
}

///////////////////////////////////////////////////////

bool USmlTagIn::setTag(const char * tagString, int tagStringCnt)
{ //
  // look for start
  tag = strchr(tagString, '<');
  valid = (tag != NULL);
  if (valid)
    // is found before end of buffer?
    valid = ((tag - tagString) < tagStringCnt);
  if (valid)
  { // start is within buffer
    tagCnt = findEndTag(tag, tagStringCnt);
    tagEndFound = (tag[tagCnt] == '>');
    reset();
  }
  return valid;
}

////////////////////////////////////////////////////////

int USmlTagIn::findEndTag(const char * data, const int dataCnt)
{
  int result = dataCnt;
  //
  if (dataCnt > 5 and strncmp(tag, "<!--", 4) == 0)
  { // an XML comment block, nested comments are not allowed
    toEnd = strstr(tag, "-->");
    if (toEnd != NULL)
      toEnd += 2;
  }
  else if (ignoreTagEndInString)
    toEnd = findStopChar(data, '>');
  else
    toEnd = strchr(data, '>');
  if (toEnd != NULL)
  { // end char is found
    result = (toEnd - data);
    if (result > dataCnt)
    { // outside buffer - revert to old end.
      result = dataCnt;
      toEnd = NULL;
    }
  }
  return result;
}

////////////////////////////////////////////////////////

bool USmlTagIn::reset()
{
  valid = (tagCnt > 0);
  //
  if (valid)
  { // set start of tag
    next = tag;
    if (tag[0] == '<')
    { // skip open bracket
      next++;
      // find tagType and
      isEndTag = (strncmp(tag, "</", 2) == 0);
      // full tag also if brackets are missing
      isFullTag = (strncmp(&tag[tagCnt-1], "/>", 2) == 0);
      // or may be a comment or other phony tag - a full tag by exception
      if (strncmp(tag, "<!", 2) == 0 or strncmp(tag, "<?", 2) == 0)
        isFullTag = true;
      // else it must be a start tag
      isStartTag = (not isFullTag) and (not isEndTag);
    }
    else
    { // breacket free tag is a full tag
      isFullTag = true;
      isEndTag = false;
      isStartTag = false;
    }
    //
    // read and return tagname at start of buffer
    valid = getNextAttribute(tagName, NULL, 0);
  }
  //
  return valid;
}

///////////////////////////////////////////////////////

void USmlTagIn::print(const char * preString)
{
  const int MSL = 150;
  char s[MSL];
  //
  if (tag == NULL)
    printf("%s - no tag\n", preString);
  else
  {
    xml2str(s, MSL, tag, tagCnt+1);
    s[mini(MSL-1, tagCnt+1)] = 0;
    if (tagCnt > MSL)
      printf("%s%s ...\n", preString, s);
    else
      printf("%s%s\n", preString, s);
  }
}

///////////////////////////////////////////////////////

bool USmlTagIn::getAttValue(const char * name, char * value, const int bufLength)
{
  bool result = false;
  char attName[MAX_SML_NAME_LENGTH];
  if (isValid())
  { // start from firt attribute
    reset();
    // search if attribute is present
    while (getNextAttribute(attName, NULL, 0))
    { // compare with wanted attribute
      result = (strcasecmp(attName, name) == 0);
      if (result)
        break;
    }
    if (result and value != NULL and bufLength > 0)
    { // get value of attribute
      reset();
      while (getNextAttribute(attName, value, bufLength))
      { // compare with wanted attribute
        result = (strcasecmp(attName, name) == 0);
        if (result)
          break;
      }
    }
  }
  return result;
}

///////////////////////////////////////////////////////////

int USmlTagIn::getAttCnt()
{
  int cnt = 0;
  if (isValid())
  { // start from firt attribute
    reset();
    // search if attribute is present
    while (getNextAttribute(NULL, NULL, 0))
      cnt++;
  }
  return cnt;
}

///////////////////////////////////////////////////////////

bool USmlTagIn::getAttValueBool(const char * name, bool * value, bool defValue)
{
  const int MVL = 30;
  char s[MVL];
  bool result;
  //
  result = getAttValue(name, s, MVL);
  if (result and value != NULL)
  { // allow use of 0 for false, 1 for true - else expect "false" or "true"
    if (s[0] == '0' and s[1] == '\0')
      *value = false;
    else if (s[0] == '1' and s[1] == '\0')
      *value = true;
    else
      *value = str2bool2(s, defValue);
  }
  return result;
}

///////////////////////////////////////////////////////////

bool USmlTagIn::getAttValueInt(const char * name, int * value)
{
  const int MVL = 30;
  char s[MVL];
  bool result;
  //
  result = getAttValue(name, s, MVL);
  if (result and strlen(s) > 0 and value != NULL)
    *value = strtol(s, NULL, 0);
  return result;
}

///////////////////////////////////////////////////////////

bool USmlTagIn::getAttValueD(const char * name, double * value)
{
  const int MVL = 30;
  char s[MVL];
  bool result;
  //
  result = getAttValue(name, s, MVL);
  if (result and strlen(s) > 0 and value != NULL)
    *value = strtod(s, NULL);
  return result;
}

///////////////////////////////////////////////////////////

bool USmlTagIn::getNextAttribute(char * name,
                                 char * value,
                                 const int valueMaxCnt,
                                const int nameMaxCnt, /* optional */
                                bool * overflow) /* optional */
{
  bool result = valid;
  bool inName = false;
  int pos, i;
  char * p1;
  //
  if (result)
    result = ((next - tag) <= tagCnt);
  if (result)
  { // make value default to empty
    if (valueMaxCnt > 0 and value != NULL)
      *value = '\0';
    while (*next != '\0')
    { // need a space character to start a new attribute
      // else it is probably just an end-character "/"
      if ((*next == '>') or
          (strncmp(next, "/>", 2) == 0))
        // end of message
        break;
      if (*next > ' ')
      { // name must start with something other than "/>"
        inName = true;
        break;
      }
      next++;
    }
    result = inName;
  }
  if (result)
  { // get attribute name
    if (inName)
    { // start of name is found
      pos = 0;
      while ((*next != '\0') and ((next - tag) <= tagCnt))
      { // name may be available already, if
        // so just skip name
        if ((*next == '=') or // attribute name finished
            (*next <= ' ') or // if next is white space then name is finished
            (*next == '>') or // an end-tag
            (strncmp(next, "/>", 2) == 0)) // full tag end
          break;
        // save attribute name
        if  (name != NULL)
          name[pos] = *next;
        next++;
        // just skip character if too long for buffer
        if (pos < nameMaxCnt - 1)
          pos++;
      }
      // terminate name string
      if (name != NULL)
      {
        name[pos] = '\0';
        // test for end-flag
        if (strcmp(name, "/") == 0)
          result = false;
      }
    }
  }
  if (result)
  { // read value if present
    while (*next == ' ')
      // skip space before (possible) '=' sign
      next++;
    if (*next == '=')
    { // there should be a value
      next++;
      // skip whitespace
      while (isspace(*next))
        next++;
      if (*next == '\'' or *next == '"')
      { // get string - accepting substrings and escapes
        next = stringGet(next, value, valueMaxCnt, overflow);
        if (*next == '\'' or *next == '"')
          next++;
      }
      else
      {
        p1 = value;
        i = 0;
        while (true)
        {
          if (*next <= ' ' or *next == '>' or strcmp(next, "/>") == 0)
            break;
          else if (p1 != NULL and i < valueMaxCnt)
            *p1++ = *next++;
          else
            // no destination, just advance
            next++;
          i++;
        }
        if (p1 != NULL)
          *p1 = '\0';
      }
      //
      // remove XML encoding from value string
      if (value != NULL)
        xml2str(value, valueMaxCnt, value, -1);
    }
  }
  //
  return result;
}


///////////////////////////////////////////////////////

bool USmlTagIn::getTime(UTime * time)
{
  bool gotSec = false;
  bool gotUSec = false;
  char name[MAX_SML_NAME_LENGTH];
  const int MVL = 20;
  char val[MVL];
  int n;
  unsigned long v, uv;
  //
  while (getNextAttribute(name, val, MVL))
  {
    if (strcasecmp(name, "sec") == 0)
    {
      n = sscanf(val, "%lu", &v);
      time->time.tv_sec = v;
      gotSec = (n == 1);
    }
    else if (strcasecmp(name, "usec") == 0)
    {
      n = sscanf(val, "%lu", &v);
      time->time.tv_usec = v;
      gotUSec = (n == 1);
    }
    else if (strcasecmp(name, "tod") == 0)
    {
      n = sscanf(val, "%lu.%lu", &v, &uv);
      time->setTime(v, uv);
      gotSec = (n == 2);
      gotUSec = (n == 2);
    }
    // time name is ignored
  }
  time->valid = gotSec and gotUSec;
  //
  return time->valid;
}

////////////////////////////////////////////////////////

bool USmlTagIn::getAttBool(const char * name, bool * value,
                          bool defaultValue)
{
  const int MSL = 50;
  char s[MSL];
  bool result;
  //
  result = getAttValue(name, s, MSL);
  if (result and value != NULL)
  {
    *value = str2bool2(s, defaultValue);
  }
  return result;
}

///////////////////////////////////////////////////////

bool USmlTagIn::getAttInteger(const char * name, int * value,
                             int defaultValue)
{
  const int MSL = 50;
  char s[MSL];
  bool result;
  //
  result = getAttValue(name, s, MSL);
  if (result and value != NULL)
  {
    if (strlen(s) > 0)
      *value = strtol(s, NULL, 0);
    else
      *value = defaultValue;
  }
  return result;
}

///////////////////////////////////////////////////////

bool USmlTagIn::getAttUnsignedLong(const char * name, unsigned long * value,
                                  unsigned long defaultValue)
{
  const int MSL = 50;
  char s[MSL];
  bool result;
  //
  result = getAttValue(name, s, MSL);
  if (result and value != NULL)
  {
    if (strlen(s) > 0)
      *value = strtol(s, NULL, 0);
    else
      *value = defaultValue;
  }
  return result;
}

///////////////////////////////////////////////////////

bool USmlTagIn::getAttDouble(const char * name, double * value,
                             double defaultValue)
{
  const int MSL = 50;
  char s[MSL];
  bool result;
  //
  result = getAttValue(name, s, MSL);
  if (result and value != NULL)
  {
    if (strlen(s) > 0)
      *value = strtod(s, NULL);
    else
      *value = defaultValue;
  }
  return result;
}

////////////////////////////////////////////////////////

bool USmlTagIn::getAttTime(const char * name, UTime * value)
{
  const int MSL = 50;
  char s[MSL];
  bool result;
  //
  result = getAttValue(name, s, MSL);
  if (result and value != NULL)
  {
    value->setTimeTod(s);
  }
  return result;
}

////////////////////////////////////////////////////////

bool USmlTagIn::getAttTime(const char * name, UTime * value,
                          double defaultValue)
{
  const int MSL = 50;
  char s[MSL];
  bool result;
  //
  result = getAttValue(name, s, MSL);
  if (result and value != NULL)
  {
    if (strlen(s) > 0)
      value->setTimeTod(s);
    else
      value->setTime(defaultValue);
  }
  return result;
}

////////////////////////////////////////////////////////

bool USmlTagIn::getAttTime(const char * name, UTime * value,
                          UTime defaultValue)
{
  const int MSL = 50;
  char s[MSL];
  bool result;
  //
  result = getAttValue(name, s, MSL);
  if (result and value != NULL)
  {
    if (strlen(s) > 0)
      value->setTimeTod(s);
    else
      *value = defaultValue;
  }
  return result;
}

////////////////////////////////////////////

void USmlTagIn::setBinaryTag(const char * name, const char * data, int dataCnt)
{
  tag = data;
  tagCnt = dataCnt;
  valid = true;
  strncpy(tagName, name, MAX_SML_NAME_LENGTH);
  isFullTag = true;
  isEndTag = false;
  isStartTag = false;


}

////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////



