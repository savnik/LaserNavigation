/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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
#ifndef USMLTAGIN_H
#define USMLTAGIN_H

#include "ucommon.h"
#include "u3d.h"

/**
Length of a SML tag or attribute name */
#define MAX_SML_NAME_LENGTH 30

/**
Common function to convert XML reserved characters to
their escaped values, expanding the length of the string.
Source and destination may be the same string
\Returns pounter to destination string.
if destination string is too short, the result is undefined. */
extern char * str2xml(char * dest, int destLength,
                      const char * source);

/**
 * Common function to convert XML minimum set of reserved
 * characters - '\<' to &lt; and '&' to &amp; - to
 * their escaped values, expanding the length of the string.
 * Source and destination may be the same string
 * \Returns pounter to destination string.
 * if destination string is too short, the result is undefined. */
extern char * str2xmlMin(char * dest, int destLength,
                      const char * source);

/**
Common function to convert an XML string with escaped characters back to
their original values.
Source and destination may be the same string.
If destination string is too short the result is undefined. */
extern char * xml2str(char * dest, int destLength,
                      const char * source, int sourceLength);

/**
 * Look in the source for the character 'stop', but do not
 * look inside strings in quotes and brackets of type (hhh) or [ggg] e.g.:
 * source= ---"aaa 'ggggg "2%2" ggggg' aaa"(aa % 2) %kkk-%-%-
 * looking for '%' will not find the % in 2%2, and not
 * in (aa % 2) but the % before the kkk
 * \returns a pointer to the found character, or NULL if none found. */
extern const char * findStopChar(const char * source, char stop);

/**
A class for analysis and construction of an Sml tag.
Sml is a minor implementation of XML.

@author Christian Andersen
*/
class USmlTagIn
{
public:
  /**
  Constructor */
  USmlTagIn();
  /**
  Destructor */
  ~USmlTagIn();
  /**
  Set and analyze tag initially.
  Tags may start with an '<', if not
  the first non-white space is assumed to be tag name. */
  bool setTag(const char * tagString);
  /**
  Set tag, when also specifying the buffer length.
  NB! requires that the brackets is available,
  e.g. a message MUST start with a '<'.
  Returns true if tag-name is found. */
  bool setTag(const char * tagString, int tagStringCnt);
  /**
  It the tag valid.
  Returns true if valid - i.e. has a name. */
  inline bool isValid()
  { return valid; };
  /**
  Set valid flag */
  inline void setValid(bool value)
  { valid = value; };
  /**
  Is last read tag (by getNextTag) a group start tag - e.g. < matrix> */
  inline bool isAStartTag()
  { return isStartTag and valid;};
  /**
  Is last read tag (by getNextTag) a group end tag - e.g. < /matrix> */
  inline bool isAnEndTag()
  { return isEndTag and valid;};
  /**
  Is last read tag (by getNextTag) a group end tag - e.g. < pos3D x=1.1 y=5.5 z=8.8/> */
  inline bool isAFullTag()
  { return isFullTag and valid;};
  /**
  Is tag a phony tag starting with a ?.
  Returns true if so */
  inline bool isAPhonyTag()
  { return ((tagName[0] == '?') or (tagName[0] == '!')) and valid;};
  /**
  Compare the tagname with this string */
  inline bool isTagA(const char * tag)
  { return (strcasecmp(tagName, tag) == 0) and valid; };
  /**
  Compare the tagname with this string.
  And return true if it is the right end tag.
  An '/' is appended to the provided tag. */
  inline bool isTagAnEnd(const char * tag)
  {
    return ((strcasecmp(&tagName[1], tag) == 0) and (tagName[0] == '/') and valid);
  };
  /**
  Is the buffer holding a full tag, then the endtag '>' is at position tagCnt.
  Returns true if buffer holds a full tag. */
  inline bool isTagEndFound()
    { return tagEndFound and valid; };
  /**
  Get last read tagname by function getNextTag();
  Returns pointer to read string. */
  inline const char * getTagName() { return tagName;};
  /**
  Return the tag length in characters. The end-tag character is at this
  position if an end-tag is found at all. */
  inline int getTagCnt()
    { return tagCnt; };
  /**
  Get a pointer to the last character in the tag, i.e. the '>' character. */
  inline const char * getToEnd()
    { return toEnd; };
  /**
  Get the pointer to the start of the tag - i.e. the '<' character in
  the source data stream */
  inline const char * getTagStart()
    { return tag; };
  /**
  Get a pointer to the start of the next attribute in the list */
  inline const char * getNext()
  { return next; };
  /**
  Get next attribute and value to these buffers
  starting from provided 'start' position.
  e.g. if buffer is: matrix name="A_matrix" rows="3" cols="3"
  and start is after matrix, then name-> will contain "name"
  and value-> will contain "A_matrix" (but no quotation marks)
  and start will point to first character after value. <br>
   * \param name is the name is not filled with more than MAX_SML_NAME_LENGTH characters (or as specified in optional nameMaxCnt parameter).
   * \param value If there is no value, i.e. a attribute name only, then
   * value will be an empty sthring. value may be NULL (value data ignored).
   * \param valMaxCnt is the length of the val buffer.
   * \param nameMaxCnt is the length of the name buffer, default is MAX_SML_NAME_LENGTH
   * \Returns true if a name were found before end of start string. */
  bool getNextAttribute(char * name, char * value,
                        const int valBufLength,
                        const int nameMaxCnt = MAX_SML_NAME_LENGTH,
                        bool * overflow = NULL);
  /**
  Get value for this named attribute.
   * \param name is the name of the searched attribute
   * \param value is a string buffer for the result. If no such attribute, then no change to buffer
   * \param bufLength is size of the provided buffer
   * \Returns false if no attribute has that name.
   * \Returns at maximum bufLength data (NB! hard limit in function too).
  The value string is returned in value, and optional quotes or apostrophs
  are removed. */
  bool getAttValue(const char * name, char * value, const int bufLength);
  /**
  Get boolean value for this named attribute.
   * \param name is the name of the searched attribute
   * \param value is where the value is returned (may be NULL), if no attribute value (an empty string)
   *              is found then set to default value. If no such attribute, then no change to value.
   * \param defaultValue is used if attribute value is empty (or is an empty string)
   * \Returns true if attribute is found and false if attribute is not found. */
  bool getAttBool(const char * name, bool * value, bool defaultValue = true);
  /**
  Get integer value for this named attribute.
   * \param name is the name of the searched attribute
   * \param value is where the value is returned (may be NULL). If no such attribute, then no change to value.
   * \param defaultValue is used if attribute is empty (or is an empty string)
   * \Returns false if attribute is not found. */
  bool getAttInteger(const char * name, int * value, int defaultValue = 0);
  /**
  Get integer value for this named attribute.
   * \param name is the name of the searched attribute
   * \param value is where the value is returned (may be NULL)
   * \param defaultValue is used if attribute is empty (or is an empty string)
   * \Returns false if attribute is not found. */
  bool getAttUnsignedLong(const char * name,
                          unsigned long * value,
                          unsigned long defaultValue = 0);
  /**
  Get double value for this named attribute.
   * \param name is the name of the searched attribute
   * \param value is where the value is returned, unchange if no such attribute (may be NULL)
   * \param defaultValue is used if attribute has no value (or an empty string)
   * \Returns false if attribute is not found. */
  bool getAttDouble(const char * name, double * value,
                    double defaultValue = 0.0);
  /**
  Get time value for this named attribute -- assumed to be in decimal seconds
   * since 1 jan 1970, with up to 6 decimals.
   * \param name is the name of the searched attribute
   * \param value is where the value is returned (unchanged if no such value or empty value)
   * \Returns false if attribute is not found. */
  bool getAttTime(const char * name, UTime * value);
  /**
  Get time value for this named attribute -- assumed to be in decimal seconds
   * since 1 jan 1970, with up to 6 decimals.
   * \param name is the name of the searched attribute
   * \param value is where the value is returned, not changed if no such attribute
   * \param defaultValue (decimal seconds) is used if attribute has no value (or an empty string)
   * \Returns false if attribute is not found. */
  bool getAttTime(const char * name, UTime * value, double defaultValue);
  /**
  Get time value for this named attribute -- assumed to be in decimal seconds
   * since 1 jan 1970, with up to 6 decimals.
   * \param name is the name of the searched attribute
   * \param value is where the value is returned, not changed if no such attribute
   * \param defaultValue is used if attribute has no value (or an empty string)
   * \Returns false if attribute is not found. */
  bool getAttTime(const char * name, UTime * value, UTime defaultValue);
  /**
  Reset to the state, where no attributes are read, i.e.
  ready to get next attribute after the tag name.
   * \Returns true if the tag is valid. */
  bool reset();
  /**
  Find first occurance of the end character '>'.
  \Returns the count form start of data to this character.
  \Returns 'dataCnt' if not found.
  Sets the 'toEnd' pointer of the object. */
  int findEndTag(const char * data, const int dataCnt);
  /**
  Print tag - or the first 100 characters if it is longer */
  void print(const char * preString);
  /**
  Decode this tag as a time tag.
  Returns true if both 'sec' and 'usec' attributes are
  present in tag. All attributes in tag are read at return. */
  bool getTime(UTime * time);
  /**
   * Get integer attribute value for the attribute with this name.
   * \param name of the attribute to be found
   * \param value pointer to where the value are to be stored (not NULL)
   * If value of attribute is empty, then 'value' is unchanged (to allow a default value)
   * \returns true if such attribute exist in  the tag. */
  bool getAttValueInt(const char * name, int * value);
  /**
   * Get boolean attribute value for the attribute with this name.
   * The allowed values are "0" or "false" for false, "1" or "true" for true,
   * if not one of these, then the default value apply. if attribute do not exist,
   * the value is not changed.
   * \param name of the attribute to be found
   * \param value pointer to where the value are to be stored (not NULL), unchanged if no such attribute.
   * \param defValue is the value if attribute string is empty or not one of the allowed values
   * \returns true if such attribute exist in  the tag. */
  bool getAttValueBool(const char * name, bool * value, bool defValue);
  /**
   * Get double attribute value for the attribute with this name.
   * \param name of the attribute to be found
   * \param value pointer to where the value are to be stored (not NULL).
   * If value of attribute is empty, then 'value' is unchanged (to allow a default value)
   * \returns true if such attribute exist in the tag. */
  bool getAttValueD(const char * name, double * value);
  /**
   * Set flag to allow tag end inside a string */
  void allowTagEndInString()
  { ignoreTagEndInString = false;};
  /**
  Get number of attributes in this tag */
  int getAttCnt();
  /**
  Set binary tag
  \param name is a string with the name of the user of the binary data.
  \param data is the rx-buffer with the binary data.
  \param dataCnt is the number of data received. */
  void setBinaryTag(const char * name, const char * data, int dataCnt);
  
protected:
  /**
  Is SML tag valid - i.e. has a name */
  bool valid;
  /**
  Name of last read tag - by getNextTag function */
  char tagName[MAX_SML_NAME_LENGTH];
  /**
  Is a group start tag - e.g. < matrix> */
  bool isStartTag;
  /**
  Is a group end tag - e.g. < /matrix> */
  bool isEndTag;
  /**
  is a full tag - e.g. < pos3D x="1.1" y="5.5" z="8.8"/> */
  bool isFullTag;
  /**
  Buffer for the complete tag */
  const char * tag;
  /**
  Pointer to the end (close) character if found
  otherwise a NULL pointer */
  const char * toEnd;
  /**
  Length of tag buffer */
  int tagCnt;
  /**
  True if tagCnt is the '>' character in the tag */
  bool tagEndFound;
  /**
  Pointer to next character to analyse */
  const char * next;
  /**
   * Allow tag end to be inside a string - strict XML allows this */
  bool ignoreTagEndInString;
};

/**
class to hold an outgoint XML element */
// class USmlTagOut
// {
// public:
//   /**
//   Constructor */
//   USmlTagOut(const char * tagName);
//
// protected:
//   /**
//   buffer to hold packed xml statement */
//   char name[MAX_SML_NAME_LENGTH];
//
//   hertil - nej ikke denne gang
//            Find eventuelt hellere et samlet XML library
// }

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////


#endif
