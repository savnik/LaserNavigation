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
#ifndef USMLTAG_H
#define USMLTAG_H

#include <ugen4/usmltagin.h>
#include <ugen4/utime.h>
#include <ugen4/uline.h>
#include <ugen4/ugmk.h>
#include <umap4/umanseq.h>
#include <umap4/uprobpoly.h>

//#include "uclientport.h"
#include "uobstacle.h"
#include "usmlsource.h"


/**
Class that codes and decodes more complex structores to and from the SML format */
class USmlTag : public USmlTagIn
{
public:
  /**
  Constructor */
  USmlTag()
  {
    cnn = NULL;
  };
  /**
  Shortcut to cnn (UClientPortSml) function */
  inline bool skipToEndTag(int msTimeout)
  {
    if (cnn == NULL)
    {
      printf("USmlTag::skipToEndTag: no cnn!\n");
      return false;
    }
    else
      return cnn->skipToEndTag( this, msTimeout);
  };
  /**
  Shortcut to cnn (UClientPortSml) function */
  bool getNextTag(USmlTag * tag, int msTimeout,
                         USmlTagIn * failEndTag = NULL,
                         char * beforeTagBuffer = NULL, int * beforeTagCnt = NULL);
  /**
  Shortcut to cnn (UClientPortSml) function */
  inline bool skipNBytes(int n, int msTimeout)
  {
    if (cnn == NULL)
    {
      printf("USmlTag::skipNBytes: no cnn!\n");
      return false;
    }
    else
      return cnn->skipNBytes(n, msTimeout);
  };
  /**
  Shortcut to cnn (UClientPortSml) function */
  inline int getNBytes(char * buffer, int n, int msTimeout)
  {
    if (cnn == NULL)
    {
      printf("USmlTag::getNBytes: no cnn!\n");
      return false;
    }
    else
      return cnn->getNBytes(buffer, n, msTimeout);
  };
  /**
  Shortcut to cnn (UClientPortSml) function */
/*  inline bool sendMsg(const char * message)
  {
    if (cnn == NULL)
    {
      printf("USmlTag::sendMsg: no cnn!\n");
      return false;
    }
    else
      return cnn->sendMsg(message);
  };*/
  /**
  Shortcut to cnn (UClientPortSml) function */
  inline bool outputData(const char * message)
  {
    if (cnn == NULL)
    {
      printf("USmlTag::outputData: no cnn!\n");
      return false;
    }
    else
      return cnn->outputData(message);
  };
  /**
  Set connection pointer */
  inline void setCnn(USmlSource * connection)
  { cnn = connection; };
  /**
  Is connection verbose */
  inline bool cnnVerbose()
  {
    if (cnn == NULL)
    {
      printf("USmlTag::cnnVerbose: no cnn!\n");
      return false;
    }
    else
      return cnn->isVerbose();
  }

public:
  /**
  Make position to SML string
  \<pos3d name="name" x="123.456" y="123.456" z="123.2"/ [extra]\>
  Returns pointer to the buffer string 's', if buffer is too short the message will be truncated.
  If name is NULL or an empty string, the name attribute will be omitted.
  The 'extra' is extra attributes appended before the '/>' close tag. */
  const char * codePosition(UPosition * pos, char * s, const unsigned int bufferLength,
                            const char * name, const char * extra = NULL);
  /**
  Slight variation of the main function with the same name */
  inline const char * codePosition(UPosition pos, char * s, const unsigned int bufferLength,
                                   const char * name, const char * extra = NULL)
  { return codePosition(&pos, s, bufferLength, name, extra); };
  /**
  Code a rotation structore into string s of length 'buffer length.
  Returns a pointer to s, if buffer length is sufficient, else NULL is returned */
  const char * codeRotation(URotation * rot, char * s, const unsigned int bufferLength, const char * name);
  /**
  Slight variation of the main function with the same name */
  inline const char * codeRotation(URotation rot, char * s, const unsigned int bufferLength, const char * name)
  { return codeRotation(&rot, s, bufferLength, name); };
  /**
  Decode a position tag.
  Returns true if all valeues were decoded. */
  bool getPosition(UPosition * pos);
  /**
  Decode rotation from this tag.
  Returns true if all thress parts are present (Omega, Phi and Kappa) */
  bool getRotation(URotation * rot);

  /**
  Code the line segment to in a XML like text of the form
  &lt;lineSeg name="name" legngth=1.234 extra/&gt;
  &lt;pos3d name="start" x=1.0e12 y=2.0 z=2.0/&gt;
  &lt;pos3d name="vec" x=9.9 y=9.9 z=9.9/&gt;
  &lt;/lineSeg&gt;.
  If extra is not NULL, then the extra string info is inserted as 'extra'
  into start tag.
  Returns true if data is fitted into buffer. */
  bool codeLineSegment(ULineSegment * seg, char * s,
                       const unsigned int bufferLength, const char * name,
                       const char * extra = NULL);
  /**
  Decode line segment from a SML-string.
  Returns true if all data were present. */
  bool getLineSegment(ULineSegment * seg);
  /**
  Code a time of dat as a single tag.
  Returns a pointer to s, if buffer length is sufficient, else NULL is returned */
  const char * codeTime(const UTime time, char * s, const unsigned int bufferLength, const char * name);
  /**
  Get time value from this tag. */
  bool getTimeofday(UTime * time, char * name);
  /**
  decode a guidemark and store result in the provided structure.
  Returns true if full guidemari is obtained. */
  bool getGmk(UGmk * gmk);
  /**
  Decode a polygon */
  bool getProbPoly(UProbPoly * poly);
  /**
  Code a manoeuvre.
  Returns a pointer to the coded string (buf). */
  char * codeManoeuvre(UManoeuvre * man, char * buf, int bufCnt, char * name);
  /**
  Code a small sequence of manoeuvres to d a pose-to-pose manoeuvre.
  Returns a pointer to the coded string (buf). */
  bool codeManPPSeq(UManPPSeq * manseq, char * buf, int bufCnt, char * name);
  /**
  Code a robot pose including velocity. All is in metric units, meter and radians
  Returns a pointer to the coded string (buf). */
  char * codePoseV(UPoseV pv, char * buf, int bufCnt, const char * name);
  /**
  * Send an UProbPoly structure.
  * Add some extra attributes in start tag if extraAtt is not NULL (or empty).
   * \param poly is the polygon to be send,
   * \param msg is the message reference for client number etc.
   * \param name is an optional value for a name attribute
   * \param extraAtt an additional (optional) string with more attributes for the polygon start tag 
   * \param exe is a server core pointer that implements the send functions
  * Returns true if send. */
  bool sendProbPoly(UProbPoly * poly, UServerInMsg * msg, const char * name, 
                    const char * extraAtt, UCmdExe * exe);
  /**
  Receive a sequense of manoeuvres */
  bool getManSeq(UManSeq * man);
  /**
  * Get a pose-velocity structure from this tag.
  * Returns true if all fields (x,y,h,v) were present. */
  bool getPoseV(UPoseV * pose);
  /**
  Code a pose to this string, and include a name attribute, if name holds a name.
  Additional attributes (like time, velocity may be placed in 'extraAtt' */
  char * codePose(UPose pose, char * buf, int bufCnt, const char * name, const char * extraAtt = NULL);
  /**
  Extract pose attributes from this tag. */
  bool getPose(UPose * pose);
  /**
  Code pose time as one tag.
  Extra attributes may be added in 'extraAtt' */
  char * codePoseTime1(UPoseTime pt, char * buf, int bufCnt, const char * name, const char * extraAtt = NULL);
  /**
  Code a pose-time structure as a combination of pose and time tags */
  char * codePoseTime(UPoseTime pt, char * buf, int bufCnt, const char * name);
  /**
  Get a pose-time structure. If there is a name attribute, then
  set it to the name structure (name buffer must be able to hold 30 characters). */
  bool getPoseTime(UPoseTime * pose, char * name);
  /**
  Get pose time in compact form.
  * \<pt od="1234567890.123456" x=0.123 y="1.234" th="0.0123"/>.
  * Returns true if all needed attributes were present. */
  bool getPoseT(UPoseTime * pose);
  /**
  Decode an obstacle group */
  bool getObstacleGroup(UObstacleGroup * oGrp, char * name);
  /**
  Decode an obstacle for connection stream */
  bool getObstacle(UObstacle * obst, char * name);
  /**
  Decode polygon from this start tag and place result in 'poly' structure.
  Return any name in 'name', and get additional charracters from 'cnn' connection. */
  bool getPolygon(UPolygon * poly, char * name);
  /**
  Code a polygon using the tagname 'polygon'.
  Handles any number of vertices, but require a buffer of
  about 90 characters per vertice (+ about 50 for frame)
   * \param poly is the polygon to code,
   * \param buf is the buffer to code to,
   * \param bufCnt is the length of the buffer,
   * \param name is an optional extra name attribute value - is added as name value in quotes.
   * \param extra is a full string of extra attributes (assumed to be coded to legal xml standard, i.e. aaa="bbb" ccc="555" ....
   * \returns pointer to buffer */
  const char * codePolygon(UPolygon * poly, char * buf, int bufCnt, char * name,
                     const char * extra);
  /**
  Code an obstacle polygon including start and end pose etc.
  Requires approc 90 bytes per vertice plus about 400 characters for other values */
  const char * codeObstacle(UObstacle * obst, char * buf, int bufCnt, char * name);
  /**
  Get interface name - if set */
  inline const char * getIfName()
  {
    if (cnn != NULL)
      return cnn->ifName;
    else
      return NULL;
  }
  /**
   * DEcode a pose with time and potentiallt velocity and quality figure.
   * \returns true if at least x,y,th and time is available. */
  bool getPoseTVQ(UPoseTVQ * pose);

protected:
  /**
  Client connection from wich to acquire more data - when needed */
  USmlSource * cnn;

};


#endif
