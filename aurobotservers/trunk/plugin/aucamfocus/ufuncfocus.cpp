/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                             *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "ufuncfocus.h"

#include <ugen4/uimage2.h>

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

#ifdef LIBRARY_OPEN_NEEDED


UFunctionBase* createFunc()
{ // create an object of this type
  //
  /** replace 'MyFunction' with your classname, as used in the headerfile */
  return new UFuncFocus();
}


#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncFocus::~UFuncFocus()
{
  // possibly remove allocated variables here - if needed
}

///////////////////////////////////////////////////

bool UFuncFocus::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("focus"))
    result = doFocus(msg, (UImage *)extra);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFuncFocus::doFocus(UServerInMsg * msg, UImage * pushImg)
{ // extract parameters first
  bool result = false;
  // decode vars
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 100;
  char attValue[VAL_BUFF_LNG];
  int n, i, np;
  // parameters default value
  int roiX = 0;
  int roiY = 0;
  int roiW = 800;
  int roiH = 600;
  int deviceNum = -1;
  char posName[MAX_MOUNT_NAME_SIZE] = "";
  // camera and source image variables
  UCamPush * cam = NULL; // camera
  UImage * img = pushImg; // image
  bool ask4help = false;
  const int MRL = 300;
  char reply[MRL];
  char reply2[MRL];
  const int MBL = 50;
  char bar[MBL*2+1];
  char trend[MBL];
  long fval, fmax = 1;
  //
  // extract parameters
  // this is one of two methodt, this one takes all provided parameters one at a time
  // and returns its name (attName) and value (attValue), and you must test for the name and
  // extract the value from the provided string.
  //
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  { // camera device
    if ((strcasecmp(attName, "device") == 0) or
        (strcasecmp(attName, "posName") == 0))
    { // either number or string
      n = sscanf(attValue, "%d", &deviceNum);
      if ((n != 1) and (strlen(attValue) > 0))
        strncpy(posName, attValue, MAX_MOUNT_NAME_SIZE);
    }
    // range of interest parameters
    else if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strncasecmp(attName, "width", 1) == 0)
      sscanf(attValue, "%d", &roiW);
    else if (strncasecmp(attName, "height", 1) == 0)
      sscanf(attValue, "%d", &roiH);
    else if (strcasecmp(attName, "y") == 0)
      sscanf(attValue, "%d", &roiY);
    else if (strcasecmp(attName, "x") == 0)
      sscanf(attValue, "%d", &roiX);
  }
  //
  if (ask4help)
  { // it was just a request for on-line help
    sendMsg(msg, "<help subject=\"FOCUS\">\n");
    sendText(msg, "----------- available FOCUS options\n");
    sendText(msg, "device=N      from device 'N', i.e. /dev/videoN\n");
    sendText(msg, "posName=name  from camera 'name', e.g. left for left camera\n");
    sendText(msg, "x=X y=Y       Top-left position of area (def = 0,0)\n");
    sendText(msg, "w=W h=H       Width and height of (def=full image)\n");
    sendText(msg, "    (maximize for best focus (in area))\n");
    sendText(msg, "---\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else
  { // if server event push, then image is available in call
    // if not, then get an image from camera
    if (img == NULL)
    { // no image, so get requested camera
      if (deviceNum >= 0)
        // get camera from device number
        cam = camPool->getCam(deviceNum);
      else if (strlen(posName) > 0)
        // else get camera from camera position name
        cam = camPool->getCam(posName);
      else
        // else use device 0
        cam = camPool->getCam(0);
      // get image buffer from image-pool
      img = imgPool->getImage(cam->getDev()->getDeviceNumber(), true);
      // get an image from camera
      result = (cam != NULL);
      if (result)
        // get an image from stream and loads it into 'img'
        result = cam->getImageSnapshot(img);
    }
    //
    result = (img != NULL);
    if (not result)
      // send standard XML-packed reply to client - like:
      // <COG warning="No image available"/>\n
      sendWarning(msg, "No image available");
    else
    { // image is avialable
      if (fmaxOld < 1000)
        fmaxOld = 1000;
      fmax = fmaxOld;
      fval = calculateFocusValue(img, roiX, roiY, roiW, roiH);
      if (fvalOld < 1000)
        fvalOld = fval;
      while (fvalOld > fmax)
        fmax *= 2;
      while (fvalOld < fmax/3)
        fmax /= 2;
      // make focus string
      n = (MBL * fval)/fmax;
      np = (MBL * fvalOld)/fmax;
      for (i = 0; i < mini(MBL*2, maxi(n, np+1)); i++)
      {
        if (i == np)
          bar[i] = '#';
        else if (i < n)
          bar[i] = '*';
        else
          bar[i] = ' ';
      }
      bar[i] = '\0';
      //
      if (fval > fvalOld)
        strncpy(trend, "->", 2);
      else if (fval == fvalOld)
        strncpy(trend, "--", 2);
      else
        strncpy(trend, "<-", 2);
      //
      fvalOld = fval + (9 * (fvalOld - fval)) / 10;
      fmaxOld = fmax;
     // send reply
     // format reply in XML style as a single tag with attributes and values.
      snprintf(reply, MRL, "value=\"%ld\" trend=\"%s\" valstr=\"%s\"", fval, trend, bar);
      // code < and > in XML format
      str2xml(reply, MRL, reply);
      // send coded string
      snprintf(reply2, MRL, "<%s %s/>\n", msg->tag.getTagName(), reply);
      sendMsg(msg, reply2);
    }
  }
  return result;
}

//////////////////////////////////////////////////////////////

long UFuncFocus::calculateFocusValue(
                       UImage * rawImg,
                       int roiX, int roiY, int roiW, int roiH)
{
  int r, c;
  long fsum = 0;
  //UPixel * pix; // pointer to a pixel
  //
  //
  unsigned char *yp, *y1, *y2, *y3;
  UPixel * p1, *p2, *p3;
  //
  // limit area to within image
  roiX = maxi(0, mini(roiX, rawImg->width() - 2));
  roiY = maxi(0, mini(roiY, rawImg->height() - 2));
  if ((roiX + roiW) > (int)rawImg->width() - 1)
    roiW = rawImg->width() - roiX - 1;
  if ((roiY + roiH) > (int)rawImg->height() - 1)
    roiH = rawImg->height() - roiY - 1;
  //
  //
  //  YUV420 image
  if (rawImg->isYUV420())
  {
    for (r = 0; r < roiH; r++)
    { // get pointer to first pixel in area
      // on line 'r'
      yp = rawImg->getYline(r + roiY);
      y2 = yp + roiX;
      y1 = y2++;              // first pixel and shift y2 to next pixel
      y3 = &y1[rawImg->width()]; // same position next line
      // then continue with the next pixels on image line
      for (c = 0; c < roiW; c++)
      { // test if average pixel-value is above treshold
        fsum += absi(*y2 - *y1) + absi(*y3 - *y1);
      }
    }
  }
  else if (rawImg->isYUV())
  {
    for (r = 0; r < roiH; r++)
    { // get pointer to first pixel in area
      // on line 'r'
      p2 = &rawImg->getLine(r + roiY)[roiX];
      p1 = p2++;              // first pixel and shift y2 to next pixel
      p3 = &p1[rawImg->width()]; // same position next line
      // then continue with the next pixels on image line
      for (c = 0; c < roiW; c++)
      { // test if average pixel-value is above treshold
        fsum += absi(p2->y - p1->y) + absi(p3->y - p1->y);
      }
    }
  }
  //
  //
  return fsum;
}
