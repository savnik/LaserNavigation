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
#include "ufunccog.h"

#include <ugen4/uimage2.h>

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

#ifdef LIBRARY_OPEN_NEEDED

void libraryOpen(void)
{ // called when server opens this plugin (i.e. calls dlopen())
  printf("Opening UFuncCog Library\n");
  // add code as needed
  // for global logfiles etc ...
}

///////////////////////////////////////////////////

void libraryClose(void)
{ // called when server unloads this plugin (i.e. calls dlclose())
  printf("Closing UFuncCog library\n");
  // add code as needed
}

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase* createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncCog' with your classname, as used in the headerfile */
  return new UFuncCog();
}

void deleteFunc(UFunctionCamBase* p)
{ // delete the object
  delete p;
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


///////////////////////////////////////////////////

UFuncCog::~UFuncCog()
{
  // possibly remove allocated variables here - if needed
}


///////////////////////////////////////////////////
bool UFuncCog::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("COG"))
    result = centerOfGravity(msg, (UImage *)extra);
  else if (msg->tag.isTagA("bark"))
    result = doBark(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFuncCog::doBark(UServerInMsg * msg)
{ // send a short reply back to the client requested the 'bark'
  const int MRL = 100;
  char reply[MRL];
  bool ask4help;
  char helpValue[30];
  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", helpValue, 30);
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    snprintf(reply, MRL, "<%s reply='I will not help you to bark!'/>\n",
             msg->tag.getTagName());
  }
  else
  { // create the reply in XML-like (html - like) format
    snprintf(reply, MRL, "<%s what='No reason to bark at me.'/>\n",
             msg->tag.getTagName());
  }
  // send this string as the reply to the client
  sendMsg(msg, reply);
  // return true if the function is handled with a positive result
  return true;
}

///////////////////////////////////////////////////

bool UFuncCog::centerOfGravity(UServerInMsg * msg, UImage * pushImg)
{ // extract parameters first
  bool result = false;
  // decode vars
  char attName[MAX_SML_NAME_LENGTH];
  const int VBL = 100;
  char attValue[VBL];
  const int MSL = 30;
  char s[MSL];
  int n;
  // parameters default value
  int roiX = 20;
  int roiY = 10;
  int roiW = 80;
  int roiH = 100;
  int threshold = 100;
  int deviceNum = -1;
  char posName[MAX_MOUNT_NAME_SIZE] = "";
  // camera and source image variables
  UCamPush * cam = NULL; // camera
  UImage * img = pushImg; // image
  bool ask4help = false;
  const int MRL = 200;
  char reply[MRL];
  double lx, ly;
  //
  // extract parameters
  // this is one of two methodt, this one takes all provided parameters one at a time
  // and returns its name (attName) and value (attValue), and you must test for the name and
  // extract the value from the provided string.
  //
  while (msg->tag.getNextAttribute(attName, attValue, VBL))
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
    else if (strncasecmp(attName, "threshold",1) == 0)
      sscanf(attValue, "%d", &threshold);
  }
  //
  if (ask4help)
  { // it was just a request for on-line help
    sendHelpStart(msg, "COG");
    sendText(msg, "---- Available COG options:\n");
    sendText(msg, "device=N      from device 'N', i.e. /dev/videoN\n");
    sendText(msg, "posName=name  from camera 'name', e.g. left for left camera\n");
    sendText(msg, "x=X y=Y       Top-left position of Range Of Interest (ROI)\n");
    sendText(msg, "w=W h=H       Width and height of ROI\n");
    sendText(msg, "threshold=B   Threshold for pixel use (0 to 255)\n");
    sendText(msg, "    (returns center of gravity position for area)\n");
    sendText(msg, "    (Uses image pool image 40 (org), 41 (result), 42 and 43)\n");
    sendText(msg, "---\n");
    sendHelpDone(msg);
  }
  else
  { // if server event push, then image is available in call
    // if not, then get an image from camera
    if (img == NULL)
    { // no image, so get requested camera
      if (gotAllResources(reply, MRL))
      {
        if (deviceNum >= 0)
          // get camera from device number
          cam = camPool->getCam(deviceNum);
        else if (strlen(posName) > 0)
          // else get camera from camera position name
          cam = camPool->getCam(posName);
        else
          // else use device 0
          cam = camPool->getCam(0);
        result = (cam != NULL);
        if (result)
        { // get image buffer from image-pool
          img = imgPool->getImage(cam->getDev()->getDeviceNumber(), true);
          // get an image from camera
          // get an image from stream and load it into 'img'
          result = cam->getImageSnapshot(img);
        }
        else
          fprintf(stderr, "No camera found\n");
      }
      else
        fprintf(stderr, "Ressoreces are missing, (no %s)\n", reply);
    }
    else
      result = true;
    //
    if (not result)
      // send standard XML-packed reply to client - like:
      // <COG warning="No image available"/>\n
      sendWarning(msg, "No image available");
    else
    { // image is avialable
      calculateCOG(img, roiX, roiY, roiW, roiH, threshold, &lx, &ly);
      // convert image timestamp to a string in format hh:mm:ss.ddd (to string s)
      img->imgTime.getTimeAsString(s, true);
      // format reply in XML style as a single tag with attributes and values.
      snprintf(reply, MRL, "<COG x=\"%f\" y=\"%f\" time=\"%s\"/>\n", lx, ly, s);
      sendMsg(msg, reply);
    }
  }
  return result;
}

//////////////////////////////////////////////////////////////

bool UFuncCog::calculateCOG(UImage * rawImg,
                       int roiX, int roiY, int roiW, int roiH,
                       int threshold,
                       double * lx, double * ly)
{
  UImage * raw2;
  UImage * img0; // RGB format image
  UImage * img1; // RGB format image
  UImage * imgBW; // RGB format image
  double sx = 0; // sum of x values
  double sy = 0; // sum of y values
  int sxi, syi;
  int n = 0;      // count of pixels
  int r, c;
  UPixel * pix; // pointer to a pixel
  unsigned char *y, *u, *v, m;

  //
  // get an image structure from the image pool - shared by all functions
  // image is created on first call (if second parameter is true)
  //
  //
  // Make a lowpass filtered image using and openCV function
  //
  // get an image from pool to store a work copy of image
  // this will be image 40 in imagepool-list
  img0 = imgPool->getImage(40, true);
  // get another image from pool to store smoothed image
  // this will be image 41 in imagepool-list
  img1 = imgPool->getImage(41, true);
  // convert raw YUV image to RGB (actually BGR, as this is standard in openCV)
  //img1->copy(rawImg);
  rawImg->toBGR(img1);
  img1->setName("img1");
  // filter image using a gaussion low pass filter
  cvSmooth(img1->cvArr(), img1->cvArr(), CV_GAUSSIAN, 3, 0, 0 );
  //
  // use filtered image
  // limit area to within image
  roiX = maxi(0, mini(roiX, img1->width() - 1));
  roiY = maxi(0, mini(roiY, img1->height() - 1));
  if ((roiX + roiW) > (int)img1->width())
    roiW = img1->width() - roiX;
  if ((roiY + roiH) > (int)img1->height())
    roiH = img1->height() - roiY;
  //
  //
  // now calculate the COG within the ROI
  for (r = 0; r < roiH; r++)
  { // get pointer to first pixel in area
    // on line 'r'
    pix = img1->getPixRef(r + roiY, roiX);
    // then continue with the next pixels on image line
    for (c = 0; c < roiW; c++)
    { // test if average pixel-value is above treshold
      if (((pix->p1 + pix->p2 + pix->p3)/3) > threshold)
      { // summ the pixel position
        n++;
        sx += (double)(roiX + c);
        sy += (double)(roiY + r);
        // change the color of this pixel
        // to mark, that it is above treshold.
        pix->p2 = 255; // green value
        pix->p1 = 128; // blue value
      }
      // advance pixel pointer to next pixel
      pix++;
    }
  }
  if (n > 0)
  { // calculate Center of Gravity
    sx /= (double)n;
    sy /= (double)n;
  }
  //
  // paint a rectangel in image, to mark the tested area
  // using an open CV function
  cvRectangle(img1->cvArr(),   // image
        cvPoint(roiX, roiY),  // one corner
        cvPoint(roiX + roiW, roiY + roiH), // second corner
        CV_RGB(255, 0, 0),   // colour (R,G,B)
        1, 8, 0);  // line-width, connectivity, offset
  // convert result to integer
  sxi = roundi(sx);
  syi = roundi(sy);
  // paint cross at center of gravity
  cvLine(img1->cvArr(), cvPoint(sxi-10, syi-10),
                       cvPoint(sxi+10, syi+10),
                       CV_RGB(0,0,255), 1, 8, 0);
  cvLine(img1->cvArr(), cvPoint(mini(img1->width()-1, sxi+10), maxi(0, syi-10)),
                       cvPoint(maxi(0, sxi-10), mini(img1->height()-1, syi+10)),
                       CV_RGB(0,0,255), 1, 8, 0);
  // tell imagepool to flag image as updated, (so that any pending push
  // commands can be effectuated (after this function is finished).
  img1->imgUpdated();
  // save result in bmp format - filtered image with result markings
  rawImg->saveBMP("testimg-org.bmp");
  img1->saveBMP("testimg-filt.bmp");
  //
  // save result to caller
  *lx = sx;
  *ly = sy;
  //
  // the rest is just to demonstrate some image maipulations
  //
  // convert to BW for some openCV maipulation in BW
  imgBW = imgPool->getImage(43, true);
  // imgBW->copy(rawImg);
  // convert to BW
  rawImg->toBW(imgBW);
  // flag as updated
  imgBW->updated();
  // copy support data to image 0
  // - including size image time, name serial number etc.
  // but not the image pixels
  img0->copyMeta(imgBW, true);
  img0->setName("img0-edge");
  // detect edges using a (openCV) canny filter
  // - and stores result in img0
  cvCanny(imgBW->cvArr(), img0->cvArr(), 0.5, 0.5, 3);
  // flag as updated
  img0->imgUpdated();
  // save the edge image
  img0->saveBMP("testimg-canny.bmp");
  //
  //
  // some image manipulation pixel by pixel in YUV420 format
  // (in YUV420 u and v has half resolution only)
  // result is saved as testing-colour.bmp
  // enhances color in the region:
  // x=50 (column) y=60 (row) (top left) to (x=350 y=310 - bottom right)
  //
  raw2 = imgPool->getImage(42, true);
  raw2->copy(rawImg);
  raw2->setName("raw2-color");
  for (r = 60; r < mini(raw2->getHeight(),310); r+=2)
  {
    y = raw2->getYline(r);
    u = raw2->getUline(r/2);
    v = raw2->getVline(r/2);
    for (c = 50; c < mini(350, raw2->getWidth()); c+=2)
    { // swap U and V
      m = *v;
      *v = *u;
      *u = m;
      //advance y pointer by 2 pixels
      y += 2;
      // advance U and V by one pixel
      u++;
      v++;
    }
  }
  // flag as updated
  raw2->imgUpdated();
  // save
  raw2->saveBMP("testing-color.bmp");
  //
  return true;
}
