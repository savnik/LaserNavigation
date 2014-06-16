/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                      *
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
 *
 * $Date: 2011-07-04 17:03:31 +0200 (Mon, 04 Jul 2011) $
 * $Id: ufuncrectify.cpp 59 2012-10-21 06:25:02Z jcan $
 *
 ***************************************************************************/

#include <urob4/ufuncplugbase.h>
#include <ugen4/uimage2.h>
#include <ucam4/ucammount.h>
#include <urob4/uimagepool.h>
#include <ucam4/ucampool.h>

/**
 * plug-in to make global variable in camera or laser scanner server available to MRC.
 * @author Christian Andersen
*/
class UFuncRectify : public UFuncPlugBase
{
public:
  /**
  Constructor */
  UFuncRectify()
  { // set the command (or commands) handled by this plugin
    setCommand("rectify", "rectify", "Rectify camera image for lens distortion");
  }
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra)
  {  // handle a plugin command
    const int MRL = 500;
    char reply[MRL];
    bool ask4help;
    const int MVL = 30;
    char value[MVL];
    bool finished = false;
    int src = 0;
    int dst = 15;
    const int MFL = 10;
    char fmt[MFL] = "";
    int camDev = 0;
    bool gotCamDev;
    UImage *imgSrc = NULL;
    UImage *imgDst = NULL;
    UCamMounted * cam = NULL;
    UCamPool * campool = NULL;
    UImagePool * imgpool = NULL;
    // check for 'help'
    ask4help = msg->tag.getAttValue("help", value, MVL);
    if (ask4help)
    { // create the reply in XML-like (html - like) format
      sendHelpStart(msg, "rectify");
      sendText(msg, "--- available RECTIFY options\n");
      sendText(msg, "Rectify camera images for lens distortion\n");
      sendText(msg, "help            This message\n");
      sendText(msg, "device=C        where C is the camera device number (holding the rectification parameters)\n");
      sendText(msg, "                default camera device is device number in pool image\n");
      sendText(msg, "src=S           where S is the source image in the image pool\n");
      sendText(msg, "dst=D           where D is destination image in the image pool\n");
      sendText(msg, "fmt=BW|RGB|BGR  Destination format (default is RGB for Bayer coded source images)\n");
      sendText(msg, "silent          No reply if rectification is successfull\n");
      sendText(msg, "see also    'var campool' for camera parameters\n");
      sendText(msg, "see also    'poolpush help' for actions when rectified image is available\n");
      sendHelpDone(msg);
      finished = true;
    }
    if (not finished)
    { // do some action and send a reply
      // test for availablity of needed resources
      campool = (UCamPool*)getStaticResource("campool", false, false);
      imgpool = (UImagePool*) getStaticResource("imgpool", false, false);
      if (campool == NULL or imgpool == NULL)
      {
        snprintf(reply, MRL, "Missing plug-in, got imagepool (%s) or campool (%s)",
                 bool2str(imgpool != NULL), bool2str(campool != NULL));
        sendWarning(reply);
        finished = true;
      }
    }
    if (not finished)
    { // got resources, find source image
      msg->tag.getAttInteger("src", &src, 0);
      msg->tag.getAttInteger("dst", &dst, 15);
      gotCamDev = msg->tag.getAttInteger("device", &camDev, 10);
      msg->tag.getAttValue("fmt", fmt, MFL);
      // get source image
      imgSrc = imgpool->getImage(src, false);
      if (imgSrc == NULL)
      {
        snprintf(reply, MRL, "no source image (img=%d)", src);
        sendWarning(reply);
        finished = true;
      }
    }
    if (not finished)
    { // got source image, find camera parameters
      if (not gotCamDev)
        camDev = imgSrc->camDevice;
      cam = campool->getCam(camDev);
      if (cam == NULL)
      {
        snprintf(reply, MRL, "no such camera device available (device=%d)", camDev);
        sendWarning(reply);
        finished = true;
      }
    }
    if (not finished)
    { // is destination image available
      imgDst = imgpool->getImage(dst, true);
      if (imgDst == NULL or dst == src)
      {
        snprintf(reply, MRL, "failed to get destination image (dst=%d), or is the same as source (src=%d)", dst, src);
        sendWarning(reply);
        finished = true;
      }
    }
    if (not finished)
    { // sort out destination format
      UImage ** imgBuf;
      bool needConvert;
      bool silent = msg->tag.getAttBool("silent", NULL, false);
      //
      needConvert = imgSrc->isBayer() or imgSrc->isYUV420();
      if (not needConvert)
        needConvert = not imgSrc->isBW() and strcasecmp(fmt, "bw") == 0;
      if (not needConvert)
        needConvert = not (imgSrc->isRGB() or imgSrc->isBGR());
      if (not needConvert)
        needConvert = imgSrc->isRGB() and strcasecmp(fmt, "bgr") == 0;
      if (not needConvert)
        needConvert = imgSrc->isBGR() and strcasecmp(fmt, "rgb") == 0;
      if (needConvert)
      { // convert and move source pointer to converted image
        imgBuf = imgSrc->getConvertBuffer();
        if (*imgBuf == NULL)
          *imgBuf = new UImage();
        if (strcasecmp(fmt, "bw") == 0)
          imgSrc->toBW(*imgBuf);
        else if (strcasecmp(fmt, "bgr") == 0)
          imgSrc->toBGR(*imgBuf);
        else
          imgSrc->toRGB(*imgBuf);
        imgSrc = *imgBuf;
      }
      // now imgSrc is in a usable format for rectification
      // do the rectification - using cvRemap(...)
      cam->removeRadialError(imgSrc, imgDst);
      // the call marks the destination image as updated for update event handling
      if (not silent)
        sendInfo("done");
    }
    // return true if the function is handled with a positive result
    return true;
  }


};


#ifdef LIBRARY_OPEN_NEEDED
/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFuncRectify' with your classname */
  return new UFuncRectify();
}
#endif
