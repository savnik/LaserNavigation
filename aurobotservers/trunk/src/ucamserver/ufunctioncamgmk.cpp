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

#include <umap4/upose.h>
#include <urob4/usmltag.h>
#include <urob4/uvarcalc.h>

#include "ufunctioncamgmk.h"
#include "ucalibrate.h"

///////////////////////////////////////////////////

// UFunctionCamGmk::UFunctionCamGmk(UCamPool * cams, UImagePool * images)
//   : UFunctionCamBase(cams, images), UPush()
// {
//   calib = NULL;
// }

///////////////////////////////////////////////////
//#include "../../plugin/aucamlocalize/ufunccamlocalize.h"

UFunctionCamGmk::UFunctionCamGmk()
{ // new version tag
  calib = NULL;
  setCommand("gmk gmkGet", "gmk", "guidemark ($Id: ufunctioncamgmk.cpp 111 2013-02-06 10:41:31Z jcan $)");
}

///////////////////////////////////////////////////

UFunctionCamGmk::~UFunctionCamGmk()
{}


///////////////////////////////////////////////////

// const char * UFunctionCamGmk::name()
// {
//   return "guidemark (2011-01-29 jca@elektro.dtu.dk)";
// }

///////////////////////////////////////////////////

// const char * UFunctionCamGmk::commandList()
// {
//   return "gmk gmkGet";
// }

void UFunctionCamGmk::createResources()
{
  // Create global variables - owned by this plug-in.
  varDefImg = addVar("defimg", 18.0, "d", "(r/w) Default image to use (-1: try default camera (guppy))");
  varImgSerial = addVar("imageSerial", "0 0", "d", "(r) camera number and serial number of image used in search");
  varCount = addVar("count", 0.0, "d", "(r) Number of guidemarks found");
  varIDs = addVar("IDs", 0.0, "d", "(r) ID codenumber (in decimal) of the found guidemarks");
  varGmkPose = addVar("gmkPose", "0 0 0 0 0 0", "6d", "(r) pose of first guidemark in robot coordinates (x,y,z,O,P,K) x=forward, y=left z=up [meter], O,P,K [radians] is rotation around the same 3 axes");
  varRobPose = addVar("robPose", "0 0 0 0 0 0", "6d", "(r) pose of robot seen from first guidemark also (x,y,z,O,P,K)");
  varCamPose = addVar("gmkcamPose", "0 0 0 0 0 0", "6d", "(r) pose of guidemark in camera coordinates (x,y,z,O,P,K)");
  varUseVert = addVar("vertical", 1.0, "d", "(r/w) expect vertical oriented guidemarks - better performance on diagonal guidemarks if 0");
  varUseDiag = addVar("diagonal", 1.0, "d", "(r/w) expect diagonal oriented guidemarks - better performance on vertical guidemarks if 0");
}


///////////////////////////////////////////////////

bool UFunctionCamGmk::handleCommand(UServerInMsg * msg, void * pushObj)
{  // message for this function
  return handleGmkGetCommand(msg, pushObj);
}

/////////////////////////////////////////////////////////////////////////////

bool UFunctionCamGmk::handleGmkGetCommand(UServerInMsg * msg, void * imgBase)
{
  bool result = false;
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 100;
  char attValue[VAL_BUFF_LNG];
  int imgDevice = -1;
  char posName[MAX_MOUNT_NAME_SIZE] = "";
  UCamPush * cam = NULL;
  UImage * img = NULL;
  bool ask4help = false;
  bool extraImg = false;
  bool gmkNear = false;
  bool poolSource = false;
  bool gmkRef = false;
  int poolImg = 0;
  double gmkBlock = 0.025;
  int i;
  const int MRL = 200;
  char reply[MRL];
  UBarcode gmkRob;
  UPosRot camPos;
  UPosRot ref; // reference position (zero)
  //
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  {
    if (strcasecmp(attName, "device") == 0)
      sscanf(attValue, "%d", &imgDevice);
    else if (strcasecmp(attName, "posName") == 0)
      strncpy(posName, attValue, MAX_MOUNT_NAME_SIZE);
    else if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strcasecmp(attName, "extra") == 0)
      extraImg = str2bool2(attValue, true);
    else if (strcasecmp(attName, "debug") == 0)
      extraImg = str2bool2(attValue, true);
    else if (strcasecmp(attName, "near") == 0)
      gmkNear = str2bool(attValue);
    else if (strcasecmp(attName, "block") == 0)
      gmkBlock = strtod(attValue, NULL);
    else if (strcasecmp(attName, "gmkRef") == 0)
      gmkRef = true;
    else if (strcasecmp(attName, "poolImg") == 0 or strcasecmp(attName, "img") == 0)
    {
      poolSource = true;
      poolImg = strtol(attValue, NULL, 10);
    }
  }
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"---- Available GMK options:\">\n");
    sendHelp(msg, "NB! looks for 7x7 frame size only\n");
    sendText(msg, "device=N         From device 'N', i.e. /dev/videoN\n");
    sendText(msg, "posName=name     From camera 'name', e.g. left for left camera\n");
    sendText(msg, "extra=true|false Extra images (def=false) (se poolList/poolget)\n");
    sendText(msg, "near=true|false  Optimize for near gmk (def=false)\n");
    sendText(msg, "block=W          Frame block size (def=0.025 m)\n");
    sendText(msg, "img=N            Use imagePool image N as image source (not from cam)\n");
    sendText(msg, "gmkRef           Get robot pos. in GMK ref system (else GMK in robot sys)\n");
    sendText(msg, "help             This help text\n");
    sendText(msg, "---\n");
    sendText(msg, "(NB! image is not rectified for lens distortion)\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg,"done");
  }
  else
  { // get image from camera
    if (not poolSource and varDefImg->getInt() >= 0)
    {
      poolSource = true;
      poolImg = varDefImg->getInt();
    }
    if (poolSource)
    { //
      //imgRaw = NULL;
      img = imgPool->getImage(poolImg, false);
      result = (img != NULL);
      if (result)
      { // should be in YUV format
        img->lock();
        if (img->isBGR())
          printf("Loaded BGR (converts to YUV)\n");
        else if (img->isRGB())
          printf("Loaded RGB (converts to YUV)\n");
        else
          printf("Converted to YUV)\n");
        img->toYUV(NULL);
        imgDevice = img->camDevice;
      }
      else
        printf("No image in image pool img=%d\n", poolImg);
      if (result)
      {
        cam = camPool->getCam(imgDevice);
        result = (cam != NULL);
        if (result == false)
          printf("No information found about camera device %d\n", imgDevice);
      }
    }
    else
    { // get new image from a camera
      img = imgPool->getImage(UCAM_IMS_CALIB_RAW, true);
      result = getCamAndRawImage(&cam, // result camera                  out
                        &img,          // result image                   out
                        &imgDevice,    // camera device number           in-out
                        //&imgRemRad,    // should/is radial error remoced in-out
                        imgBase,       // pushed image                   in
                        posName,       // camera position name           in
                        -1); //imgDevice + 3); // destination for rectified image (-1=no rectify)
      if (result)
        result = (img != NULL);
      else
        sendWarning("Failed to get image directly from camera source (try imagepool? using img=N)");
      if (result)
      { // need to be locked, as format is changing
        img->lock();
        result = img->toYUV(NULL);
      }
    }
    if (result)
    {
      if (strlen(img->name) < 2)
        snprintf(img->name, MAX_IMG_NAME_SIZE, "YUV24bit-%08lu-source", img->imageNumber);
    }
    if (calib == NULL)
      calib = new UCalibrate();
    if (result and not calib->isBufferImagesAvailable())
      result = calib->allocateBufferImages(imgPool);
    if (result)
    {
      result = calib->findGmk(cam, // camera ref
                              img, // image to analyse
                              7, // frame block size
                              2, // frame/code size factor
                              gmkBlock, // size of a frame block
                              16,     // max codes to look for
                              true,  // find codes
                              false, // find camera position
                              false, // find camera rotation
                              0.32,  // chart center height above floor
                              true, // find guidemark chart position
                              gmkNear, // near (else far) parameter setting
                              varUseVert->getBool(),
                              varUseDiag->getBool(),
                              extraImg
                              );
    }
    if (img != NULL)
      img->unlock();
    if (not result)
    {// send result to client
      sendWarning("None found");
            // send also as help
      sendMsg("<help subject=\"No Guidemarks found\">\n");
      sendMsg("</help>\n");
      if (img != NULL)
      {
        varImgSerial->setInt(img->imageNumber, 1);
        varImgSerial->setInt(img->camDevice, 0);
      }
      varCount->setInt(0, 0);
      varIDs->setSize(1);
      varIDs->setInt(0);
    }
    else
    {
      snprintf(reply, MRL, "<%s gmkCnt=\"%d\">\n",
              msg->tag.getTagName(),
              calib->getGmksCnt());
      sendMsg(msg, reply);
      //
      camPos.set(cam->getPos(), cam->getRot());

      varImgSerial->setInt(img->imageNumber, 1);
      varImgSerial->setInt(img->camDevice, 0);
      varCount->setInt(calib->getGmksCnt(), 0);
      varIDs->setSize(calib->getGmksCnt());
      for (i = 0; i < calib->getGmksCnt(); i++)
      {
        UPosRot pc, pg, pr;
        gmkRob = *calib->getGmk(i);
        varIDs->setInt(gmkRob.getCodeInt(), i, true);
        pc = gmkRob;
        // rotation is mirrored, so cnhange
        pc.Omega *= -1.0;
        pc.Phi *= -1.0;
        pc.Kappa = limitToPi(pc.Kappa + M_PI);
        // debug send unconverted
        // sendGmk(msg, &gmkRob, cam->getPosName(), cam->getDeviceNumber());
        // debug end
        // convert from camera to robot coordinates (and mirror guidemark)
        gmkRob.setCtoR(&camPos, gmkRob.getPosRot(), true);
        pg = gmkRob;
        //gmkRob.print("gmk pos");
        if (gmkRef)
        { // convert robot position as seen frm guidemark
          gmkRob.setRtoC(gmkRob.getPosRot(), &ref);
          pr = gmkRob;
          //gmkRob.print("rob pos");
        }
        sendGmk(msg, &gmkRob, cam->getPosName(), cam->getDev()->getDeviceNumber());
        if (not gmkRef)
        { // convert robot position as seen frm guidemark - if not done already
          gmkRob.setRtoC(gmkRob.getPosRot(), &ref);
          pr = gmkRob;
          //gmkRob.print("rob pos");
        }
        if (i == 0)
        { // save position of first GMK
          varGmkPose->set6D(&pg);
          varRobPose->set6D(&pr);
          varCamPose->set6D(&pc);
        }
        if (true)
        { // update var pool with all detected guidemarks
          UVarPool * vp;
          const int MIL = 40;
          char id[MIL];
          //
          snprintf(id, MIL, "gmk%lu", gmkRob.getCodeInt());
          vp = getVarPool()->getStruct(id);
          if (vp == NULL)
          {
            vp = getVarPool()->addStructLocal(id, "guidemark position", false);
            if (vp != NULL)
            { // 0=ID, 1=GMK, 2=rob, 3=cam 4=imgSerial, 5=time
              vp->addVar("ID", 0.0, "d", "(r) ID codenumber (in decimal) of the found guidemarks.");
              vp->addVarA("gmkPose", "0 0 0 0 0 0", "6d", "(r) pose of first guidemark in robot coordinates [x,y,z,O,P,K].");
              vp->addVarA("robPose", "0 0 0 0 0 0", "6d", "(r) pose of robot seen from first guidemark [x,y,z,O,P,K].");
              vp->addVarA("gmkcamPose", "0 0 0 0 0 0", "6d", "(r) pose of guidemark in camera coordinates [x,y,z,O,P,K].");
              vp->addVar("imageSerial", -1.0, "d", "(r) serial number for image with this code.");
              vp->addVar("time", -1.0, "t", "(r) image time of image with guidemark");
              vp->addVar("crcOK", 0.0, "d", "(r) 1: 8-bit CRC is sucessfull; 0: crc failed, ID includes crc.");
            }
          }
          if (vp != NULL)
          {
            UVariable * var;
            vp->setLocalVar("ID", gmkRob.getCodeInt(), true, UVariable::d, NULL);
            var = vp->getLocalVar(1);
            var->set6D(&pg);
            var = vp->getLocalVar(2);
            var->set6D(&pr);
            var = vp->getLocalVar(3);
            var->set6D(&pc);
            var = vp->getLocalVar(4);
            var->setInt(img->imageNumber, 0);
            var = vp->getLocalVar(5);
            var->setTime(img->getImageTime(), 0);
            var = vp->getLocalVar(6);
            var->setBool(gmkRob.isCrcValid(), 0);
          }
        }
      }
      //
      snprintf(reply, MRL, "</%s>\n", msg->tag.getTagName());
      sendMsg(reply);
      //
      //
      // send also as help
      sendMsg("<help subject=\"Guidemark reply\">\n");
      //
      camPos.set(cam->getPos(), cam->getRot());
      for (i = 0; i < calib->getGmksCnt(); i++)
      {
        gmkRob = *calib->getGmk(i);
        // convert from camera to robot coordinates (and mirror guidemark)
        gmkRob.setCtoR(&camPos, gmkRob.getPosRot(), true);
        //gmkRob.print("gmk pos");
        if (gmkRef)
        { // convert robot position as seen frm guidemark
          gmkRob.setRtoC(gmkRob.getPosRot(), &ref);
          //gmkRob.print("rob pos");
        }
        snprintf(reply, MRL, "gmk %d code %lu (%s) crc=%s at %6.2fx, %6.2fy, %6.2fz, %6.4fo, %6.4fp, %6.4fk\n",
                i, gmkRob.getCodeInt(), gmkRob.getCode(), bool2str(gmkRob.isCrcValid()),
                gmkRob.x, gmkRob.y, gmkRob.z, gmkRob.Omega, gmkRob.Phi, gmkRob.Kappa);
        //sendGmk(msg, &gmkRob, cam->getPosName(), cam->getDev()->getDeviceNumber());
        sendText(reply);
      }
      //
      sendMsg(msg, "</help>\n");
    }
  }
  //
  return result;
}

/////////////////////////////////////////////////////////////////////////////


bool UFunctionCamGmk::sendGmk(UServerInMsg * msg, UBarcode * gmk,
            const char * devName, int dev)
{
  bool result;
  const unsigned int MRL = 200;
  char reply[MRL];
  const char * gmkTag = "gmk";
  USmlTag tag;
  //
  snprintf(reply, MRL, "<%s code=\"%s\" id=\"%lu\" cam=\"%s\" device=\"%d\" crcOK=\"%d\">\n",
           gmkTag, gmk->getCode(), gmk->getCodeInt(), devName, dev, gmk->isCrcValid());
  result = sendMsg(msg, reply);
  if (result)
  { // send time
    tag.codeTime(gmk->getTime(), reply, MRL, NULL);
    result = sendMsg(msg, reply);
    // send position
  }
  if (result)
    result = sendMsg(msg, tag.codePosition(gmk->getPos(), reply, MRL, "gmkPosition"));
  if (result)
    result = sendMsg(msg, tag.codeRotation(gmk->getRot(), reply, MRL, "gmkRotation"));
  // send end tag
  snprintf(reply, MRL, "</%s>\n", gmkTag);
  result = sendMsg(msg, reply);
  return result;
}
