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

#include <string.h>

#include "ucomcam.h"


UComCam::UComCam()
{
  // PWC_WB_AUTO, PWC_WB_MANUAL, PWC_WB_INDOOR, PWC_WB_OUTDOOR, PWC_WB_FL (flurocent).
}

/////////////////////////////////////////////

UComCam::~UComCam()
{
}

//////////////////////////////////////////////

void UComCam::clear()
{ // clear all valid flags (16)
  deviceValid = false;
  deviceValue = -1;
  openValid = false;
  openValue = false;
  inCtrlValid = false;
  inCtrlValue = false;
  ledValid = false;
  ledOn = true;
  imageNumberValid = false;
  imageNumber = 0;
  compressionValid = false;
  compressionValue = 0;
  //
  sizeValid = false;
  sizeHeight = 240;
  sizeWidth = 320;
  fpsValid = false;
  fpsValue = 10;
  gainValid = false;
  gainAutomatic = true;
  gainValue = 0x2000;
  shutterValid = false;
  shutterAutomatic = true;
  shutterValue = 0x8000;
  //
  videoValid = false;
  gammaValue = 0xfff0;
  brightnessValue = 0x8000;
  contrastValue = 0x8000;
  colourValue = 0x8000;
  //
  whiteBalValid = false;
  // PWC_WB_AUTO, PWC_WB_MANUAL, PWC_WB_INDOOR, PWC_WB_OUTDOOR, PWC_WB_FL (flurocent).
  whiteBalMode = AU_WB_AUTO;
  whiteBalGainBlue = 0x4000;
  whiteBalGainRed = 0x4000;
  contourValid = false;
  contourValue = 0;
  //
  channelNumber = 0;
  channelNumberValid = false;
  channelName[0] = '\0';
  //
  nameValid = false;
  name[0] = 0;
  hdrValid = false;
  hdrMode = 0;
  saturationCtrlValid = false;
  saturationCtrlMode = 0;
  //
  streamValid = false;
  streamOn = false;
  streamFps = 1;
  //
  cmdValid = false;
  cmdUse = false;
  //cmdCmd[0] = 0;
}

//////////////////////////////////////////////

// unsigned int UComCam::pack(unsigned char * buff,
//                            unsigned int buffLng,
//                            bool toCamServer,
//                            unsigned int msgSerial)
// {
//   int result = -1;
//   const unsigned int fixLng = 52;
//   unsigned int nameStart;
//   unsigned int i;
//   bool isOK;
//   //
//   isOK = (buffLng >= fixLng);
//   if (not cmdValid)
//     cmdCmd[0] = '\0';
//   nameStart = fixLng + strlen(cmdCmd);
//   if (isOK and nameValid)
//     isOK = (buffLng >= (nameStart + strlen(name)));
//   //
//   if (isOK)
//   {
//     result = nameStart + strlen(name);
//     //ensure unused bits are zero
//     for (i = 2; i < fixLng; i++)
//       buff[i] = 0;
//     //
//     setLong(&buff[0], 2, result);   // length
//     // byte 2 is reserved for message serial (truncated)
//     buff[2] = (msgSerial & 0x7f);
//     //
//     // message type
//     if (toCamServer)
//       buff[3] = 0xc0 +  MSG_CAM_SET_CAMERA_STATE;
//     else
//       buff[3] = 0xc0 +  MSG_CAM_CAMERA_STATE;
//     //
//     // Valid flags (general)
//     if (deviceValid)
//     { // 9
//       buff[4] |= 0x01;
//       buff[9] = deviceValue & 0x3f;
//     }
//     if (openValid)
//     { // 10
//       buff[4] |= 0x02;
//       buff[10] = openValue;
//     }
//     if (inCtrlValid)
//     { // 11
//       buff[4] |= 0x04;
//       buff[11] = inCtrlValue;
//     }
//     if (ledValid)
//     { // 12
//       buff[4] |= 0x08;
//       buff[12] = ledOn;
//     }
//     if (imageNumberValid)
//     { // bytes 13, 14, 15, 16
//       buff[4] |= 0x10;
//       setLong(&buff[13], 4, imageNumber);
//     }
//     if (compressionValid)
//     { // 17
//       buff[4] |= 0x20;
//       buff[17] = compressionValue;
//       // debug
//       //printf("UComCam::pack: compression packed is %d\n", compressionValue);
//       // debug end
//     }
//     //
//     // size and gain
//     if (sizeValid)
//     { // 18, 19, 20, 21
//       buff[5] |= 0x01;
//       setLong(&buff[18], 2, sizeWidth);
//       setLong(&buff[20], 2, sizeHeight);
//     }
//     if (fpsValid)
//     { // 22
//       buff[5] |= 0x02;
//       buff[22] = fpsValue;
//     }
//     if (gainValid)
//     { // 23, 24, 25
//       buff[5] |= 0x04;
//       buff[23] = gainAutomatic;
//       setLong(&buff[24], 2, gainValue);
//     }
//     if (shutterValid)
//     { // 26, 27, 28
//       buff[5] |= 0x08;
//       buff[26] = shutterAutomatic;
//       setLong(&buff[27], 2, shutterValue);
//     }
//     if (videoValid)
//     { // 29, 30
//       buff[5] |= 0x10;
//       setLong(&buff[29], 2, gammaValue);
//       // also includes other video values - byte 46..51
//     }
//     if (whiteBalValid)
//     { // 31, 32, 33, 34, 35
//       buff[5] |= 0x20;
//       buff[31] = whiteBalMode;
//       setLong(&buff[32], 2, whiteBalGainRed);
//       setLong(&buff[34], 2, whiteBalGainBlue);
//     }
//     if (contourValid)
//     { // 36, 37
//       buff[5] |= 0x40;
//       setLong(&buff[36], 2, contourValue);
//     }
//     // other settings
//     if (nameValid)
//     { // 38 + end
//       buff[6] |= 0x01;
//       // position of start of camera name.
//       buff[38] = nameStart;
//       // debug
//       //printf("UComCam::pack name start at %d length %d\n", nameStart, result - nameStart);
//       // debug end
//       // length is to end of message
//       // camera name at the end
//       strncpy((char *)&buff[nameStart], name, result - nameStart);
//     }
//     if (hdrValid)
//     { // 39
//       buff[6] |= 0x02;
//       buff[39] = hdrMode;
//     }
//     if (saturationCtrlValid)
//     { // 40
//       buff[6] |= 0x04;
//       buff[40] = saturationCtrlMode;
//     }
//     if (streamValid)
//     { // 41, 42
//       buff[6] |= 0x08;
//       buff[41] = streamOn;
//       buff[42] = streamFps; // signed
//     }
//     if (cmdValid)
//     { // 43, 44, 45
//       buff[6] |= 0x10;
//       buff[43] = cmdUse;
//       buff[44] = fixLng;
//       buff[45] = strlen(cmdCmd);
//       strncpy((char *)&buff[fixLng], cmdCmd, buff[45]);
//       // debug
//       //printf("UComCam::pack Cmd pack %d lng %d use %s <%s>\n", fixLng, buff[45],
//       //           bool2str(cmdUse), cmdCmd);
//       // debug end
//     }
//     if (videoValid)
//     { // 46, 47, 48, 49, 50, 51
//       setLong(&buff[46], 2, brightnessValue);
//       setLong(&buff[48], 2, contrastValue);
//       setLong(&buff[50], 2, colourValue);
//     }
//     // reserved for other flags etc
//     buff[7] = 0;
//     buff[8] = 0;
//     //
//   }
//   return result;
// }
//
// ///////////////////////////////////////////////////
//
// int UComCam::peekDevice(unsigned char * buff)
// {
//   int result = -1;
//   if ((buff[4] & 0x01) != 0)
//     result = (buff[9] & 0x3f);
//   return result;
// }
//
// ///////////////////////////////////////////////////
//
// bool UComCam::unpack(unsigned char * buff)
// {
//   bool result = true;
//   unsigned int fixedLng;
//   unsigned int lng;
//   unsigned int cmdStart;
//   unsigned int cmdLength;
//   //
//   result = buff != NULL;
//   if (result)
//   {
//     //
//     lng = getLong(&buff[0], 2);
//     //
//     // Valid flags (general)
//     if ((buff[4] & 0x01) != 0)
//     {
//       deviceValid = true;
//       deviceValue = (buff[9] & 0x3f);
//     }
//     //
//     if ((buff[4] & 0x02) != 0)
//     {
//       openValid = true;
//       openValue = (buff[10] != 0);
//     }
//     if ((buff[4] & 0x04) != 0)
//     {
//       inCtrlValid = true;
//       inCtrlValue = (buff[11] != 0);
//     }
//     if ((buff[4] & 0x08) != 0)
//     {
//       ledValid = true;
//       ledOn = (buff[12] != 0);
//     }
//     if ((buff[4] & 0x10) != 0)
//     { // bytes 13, 14, 15, 16
//       imageNumberValid = true;
//       imageNumber = getLong(&buff[13], 4);
//     }
//     if ((buff[4] & 0x20) != 0)
//     {
//       compressionValid = true;
//       compressionValue = buff[17];
//       // debug
//       //printf("UComCam::unpack: compression unpacked is %d\n", compressionValue);
//       // debug end
//     }
//     //
//     // size and gain
//     if ((buff[5] & 0x01) != 0)
//     {
//       sizeValid = true;
//       sizeWidth = getLong(&buff[18], 2);
//       sizeHeight = getLong(&buff[20], 2);
//     }
//     if ((buff[5] & 0x02) != 0)
//     { // fps from camera device
//       fpsValid = true;
//       fpsValue = (char)buff[22]; // signed, but not used
//     }
//     if ((buff[5] & 0x04) != 0)
//     { // 23, 24, 25
//       gainValid = true;
//       gainAutomatic = (buff[23] != 0);
//       gainValue = getLong(&buff[24], 2);
//     }
//     if ((buff[5] & 0x08) != 0)
//     { // 26, 27, 28
//       shutterValid = true;
//       shutterAutomatic = (buff[26] != 0);
//       shutterValue = getLong(&buff[27], 2);
//     }
//     if ((buff[5] & 0x10) != 0)
//     { // 29, 30
//       videoValid = true;
//       gammaValue = getLong(&buff[29], 2);
//     }
//     if ((buff[5] & 0x20) != 0)
//     { // 31, 32, 33, 34, 35
//       whiteBalValid = true;
//       whiteBalMode = buff[31];
//       whiteBalGainRed = getLong(&buff[32], 2);
//       whiteBalGainBlue = getLong(&buff[34], 2);
//     }
//     if ((buff[5] & 0x40) != 0)
//     { // 36, 37
//       contourValid = true;
//       contourValue = getLong(&buff[36], 2);
//     }
//     // other settings
//     if ((buff[6] & 0x01) != 0)
//     { // 38 + end
//       nameValid = true;
//       // position of start of camera name.
//       fixedLng = buff[38];
//       result = fixedLng <= lng;
//       // camera name at the end
//       if (result)
//       { // copt name from end of buffer
//         strncpy(name, (char *)&buff[fixedLng], lng - fixedLng);
//         // terminate name
//         name[lng - fixedLng] = 0;
//       }
//     }
//     if ((buff[6] & 0x02) != 0)
//     { // 39
//       hdrValid = true;
//       hdrMode = buff[39];
//     }
//     if ((buff[6] & 0x04) != 0)
//     { // 40
//       saturationCtrlValid = true;
//       saturationCtrlMode = buff[40];
//     }
//     if ((buff[6] & 0x08) != 0)
//     { // 41, 42
//       streamValid = true;
//       streamOn = ((buff[41] & 0x01) != 0);
//       if ((buff[42] & 0x80) == 0)
//         // positive
//         streamFps = int(buff[42]);
//       else
//         // negative
//         streamFps = -int(buff[42] & 0x7f);
//     }
//     if ((buff[6] & 0x10) != 0)
//     { // 43, 44, 45
//       cmdValid = true;
//       cmdUse = ((buff[43] & 0x01) != 0);
//       cmdStart = buff[44];
//       cmdLength = mini(MAX_CMD_LENGTH-1, buff[45]);
//       if (cmdLength > 0)
//         strncpy(cmdCmd, (char *)&buff[cmdStart],cmdLength);
//       // terminate name
//       cmdCmd[cmdLength] = 0;
//       // debug
//       printf("---UComCam::unpack: command use %s <%s>\n",
//            bool2str(cmdUse), cmdCmd);
//       // debug end
//     }
//     if (videoValid)
//     { // combined as video settings - combined with gamma
//       // 46, 47, 48, 49, 50, 51
//       brightnessValue = getLong(&buff[46], 2);
//       contrastValue = getLong(&buff[48], 2);
//       colourValue = getLong(&buff[50], 2);
//     }
//     // reserved for other flags etc
//     buff[7] = 0;
//     buff[8] = 0;
//     //
//   }
//   return result;
// }

///////////////////////////////////////////////////////

bool UComCam::setFromCam(UCamDevBase * dev, bool probe,
                         bool clientInCtrl)
{
  bool result;
  int v;
  bool foundClosed;
  char * camName;
  const char * s2;
  //
  // debug
  //printf("UComCam::setFromCam started ... (cam = %x)\n", cam);
  // debug end
  //
  // clear first, so default is not valid
  clear();
  // get camera name - this opens device (if needed)
  foundClosed = (not dev->isCameraOpen());
  if (foundClosed and probe)
    // open camera for info gathering
    result = dev->openDeviceDefault();
  else
    result = true;
  //
  //result = dev->isInfoValid();
  //
  // debug
  //printf("UComCam::setFromCam ... camera open (%s) ...\n", bool2str(result));
  // debug end
  //
  //
  if (true)
  { // name
    camName = dev->getCameraName();
    if (camName != NULL)
    {
      strncpy(name, camName, dev->MAX_CAM_DEV_NAME_LENGTH);
      nameValid = true;
    }
    else
    {
      strncpy(name, "No cam device", dev->MAX_CAM_DEV_NAME_LENGTH);
      nameValid = true;
    }
    // device
    deviceValid = true;
    deviceValue = dev->getDeviceNumber();
    //
    openValid = dev->isCameraOpen(); // true;
    openValue = not foundClosed; //dev->isCameraOpen();
    // master controller
    inCtrlValid = true;
    inCtrlValue = clientInCtrl;
    //
    imageNumberValid = true;
    imageNumber = dev->getImageNumber();
    //
    if (openValid and openValue)
    { // most values are only valid if camera is open
      ledValid = true;
      //ledOn = dev->isLedOn(probe);
      //
      // data channel e.g. "television" (not valid for webcams)
      channelNumber = dev->getDataChannel();
      channelNumberValid = channelNumber >= 0;
      s2 = dev->getDataChannelName();
      if (s2 != NULL)
        strncpy(channelName, dev->getDataChannelName(), MAX_CHANNEL_NAME_LENGTH);
      else
        channelName[0] = '\0';
  // debug
  //printf("UComCam::setFromCam ... compression (probe %s) ...\n", bool2str(probe));
  // debug end
  //
      compressionValid = true;
      compressionValue = dev->getCompPref(probe);
      //
      // debug
      // printf("UComCam::setFromCam compression value got %d\n", compressionValue);
      // debug end
      //
      sizeValid = true;
      sizeWidth = dev->getWidth();
      sizeHeight = dev->getHeight();
      //
      fpsValid = true;
      fpsValue = dev->getFrameRate();
      //
  //
  // debug
  //printf("UComCam::setFromCam ... gain (probe %s) ...\n", bool2str(probe));
  // debug end
  //
      gainValid = true;
      v = dev->getGain(probe, NULL, &gainAutomatic);
      //gainAutomatic = (v < 0);
      gainValue = absi(v);
      //
      shutterValid = true;
  //
  // debug
  // printf("UComCam::setFromCam ... shutter (probe %s) ...\n", bool2str(probe));
  // debug end
  //
      // can not be probed on PWC devices!, but last set value is returned
      shutterValue = absi(dev->getShutter(probe, NULL, &shutterAutomatic));
      //shutterAutomatic = (dev->getShutter(false) <= 0);
      //
  //
  // debug
  //printf("UComCam::setFromCam ... gamma (probe %s) ...\n", bool2str(probe));
  // debug end
  //
      videoValid = true;
      gammaValue = dev->getGamma(probe);
  //
  // debug
  //printf("UComCam::setFromCam ... brightness (probe %s) ...\n", bool2str(probe));
  // debug end
  //
      brightnessValue = dev->getBrightness(false);
  //
  // debug
  //printf("UComCam::setFromCam ... contrast (probe %s) ...\n", bool2str(probe));
  // debug end
  //
      contrastValue = dev->getContrast(false);
  //
  // debug
  //printf("UComCam::setFromCam ... colour (probe %s) ...\n", bool2str(probe));
  // debug end
  //
      colourValue = dev->getColour(false);
      //
  //
  // debug
  //printf("UComCam::setFromCam ... white ballance (probe %s) ...\n", bool2str(probe));
  // debug end
  //
      whiteBalValid = dev->getWhiteBalance(probe,
               (int *)&whiteBalGainRed,
               (int *)&whiteBalGainBlue,
               (int *)&whiteBalMode);
      //

  //
  // debug
  //printf("UComCam::setFromCam ... contour (probe %s) ...\n", bool2str(probe));
  // debug end
  //
      contourValid = true;
      contourValue = dev->getContour(probe);
      //
  //
  // debug
  //printf("UComCam::setFromCam ... hdr/sat mode (probe %s) ...\n", bool2str(probe));
  // debug end
  //
/*      hdrValid = true;
      hdrMode = dev->getHdrMode();
      //
      saturationCtrlValid = true;
      saturationCtrlMode = dev->getStaurationCtrlMode();*/
    }
  }
  /*
  if (foundClosed and dev->isCameraOpen())
    // if found closed, then lose on exit
    dev->setDeviceClosed();
  */
  //
  //
  // debug
  //printf("UComCam::setFromCam ... returned (%s) ...\n", bool2str(result));
  // debug end
  //
  return result;
}


///////////////////////////////////////////////////////

bool UComCam::setCamDevice(UCamDevBase * dev)
{ // set camera from these data
  bool result;
  int w=0, h = 0, fps = 0;
  //
  // debug
  //printf("UComCam::setCamDevice: in\n");
  // debug end
  // do not check to see if it is the right device
  result = (dev != NULL);
  if (result)
  {
    // set size
    if (sizeValid or fpsValid)
    {
      if (sizeValid)
      {
        w = sizeWidth;
        h = sizeHeight;
      }
      if (fpsValid)
        fps = fpsValue;
      // debug
      //printf("UComCam::setCamDevice: set w,h,fps ...\n");
      // debug end
      dev->setDevice(w, h, fps);
    }
    if (openValid and not openValue)
    { // close is requested
      // debug
      //printf("UComCam::setCamDevice: close device ...\n");
      // debug end
      dev->closeDevice();
    }
    else
    { // if not close order, then open
      if (not dev->isCameraOpen())
        dev->openDevice();
    }
  }
  // other settings that require camera to be open
  if (result and dev->isCameraOpen())
  { // compression factor
    if (compressionValid)
    {
      // debug
      //printf("UComCam::setCamDevice: set comp value ...\n");
      // debug end
      dev->setCompPref(compressionValue);
    }
    // contour
    if (contourValid)
    {
      // debug
      //printf("UComCam::setCamDevice: set compression ...\n");
      // debug end
      dev->setContour(contourValue);
    }
    // gain
    if (gainValid)
    {
      // debug
      //printf("UComCam::setCamDevice: set gain ...\n");
      // debug end
      if (gainAutomatic)
        dev->setGain(-1);
      else
        dev->setGain(gainValue);
    }
    // shutter
    if (shutterValid)
    {
      // debug
      //printf("UComCam::setCamDevice: set shutter ...\n");
      // debug end
      if (shutterAutomatic)
        dev->setShutter(-1);
      else
        dev->setShutter(shutterValue);
    }
    // gamma
    if (videoValid)
    {
      // debug
      //printf("UComCam::setCamDevice: set video - 4 settings ...\n");
      // debug end
      dev->setVideoCap(brightnessValue, contrastValue,
                       gammaValue, colourValue);
    }
    // white ballance
    if (whiteBalValid)
    {
      // debug
      //printf("UComCam::setCamDevice: set white balance ...\n");
      // debug end
      dev->setWhiteBalance(whiteBalMode,
              whiteBalGainRed, whiteBalGainBlue);
    }
    // image serial number
    if (imageNumberValid)
    {
      // debug
      //printf("UComCam::setCamDevice: set image number ...\n");
      // debug end
      dev->setImageNumber(imageNumber);
    }
    // LED
/*    if (ledValid)
    {
      // debug
      //printf("UComCam::setCamDevice: set led ...\n");
      // debug end
      dev->setLedOn(ledOn);
    }*/
    if (channelNumberValid)
    {
      dev->setDataChannel(channelNumber);
    }
    // dhr
/*    if (hdrValid)
    {
      // debug
      //printf("UComCam::setCamDevice: set HDR mode %d ...\n", hdrMode);
      // debug end
      dev->setHdrMode(hdrMode);
    }
    // saturation control
    if (saturationCtrlValid)
    {
      // debug
      //printf("UComCam::setCamDevice: set saturation control ...\n");
      // debug end
      dev->setSaturationCtrlMode(saturationCtrlMode);
    }*/
  }
  return result;
}

//////////////////////////////////////////////

void UComCam::print(const char * prestring)
{
  printf("%s\n", prestring);
  //deviceValid = false;
  //deviceValue = 0;
  printf("-Device valid (%s) and value %d\n",
              bool2str(deviceValid), deviceValue);
  //openValid = false;
  //openValue = false;
  printf("-Open valid (%s) and open (%s)\n",
              bool2str(openValid), bool2str(openValue));
  //inCtrlValid = false;
  //inCtrlValue = false;
  printf("-InCtrl valid (%s) and value (%s)\n",
              bool2str(inCtrlValid), bool2str(inCtrlValue));
  //ledValid = false;
  //ledOn = true;
  printf("-LED valid (%s) and on (%s)\n",
              bool2str(ledValid), bool2str(ledOn));
  //imageNumberValid = false;
  //imageNumber = 0;
  printf("-Image number valid (%s) is %ld\n",
               bool2str(imageNumberValid), imageNumber);
  //compressionValid = false;
  //compressionValue = 0;
  printf("-Compression valid (%s) and value (%d)\n",
                 bool2str(compressionValid), compressionValue);
  //
  //sizeValid = false;
  //sizeHeight = 240;
  //sizeWidth = 320;
  printf("-Size valid (%s) and (w,h)(%d,%d)\n",
                 bool2str(sizeValid), sizeWidth, sizeHeight);
  //fpsValid = false;
  //fpsValue = 10;
  printf("-FPS valid (%s) and is %d fps\n",
               bool2str(fpsValid), fpsValue);
  //gainValid = false;
  //gainAutomatic = true;
  //gainValue = 0x2000;
  printf("-Gain valid (%s) and auto (%s), gain 0x%x\n",
       bool2str(gainValid), bool2str(gainAutomatic), gainValue);
  //shutterValid = false;
  //shutterAutomatic = true;
  //shutterValue = 0x8000;
  printf("-Shutter valid (%s) and auto (%s), value 0x%x\n",
       bool2str(shutterValid), bool2str(shutterAutomatic), shutterValue);
  //gammaValid = false;
  //gammaValue = 0;
  printf("-Video valid (%s) and bright: 0x%04x, conrast 0x%04x, gamma 0x%04x, color 0x%04x\n",
       bool2str(videoValid), brightnessValue, contrastValue, gammaValue, colourValue);
  //whiteBalValid = false;
  // PWC_WB_AUTO, PWC_WB_MANUAL, PWC_WB_INDOOR, PWC_WB_OUTDOOR, PWC_WB_FL (flurocent).
  //whiteBalMode = PWC_WB_AUTO;
  //whiteBalGainBlue = 0x4000;
  //whiteBalGainRed = 0x4000;
  printf("-White bal valid (%s), mode (%d:%s), gain red 0x%x, gain blue 0x%x\n",
     bool2str(whiteBalValid),
     whiteBalMode, getWhiteModeStr(whiteBalMode),
     whiteBalGainRed, whiteBalGainBlue);
  //contourValid = false;
  //contourValue = 0;
  printf("-Contour valid (%s) value 0x%x\n",
      bool2str(contourValid), contourValue);
  // data channel number
  printf("-Channel valid (%s) value %d name %s\n",
         bool2str(channelNumberValid),
         channelNumber, channelName);
  //
  //nameValid = false;
  //name[0] = 0;
  printf("-Name valid (%s), is '%s'\n", bool2str(nameValid), name);
  //hdrValid = false;
  //hdrMode = 0;
  printf("-HDR valid (%s) mode %d\n", bool2str(hdrValid), hdrMode);
  //saturationCtrlValid = false;
  //saturationCtrlAuto = false;
  printf("-Saturation ctrl valid (%s) mode is (%d) %s\n",
       bool2str(saturationCtrlValid), saturationCtrlMode,
       getSaturationModeStr(saturationCtrlMode));
  //
  // stream mode
  printf("-Streaming valid (%s) is on (%s) at %d fps\n",
       bool2str(streamValid), bool2str(streamOn), streamFps);
  // command
/*  printf("-Command valid (%s) is on (%s) cmd: '%s'\n",
       bool2str(cmdValid), bool2str(cmdUse), cmdCmd);*/
}

////////////////////////////////////////////////////////

const char * UComCam::getWhiteModeStr(int mode)
{
  const char * result;
  //
  switch (mode)
  {
    case AU_WB_AUTO:
      result = "Auto";
      break;
    case AU_WB_MANUAL:
      result = "Manual";
      break;
    case AU_WB_INDOOR:
      result = "Indoor";
      break;
    case AU_WB_OUTDOOR:
      result = "Outdoor";
      break;
    case AU_WB_FL:
      result = "Fluorcent";
      break;
    default:
      result = "Unknown";
      break;
  }
  return result;
}

///////////////////////////////////////////////////////////

const char * UComCam::getSaturationModeStr(int mode)
{
  const char * result = NULL;
  switch (mode)
  {
    case 0:
      result = "none";
      break;
    case 1:
      result = "disp only";
      break;
    case 2:
      result = "shut";
      break;
    case 3:
      result = "max";
      break;
    case 4:
      result = "high frac.";
      break;
    case 5:
      result = "Avg.";
      break;
    case 6:
      result = "Avg.low.";
      break;
    default:
      result = "unknown";
      break;
  }
  return result;
}

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

bool UComCamSml::pack(UCamMounted * cam, int client, char * message, int messageSize)
{
  bool result;
  char * c = message;
  int l = 0;
  UComCamSml fromCam;
  const int MSL=20;
  char s[MSL];
  //
  result = fromCam.setFromCam(cam, false, cam->isClientInCharge(client));
  //
  if (true)
  {
    if (posNameValid)
    {
      snprintf(c, messageSize - l, " posName=\"%s\"", fromCam.posName);
      l += strlen(c);
      c = &message[l];
    }
    if (videoValid)
    {
      snprintf(c, messageSize - l, " gamma=\"%u\" brightness=\"%u\" contrast=\"%u\" colour=\"%u\"",
          fromCam.gammaValue, fromCam.brightnessValue,
          fromCam.contrastValue, fromCam.colourValue);
      l += strlen(c);
      c = &message[l];
    }
    if (contourValid)
    {
      snprintf(c, messageSize - l, " colntour=\"%u\"", fromCam.contourValue);
      l += strlen(c);
      c = &message[l];
    }
    if (deviceValid)
    {
      snprintf(c, messageSize - l, " device=\"%u\"", fromCam.deviceValue);
      l += strlen(c);
      c = &message[l];
    }
    if (fpsValid)
    {
      snprintf(c, messageSize - l, " fps=\"%u\"", fromCam.fpsValue);
      l += strlen(c);
      c = &message[l];
    }
    if (sizeValid)
    {
      snprintf(c, messageSize - l, " height=\"%u\" width=\"%u\"",
           fromCam.sizeHeight, fromCam.sizeWidth);
      l += strlen(c);
      c = &message[l];
    }
    if (gainValid)
    {
      snprintf(c, messageSize - l, " gainAuto=\"%s\" gain=\"%d\"",
           bool2str(fromCam.gainAutomatic), fromCam.gainValue);
      l += strlen(c);
      c = &message[l];
    }
    if (channelNumberValid)
    {
      snprintf(c, messageSize - l, " channelNumber=\"%d\" channelName=\"%s\"",
               fromCam.channelNumber, fromCam.channelName);
      l += strlen(c);
      c = &message[l];
    }
    if (hdrValid)
    {
      snprintf(c, messageSize - l, " hdrMode=\"%u\"", fromCam.hdrMode);
      l += strlen(c);
      c = &message[l];
    }
    if (imageNumberValid)
    {
      snprintf(c, messageSize - l, " serial=\"%lu\" ",
            fromCam.imageNumber);
      l += strlen(c);
      c = &message[l];
    }
    if (inCtrlValid)
    {
      snprintf(c, messageSize - l, " inCharge=\"%s\"",
           bool2str(cam->isClientInCharge(client)));
      l += strlen(c);
      c = &message[l];
    }
    if (ledValid)
    {
      snprintf(c, messageSize - l, " ledOn=\"%s\"", bool2str(fromCam.ledOn));
      l += strlen(c);
      c = &message[l];
    }
    if (nameValid)
    {
      snprintf(c, messageSize - l, " name=\"%s\"", fromCam.name);
      l += strlen(c);
      c = &message[l];
    }
    if (openValid)
    {
      snprintf(c, messageSize - l, " open=\"%s\"", bool2str(fromCam.openValue));
      l += strlen(c);
      c = &message[l];
    }
    if (saturationCtrlValid)
    {
      snprintf(c, messageSize - l, " saturationCtrlMode=\"%u\"", fromCam.saturationCtrlMode);
      l += strlen(c);
      c = &message[l];
    }
    if (shutterValid)
    {
      snprintf(c, messageSize - l, " shutterAuto=\"%s\" shutter=\"%d\"",
               bool2str(fromCam.shutterAutomatic), fromCam.shutterValue);
      l += strlen(c);
      c = &message[l];
    }
    if (camParValid)
    {
      snprintf(c, messageSize - l, " focalLength=\"%g\" K1=\"%e\" K2=\"%e\" headX=\"%g\" headY=\"%g\"",
          fromCam.camParFocalLength,
          fromCam.camParRadialK1,
          fromCam.camParRadialK2,
          fromCam.camParHeadX,
          fromCam.camParHeadY
          );
      l += strlen(c);
      c = &message[l];
    }
    if (relPosValid)
    {
      snprintf(c, messageSize - l, " posX=\"%.3f\" posY=\"%.3f\" posZ=\"%.3f\"",
          fromCam.relPos.x,
          fromCam.relPos.y,
          fromCam.relPos.z
          );
      l += strlen(c);
      c = &message[l];
    }
    if (relRotValid)
    {
      snprintf(c, messageSize - l, " rotOmega=\"%.6f\" rotPhi=\"%.6f\" rotKappa=\"%.6f\"",
          fromCam.relRot.Omega,
          fromCam.relRot.Phi,
          fromCam.relRot.Kappa
          );
      l += strlen(c);
      c = &message[l];
    }

    if (whiteBalValid)
    {
      switch (fromCam.whiteBalMode)
      {
        case AU_WB_AUTO:    sprintf(s, "auto");  break;
        case AU_WB_MANUAL:  sprintf(s, "manual"); break;
        case AU_WB_INDOOR:  sprintf(s, "indoor"); break;
        case AU_WB_OUTDOOR: sprintf(s, "outdoor"); break;
        case AU_WB_FL:      sprintf(s, "flurocent"); break;
        default:             sprintf(s, "unknown  "); break;
      }
      snprintf(c, messageSize - l, " whiteBalMode=\"%s\" whiteBalRed=\"%u\" whiteBalBlue=\"%u\"",
        s, fromCam.whiteBalGainRed, fromCam.whiteBalGainBlue);
      l += strlen(c);
      c = &message[l];
    }
    // pan tilt
    if (panTiltValid)
    { // send pan-tilt status
      if (fromCam.panTiltSupportValue)
        snprintf(c, messageSize - l, " panTiltSupport=\"true\""
           " panPos=\"%d\" panMin=\"%d\" panMax=\"%d\""
           " tiltPos=\"%d\" tiltMin=\"%d\" tiltMax=\"%d\"",
          fromCam.panPosValue, fromCam.panMinValue, fromCam.panMaxValue,
          fromCam.tiltPosValue, fromCam.tiltMinValue, fromCam.tiltMaxValue);
      else
        snprintf(c, messageSize - l, " panTiltSupport=\"false\"");
      l += strlen(c);
      c = &message[l];
    }
  }
  if (not result)
    snprintf(c, messageSize - l, " deviceReal=\"false\"");
  //
  return result;
}

///////////////////////////////////////////////////////////////

bool UComCamSml::unpack(UServerInMsg * msg)
{
  USmlTagIn * tag;
  const int MVL = 200;
  char value[MVL];
  char att[MAX_SML_NAME_LENGTH];
  int n, v, used = 0;
  bool ask4help = false;
  //
  tag = msg->getTag();
  tag->reset();
  while (tag->getNextAttribute(att, value, MVL))
  {
    used++;
    // videoValid = false;
    // gammaValue = 0xfff0;
    // brightnessValue = 0x8000;
    // contrastValue = 0x8000;
    // colourValue = 0x8000;
    if (strcasecmp(att, "help") == 0)
      ask4help = true;
    else if (strncasecmp(att, "colour", 4) == 0)
    {
      sscanf(value, "%u", &colourValue);
      videoValid = true;
    }
    else if (strcasecmp(att, "gamma") == 0)
    {
      sscanf(value, "%u", &gammaValue);
      videoValid = true;
    }
    else if (strncasecmp(att, "brightness", 6) == 0)
    {
      sscanf(value, "%u", &brightnessValue);
      videoValid = true;
    }
    else if (strcasecmp(att, "contrast") == 0)
    {
      sscanf(value, "%u", &contrastValue);
      videoValid = true;
    }
    else if (strcasecmp(att, "video") == 0)
      videoValid = true;
    //
    else if (strcasecmp(att, "contour") == 0)
    { // edge enhancement
      sscanf(value, "%u", &contourValue);
      contourValid = true;
    }
    else if (strcasecmp(att, "camera") == 0)
    { // value can be either a number, a name or nothing
      n = sscanf(value, "%d", &deviceValue);
      if (n == 1)
        deviceValid = true;
      else
        posNameValid = true;
      if (posNameValid and strlen(value) > 0)
        strncpy(posName, value, MAX_MOUNT_NAME_SIZE);
    }
    else if (strcasecmp(att, "posName") == 0)
    {
      posNameValid = true;
      if (strlen(value) > 0)
        strncpy(posName, value, MAX_MOUNT_NAME_SIZE);
    }
    else if (strcasecmp(att, "device") == 0)
    {
      sscanf(value, "%d", &deviceValue);
      deviceValid = true;
    }
    else if (strcasecmp(att, "fps") == 0)
    {
      sscanf(value, "%d", &fpsValue);
      fpsValid = true;
    }
    else if (strcasecmp(att, "gain") == 0)
    {
      n = sscanf(value, "%d", &v);
      if (n == 1)
      {
        gainAutomatic = (v == -1);
        if (not gainAutomatic)
          gainValue = (unsigned int)v;
      }
      gainValid = true;
    }
    else if (strcasecmp(att, "channel") == 0)
    {
      sscanf(value, "%d", &channelNumber);
      channelNumberValid = true;
    }
    else if (strcasecmp(att, "hdrMode") == 0)
    {
      sscanf(value, "%u", &hdrMode);
      hdrValid = true;
    }
    else if (strncasecmp(att, "serial", 8) == 0)
    {
      sscanf(value, "%lu", &imageNumber);
      imageNumberValid = true;
    }
    else if (strcasecmp(att, "inCharge") == 0)
    {
      inCtrlValid = true;
    }
    else if (strcasecmp(att, "ledOn") == 0)
    {
      ledOn = str2bool(value);
      ledValid = true;
    }
    else if (strcasecmp(att, "name") == 0)
    {
      nameValid = true;
      posNameValid=true;
      if (strlen(value) > 0)
        strncpy(posName, value, MAX_MOUNT_NAME_SIZE);
    }
    else if (strcasecmp(att, "open") == 0)
    {
      openValid = true;
      openValue = true;
    }
    else if (strncasecmp(att, "close", 5) == 0)
    {
      openValid = true;
      openValue = false;
    }
    else if (strncasecmp(att, "saturationCtrlMode", 5) == 0)
    {
      sscanf(value, "%u", &saturationCtrlMode);
      saturationCtrlValid = true;
    }
    else if (strcasecmp(att, "shutter") == 0)
    {
      shutterValid = true;
      n = sscanf(value, "%d", &v);
      if (n == 1)
      {
        shutterAutomatic = (v == -1);
        if (not shutterAutomatic)
          shutterValue = (unsigned int)v;
      }
    }
    else if (strcasecmp(att, "width") == 0)
    {
      n = sscanf(value, "%u", &sizeWidth);
      if (n == 1 and not sizeValid)
          sizeHeight = (sizeWidth * 3) / 4;
      sizeValid = true;
    }
    else if (strcasecmp(att, "height") == 0)
    {
      n = sscanf(value, "%u", &sizeHeight);
      if (n == 1 and not sizeValid)
          sizeWidth = (sizeHeight * 4)/3;
      sizeValid = true;
    }
    else if (strcasecmp(att, "size") == 0)
    {
      sizeValid = true;
    }
    // white ballance
    else if (strcasecmp(att, "whiteBal") == 0)
    {
      n = sscanf(value, "%u", &whiteBalMode);
      whiteBalValid = true;
      if (n != 1)
      {
        if (strcasecmp(value, "auto") == 0)
          whiteBalMode = AU_WB_AUTO;
        else if (strncasecmp(value, "manual", 3) == 0)
          whiteBalMode = AU_WB_MANUAL;
        else if (strncasecmp(value, "indoor", 2) == 0)
          whiteBalMode = AU_WB_INDOOR;
        else if (strncasecmp(value, "outdoor", 3) == 0)
          whiteBalMode = AU_WB_OUTDOOR;
        else if (strncasecmp(value, "flurocent", 2) == 0)
          whiteBalMode = AU_WB_FL;
        else
          whiteBalMode = AU_WB_AUTO;
      }
    }
    else if (strcasecmp(att, "whiteBalRed") == 0)
    {
      whiteBalValid = true;
      n = sscanf(value, "%u", &whiteBalGainRed);
      if (n == 1)
        whiteBalMode = AU_WB_MANUAL;
    }
    else if (strcasecmp(att, "whiteBalBlue") == 0)
    {
      whiteBalValid = true;
      n = sscanf(value, "%u", &whiteBalGainBlue);
      if (n == 1)
        whiteBalMode = AU_WB_MANUAL;
    }
    else if (strcasecmp(att, "posX") == 0)
    {
      relPosXValid = true;
      sscanf(value, "%lf", &relPos.x);
    }
    else if (strcasecmp(att, "posY") == 0)
    {
      relPosYValid = true;
      sscanf(value, "%lf", &relPos.y);
    }
    else if (strcasecmp(att, "posZ") == 0)
    {
      relPosZValid = true;
      sscanf(value, "%lf", &relPos.z);
    }
    else if (strcasecmp(att, "pos") == 0)
    { // if just request position
      relPosValid = true;
    }
    else if (strncasecmp(att, "rotPhi", 4) == 0)
    {
      relRotPValid = true;
      sscanf(value, "%lf", &relRot.Phi);
    }
    else if (strncasecmp(att, "rotOmega", 4) == 0)
    {
      relRotOValid = true;
      sscanf(value, "%lf", &relRot.Omega);
    }
    else if (strncasecmp(att, "rotKappa", 4) == 0)
    {
      relRotKValid = true;
      sscanf(value, "%lf", &relRot.Kappa);
    }
    else if (strcasecmp(att, "rot") == 0)
    { // if just request position
      relRotValid = true;
    }
    else if (strncasecmp(att, "camPar", 6) == 0)
    {
      camParValid = true;
    }
    else if (strncasecmp(att, "focalLength", 3) == 0)
    {
      camParValid = true;
      sscanf(value, "%f", &camParFocalLength);
    }
    else if (strcasecmp(att, "k1") == 0)
    {
      camParValid = true;
      sscanf(value, "%f", &camParRadialK1);
    }
    else if (strcasecmp(att, "k2") == 0)
    {
      camParValid = true;
      sscanf(value, "%f", &camParRadialK2);
    }
    else if (strcasecmp(att, "headX") == 0)
    {
      camParValid = true;
      sscanf(value, "%f", &camParHeadX);
    }
    else if (strcasecmp(att, "headY") == 0)
    {
      camParValid = true;
      sscanf(value, "%f", &camParHeadY);
    }
    // pan tilt
    else if (strcasecmp(att, "panTiltHome") == 0)
    {
      panTiltHome = true;
    }
    else if (strcasecmp(att, "panPos") == 0)
    {
      panPosValid = true;
      sscanf(value, "%d", &panPosValue);
    }
    else if (strcasecmp(att, "panRel") == 0)
    {
      panRelValid = true;
      sscanf(value, "%d", &panRelValue);
    }
    else if (strcasecmp(att, "tiltPos") == 0)
    {
      tiltPosValid = true;
      sscanf(value, "%d", &tiltPosValue);
    }
    else if (strcasecmp(att, "tiltRel") == 0)
    {
      tiltRelValid = true;
      sscanf(value, "%d", &tiltRelValue);
    }
    else if (strncasecmp(att, "panTilt", 7) == 0)
    { // if none else, then status request for pan-tilt
      panTiltValid = true;
    }

    else
    { // add not used attributes to a notUsedList
      if (notUsedPars[0] != 0)
        strncat(notUsedPars, " ", MAX_MESSAGE_LENGTH_TO_CAM);
      strncat(notUsedPars, att, MAX_MESSAGE_LENGTH_TO_CAM);
      used--;
    }
  }

  //
  return ask4help;
}

///////////////////////////////////////////////////////////////

void UComCamSml::clear()
{
  UComCam::clear();
  notUsedPars[0] = 0;
  relPosValid = false;
  relPosXValid = false;
  relPosYValid = false;
  relPosZValid = false;
  relPos.clear();
  relRotValid = false;
  relRotOValid = false;
  relRotPValid = false;
  relRotKValid = false;
  relRot.clear();
  camParValid= false;
  camParFocalLength = 0.0;
  camParRadialK1 = 0.0;
  camParRadialK2 = 0.0;
  camParHeadX = 0.0;
  camParHeadY = 0.0;
  posNameValid = false;
  posName[0] = 0;
  //-
  panPosValid = false;
  panRelValid = false;
  panTiltValid = false; // requst/status
  tiltPosValid = false;
  tiltRelValid = false;
  panTiltHome = false;
}

///////////////////////////////////////////////////////////////

bool UComCamSml::setFromCam(UCamMounted * cam, bool probe, int client)
{
  bool result;
  UCamPar * camPar;

  result = UComCam::setFromCam(cam->getDev(), probe, client);
  if (true)
  {
     relPosValid = true;
     relPos = cam->getPos();
     relRot = cam->getRot();
     //
     camPar = cam->getCamPar();
     camParValid = camPar->isValid();
     camParFocalLength = camPar->getFocalLength();
     camParRadialK1 = camPar->getK1();
     camParRadialK2 = camPar->getK2();
     camParHeadX = camPar->getHx();
     camParHeadY = camPar->getHy();
     //
     posNameValid = true;
     strcpy(posName, cam->getPosName());
     //
     // pan-tilt
     panTiltValid = true;
     cam->pantiltUpdatePosition();
     panTiltSupportValue = cam->isPantiltSupported();
     if (panTiltSupportValue)
     {
       panMaxValue = cam->getPanMaxRange();
       panMinValue = cam->getPanMinRange();
       panPosValid = true;
       panPosValue = cam->getPanPos();
       tiltMaxValue = cam->getTiltMaxRange();
       tiltMinValue = cam->getTiltMinRange();
       tiltPosValid = true;
       tiltPosValue = cam->getTiltPos();
     }
  }
  return result;
}

/////////////////////////////////////////////////

bool UComCamSml::setCamDevice(UCamMounted * cam)
{
  bool result;
  UPosition pos;
  URotation rot;
  UCamPar camPar;
  float k1, k2, f, hx, hy;
  //
  result = UComCam::setCamDevice(cam->getDev());
  //
  // NB! can not set relative camera position from
  //     other components
  // NB! can not set camera parameters from clients
  //
  // pan-tilt
  if (cam->isPantiltSupported())
  {
    if (panTiltHome)
      cam->pantiltToHomePosition();
    if (panRelValid or tiltRelValid)
    {
      if (not panRelValid)
        panRelValue = 0;
      if (not tiltRelValid)
        tiltRelValue = 0;
      cam->pantiltSetPosition(true, panRelValue, tiltRelValue);
    }
    if (panPosValid or tiltPosValid)
    {
      if (not panPosValid)
        panPosValue = cam->getPanPos();
      if (not tiltPosValid)
        tiltPosValue = cam->getTiltPos();
      cam->pantiltSetPosition(false, panPosValue, tiltPosValue);
    }
  }
  if (relPosXValid or relPosYValid or relPosZValid or
      relRotOValid or relRotPValid or relRotKValid)
  { // @todo lock on camera parameters?
    // cam->lock();
    pos = cam->getPos();
    rot = cam->getRot();
    if (relPosXValid)
      pos.x = relPos.x;
    if (relPosYValid)
      pos.y = relPos.y;
    if (relPosZValid)
      pos.z = relPos.z;
    if (relRotOValid)
      rot.Omega = relRot.Omega;
    if (relRotPValid)
      rot.Phi = relRot.Phi;
    if (relRotKValid)
      rot.Kappa = relRot.Kappa;
    cam->setPosOnRobot(&pos, &rot);
    // cam->unlock();
  }
  if (camParValid)
  { //
    camPar = *cam->getCamPar();
    camPar.setPixelSize(1.0);
    f = camPar.getFocalLength();
    k1 = camPar.getK1();
    k2 = camPar.getK2();
    hx = camPar.getHx();
    hy = camPar.getHy();
    if (camParFocalLength > 0.0)
      f = camParFocalLength;
    if (camParRadialK1 > 0.0)
      k1 = camParRadialK1;
    if (camParRadialK2 > 0.0)
      k2 = camParRadialK2;
    if (camParHeadX > 0.0)
      hx = camParHeadX;
    if (camParHeadY > 0.0)
      hy = camParHeadY;
    cam->setCameraParameters(hx, hy, k1, k2, f);
  }
  if (posNameValid and (strlen(posName) > 0))
  {
    cam->setMountName(posName);
  }
  return result;
}
