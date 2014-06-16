/***************************************************************************
 *   Copyright (C) 2004 by Christian Andersen                              *
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
#ifndef UIMGPUSHTEST_H
#define UIMGPUSHTEST_H

#include <ugen4/uimage2.h>
//#include <ugen4/urawimage.h>

#include "uimgsockserv.h"
#include "ucomcam.h"
#include "usockclient.h"
/**
Test for image push speed

@author Christian Andersen
*/

class UImgSockServPushTestConn : public UImgSockServConn
{
public:
  /**
  alled when new (possibly partial) message is received */
  virtual void gotNewMessage(unsigned char * message, int length);

};

/////////////////////////////////////////////////////

/**
This is ... */
class UImgSockServPushTest : public UImgSockServ
{
public:
  UImgSockServPushTest();
  /**
  Sets up pointers to client structures */
  void initialize();

public:
  UImgSockServPushTestConn * clients[MAX_CLIENTS_SERVED];
  bool finished;
};


////////////////////////////////////////////////////////////

/**
Image client function */
class UImgClientTest : public USockClient
{
public:
  /**
  Constructor */
  UImgClientTest();
  virtual ~UImgClientTest();
  /**
  Send message block, but first log, what is transmitted. */
  bool blockSend(const char * buffer, int length);
  /**
  Request camera time */
  bool sendTimeRequest();
  /**
  Request camera information - such as device number,
  frame speed, device name and frame size.
  Returns true if send. */
  bool sendCameraInfoRequest(bool allCams);
  /**
  Request camara parameter info, such as focus length,
  radial error parameters and head point.
  Returns true if send. */
  bool sendCameraParameterRequest();
  /**
  Request a image metadata to be taken with default camera parameters.
  NB! actual image is not send, but will have to be requested
  separately.  */
  bool sendImageMetaRequest(int sourceNum);
  /**
  Send request to capture a new image, with or without
  radial error removed.
  Returns true if send. */
  bool sendImageCaptureRequest(bool removeRadialError,
                               int imageFormat);
  /**
  Request image, either packed or unpacked.
  Source number is image source at camera server.
  Destination is saved until image is retrived.
  If line number != -1, then just the specified line is send.
  If format (color) is -1 then the default format is used
  as set by tryColorFormat parameter.
  Format can be 1 = YUV, 2 = RGB or 3 = BW.
  Returns true if send. */
  bool sendGetImage(int sourceNum,
                    unsigned int device,
                    bool tryPacked,
                    int line = -1,
                    int format = -1);
  /**
  Request new camera settings,
  as defined in UComCam structure.
  Returns true if send. */
  bool sendNewCameraSettings2(UComCam * cs);
  /**
  Request that camera settings are changed to this
  device ('/dev/videoX' with X = 0..5),
  Width (w) and height (h) normally from 160x120 to 640x480.
  (0,0) means no change. <br>
  Frames per seconf (fps) normally 5, 10, 15 or 30. <br>
  fps = -1 (0xff) means no change. <br>
  Image number is camera serial number will be set to this number.<br>
  Relative camera position on robot is used when reporting
  estimated single camera positions of e.g. barchart position.
  The position is set if 'newPosition' is true. <br>
  The rotation is set if 'newRotation' is true. <br>
  imNum = -1 means no change. <br>
  Returns true if send. */
  bool sendNewCameraSettings(int dev, int w, int h,
                                 int fps, long imNum,
                                 bool repeatNewImages,
                                 bool newPosition,
                                 UPosition newPos,
                                 bool newRotation,
                                 URotation newRot,
                                 bool ensureOpen);
  /**
  Send specific camera settings.
  Radial error parameters K1 must be larger than 1e-12 and
  parameter K2 must be larger than 1e-15.
  Focus length with 2 decomals in pixels.
  headX center point in pixels.
  headY center point in pixels. */
  bool sendNewCameraParameters(
                   const float radialK1,
                   const float radialK2,
                   const float focalLength,
                   const float headX,
                   const float headY);
  /**
  Send camera calibration request for estimation of
  radial error parameters */
  bool sendCameraCalibrateRequest(
                           bool  analyzeImage,
                           int  imageSource,
                           bool  clearAllData,
                           bool  clearSpecificData,
                           float  blockSize,
                           bool  findCameraParams,
                           bool alsoFocalLength,
                           bool alsoK1, bool alsoK2,
                           unsigned long  setsToUse,
                           bool extraImages);
  /**
  Send request for analyzing current image for barcode data.
  Barcode frame size must be (frameSize), and each block is a
  'blockSize' cm square. booleand flag indicates different
  calculation and debugging options.
  Returns true if send.  */
  bool sendBarcodeExecute(int sourceImage,
                     bool findCode,
                     bool extraImages,
                     bool nearGMK,
                     bool findPosition, bool findRotation,
                     bool findChartPosition,
                     bool clearOldData, bool findCameraParameters,
                     int frameSize, float blockSize, int codeBlockFactor,
                     int maxCodesToLookFor,
                     float chartHeight, // height of barcode center for cam calib
                     unsigned char specificCodeLng,
                     int specificCode[]);
  /**
  Send request to close device.
  Returns true if send. */
  bool sendCameraClose();
  /**
  Send request to be master for controlling this camera */
  bool sendCameraMaster();
  /**
  Send request for pan-tilt position and range for this camera */
  bool sendPantiltInfoRequest();
  /**
  Request a pan-tilt camera to move to a desired position */
  bool sendPantiltPosRequest(
                  bool reset,
                  bool relative,
                  int pan, int tilt);


protected:
  /**
  Decode this time message */
  bool decodeTimeMsg(unsigned char * msg);
  /**
  Decode message with camera info - like image size and
  device name.
  Returns true if no errors were found. */
  bool decodeCameraInfoMsg(unsigned char * msg);
  /**
  Full camera device state message (replaces first version)
  Returns true if no errors were found. */
  bool decodeCameraInfoMsg2(unsigned char * msg);
  /**
  Decode message with camera parameters - like focus length and
  radial error parameters.
  Returns true if no errors were found. */
  bool decodeCameraParamMsg(unsigned char * msg);
  /**
  Decode received image meta data */
  bool decodeImageMetaData(unsigned char * msg);
  /**
  Decode message with image pixel data (packed or unpacked)
  returns true if no format error were found.
  The full message consist of several messages of this type.
  The last message is marked, and could trigger further action. */
  bool decodeImageData(unsigned char * msg);
  /**
  Called by receive thread every time some bytes have been received.
  There is no garantee that data is separated at a message boundary. */
  void processMessage(unsigned char * data, int length);
  /**
  Decode a received message.
  Returns number of bytes used. */
  int decodeMessages();
  /**
  Decode done message, that are the default
  message, if no other data are returned.
  This message may include a string. if so, then this is logged.
  Returns true. */
  bool decodeDone(unsigned char * msg);
  /**
  Decode message with barcode code.
  Returns true if decoded OK. */
  bool decodeBarcodeCode(unsigned char * msg);
  /**
  Decode message with pan-tilt support and position info.
  Returns true if message is valid */
  bool decodePantiltPosition(unsigned char * msg);
  /**
  Called when image meta data is updated */
  virtual void imageMetaUpdated(int imageSlot);
  /**
  Called when image is updated.
  if only one row, then row specified. if full image, then
  row is -1.
  If 'isBW' is true, then only Y channel is used (U,V is 128).
   */
  virtual void imageUpdated(unsigned int serial,
                            int source,
                            int row, bool isBW);
  /**
  Called, when camera information is updated */
  virtual void cameraDataUpdated(int device);
  /**
  Barcode data updated */
  //virtual void barcodeDataUpdated(int codeNumber);
  /**
  Version or server time updated */
  virtual void timeUpdated() {;};
  /**
  Called when something is to be displayed or logged.
  if not handled by virtual function, then
  messages are just printed to console. */
  virtual void sInfo(const char * message, int type);

public:
  /**
  Received parameters */
  /** server time (set when requested only) */
  UTime timeServ;
  /** server version major (set on connect) */
  int   formatMajor;
  /** server version major (set on connect) */
  int   formatMinor;
  //
  // image format
  // bool tryPacked;
  /** This is ... */
  int tryColorFormat;
  /** requested image data */
  bool reqOutstanding;
  int reqMessageSerial;
  UTime reqTime;
  /** Timing test - time for received and unpacked */
  UTime recvTime;
  /** Timing test - time start of test */
  UTime startTime;
  /** Timing test sime of reception of first image meta */
  UTime startTimeClient;
  /**
  Image count */
  int imagesReceived;

  // partially received image
  UImage640 * rgbImage; // unpacked image data
  URawImage * rawImage; // image in YUV:4:2:0 format
  // received image data
  bool dataBuffersCreated;
  // debug and control
  bool verboseMessages;
  //
  UComCam ccam;
protected:
  bool receiving;
  /** buffer for incomplete messages plus new full buffer of data */
  unsigned char message[MAX_MESSAGE_LENGTH_FROM_CAM * 2];
  /** bytes in buffer */
  int messageCnt;
  /** message serial number */
  int serial;
  /** Current device number for info-messages etc.
     i.e. when a message is send or received without
     explicit device number. */
  int selectedDeviceNumber;
};

#endif
