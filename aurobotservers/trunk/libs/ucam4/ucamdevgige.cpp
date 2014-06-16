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

#include <iostream>

#include <ugen4/uimage2.h>

#include "ucamdevgige.h"

/*
| ==============================================================================
| Copyright (C) 2005-2007 Prosilica.  All Rights Reserved.
|
| Redistribution of this header file, in original or modified form, without
| prior written consent of Prosilica is prohibited.
|
|==============================================================================
|
| This sample code, open the first camera found on the host computer and set it
| for capturing. It then wait for the user to press a key before enqueuing a
| frame and saving it to a TIFF file if the capture was successful
|
|==============================================================================
|
| THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
| WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
| NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
| DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
| INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
| LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
| OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  AND ON ANY THEORY OF
| LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
| NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
| EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
|
|==============================================================================
*/

// #ifdef _WINDOWS
// #include "StdAfx.h"
// #endif
//
// #include <stdio.h>
// #include <string.h>
//
// #ifdef _WINDOWS
// #define WIN32_LEAN_AND_MEAN
// #include <Windows.h>
// #endif

#if defined USE_GIGE

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
#include <unistd.h>
#include <time.h>
#include <signal.h>
#endif

UCamDevGigE * pCamDev[MAX_GIGE_DEVS] = {NULL, NULL, NULL, NULL};
bool gigeApiInitialized = false;

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
void Sleep(unsigned int time)
{
    struct timespec t,r;

    t.tv_sec    = time / 1000;
    t.tv_nsec   = (time % 1000) * 1000000;

    while(nanosleep(&t,&r)==-1)
        t = r;
}
#endif

void  CameraEventCB_0(void* Context, tPvInterface Interface,
                      tPvLinkEvent Event, unsigned long UniqueId)
{
  UCamDevGigE * obj = (UCamDevGigE*)Context;
  if (obj != NULL)
    obj->cameraEventCB(Context, Interface, Event, UniqueId);
  printf("CameraEventCB_0 - %lu\n", UniqueId);
}

/////////////////////////////////////////////////////////////////////////

// callback called when the camera is plugged/unplugged
void  UCamDevGigE::cameraEventCB(void* Context,
                             tPvInterface Interface,
                             tPvLinkEvent Event,
                             unsigned long UniqueId)
{
    switch(Event)
    {
        case ePvLinkAdd:
        {
            printf("camera %lu plugged\n",UniqueId);
            break;
        }
        case ePvLinkRemove:
        {
            printf("camera %lu unplugged\n",UniqueId);
            break;
        }
        default:
            break;
    }
}

/////////////////////////////////////////////////////////////////////////

void  FrameDoneCB_0(tPvFrame* pFrame)
{
  //printf("GigE 0 frame\n");
  if (pCamDev[0] != NULL)
    pCamDev[0]->frameDoneCB(pFrame);
}

void  FrameDoneCB_1(tPvFrame* pFrame)
{
  //printf("GigE 1 frame\n");
  if (pCamDev[1] != NULL)
    pCamDev[1]->frameDoneCB(pFrame);
}

void  FrameDoneCB_2(tPvFrame* pFrame)
{
  //printf("GigE 2 frame\n");
  if (pCamDev[2] != NULL)
    pCamDev[2]->frameDoneCB(pFrame);

}

void  FrameDoneCB_3(tPvFrame* pFrame)
{
  //printf("GigE 3 frame\n");
  if (pCamDev[3] != NULL)
    pCamDev[3]->frameDoneCB(pFrame);
}

/////////////////////////////////////////////////////////////////////////

// callback called when a frame is done
void  UCamDevGigE::frameDoneCB(tPvFrame* pFrame)
{
  // if the frame was completed we re-enqueue it
/*  printf("Got Frame %lux%lu format %u with %lu bytes frame %lu\n",
         pFrame->Height, pFrame->Width,
         pFrame->Format, pFrame->ImageSize,
         pFrame->FrameCount);*/
  frameWidth = pFrame->Width;
  frameHeight = pFrame->Height;
  frameFormat = pFrame->Format;
  if (captureDo.tryWait())
  { // there is a custumor for an image
    frameToImage(captureImage, pFrame);
    captureDone.post();
  }
  if (pFrame->Status != ePvErrUnplugged and pFrame->Status != ePvErrCancelled)
  {
    imCnt = pFrame->FrameCount;
    switch(getGigEIdx())
    {
      case 0: PvCaptureQueueFrame(Camera.Handle, pFrame, FrameDoneCB_0); break;
      case 1: PvCaptureQueueFrame(Camera.Handle, pFrame, FrameDoneCB_1); break;
      case 2: PvCaptureQueueFrame(Camera.Handle, pFrame, FrameDoneCB_2); break;
      case 3: PvCaptureQueueFrame(Camera.Handle, pFrame, FrameDoneCB_3); break;
    }
  }
}

/////////////////////////////////////////////////////////////////////////

// wait for a camera to be plugged
void WaitForCamera()
{
  int i = 0;
    //printf("waiting for GigE camera ");
    while (PvCameraCount() == 0)
    {
        //printf(".");
        Sleep(250);
        if (i++ >= 6)
        {
          printf("Waited too long for camera (1.5 sec)\n");
          break;
        }
    }
    // printf("\n");
}

// get the first camera found
bool CameraGet(tCamera* Camera, int gigeIdx)
{
    const int MAX_CAM_CNT = 4;
    tPvUint32 count,connected;
    tPvCameraInfo list[MAX_CAM_CNT];

    count = PvCameraList(list, MAX_CAM_CNT,&connected);
    if((int)count > gigeIdx)
    {
      Camera->UID = list[gigeIdx].UniqueId;
      //printf(" - got GigE camera %s_%lu\n", list[gigeIdx].DisplayName, list[gigeIdx].UniqueId);
      snprintf(Camera->Filename, 20, "%s_%lu", list[gigeIdx].DisplayName, list[gigeIdx].UniqueId);
      return true;
    }
    else
      return false;
}

//////////////////////////////////////////////////////////////////

// open the camera
bool CameraSetup(tCamera* Camera)
{
  return !PvCameraOpen(Camera->UID, ePvAccessMaster, &Camera->Handle);
}

//////////////////////////////////////////////////////////////////

// setup and start streaming
bool UCamDevGigE::CameraStart(tCamera* Camera)
{
    unsigned long FrameSize = 0;
    bool result = false;
    int idx;
    // Auto adjust the packet size to max supported by the network, up to a max of 8228.
    // NOTE: In Vista, if the packet size on the network card is set lower than 8228,
    //       this call may break the network card's driver. See release notes.
    //
    //PvCaptureAdjustPacketSize(Camera->Handle,8228);

    // how big should the frame buffers be?
    if(!PvAttrUint32Get(Camera->Handle,"TotalBytesPerFrame",&FrameSize))
    {
      bool failed = false;
      // debug
      printf("GigE: got framesize to %lu for %d frame buffers\n", FrameSize, FRAMESCOUNT);
      // allocate the buffer for each frames
      for(int i = 0; i < FRAMESCOUNT; i++)
      {
        Camera->Frames[i].ImageBuffer = new char[FrameSize];
        if (Camera->Frames[i].ImageBuffer)
          Camera->Frames[i].ImageBufferSize = FrameSize;
        else
        {
          failed = true;
          break;
        }
      }
      //
      if (!failed)
      {
        // set the camera is capture mode
        if (!PvCaptureStart(Camera->Handle))
        {
          printf("GigE: start to capture\n");
          // set the camera in continuous acquisition mode
          //if(!PvAttrEnumSet(Camera->Handle,"FrameStartTriggerMode","Freerun"))
          {
            // and set the acquisition mode into continuous
            if (PvCommandRun(Camera->Handle, "AcquisitionStart"))
            {
              // if that fail, we reset the camera to non capture mode
              PvCaptureEnd(Camera->Handle) ;
            }
            else
            {
              idx = getGigEIdx();
              printf("GigE: AcquisitionStarted cam %d - callback functions for all frame buffers\n", idx);
              for(int i = 0; i < FRAMESCOUNT; i++)
              {
                switch(idx)
                {
                  case 0: PvCaptureQueueFrame(Camera->Handle,&(Camera->Frames[i]),FrameDoneCB_0); break;
                  case 1: PvCaptureQueueFrame(Camera->Handle,&(Camera->Frames[i]),FrameDoneCB_1); break;
                  case 2: PvCaptureQueueFrame(Camera->Handle,&(Camera->Frames[i]),FrameDoneCB_2); break;
                  case 3: PvCaptureQueueFrame(Camera->Handle,&(Camera->Frames[i]),FrameDoneCB_3); break;
                }
              }
              printf("frames queued ...\n");
              result = true;
            }
          }
        }
      }
    }
    return result;
}

///////////////////////////////////////////////////////////////

// stop streaming
void CameraStop(tCamera* Camera)
{
    PvCommandRun(Camera->Handle,"AcquisitionStop");
    PvCaptureEnd(Camera->Handle);
}

// snap and save a frame from the camera
// bool CameraSnap(tCamera* Camera)
// {
//   bool result = false;
//   result = (PvCaptureQueueFrame(Camera->Handle,&(Camera->Frame),NULL) != 0);
//   if (result)
//   {
//       //printf("waiting for the frame to be done ...\n");
//       while(PvCaptureWaitForFrameDone(Camera->Handle,&(Camera->Frame),100) == ePvErrTimeout)
//           printf("... still waiting for camera ...\n");
//       if(Camera->Frame.Status == ePvErrSuccess)
//       {
//         printf("GigE got Image size %dh%dw\n", (int)Camera->Frame.Height, (int) Camera->Frame.Width);
//       }
//       else
//       {
//         result = false;
//         printf("the frame failed to be captured ...\n");
//       }
//   }
//   else
//   {
//     printf("failed to enqueue the frame\n");
//   }
//   return result;
// }

//////////////////////////////////////////////////////////////////////

// unsetup the camera
void CameraUnsetup(tCamera* Camera)
{
  // dequeue all the frame still queued (this will block until they all have been dequeued)
  PvCaptureQueueClear(Camera->Handle);
  
  PvCameraClose(Camera->Handle);
  // delete all the allocated buffers
  for(int i = 0; i < FRAMESCOUNT; i++)
    delete [] (char*)Camera->Frames[i].ImageBuffer;
  
  Camera->Handle = NULL;
}

////////////////////////////////////////////////////////


bool Value2String(tPvHandle aCamera,const char* aLabel,tPvDatatype aType,char* aString,unsigned long aLength)
{
  switch(aType)
  {
    case ePvDatatypeString:
    {
      if(!PvAttrStringGet(aCamera,aLabel,aString,aLength,NULL))
        return true;
      else
        return false;
    }
    case ePvDatatypeEnum:
    {
      if(!PvAttrEnumGet(aCamera,aLabel,aString,aLength,NULL))
        return true;
      else
        return false;
    }
    case ePvDatatypeUint32:
    {
      tPvUint32 lValue;
      
      if(!PvAttrUint32Get(aCamera,aLabel,&lValue))
      {
        sprintf(aString,"%lu",lValue);
        return true;
      }
      else
        return false;
      
    }
    case ePvDatatypeFloat32:
    {
      tPvFloat32 lValue;
      
      if(!PvAttrFloat32Get(aCamera,aLabel,&lValue))
      {
        sprintf(aString,"%g",lValue);
        return true;
      }
      else
        return false;
    }
    default:
      return false;
  }
}

///////////////////////////////////////////////////////////////////

/*
data type enum   (w=2, cmd=0): AcqEndTriggerEvent = EdgeRising;
data type enum   (w=2, cmd=0): AcqEndTriggerMode = Disabled;
data type enum   (w=2, cmd=0): AcqRecTriggerEvent = EdgeRising;
data type enum   (w=2, cmd=0): AcqRecTriggerMode = SyncIn1;
data type enum   (w=2, cmd=0): AcqStartTriggerEvent = EdgeRising;
data type enum   (w=2, cmd=0): AcqStartTriggerMode = Disabled;
data type uint32 (w=2, cmd=0): AcquisitionFrameCount = 1;
data type enum   (w=2, cmd=0): AcquisitionMode = Continuous;
data type enum   (w=2, cmd=0): BandwidthCtrlMode = StreamBytesPerSecond;
data type uint32 (w=2, cmd=0): BinningX = 1;
data type uint32 (w=2, cmd=0): BinningY = 1;
data type string (w=2, cmd=0): CameraName = GC650C;
data type enum   (w=2, cmd=0): ConfigFileIndex = Factory;
data type enum   (w=2, cmd=0): ConfigFilePowerUp = 1;
data type uint32 (w=2, cmd=0): DSPSubregionBottom = 2147483647;
data type uint32 (w=2, cmd=0): DSPSubregionLeft = 0;
data type uint32 (w=2, cmd=0): DSPSubregionRight = 2147483647;
data type uint32 (w=2, cmd=0): DSPSubregionTop = 0;
data type string (w=0, cmd=0): DeviceEthAddress = 00-0f-31-01-92-64;
data type string (w=0, cmd=0): DeviceFirmwareVersion = 1.36.0 ;
data type string (w=0, cmd=0): DeviceIPAddress = 169.254.58.87;
data type enum   (w=0, cmd=0): DeviceScanType = Areascan;
data type string (w=0, cmd=0): DeviceSerialNumber = 02-2111A-06552;
data type string (w=0, cmd=0): DeviceVendorName = Prosilica;
data type uint32 (w=2, cmd=0): ExposureAutoAdjustTol = 5;
data type enum   (w=2, cmd=0): ExposureAutoAlg = Mean;
data type uint32 (w=2, cmd=0): ExposureAutoMax = 500000;
data type uint32 (w=2, cmd=0): ExposureAutoMin = 8;
data type uint32 (w=2, cmd=0): ExposureAutoOutliers = 0;
data type uint32 (w=2, cmd=0): ExposureAutoRate = 100;
data type uint32 (w=2, cmd=0): ExposureAutoTarget = 50;

data type enum   (w=2, cmd=0): ExposureMode = Auto;
data type uint32 (w=2, cmd=0): ExposureValue = 2053;
data type uint32 (w=0, cmd=0): FirmwareVerBuild = 0;
data type uint32 (w=0, cmd=0): FirmwareVerMajor = 1;
data type uint32 (w=0, cmd=0): FirmwareVerMinor = 36;

data type float  (w=2, cmd=0): FrameRate = 10;
data type uint32 (w=2, cmd=0): FrameStartTriggerDelay = 0;
data type enum   (w=2, cmd=0): FrameStartTriggerEvent = EdgeRising;
data type enum   (w=2, cmd=0): FrameStartTriggerMode = FixedRate;
data type uint32 (w=2, cmd=0): GainAutoAdjustTol = 5;
data type uint32 (w=2, cmd=0): GainAutoMax = 19;
data type uint32 (w=2, cmd=0): GainAutoMin = 0;
data type uint32 (w=2, cmd=0): GainAutoOutliers = 0;
data type uint32 (w=2, cmd=0): GainAutoRate = 100;
data type uint32 (w=2, cmd=0): GainAutoTarget = 50;

data type enum   (w=2, cmd=0): GainMode = Auto;
data type uint32 (w=2, cmd=0): GainValue = 16;
data type uint32 (w=2, cmd=0): GvcpRetries = 5;
data type uint32 (w=2, cmd=0): GvspLookbackWindow = 25;
data type float  (w=2, cmd=0): GvspResendPercent = 1;
data type uint32 (w=2, cmd=0): GvspRetries = 3;
data type enum   (w=2, cmd=0): GvspSocketBuffersCount = 512;
data type uint32 (w=2, cmd=0): GvspTimeout = 50;
data type uint32 (w=2, cmd=0): HeartbeatInterval = 3500;
data type uint32 (w=2, cmd=0): HeartbeatTimeout = 6000;
data type uint32 (w=2, cmd=0): Height = 493;
data type string (w=0, cmd=0): HostEthAddress = 00-1c-c0-81-cc-dd;
data type string (w=0, cmd=0): HostIPAddress = 169.254.1.1;
data type uint32 (w=2, cmd=0): IrisAutoTarget = 50;
data type enum   (w=2, cmd=0): IrisMode = Disabled;
data type uint32 (w=0, cmd=0): IrisVideoLevel = 0;
data type uint32 (w=2, cmd=0): IrisVideoLevelMax = 110;
data type uint32 (w=2, cmd=0): IrisVideoLevelMin = 90;
data type enum   (w=2, cmd=0): MirrorX = Off;
data type string (w=0, cmd=0): ModelName = GC650C;
data type uint32 (w=2, cmd=0): PacketSize = 8964;
data type uint32 (w=0, cmd=0): PartClass = 2;
data type uint32 (w=0, cmd=0): PartNumber = 2111;
data type string (w=0, cmd=0): PartRevision = A;
data type string (w=0, cmd=0): PartVersion = A;
data type enum   (w=2, cmd=0): PixelFormat = Yuv422;
data type uint32 (w=2, cmd=0): RecorderPreEventCount = 0;
data type uint32 (w=2, cmd=0): RegionX = 0;
data type uint32 (w=2, cmd=0): RegionY = 0;
data type uint32 (w=0, cmd=0): SensorBits = 12;
data type uint32 (w=0, cmd=0): SensorHeight = 493;
data type enum   (w=0, cmd=0): SensorType = Bayer;
data type uint32 (w=0, cmd=0): SensorWidth = 659;
data type string (w=0, cmd=0): SerialNumber = 6552;
data type enum   (w=0, cmd=0): StatDriverType = Standard;
data type string (w=0, cmd=0): StatFilterVersion = n/a;
data type float  (w=0, cmd=0): StatFrameRate = 0;
data type uint32 (w=0, cmd=0): StatFramesCompleted = 0;
data type uint32 (w=0, cmd=0): StatFramesDropped = 0;
data type uint32 (w=0, cmd=0): StatPacketsErroneous = 0;
data type uint32 (w=0, cmd=0): StatPacketsMissed = 0;
data type uint32 (w=0, cmd=0): StatPacketsReceived = 0;
data type uint32 (w=0, cmd=0): StatPacketsRequested = 0;
data type uint32 (w=0, cmd=0): StatPacketsResent = 0;
data type uint32 (w=2, cmd=0): StreamBytesPerSecond = 115000000;
data type uint32 (w=0, cmd=0): StreamHoldCapacity = 25;
data type enum   (w=2, cmd=0): StreamHoldEnable = Off;
data type enum   (w=2, cmd=0): Strobe1ControlledDuration = Off;
data type uint32 (w=2, cmd=0): Strobe1Delay = 0;
data type uint32 (w=2, cmd=0): Strobe1Duration = 0;
data type enum   (w=2, cmd=0): Strobe1Mode = FrameTrigger;
data type uint32 (w=0, cmd=0): SyncInLevels = 0;
data type enum   (w=2, cmd=0): SyncOut1Invert = Off;
data type enum   (w=2, cmd=0): SyncOut1Mode = Exposing;
data type enum   (w=2, cmd=0): SyncOut2Invert = Off;
data type enum   (w=2, cmd=0): SyncOut2Mode = Exposing;
data type uint32 (w=2, cmd=0): SyncOutGpoLevels = 0;
data type uint32 (w=0, cmd=0): TimeStampFrequency = 36764706;
data type uint32 (w=0, cmd=0): TimeStampValueHi = 0;
data type uint32 (w=0, cmd=0): TimeStampValueLo = 0;
data type uint32 (w=0, cmd=0): TotalBytesPerFrame = 631040;
data type uint32 (w=0, cmd=0): UniqueId = 103012;
data type uint32 (w=2, cmd=0): WhitebalAutoAdjustTol = 5;
data type uint32 (w=2, cmd=0): WhitebalAutoRate = 100;
data type enum   (w=2, cmd=0): WhitebalMode = Auto;
data type uint32 (w=2, cmd=0): WhitebalValueBlue = 211;
data type uint32 (w=2, cmd=0): WhitebalValueRed = 133;
data type uint32 (w=2, cmd=0): Width = 640;
*/


void UCamDevGigE::resetTimestamp()
{
/*  const int MVL = 128;
  char lValue[MVL];*/
  tPvUint32 vui32;
//  tPvFloat32 vf32;
//  unsigned long n;
//  bool vAuto;
  tPvErr err;
  //
  err = PvAttrUint32Get(Camera.Handle, "TimeStampFrequency", &vui32);
  if (err == ePvErrSuccess)
  {
    timestampFrequency = vui32;
    printf("Timestamp frequency %.3fMHz\n", timestampFrequency / 1e6);
  }
  else
    printf("GigE: failed TimeStampFrequency - err = %d\n", err);
  //
  err = PvCommandRun(Camera.Handle, "TimeStampReset");
  if (err != ePvErrSuccess)
    printf("GigE: failed TimeStampReset call - err = %d\n", err);
  timestampRefTime.now();
}


/// set the value of a given attribute from a value encoded in a string
/// \returns false if no error
bool setAttribute(tPvHandle aCamera,
                  const char* aLabel,
                  tPvDatatype aType,
                  char* aValue)
{
  switch(aType)
  {
    case ePvDatatypeString:
    {
      if(!PvAttrStringSet(aCamera,aLabel,aValue))
        return true;
      else
        return false;
    }
    case ePvDatatypeEnum:
    {
      if(!PvAttrEnumSet(aCamera,aLabel,aValue))
        return true;
      else
        return false;
    }
    case ePvDatatypeUint32:
    {
      tPvUint32 lValue = atol(aValue);
      tPvUint32 lMin,lMax;
      
      if(!PvAttrRangeUint32(aCamera,aLabel,&lMin,&lMax))
      {
        if(lMin > lValue)
          lValue = lMin;
        else
          if(lMax < lValue)
            lValue = lMax;
          
          if(!PvAttrUint32Set(aCamera,aLabel,lValue))
            return true;
          else
            return false;
      }
      else
        return false;
    }
    case ePvDatatypeFloat32:
    {
      tPvFloat32 lValue = (tPvFloat32)atof(aValue);
      tPvFloat32 lMin,lMax;
      
      if(!PvAttrRangeFloat32(aCamera,aLabel,&lMin,&lMax))
      {
        if(lMin > lValue)
          lValue = lMin;
        else
          if(lMax < lValue)
            lValue = lMax;
          
          if(!PvAttrFloat32Set(aCamera,aLabel,lValue))
            return true;
          else
            return false;
      }
      else
        return false;
    }
    default:
      return false;
  }
}

///////////////////////////////////////////////////////

// set the value of a given attribute from a value encoded in a string
bool String2Value(tPvHandle aCamera, const char* aLabel, tPvDatatype aType, char* aValue)
{
  switch(aType)
  {
    case ePvDatatypeString:
    {
      if(!PvAttrStringSet(aCamera,aLabel,aValue))
        return true;
      else
        return false;
    }
    case ePvDatatypeEnum:
    {
      if(!PvAttrEnumSet(aCamera,aLabel,aValue))
        return true;
      else
        return false;
    }
    case ePvDatatypeUint32:
    {
      tPvUint32 lValue = atol(aValue);
      tPvUint32 lMin,lMax;
      
      if(!PvAttrRangeUint32(aCamera,aLabel,&lMin,&lMax))
      {
        if(lMin > lValue)
          lValue = lMin;
        else
          if(lMax < lValue)
            lValue = lMax;
          
          if(!PvAttrUint32Set(aCamera,aLabel,lValue))
            return true;
          else
            return false;
      }
      else
        return false;
    }
    case ePvDatatypeFloat32:
    {
      tPvFloat32 lValue = (tPvFloat32)atof(aValue);
      tPvFloat32 lMin,lMax;
      
      if(!PvAttrRangeFloat32(aCamera,aLabel,&lMin,&lMax))
      {
        if(lMin > lValue)
          lValue = lMin;
        else
          if(lMax < lValue)
            lValue = lMax;
          
          if(!PvAttrFloat32Set(aCamera,aLabel,lValue))
            return true;
          else
            return false;
      }
      else
        return false;
    }
    default:
      return false;
  }
}


///////////////////////////////////////////////////////

bool ReadAttribute(tPvHandle aCamera, const char* lLabel, char * lValue)
{
  bool result = false;
  tPvAttributeInfo lInfo;
  //
  if (strlen(lLabel) > 0 and strlen(lValue) > 0)
  {
    if (!PvAttrInfo(aCamera,lLabel,&lInfo))
    {
      if(lInfo.Datatype != ePvDatatypeCommand &&
        (lInfo.Flags & ePvFlagWrite))
      {
        if(String2Value(aCamera, lLabel, lInfo.Datatype, lValue))
          fprintf(stderr,"attribute %s couldn't be loaded\n",lLabel);
      }
    }
  }
  return result;
}


// int main(int argc, char* argv[])
// {
//     // initialise the Prosilica API
//     if(!PvInitialize())
//     {
//         tCamera Camera;
//
//         memset(&Camera,0,sizeof(tCamera));
//
//         // wait for a camera to be plugged
//         WaitForCamera();
//
//         // get a camera from the list
//         if(CameraGet(&Camera))
//         {
//             // setup the camera
//             if(CameraSetup(&Camera))
//             {
//                 // strat streaming from the camera
//                 if(CameraStart(&Camera))
//                 {
//                     printf("camera is ready now. Press q to quit or s to take a picture\n");
//                     // wait for the user to quit or snap
//                     if(WaitForUserToQuitOrSnap())
//                     {
//                         // snap now
//                         CameraSnap(&Camera);
//                     }
//
//                     // stop the streaming
//                     CameraStop(&Camera);
//                 }
//                 else
//                     printf("failed to start streaming\n");
//
//                 // unsetup the camera
//                 CameraUnsetup(&Camera);
//             }
//             else
//                 printf("failed to setup the camera\n");
//         }
//         else
//             printf("failed to find a camera\n");
//
//         // uninitialise the API
//         PvUnInitialize();
//     }
//     else
//         printf("failed to initialise the API\n");
//
//     return 0;
// }

#endif

/////////////////////////////////////////////////////////

#ifndef USE_GIGE
/*  bool PvInitialize()
  { return false; };
  bool PvUnInitialize()
  { return false; };*/
  bool PvCameraClose(tCamera * a)
  { return false; };
  bool CameraUnsetup(tCamera * a)
  { return false; };
  bool CameraGet(tCamera * a, int b)
  {return false; };
  bool CameraSetup(tCamera * a)
  {return false; };
  bool UCamDevGigE::CameraStart(tCamera* Camera)
  {return false; };
  bool CameraStop(tCamera * a)
  {return false; };
  bool CameraSnap(tCamera * a)
  { return false; };
  bool WaitForCamera()
  { return false; };
#endif

////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////



UCamDevGigE::UCamDevGigE()
 : UCamDevBase()
{
  camType = CAM_DEV_GIGE;
  imgBuffNext = 0;
  imgBuff[0] = NULL;
  frameRate = 0;
  frameFormat = -1;
  // lock semaphores in no post state
  captureDo.tryWait();
  captureDone.tryWait();
  // initialise the API
#if defined USE_GIGE
  if (not gigeApiInitialized)
  {
    tPvErr err = PvInitialize();
    if (err != ePvErrSuccess)
      printf("Failed to initialize GigE API - continues - err = %d\n", err);
    gigeApiInitialized = true;
  }
#endif
}

///////////////////////////////////////////

UCamDevGigE::~UCamDevGigE()
{
  closeDevice();
  // uninitialise the API
#if defined USE_GIGE
  if (initialized)
  { // remove setup
    printf("GigE: camera unsetup %d\n", getGigEIdx());
    PvLinkCallbackUnRegister(CameraEventCB_0,ePvLinkAdd);
    PvLinkCallbackUnRegister(CameraEventCB_0,ePvLinkRemove);
    //
    PvCaptureEnd(Camera.Handle);
    CameraUnsetup(&Camera);
    initialized = false;
  }
  if (gigeApiInitialized and getGigEIdx() == 0)
  {
    PvUnInitialize();
    gigeApiInitialized = false;
    printf("GigE: Uninitialized interface %d\n", getGigEIdx());
  }
#endif
}

////////////////////////////////////////////

bool UCamDevGigE::frameToImage(UImage * destination, tPvFrame * frame)
{
  bool result;
  int j = 0, n, bytes;
  UImage * img = destination;
  const int MSL = 50;
  char s[MSL];
  double dt;
  //
  result = frame != NULL and destination != NULL;
  if (not result)
  {
    fprintf( stderr, "GigE (callback): Invalid source frame or destination image\n");
  }
  if (result)
  {
//#if defined USE_GIGE
    img->setSize(frame->Height, frame->Width, 1, 8, "BGGR");
    j = frame->Format;
//#endif
    switch(j)
    {
/*   ePvFmtMono8         = 0,            // Monochrome, 8 bits
    ePvFmtMono16        = 1,            // Monochrome, 16 bits, data is LSB aligned
    ePvFmtBayer8        = 2,            // Bayer-color, 8 bits
    ePvFmtBayer16       = 3,            // Bayer-color, 16 bits, data is LSB aligned
    ePvFmtRgb24         = 4,            // RGB, 8 bits x 3
    ePvFmtRgb48         = 5,            // RGB, 16 bits x 3, data is LSB aligned
    ePvFmtYuv411        = 6,            // YUV 411
    ePvFmtYuv422        = 7,            // YUV 422
    ePvFmtYuv444        = 8,            // YUV 444
    ePvFmtBgr24         = 9,            // BGR, 8 bits x 3
    ePvFmtRgba32        = 10,           // RGBA, 8 bits x 4
    ePvFmtBgra32        = 11,           // BGRA, 8 bits x 4
    ePvFmtMono12Packed  = 12,           // Monochrome, 12 bits,
    ePvFmtBayer12Packed = 13,           // Bayer-color, 12 bits, packed
    __ePvFmt_force_32   = 0xFFFFFFFF */
      case ePvFmtMono8: img->setColorType(PIX_PLANES_BW); break;
      case ePvFmtMono16: img->setColorType(PIX_PLANES_BW16S); break;
      case ePvFmtBayer8:
        switch (frame->BayerPattern)
        {
          case ePvBayerRGGB: img->setColorType(PIX_PLANES_RGGB); break;
          case ePvBayerBGGR: img->setColorType(PIX_PLANES_BGGR); break;
          case ePvBayerGBRG: img->setColorType(PIX_PLANES_GBRG); break;
          case ePvBayerGRBG: img->setColorType(PIX_PLANES_GRBG); break;
          default:
            img->setColorType(PIX_PLANES_RGGB);
            printf("GigE: unsupported Bayer format\n");
            break;
        }
        break;
      case ePvFmtRgb24: img->setColorType(PIX_PLANES_RGB); break;
      case ePvFmtBgr24: img->setColorType(PIX_PLANES_BGR); break;
      case ePvFmtRgba32: img->setColorType(PIX_PLANES_RGBA); break;
      case ePvFmtBgra32: img->setColorType(PIX_PLANES_BGRA); break;
      case ePvFmtYuv444: img->setColorType(PIX_PLANES_YUV); break;
      default:
       img->setColorType(PIX_PLANES_BW);
       printf("GigE (callback): Did not recognise the frame color format %d\n", frame->Format);
       break;
    }
    n = img->getDataSize();
    bytes = img->getBufferSize();
    if (n <= bytes)
    {
      memcpy(img->getData(), frame->ImageBuffer, n);
      //printf("GigE (callback): Copied %d bytes to image\n", n);
    }
    img->camDevice = getDeviceNumber();
    img->cam = NULL;
    dt = (frame->TimestampHi * 0xFFFFFFFF + frame->TimestampLo)/ double(timestampFrequency);
    img->imgTime = timestampRefTime + dt;
    //img->imgTime.setTimeU(camera.filltime.tv_sec, camera.filltime.tv_usec);
    // debug
    img->imgTime.getTimeAsString(s, true);
    //printf("GigE (callback): frame time %s  %lu.%06lu serial %lu\n", s,
    //       img->imgTime.getSec(), img->imgTime.getMicrosec(), imageNumber);
    // debug end
    //
    img->imageNumber = frame->FrameCount;
    img->valid = true;
  }
  imageNumber++;
  return result;
}

/////////////////////////////////////////////////////////////


bool UCamDevGigE::setGain(int agc)
{
  bool result = false;
#if defined USE_GIGE
  tPvUint32 lValue;
  tPvUint32 lMin = 0,lMax = 19;
  tPvErr err;

  if (agc < 0)
  {
    err = PvAttrEnumSet(Camera.Handle, "GainMode", "Auto");
    result = err == ePvErrSuccess;
    if (not result)
      printf("GigE: failed to set gain mode to auto - err = %d\n", err);
  }
  else
  {
    err = PvAttrEnumSet(Camera.Handle, "GainMode", "Manual");
    result = err == ePvErrSuccess;
    if (not result)
      printf("GigE: failed to set gain mode to manual - err = %d\n", err);
    //
    err = PvAttrRangeUint32(Camera.Handle,"GainValue",&lMin,&lMax);
    if (err == ePvErrSuccess)
    {
      lValue = agc;
      if(lMin > lValue)
        lValue = lMin;
      else
        if(lMax < lValue)
          lValue = lMax;

      err = PvAttrUint32Set(Camera.Handle, "gainValue", lValue);
      result = err == ePvErrSuccess;
      if (not result)
        printf("GigE: failed to set video gain - err = %d\n", err);
      if (agc > (int)lMax)
        printf("GigE: range for video gain is %lu to %lu (db)\n", lMin, lMax);
    }
    else
      printf("GigE: failed to get video gain range - err = %d\n", err);
  }
#endif
  return result;
}

/////////////////////////////////////////////////

int UCamDevGigE::getGain(bool probe, bool * dataValid, bool * isOnAuto)
{
#if defined USE_GIGE
  const int MVL = 128;
  char lValue[MVL];
  tPvUint32 vui32;
  unsigned long n;
  bool vAuto;
  tPvErr err;
  //
  if (probe)
  {
    vgain = -2;
    err = PvAttrStringGet(Camera.Handle, "GainMode", lValue, MVL, &n);
    if (err == ePvErrSuccess)
    {
      vAuto = strcasecmp(lValue, "auto") == 0;
      printf("GigE: videoMode auto=%s\n", bool2str(vAuto));
      if (vAuto)
        vgain = -1;
    }
    else
      printf("GigE: failed GainMode - err = %d\n", err);
    //
    err = PvAttrUint32Get(Camera.Handle, "GainValue", &vui32);
    if (err == ePvErrSuccess)
    {
      vgain = vui32;
      printf("GigE: videoGain=%d\n", vgain);
    }
    else
      printf("GigE: failed GainValue - err = %d\n", err);
  }
  if (dataValid != NULL)
    *dataValid = vgain >= -1;
  if (isOnAuto != NULL)
    *isOnAuto = vgain == -1;
#endif
  return vgain;
}

/////////////////////////////////////////////////

bool UCamDevGigE::setShutter(int value)
{
  bool result = false;
#if defined USE_GIGE
  tPvUint32 lValue;
  tPvUint32 lMin = 0,lMax = 19;
  tPvErr err;

  if (value < 0)
  {
    err = PvAttrEnumSet(Camera.Handle, "ExposureMode", "Auto");
    result = err == ePvErrSuccess;
    if (not result)
      printf("GigE: failed to set exposure mode to auto - err = %d\n", err);
    err = PvAttrEnumSet(Camera.Handle, "ExposureAutoAlg", "Mean");
    result = err == ePvErrSuccess;
    if (not result)
      printf("GigE: failed to set exposure alg to Mean - err = %d\n", err);
  }
  else
  {
    err = PvAttrEnumSet(Camera.Handle, "ExposureMode", "Manual");
    result = err == ePvErrSuccess;
    if (not result)
      printf("GigE: failed to set exposure mode to manual - err = %d\n", err);
    //
    err = PvAttrRangeUint32(Camera.Handle,"ExposureValue",&lMin,&lMax);
    if (err == ePvErrSuccess)
    {
      lValue = value;
      if(lMin > lValue)
        lValue = lMin;
      else
        if(lMax < lValue)
          lValue = lMax;
        
      err = PvAttrUint32Set(Camera.Handle, "ExposureValue", lValue);
      result = err == ePvErrSuccess;
      if (not result)
        printf("GigE: failed to set exposure value - err = %d\n", err);
      if (value > (int)lMax)
        printf("GigE: range for exposure is %lu to %lu (set to %lu)\n", lMin, lMax, lValue);
    }
    else
      printf("GigE: failed to get exposure value gain range - err = %d\n", err);
  }
#endif
  return result;
}

/////////////////////////////////////////////////

int UCamDevGigE::getShutter(bool probe, bool * dataValid, bool * isOnAuto)
{
#if defined USE_GIGE
  const int MVL = 128;
  char lValue[MVL];
  tPvUint32 vui32;
  unsigned long n;
  bool vAuto = false;
  tPvErr err;
  //
  if (probe)
  {
    vshutter = -2;
    err = PvAttrStringGet(Camera.Handle, "ExposureMode", lValue, MVL, &n);
    if (err == ePvErrSuccess)
    {
      vAuto = (strcasecmp(lValue, "auto") == 0);
      if (vAuto)
        vshutter = -1;
    }
    else
      printf("GigE: failed ExposureMode - err = %d\n", err);
    //
    err = PvAttrUint32Get(Camera.Handle, "ExposureValue", &vui32);
    if (err == ePvErrSuccess)
    {
      if (not vAuto)
        vshutter = vui32;
    }
    else
      printf("GigE: failed ExposureValue - err = %d\n", err);
    //
    printf("GigE: exposureMode auto=%s (value=%lu) err=%d\n", bool2str(vAuto), vui32, err);
  }
  if (dataValid != NULL)
    *dataValid = vshutter >= -1;
  if (isOnAuto != NULL)
    *isOnAuto = vshutter == -1;
#endif
  return vshutter;
}

//////////////////////////////////////////////////

bool UCamDevGigE::deviceExist()
{ // open device and get default parameters
  bool result;
  int gIdx;
  //
  if (not cameraOpen)
  {
    memset(&Camera, 0, sizeof(tCamera));
    // wait for a camera to be plugged
    WaitForCamera();
    gIdx = getGigEIdx();
    //printf("GigE: index %d - exist?\n", gIdx);
    // get a camera from the list
    result = CameraGet(&Camera, gIdx);
    if (result)
    {
      setTypeName(Camera.Filename);
      printf("Found a %s\n", Camera.Filename);
    }
    else if (gIdx == 0)
    {
#if defined USE_GIGE
      printf("GigE: Failed to find GigE camera %d\n", getGigEIdx());
#else
      printf("GigE: disabled, add '-D USE_GIGE' in makefile for ucam4 and ucamserver to use\n");
#endif
    }
    if (result)
    { // start plug-unplug monitoring
#if defined USE_GIGE
      pCamDev[gIdx] = this;
      PvLinkCallbackRegister(CameraEventCB_0,ePvLinkAdd, this);
      PvLinkCallbackRegister(CameraEventCB_0,ePvLinkRemove,this);
#endif
      printf("GigE: ---- camera setup call\n");
      result = CameraSetup(&Camera);
      printf("GigE: ---- camera setup call returned\n");
      if (not result)
      {
        printf("GigE: failed to setup camera %d\n", getGigEIdx());
        CameraUnsetup(&Camera);
      }
    }
    if (result)
    {
      result = CameraStart(&Camera);
      if (not result)
      {
        result = false;
        CameraStop(&Camera);
        printf("GigE: Failed to start AcquisitionStart\n");
      }
    }
    initialized = result;
  }
  else
  {
    printf("GigE: Camera already open!\n");
    result = true;
  }
  //
  return result;
}

////////////////////////////////////////////////////

bool UCamDevGigE::openDeviceDefault()
{
  bool result = true;
  //
  if (imgBuff[0] == NULL)
  { // make push buffer (if needed)
    imgBuff[0] = new UImage800();
    // initialize push buffer
    imgBuff[0]->camDevice = devNum;
    imgBuff[0]->setSize(493, 659, 3, 8, "RGB");
    imgBuff[0]->valid = false;
    imgBuffNext = 0;
  }
  //
  if (cameraOpen)
    closeDevice();
  //
  // start streaming from the camera
  //result = CameraStart(&Camera);
#if defined USE_GIGE
  tPvErr err;
  // get basis for timestamp
  resetTimestamp();
  //
  err = PvAttrEnumSet(Camera.Handle, "PixelFormat", "Bayer8");
  result = err == ePvErrSuccess;
  if (not result)
    printf("GigE: failed to set pixel format to Bayer8 - err = %d\n", err);
  //
  err = PvAttrEnumSet(Camera.Handle, "PixelFormat", "Bayer8");
  result = err == ePvErrSuccess;
  if (not result)
    printf("GigE: failed to set pixel format to Bayer8 - err = %d\n", err);
  // start frame streaming
  if (PvCommandRun(Camera.Handle, "AcquisitionStart"))
  {
    result = false;
    CameraStop(&Camera);
    printf("GigE: Failed to AcquisitionStart\n");
  }
  if (result)
  {
    if (frameHeight < 0)
      frameHeight = 480;
    if (frameWidth < 0)
      frameWidth = 640;
  }
#else
  result = false;
#endif
  cameraOpen = result;
  printf("GigE: camera open=%s\n", bool2str(cameraOpen));
  return result;
}

////////////////////////////////////////////////////

void UCamDevGigE::closeDevice()
{
  if (cameraOpen)
  { // stop the streaming
#if defined USE_GIGE
  PvCommandRun(Camera.Handle,"AcquisitionStop");
    //PvCaptureEnd(Camera->Handle);
    // CameraStop(&Camera);
    printf("GigE: device %d AcquisitionStop\n", getDeviceNumber());
    //CameraUnsetup(&Camera);
#endif
    cameraOpen = false;
  }
}

////////////////////////////////////////////////////

bool UCamDevGigE::getImageSnapshot(UImage * image)
{
  int i;
  bool result;
  //
  if (not isCameraOpen())
  { // open on data request
    openDevice();
  }
  // request a snapshot to this image buffer
  if (image != NULL)
  { // image pointer may be NULL, just to trigger a snapshot event
    image->valid=false;
    // set destination pointer
    captureImage = image;

    // post the read thread to do the capture
    captureDo.post();
    // wait for the read thread to have finished
    for (i = 0; i < 200; i++)
    { // wait for read thread to post in captureDone
      if (captureDone.tryWait())
        break;
      Wait(0.01);
    }
    if ((i >= 200))
      printf("GigE: UCamDevGigE::getImageSnapshot: Gave up "
          "waiting for image (device %d)\n", devNum);
    else
    { // remove buffer pointer - just to avoid misuse
      captureImage = NULL;
      printf("GigE got image %s (waited %d)\n", bool2str(image->valid), i);
    }
    result = image->valid;
  }
  else
  {
    result = false;
    printf("GigE: No destination image buffer\n");
  }
  //
  return result;
}

///////////////////////////////////////////////////////////

int UCamDevGigE::getFrameRate()
{
//  tPvUint32 vui32;
/*  tPvFloat32 vf32;
  if (PvAttrFloat32Get(Camera.Handle, "FrameRate", &vf32) == ePvErrSuccess)
  {
    frameRate = roundi(vf32);
    printf("Framerate %f\n", vf32);
  }*/
  return frameRate;
}

///////////////////////////////////////////////////////////

bool UCamDevGigE::setDevice(const int width, const int height,
               const int framesPerSec)
{
  bool result = false;
#if defined USE_GIGE
  tPvUint32 lValue;
  tPvUint32 lMin = 0,lMax = 19;
  tPvErr err;
  //
  if (frameRate != framesPerSec)
  {
    err = PvAttrRangeUint32(Camera.Handle,"FrameRate",&lMin,&lMax);
    if (err == ePvErrSuccess)
    {
      lValue = maxi(mini(framesPerSec, lMax), lMin);
      err = PvAttrUint32Set(Camera.Handle, "FrameRate", lValue);
      result = err == ePvErrSuccess;
      if (not result)
        printf("GigE: failed to set framerate - err = %d\n", err);
      if (framesPerSec > int(lMax) or framesPerSec < int(lMin))
        printf("GigE: framerate range is from %lu to %lu (set to %lu)\n", lMin, lMax, lValue);
      if (result)
        frameRate = framesPerSec;
    }
    else
      printf("GigE: Failed to get framerate range - err = %d\n", err);
  }
  if (frameWidth != width)
  {
    err = PvAttrRangeUint32(Camera.Handle,"ExposureValue",&lMin,&lMax);
    if (err == ePvErrSuccess)
    {
      lValue = maxi(mini(width, lMax), lMin);
      err = PvAttrUint32Set(Camera.Handle, "Width", lValue);
      result = err == ePvErrSuccess;
      if (not result)
        printf("GigE: failed to set image width - err = %d\n", err);
      if (width > int(lMax))
        printf("GigE: Width range is from %lu to %lu (set to %lu)\n", lMin, lMax, lValue);
    }
    else
      printf("GigE: Failed to get width range - err = %d\n", err);
  }
  if (frameHeight != height);
  {
    err = PvAttrRangeUint32(Camera.Handle,"ExposureValue",&lMin,&lMax);
    if (err == ePvErrSuccess)
    {
      lValue = maxi(mini(height, lMax), lMin);
      err = PvAttrUint32Set(Camera.Handle, "Height", lValue);
      result = err == ePvErrSuccess;
      if (not result)
        printf("GigE: failed to set image Height - err = %d\n", err);
      if (height > int(lMax))
        printf("GigE: Height range is from %lu to %lu (set to %lu)\n", lMin, lMax,lValue);
    }
    else
      printf("GigE: Failed to get height range - err = %d\n", err);
  }
  //
  imageSizeChanged(MAX_IMAGE_WIDTH / float(frameWidth));
#endif
  return result;
}

////////////////////////////////////////////

bool UCamDevGigE::getWhiteBalance(bool probe, int * red, int * blue, int * mode)
{
#if defined USE_GIGE
  const int MVL = 128;
  char lValue[MVL];
  tPvUint32 vui32;
  unsigned long n;
  tPvErr err;
  //
  if (probe)
  {
    err = PvAttrStringGet(Camera.Handle, "WhitebalMode", lValue, MVL, &n);
    if (err == ePvErrSuccess)
    {
      whitebalAuto = strcasecmp(lValue, "auto") == 0;
      printf("GigE: white ballance mode auto=%s\n", bool2str(whitebalAuto));
      if (PvAttrUint32Get(Camera.Handle, "WhitebalValueBlue", &vui32) == ePvErrSuccess)
        whitebalBlue = vui32;
      if (PvAttrUint32Get(Camera.Handle, "WhitebalValueRed", &vui32) == ePvErrSuccess)
        whitebalRed = vui32;
      printf("GigE: white ballance mode auto=%s, %dred, %dblue\n", bool2str(whitebalAuto), whitebalRed, whitebalBlue);
    }
    else
      printf("GigE: failed to get WhitebalMode - err = %d\n", err);
  }
  if (red != NULL)
    *red = whitebalRed;
  if (blue != NULL)
    *blue = whitebalRed;
  if (mode != NULL)
  {
    if (whitebalAuto)
      *mode = AU_WB_AUTO;
    else
      *mode = AU_WB_MANUAL;
  }
#endif
  return true;
}









