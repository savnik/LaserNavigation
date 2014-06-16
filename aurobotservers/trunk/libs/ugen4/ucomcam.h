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
#define UCOMCAM_H
// NOT USED anymore - moved to URob4
#ifndef UCOMCAM_H
#define UCOMCAM_H

/**
Structure to hold camera settings information and communicate this on a binary packed serial device.

NBNBNB! not used - moved to urob4


@author Christian Andersen
*/

NOT USED

class UComCamErr{ // NOT USED
public:
    UComCamErr();

    ~UComCamErr();
    /**
    Pack camera settings to string.
    Packs all valid values into the string and
    returns the number of bytes used.
    The format  */
    unsigned int pack(char * buff, unsigned int bufflng);

public:
  /** 
  Camera device open/closed is valis or not */
  bool openValid;
  /** Camera device is open/closed */
  bool openValue;
  /**
  Video device number - usually means directly the '/dev/video0'
  device number series.
  - is info valid. */
  bool deviceValid;
  /**
  Video device number - usually means directly the '/dev/video0'
  device number series (0..32). */
  int deviceValue;
  /** 
  Is client in control of camera - or take control 
  - is info valid */
  bool inCtrlValid;
  /** Is client in control of camera - or take control */
  bool inCtrlValue;
  /** 
  Frame size from camera - is info valid */
  bool sizeValid;
  /** Frame size from camera - width in pixels */
  unsigned int sizeWidth;
  /** Frame size from camera - height in pixels */
  unsigned int sizeHeight;
  /** 
  Framerate in images per second or 
  seconds per frame if negative 
  - is info valid */
  bool fpsValid;
  /** Framerate in images per second if positive or
      in seconds per frame if negative. */
  int fpsValue;
  /**
  Image number setting (serial number for used images)
  is info valid. */
  bool imageNumberValid;
  /**
  Image number setting (serial number for used images)
  A long positive number increased for every
  used image frame, and placed in image meta data. */
  unsigned long imageNumber;
  /**
  Camera position relative to robot center 
  - is info valid. */
  bool posValid;
  /**
  Camera position relative to robot center. in meter
  x is left, y is up and z is behind - robot center. */
  UPosition posValue;
  /**
  Camera position relative to robot center 
  - is info valid. */
  bool rotValid;
  /**
  Camera rotation relative to robot. in radians
  Omagea is rotation about x,
  Phi is rotation about y and Kappa is rotation about z. */
  UPosition rotValud;
  /**
  Camera gain control-
  is value valid. */
  bool gainValid;
  /**
  Camera gain value:
  Manual gain values are in range from  to 0xFFFF.
  Only some (typically 6 bit) of the MSB bits are 
  actually implemented in camera. */
  unsigned int gainValue;
  /** Camera gain is set to automatic */
  bool gainAutomatic;
  /**
  Shutter control - is value valid. */
  bool shutterValid;
  /**
  Shutter control value in range 0 to 0xFFFF, where the 
  most significant (typically 5 bits) are implemented only. */
  unsigned int shutterValue;
  /**
  Is shutter set to automatic. NB! if automatic, then no values are 
  available for actual setting. */
  bool shutterAutomatic;
  /**
  Video gain contour control - is info valid */
  bool contourValid;
  /**
  Video gain contour - lead filter.
  Enhances edges in horizontal lines. Value is from 0 to 0xFFFF, but
  only some of the MSBs are implemented.
  The value 0 means no enhancement. */
  unsigned int contourValue;
  /**
  Camera LED (on/off control) is info valid. */
  bool ledValid;
  /**
  Camera led value - true if on. */
  bool ledOn;
  /**
  Compression level used in image transfer. Is info valid. */
  bool compressionValid;
  /**
  Compression is needed in some image sizes and is optional
  in others. This value is the preferred cmpression if
  bandwidth allows. */
  unsigned int compressionValue;
  /**
  Gamma correction - is info valid. */
  bool gammaValid;
  /**
  Gamma correction value can change the the number of intensity 
  bits used in the dark part of the image. A preliminary analysis
  suggests that a value of 0 means linear relation from intensity 
  to coding in the 8-bit image. a higher value (up to 0xFFFF) makes
  the intensity coding non-linear compressing the bright parts and 
  expanding the darker part. A value of 0x8000 is the default 
  setting (TV-style gamma correction */
  unsigned int gammaValue;
  /**
  White ballance - is info valid. */
  bool whiteBalValid;
  /**
  White ballance can be set to automatic, a few stanrd settings and
  a manual value, the following values may be used:. 
  PWC_WB_AUTO: Automatic
  PWC_WB_MANUAL: Manual
  PWC_WB_INDOOR: Indoor setting using high temperature wolfram light
  PWC_WB_OUTDOOR: An outdoor setting.
  PWC_WB_FL: Fluorcent lightning
  The values are defined in <pwc-ioctl.h> 
  (in /usr/src/linux/drivers/usb(/media)   */
  unsigned int whiteBalMode;
  /**
  The white ballance gain can in some modes be moditored and can
  be set in manual mode. The values are gain in the red and blue channel
  (compared to the green).
  The range is 0 to 0x7FFF - but only a few of the MSBs are impelmented.
  A value of 0x4000 is assumed to be neutral (gain == 1.0), a higher
  value increases the gain. */
  unsigned int whiteBalGainRed, whiteBalGainBlue;
  /**
  Camera name valid */
  bool nameValid;
  /**
  Camera name */
  char name[MAX_CAMERA_NAME_LENGTH + 1];  
};

#endif
