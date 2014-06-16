/*******************************************************************************#
#           guvcview              http://guvcview.berlios.de                    #
#                                                                               #
#           Paulo Assis <pj.assis@gmail.com>                                    #
#                                                                               #
# This program is free software; you can redistribute it and/or modify          #
# it under the terms of the GNU General Public License as published by          #
# the Free Software Foundation; either version 2 of the License, or             #
# (at your option) any later version.                                           #
#                                                                               #
# This program is distributed in the hope that it will be useful,               #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 #
# GNU General Public License for more details.                                  #
#                                                                               #
# You should have received a copy of the GNU General Public License             #
# along with this program; if not, write to the Free Software                   #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA     #
#                                                                               #
********************************************************************************/

#ifndef V4L2UVC_H
#define V4L2UVC_H

#include <linux/videodev2.h>
#include <libudev.h>
#include "globals.h"
#include "v4l2_devices.h"
#include "v4l2_formats.h"
#include "v4l2_controls.h"

#define NB_BUFFER 4

#define VDIN_DYNCTRL_OK            3
#define VDIN_SELETIMEOUT_ERR       2
#define VDIN_SELEFAIL_ERR          1
#define VDIN_OK                    0
#define VDIN_DEVICE_ERR           -1
#define VDIN_FORMAT_ERR           -2
#define VDIN_REQBUFS_ERR          -3
#define VDIN_ALLOC_ERR            -4
#define VDIN_RESOL_ERR            -5
#define VDIN_FBALLOC_ERR          -6
#define VDIN_UNKNOWN_ERR          -7
#define VDIN_DEQBUFS_ERR          -8
#define VDIN_DECODE_ERR           -9
#define VDIN_QUERYCAP_ERR        -10
#define VDIN_QUERYBUF_ERR        -11
#define VDIN_QBUF_ERR            -12
#define VDIN_MMAP_ERR            -13
#define VDIN_READ_ERR            -14
#define VDIN_STREAMON_ERR        -15
#define VDIN_STREAMOFF_ERR       -16
#define VDIN_DYNCTRL_ERR         -17

#define HEADERFRAME1 0xaf

//set ioctl retries to 4 - linux uvc as increased timeout from 1000 to 3000 ms  
#define IOCTL_RETRY 4

enum  v4l2_uvc_exposure_auto_type 
{
	V4L2_UVC_EXPOSURE_MANUAL = 1,
	V4L2_UVC_EXPOSURE_AUTO = 2,
	V4L2_UVC_EXPOSURE_SHUTTER_PRIORITY = 4,
	V4L2_UVC_EXPOSURE_APERTURE_PRIORITY = 8
};

static const int exp_vals[]=
{
	V4L2_UVC_EXPOSURE_MANUAL,
	V4L2_UVC_EXPOSURE_AUTO,
	V4L2_UVC_EXPOSURE_SHUTTER_PRIORITY, 
	V4L2_UVC_EXPOSURE_APERTURE_PRIORITY
};

#define MAX_DEV_NAME_CNT 500

struct vdIn 
{
	pthread_mutex_t mutex;                      // VdIn struct mutex
    struct udev *udev;                  // pointer to a udev struct (lib udev)
    struct udev_monitor *udev_mon;      // udev monitor
    int udev_fd;                        // udev monitor file descriptor
	int fd;                             // device file descriptor
	char videodevice[MAX_DEV_NAME_CNT];                  // video device string (default "/dev/video0)"
	int cap_meth;                       // capture method : IO_MMAP (1)  IO_READ (0)
	struct v4l2_capability cap;         // v4l2 capability struct
	struct v4l2_format fmt;             // v4l2 formar struct
	struct v4l2_buffer buf;             // v4l2 buffer struct
	struct v4l2_requestbuffers rb;      // v4l2 request buffers struct
	//struct v4l2_timecode timecode;      // v4l2 timecode struct
	struct v4l2_streamparm streamparm;  // v4l2 stream parameters struct
	struct v4l2_jpegcompression jpgcomp;// v4l2 jpeg compression settings
	void *mem[NB_BUFFER];               // memory buffers for mmap driver frames
	UINT32 buff_length[NB_BUFFER];      // memory buffers length as set by VIDIOC_QUERYBUF
	UINT32 buff_offset[NB_BUFFER];      // memory buffers offset as set by VIDIOC_QUERYBUF
	unsigned char *tmpbuffer;           // temp buffer for decoding compressed data
	unsigned char *framebuffer;         // frame buffer (YUYV), for rendering in SDL overlay
	int isstreaming;                    // video stream flag (1- ON  0- OFF)
	int isbayer;                        // raw bayer flag
	int pix_order;                      // raw bayer pixel order (rg/gb, bg/gr, ...)
	int setFPS;                         // set FPS flag (0-do nothing, 1-change fps value, 2-query and queue buffer)
	int setJPEGCOMP;                    // set jpeg compression flag (0-do nothing, 1-change compression value, 2-query and queue buffer)
	int grabmethod;                     // only mmap available UVC doesn't support read
	struct timeval timestamp;                   //video frame time stamp
	const char *VidFName;                     // Video File name (with full path)
	int capImage;                       // Image capture flag (raised for capturing a frame)
	const char *ImageFName;                   // Image File name (with full path)
	int cap_raw;                        // raw frame capture flag
	int available_exp[4];               //backward compatible (old v4l2 exposure menu interface)
	int PanTilt;                        //1-if PanTilt Camera 0-otherwise
	gboolean signalquit;                // video loop exit flag
	gboolean capVid;                    // Video capture flag (raised while capturing)
	gboolean VidCapStop;                // Video capture stop flag (raised when video capture has stopped)
	gboolean IOfinished;                // Signals that IO thread has finished
	LFormats *listFormats;              // structure with frame formats list
	LDevices *listDevices;              // structure with devices list
};

/** ioctl with a number of retries in the case of I/O failure
* args:
* fd - device descriptor
* IOCTL_X - ioctl reference
* arg - pointer to ioctl data
* returns - ioctl result
*/
int xioctl(int fd, int IOCTL_X, void *arg);

/** Init VdIn struct with default and/or global values
 * args:
 * vd: pointer to a VdIn struct ( must be allready allocated )
 * global: pointer to a GLOBAL struct ( must be allready initiated )
 *
 * returns: error code ( 0 - VDIN_OK)
*/
int init_videoIn(struct vdIn *vd, struct GLOBAL *global);

/** Grabs video frame and decodes it if necessary
 * args:
 * \param vd: pointer to a VdIn struct ( must be allready initiated)
 * \param format: pixel v4l2_format
 * \param width: frame width
 * \param height: frame height
 * \param fps: desired frame rate
 * \param fps_num: ???
 * \param jbuf is external jpeg buffer - if jpeg image is desired external - else set to NULL
 * \param jbufCnt is the size of the external jpeg buffer (set to 0 if no external buffer)
 * is set to the size of the jpeg image on return - or 0 if no jpeg is available
 * \returns: error code ( 0 - VDIN_OK)
*/
int uvcGrab(struct vdIn *vd, int format, int width, int height, 
            int *fps, int *fps_num, 
            unsigned char * jbuff, int * jbufCnt);

/** cleans VdIn struct and allocations
 * args:
 * pointer to initiated vdIn struct
 *
 * returns: void
*/
void close_v4l2(struct vdIn *vd, gboolean control_only);

/** restarts v4l2 device with new format
 * args:
 * vd: pointer to vdIn struct
 * global: poiter to GLOBAL struct 
 *
 * returns: error code ( 0 - VDIN_OK)
*/
int restart_v4l2(struct vdIn *vd, struct GLOBAL *global);

/** sets video device frame rate
 * args:
 * vd: pointer to a VdIn struct ( must be allready initiated)
 * fps: pointer to int containing fps value
 * fps_num: pointer to int containing fps numerator value
 *
 * returns: VIDIOC_S_PARM ioctl result value
 * sets fps and fps_num to device value
*/
int input_set_framerate (struct vdIn * device, int *fps, int *fps_num);

/** gets video device defined frame rate (not real - consider it a maximum value)
 * args:
 * vd: pointer to a VdIn struct ( must be allready initiated)
 * fps: pointer to int containing fps value
 * fps_num: pointer to int containing fps numerator value
 *
 * returns: VIDIOC_G_PARM ioctl result value
 * sets fps and fps_num to device value
*/
int input_get_framerate (struct vdIn * device, int *fps, int *fps_num);

/** gets video stream jpeg compression parameters
 * args:
 * vd: pointer to a VdIn struct ( must be allready initiated)
 *
 * returns: VIDIOC_G_JPEGCOMP ioctl result value
*/
int get_jpegcomp(struct vdIn *vd);

/** sets video stream jpeg compression parameters
 * args:
 * vd: pointer to a VdIn struct ( must be allready initiated)
 *
 * returns: VIDIOC_S_JPEGCOMP ioctl result value
*/
int set_jpegcomp(struct vdIn *vd);

/** Enable video stream
 * args:
 * vd: pointer to a VdIn struct ( must be allready initiated)
 *
 * returns: VIDIOC_STREAMON ioctl result (0- OK)
*/
int video_enable(struct vdIn *vd);

/** Disable video stream
 * args:
 * vd: pointer to a VdIn struct ( must be allready initiated)
 *
 * returns: VIDIOC_STREAMOFF ioctl result (0- OK)
*/
int video_disable(struct vdIn *vd);

#endif

