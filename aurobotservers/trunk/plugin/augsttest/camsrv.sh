#!/bin/bash
gst-launch -v v4l2src device=/dev/video0 ! 'video/x-raw-yuv,width=640,height=480,framerate=5/1' !\
 x264enc pass=qual quantizer=20 tune=zerolatency ! rtph264pay ! udpsink host=127.0.0.1 port=23000

# gst-launch v4l2src device=/dev/video0 ! 'video/x-raw-yuv,width=640,height=480' !\
#  jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=23000
 