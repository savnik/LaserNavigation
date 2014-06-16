
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


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <math.h>

#include <ugen4/utime.h>
#include <ugen4/uimage2.h>
#include <ugen4/u2dline.h>
#include <ugen4/uline.h>
#include <ugen4/usmltagin.h>
#include <ugen4/uposrot.h>
#include <ugen4/ufuzzysplit.h>
#include <ugen4/ufuzzypixel.h>
#include <ugen4/upolygon.h>
//#include "urawimage.h"
#include "ucamrad.h"
#include "ucammount.h"
#include "ucamdevieee1394.h"
#include "ucamdevguppy.h"

// sound - text to sound - missing some libraries -lFestival not found
//#include <festival.h>

// prototypes
void printHelp();
int countStatementsInFiles();
int countCodeStatements(const char * filename);
int testTime();
int testImageCopy();
int test2Dline();
bool testRadErr();
bool testRadErr2();
bool testImageLoadSave();
bool testRawImageLoadSave();
bool testMatrixDensity();
bool testEigenvector();
bool testXmlFile();
bool harris();
bool harris2();
bool testLinearAlgebra();
bool testMatrix();
bool testMatrixVec();
bool testMatrixVecBig();
bool testMatrixResize();
bool testImageHandling();
bool testImageHandlingRaw();
bool testOpticalFlow();
bool harris3();
bool testCameraSettings();
bool testCameraHDR();
bool testSegmentation1();
bool testSobel();
bool testCamPar();
bool testLinePlane();
bool testGetKey();
bool testLineSegment();
bool testImageTransfer();
void testHex();
void testPath();
void testImageShift();
void testCylinderCrossing();
bool testImageSize();
void testXmlEscape();
void testTime2();
void testTangetPoint();
void testPlane();
void testPng();
void testZlib();
void testZlibRawImg();
void testPolygon();
void testSignedDistance();
void testUPosRot();
void testOpenCv();
void testVideo();
void test2DlineFit();
void testVideo2();
void testFuzzySplit();
void testFuzzySplit2();
void testCromaImage();
void test3DtoPix();
void testPolygonCog();
void testUImage();
void testColourConvert();
void testPolyOverlap();
void testPolyDistance();
void testVideo3();
void testVideo1394();
void testSSS();
void testGuppy();

//const int MAX_PATH_LENGTH = 200;
//char imagePath[MAX_PATH_LENGTH] = "/home/jca/chr/images";
//char dataPath[MAX_PATH_LENGTH] = "/home/jca/chr/results";

int main(int argc, char *argv[])
{
  bool ask4help;
  appName = strrchr(argv[0], '/');
  //
  ask4help = setCommonPathAndOtherOptions(argc, argv, "localhost", 24920);
  if (ask4help)
    printHelp();
  // testprograms:
  //
  printf("Starting a test ...\n");
  //testTime();
  //countStatementsInFiles();
  //testImageCopy();
  //test2Dline();
  //testRadErr();
  //testImageLoadSave();
  //testRawImageLoadSave();
  //testMatrixDensity();
  //testEigenvector();
  //testXmlFile();
  //harris2();
  //testLinearAlgebra();
  //testMatrix();
  //testMatrixResize();
  //testMatrixVec();
  //testMatrixVecBig();
  //testImageHandling();
  //testImageHandlingRaw();
  //testOpticalFlow();
  //harris3() -- not tested???
  //testCameraSettings();
  //testCameraHDR();
  //testSegmentation1();
  //testSobel();
  //testCamPar();
  //testLinePlane();
  //testGetKey();
  //testLineSegment();
  //testImageTransfer();
  //testCamPool();
  //testHex();
  //testPath();
  //testImageShift();
  //testCylinderCrossing();
  //testImageSize();
  //testXmlEscape();
  //testTime2();
  //testTangetPoint();
  //testPlane();
  //testPng();
  //testZlib();
  //testZlibRawImg();
  //testPolygon();
  //testSignedDistance();
  //testUPosRot();
  //testOpenCv();
  //testVideo();
  //test2DlineFit();
  //testVideo2();
  //testFuzzySplit2();
  //testCromaImage();
  //test3DtoPix();
  //testPolygonCog();
  //testUImage();
  //testColourConvert();
  //testPolyOverlap();
  //testPolyDistance();
  //testVideo3();
  //testVideo1394();
  //testSSS();
  testGuppy();

  printf("... finished test\n");
  printf("This program has terminated\n");
  return EXIT_SUCCESS;
}

void printHelp()
{
  printf("This program includes just common library classes \n");
  printf("(and some testfunctions in main.c for the library)\n");
  printf("- matrix and vector calculation \n");
  printf("- Configuration file class\n");
  printf("- Time class (seconds and microseconds)\n");
  printf("- 3D Position and Rotation functions\n");
  printf("- Image classes and a number of other general classes\n");
  printf("- Common functions like 'int absi(float)' and 'double sqr(double)' etc\n");
  //
  printf("\n");
  printf("options for library tests (most are probably not relevant for test):\n");
  printf(" -h --help This help message\n");
  printf(" -i --imagepath <path>\n");
  printf("    Sets path where images are expected to be found and written\n");
  printf(" -d --datapath <path>\n");
  printf("    Sets path where data other than images are expected to be found and written\n");
  printf(" -c --config <file>\n");
  printf("    Set path and filename for configuration file (default './robcam.conf')\n");
  printf(" -s --server <server host name>\n");
  printf("    sets the default server name, usable by a socket client applications\n");
  printf(" -p --port <port>\n");
  printf("    sets the default port number, usable by a socket server application\n");
  printf("\n");
}
////////////////////////////////////////////////////

int testTime()
{
  UTime t, t2;
  int year;
  int month;
  int day;
  int date;
  char sd[30];
  //
  t.Now();
  year = t.GetYear(&day);
  month = t.GetMonth(&date);
  printf("Today is %d month %d, day %d, at  %d:%d:%d\n",
     year, month, date, t.GetHour(), t.GetMin(), t.GetJustSec());
  t2.SetTime(year, month, date, t.GetHour(), t.GetMin(), t.GetJustSec(), t.GetMicrosec());
  //
  year = t2.GetYear(&day);
  month = t2.GetMonth(&date);
  t2.GetDateString(sd);
  printf("Today is %s at  %d:%d:%d\n", sd, t2.GetHour(), t2.GetMin(), t2.GetJustSec());
  t2.SetTime(2003,1,1);
  t2.GetDateString(sd);
  printf("1 jan 2003 is %s at  %d:%d:%d\n", sd, t2.GetHour(), t2.GetMin(), t2.GetJustSec());
  t2.SetTime(2002,12,31);
  t2.GetDateString(sd);
  printf("31 dec 2002 is %s at  %d:%d:%d\n", sd, t2.GetHour(), t2.GetMin(), t2.GetJustSec());
  t2.SetTime(24*60*60,0);
  t2.GetDateString(sd);
  printf("2 jan 1970 is %s at  %d:%d:%d\n", sd, t2.GetHour(), t2.GetMin(), t2.GetJustSec());
  t2.SetTime(0,0);
  t2.GetDateString(sd);
  printf("1 jan 1970 is %s at  %d:%d:%d\n", sd, t2.GetHour(), t2.GetMin(), t2.GetJustSec());
  t2.SetTime(2*(-24*60*60),0);
  t2.GetDateString(sd);
  printf("30 dec 1969 is %s at  %d:%d:%d\n", sd, t2.GetHour(), t2.GetMin(), t2.GetJustSec());
  t2.SetTime(365*(-24*60*60),0);
  t2.GetDateString(sd);
  printf("1 jan 1969 is %s at  %d:%d:%d\n", sd, t2.GetHour(), t2.GetMin(), t2.GetJustSec());
  return 0;
}

///////////////////////////////////////

int countStatementsInFiles()
{
  int result = 0;
  int g = 0;
  // Gen
  result += countCodeStatements("main.cpp");
  result += countCodeStatements("u3d.h");
  result += countCodeStatements("u3d.cpp");
  result += countCodeStatements("conf.h");
  result += countCodeStatements("conf.cpp");
  result += countCodeStatements("ucommon.h");
  result += countCodeStatements("ucommon.cpp");
  result += countCodeStatements("uimage.h");
  result += countCodeStatements("uimage.cpp");
  result += countCodeStatements("uimage2.h");
  result += countCodeStatements("uimage2.cpp");
  result += countCodeStatements("uline.h");
  result += countCodeStatements("uline.cpp");
  result += countCodeStatements("umatrix.h");
  result += countCodeStatements("umatrix.cpp");
  //
  printf("---- Gen:     %d statements\n\n", result - g);
  g = result;
  // Vita
  result += countCodeStatements("../../vita/vita/main.cpp");
  result += countCodeStatements("../../vita/vita/uevitacommon.h");
  result += countCodeStatements("../../vita/vita/uevitacommon.cpp");
  result += countCodeStatements("../../vita/vita/uevita.h");
  result += countCodeStatements("../../vita/vita/uevita.cpp");
  result += countCodeStatements("../../vita/vita/umemmodule.h");
  result += countCodeStatements("../../vita/vita/umemmodule.cpp");
  result += countCodeStatements("../../vita/vita/usem.h");
  result += countCodeStatements("../../vita/vita/usem.cpp");
  result += countCodeStatements("../../vita/vita/usensordata.h");
  result += countCodeStatements("../../vita/vita/usensordata.cpp");
  result += countCodeStatements("../../vita/vita/usocketserver.h");
  result += countCodeStatements("../../vita/vita/usocketserver.cpp");
  result += countCodeStatements("../../vita/vita/usysstate.h");
  result += countCodeStatements("../../vita/vita/usysstate.cpp");
  //
  printf("---- vita:    %d statements\n\n", result - g);
  g = result;
  //
  result += countCodeStatements("../../vision/vision/main.cpp");
  result += countCodeStatements("../../vision/vision/camera.h");
  result += countCodeStatements("../../vision/vision/camera.cpp");
  result += countCodeStatements("../../vision/vision/ucalibrate.h");
  result += countCodeStatements("../../vision/vision/ucalibrate.cpp");
  result += countCodeStatements("../../vision/vision/ucluster.h");
  result += countCodeStatements("../../vision/vision/ucluster.cpp");
  result += countCodeStatements("../../vision/vision/udetection.h");
  result += countCodeStatements("../../vision/vision/udetection.cpp");
  result += countCodeStatements("../../vision/vision/udetectlist.h");
  result += countCodeStatements("../../vision/vision/udetectlist.cpp");
  result += countCodeStatements("../../vision/vision/uevisioncommon.h");
  result += countCodeStatements("../../vision/vision/uevisioncommon.cpp");
  result += countCodeStatements("../../vision/vision/uevision.h");
  result += countCodeStatements("../../vision/vision/uevision.cpp");
  result += countCodeStatements("../../vision/vision/uimageset.h");
  result += countCodeStatements("../../vision/vision/uimageset.cpp");
  result += countCodeStatements("../../vision/vision/uimagesetscan.h");
  result += countCodeStatements("../../vision/vision/uimagesetscan.cpp");
  result += countCodeStatements("../../vision/vision/upremap.h");
  result += countCodeStatements("../../vision/vision/upremap.cpp");
  result += countCodeStatements("../../vision/vision/uvimage.h");
  result += countCodeStatements("../../vision/vision/uvimage.cpp");
  //
  printf("---- vision:  %d statements\n\n", result - g);
  g = result;
  //
  result += countCodeStatements("../../drive/drive/main.cpp");
  result += countCodeStatements("../../drive/drive/udrive.h");
  result += countCodeStatements("../../drive/drive/udrive.cpp");
  result += countCodeStatements("../../drive/drive/uedrivecommon.h");
  result += countCodeStatements("../../drive/drive/uedrivecommon.cpp");
  result += countCodeStatements("../../drive/drive/uedrive.h");
  result += countCodeStatements("../../drive/drive/uedrive.cpp");
  result += countCodeStatements("../../drive/drive/uirsensor.h");
  result += countCodeStatements("../../drive/drive/uirsensor.cpp");
  result += countCodeStatements("../../drive/drive/ulinesensor.h");
  result += countCodeStatements("../../drive/drive/ulinesensor.cpp");
  result += countCodeStatements("../../drive/drive/urobot.h");
  result += countCodeStatements("../../drive/drive/urobot.cpp");
  //
  printf("---- drive:   %d statements\n\n", result - g);
  g = result;
  //
  result += countCodeStatements("../../map/map/main.cpp");
  result += countCodeStatements("../../map/map/uemapcommon.h");
  result += countCodeStatements("../../map/map/uemapcommon.cpp");
  result += countCodeStatements("../../map/map/uemap.h");
  result += countCodeStatements("../../map/map/uemap.cpp");
  result += countCodeStatements("../../map/map/umap.h");
  result += countCodeStatements("../../map/map/umap.cpp");
  result += countCodeStatements("../../map/map/umapmake.h");
  result += countCodeStatements("../../map/map/umapmake.cpp");
  //
  printf("---- map:     %d statements\n\n", result - g);
  g = result;
  //
  printf("**** total    %d statements\n", result);
  //
  return result;
}

////////////////////////////////////////////////////

int countCodeStatements(const char * filename)
{  // count number of ';' in this file
   FILE * f = NULL;
   char c;
   int result = 0;
   //
   f = fopen(filename, "r");
   if (f != NULL)
   {
     while (not feof(f))
     {
       c = fgetc(f);
       if (c == ';')
         result++;
     }
     fclose(f);
   }
   printf("Found %d statements in %s\n", result, filename);
   //
   return result;
}

//////////////////////////////////////////////////////

int test2Dline()
{
  bool result;
   U2Dline l1(-1.0, -1.0, 5.0); // y = -x + 5
   U2Dline l2;
   U2Dline l3(0.0, 1.0, 0.0); // y = 0
   float x[10] = {1.0, 3.0, 5.0, 6.0, 8.0, 9.0, 10.0, 10.0, 11.0, 12.0};
   float y[10] = {1.0, 2.0, 2.0, 3.0, 3.0, 5.0,  4.0,  3.0,  4.0,  5.0};
   float d,e,f,g;
   float E; // mean
   float V; // variance
   float cx, cy;
   int err;
   //
   err = l2.set(0.0, 0.0, 1.0); // error
   err = l2.set(1.0, 0.0, -7.0); // OK
   result = l2.getCrossing(l3, &cx, &cy); // x = 7, y = 0
//   err = l2.getCrossingA(l3, &cx, &cy);
   result = l3.getCrossing(l1, &cx, &cy); // x = 5, y = 0
//   err = l3.getCrossingA(l1, &cx, &cy);
   l1.set(1.0, -1.0,  0.0);
   l2.set(1.0,  1.0, -3.0);
   result = l2.getCrossing(l1, &cx, &cy); // x = 1.5, y = 1.5
//   err = l2.getCrossingA(l1, &cx, &cy);
   l1.set(1.0, 1.0, -4.0); // parallel to l2
   l2.set(1.0, 1.0, -3.0);
   result = l1.getCrossing(l2, &cx, &cy); // false
//   err = l1.getCrossingA(l2, &cx, &cy);
   l1.set(0.0, -1.0,  0.0);
   l2.set(0.0, -1.0, -3.0);
   result = l1.getCrossing(l2, &cx, &cy); // false too
//   err = l1.getCrossingA(l2, &cx, &cy);

   // set from series of points - best fit line
   l2.set(x,y,10);
   V = l2.variance(x, y, 10, &E); // E should be ~0.0
   V = l3.variance(x, y, 10, &E); // E should be ~3
   // get line in point-vector format
   l1.getPV(&d, &e, &f, &g);
   l3.setPV(d, e, f, g);
   // set from 2 points
   l3.set2P(0.0, -2.0, 5.0, -2.0);
   l3.set2P(30.0, 5.0, 30.0, 0.0);
   l3.set2P(0.0, 5.0, 5.0, 0.0);
   // point on line near x,y)
   l3.getOnLine(0.0, 0.0, &cx, &cy); // (cx,cy) should be (2.5,2.5)
   l3.getOnLine(7.0, 7.0, &cx, &cy); // (cx,cy) should be (2.5,2.5)
   l3.getOnLine(3.0, 2.0, &cx, &cy); // (cx,cy) should be (3.0,2.0)
   //
   l3.set2P(0.0, 5.0, 1.0, 0.0);
   // point on line near x,y)
   l3.getOnLine(0.0, 0.0, &cx, &cy); // (cx,cy) should be app.(1,0)
   l3.getOnLine(7.0, 7.0, &cx, &cy); // (cx,cy) should be app.(0,6)
   l3.getOnLine(2.0, -5.0, &cx, &cy); // (cx,cy) should be (2.0,-5.0)
   //
   l3.set2P(0.0, 1.0, 5.0, 0.0);
   // point on line near x,y)
   l3.getOnLine(0.0, 0.0, &cx, &cy); // (cx,cy) should be app.(0,1)
   l3.getOnLine(7.0, 7.0, &cx, &cy); // (cx,cy) should be app.(6,0)
   l3.getOnLine(10.0, -1.0, &cx, &cy); // (cx,cy) should be (10.0,-1.0)
   return err;
}


//#ifndef NO_OPENCV_HIGHGUI

bool testRadErr()
{
  bool result = true;
  UImage320 rawerr; // distorted image
  UImage320 rawnoerr; // clean image
  //UCamRad cam;
  UCamRad * cam = NULL;
  UCamDevBase * dev;
  //char s[MAX_CAMERA_NAME_LENGTH];
  char * cn = NULL;
  //unsigned int r,c;
  //bool isOK = false;
  //const char path[] = "/home/jca/jca/images/";
  int i;
  //
  printf("Testing radial error from /dev/video0 create camera\n");
  printf("create camera ...\n");
  //
  dev = new UCamDevBase();
  dev->setDeviceNumber(1);
  cam = new UCamRad(dev);
  //
  printf("Getting camera name ... \n");
  cn = dev->getCameraName();
  if (cn == NULL)
    printf("No camera available\n");
  else
    printf("Got camera %s\n", cn);

  //
  if (true and cn != NULL)
  {
    dev->setDevice(320, 240, 5);
    dev->openDevice();
    cam->setCameraParameters(
                      dev->getWidth(), dev->getHeight(),
                      2.3e-7, -2.8e-13,
                      1049);
    //Wait(1.1);
    for (i = 0; i < 4; i++)
    { // wait for next image at 5 frames/sec
      Wait(0.2);
      if (i %2 == 0)
        result = cam->getImageSnapshot(&rawerr); //, false);
      else
        result = cam->getImageSnapshot(&rawnoerr); //, false);
      if (result)
        printf("Got image\n");
      else
        printf("Failed to get image\n");
    }
    // test save image
    if (rawerr.valid)
    {
      rawerr.toBGR(NULL);
      rawerr.saveBMP("imgRadErr.bmp");
    }
    if (rawnoerr.valid)
    {
      //rawnoerr.toRGB();
      rawnoerr.saveBMP("imgRadErrNot.bmp");
    }
  }
  //
  if (cam != NULL)
    delete cam;
  return result;
}

/////////////////////////////////////////////////////////////////////

// bool testRadErr2()
// {
//   bool result = true;
//   URawImage raw; // distorted image
//   URawImage img; // clean image
//   UImage320 rgbRaw;     // used to save result
//   UImage320 rgbImg;     // used to save result
//   UCamRad cam;
//   char s[MAX_CAMERA_NAME_LENGTH];
//   char * cn;
//   unsigned int r,c;
//   //bool isOK = false;
//   //const char path[] = "/home/jca/jca/images/";
//   int i;
//   //
//   printf("Testing radial error from /dev/video0\n");
//   printf("Setting device number ...\n");
//   //
//   cam.setDeviceNumber(0);
//   printf("Getting camera name ... \n");
//   cn = cam.getCameraName();
//   if (cn == NULL)
//     printf("No camera available\n");
//   else
//     printf("Got camera %s\n", cn);
//   //
//   if (true and cn != NULL)
//   {
//     result = cam.openAndSetDevice(320, 240, 5);
//     //Wait(1.1);
//     for (i = 0; i < 4; i++)
//     { // wait for next image at 5 frames/sec
//       //Wait(0.2);
//       if (result)
//         result = cam.getImageSnapshot(&raw, false);
//       if (result)
//         printf("Got image\n");
//       else
//         printf("Failed to get image\n");
//     }
//     if (result)
//     { // set parameters and remove radial error
//       cam.getCamPar()->setCameraParameters(320, 240, // head point
//                   5.4e-7, -7.8e-13,  1050,
//                   2.0);
//       cam.setRadialErrorMatrix();
//       snprintf(s, MAX_CAMERA_NAME_LENGTH,
//              "%s/radmatrix_barrol.txt", imagePath);
//       cam.saveRadialCorrectionMatrix(s);
//       result = cam.removeRadialError(&raw, &img);
//     }
//     if (result)
//     {
//       result = raw.moveToRGB(&rgbRaw, true);
//       if (result)
//         rgbRaw.saveBMP(imagePath, "rad", int(raw.imageNumber), "raw");
//       result = img.moveToRGB(&rgbImg, true);
//       if (result)
//         rgbImg.saveBMP(imagePath, "rad", int(raw.imageNumber), "img");
//     }
//   }
//   if (false)
//   { // make square image to test on synthetic image
//     raw.setSize(240, 320);
//     raw.radialErrorRemoved = false;
//     raw.isBW = false;
//     raw.imgTime.Now();
//     for (r = 0; r < raw.getHeight(); r = r + 2)
//       for (c = 0; c < raw.getWidth(); c = c + 2)
//       {
//         if (((r % 20) == 0) or ((c % 20) == 0))
//         {
//           if (((r % 40) == 0) or ((c % 40) == 0))
//             raw.setQuadPix(r, c, 0, 40, 80, 128, 128, 128);
//           else
//             if ((r % 40) == 20)
//               raw.setQuadPix(r, c, 200, 200, 200, 200, 32, 128);
//             else
//               raw.setQuadPix(r, c, 200, 200, 200, 200, 128, 32);
//         }
//         else
//           raw.setQuadPix(r, c, 255, 255, 255, 255, 128, 128);
//       }
//     raw.valid = true;
//     raw.moveToRGB(&rgbRaw, true);
//     rgbRaw.saveBMP(imagePath, "squares", 0, "");
//     //
//     cam.getCamPar()->setCameraParameters(320, 240, // head point
//                   5.4e-7, -7.8e-13,   // a3 (K1), a5 (K2)
//                   1050,               // focal length
//                   2.0                // pixel size
//                   );     // camera name
//     cam.setRadialErrorMatrix();
//     // URawImage based
//     result = cam.removeRadialError(&raw, &img);
//     img.moveToRGB(&rgbImg, true);
//     rgbImg.saveBMP(imagePath, "rad", int(raw.imageNumber) , "barrol");
//     // use UPixel image
//     result = cam.removeRadialError(&rgbRaw, &rgbImg);
//     rgbImg.saveBMP(imagePath, "rad", int(raw.imageNumber), "barrolRgb");
//     // pude
//     cam.getCamPar()->setCameraParameters(320, 240, // head point
//                 -5.4e-7, 7.8e-13,     // k1, k2
//                 1050,                 // focal length
//                 2.0                  // pixel size
//                 );// camera name
//     cam.setRadialErrorMatrix();
//     snprintf(s, MAX_CAMERA_NAME_LENGTH,
//            "%sradmatrix_pillow.txt", imagePath);
//     cam.saveRadialCorrectionMatrix(s);
//     // URawImage based
//     result = cam.removeRadialError(&raw, &img);
//     img.moveToRGB(&rgbImg, true);
//     rgbImg.saveBMP(imagePath, "rad", int(raw.imageNumber), "pillow");
//     // use UPixel image
//     result = cam.removeRadialError(&rgbRaw, &rgbImg);
//     rgbImg.saveBMP(imagePath, "rad", int(raw.imageNumber), "pillowRgb");
//   }
//   printf("Finished radial error removal test\n");
//   return result;
// }

///////////////////////////////////////////////////////
#ifndef NO_OPENCV_HIGHGUI

bool testImageLoadSave()
{
  UImage640 img;
  UImage640 img2;
  bool result;
  //
  result = img.load("log01040907_082835.245_2", "bmp", imagePath);
  if (not result)
    printf("Load failed");
  //
  img.imageNumber = 12649;
  strcpy(img.name, "log01040907_082835.245_2a");
  Wait(0.1);
  img.imgTime.Now();
  img.pos.set(1.0, 2.0, 3.0);
  img.rot.set(0.01, 0.02, 0.03);
  //
  result = img.saveMeta(NULL, "xml", imagePath);
  if (not result)
    printf("save meta failed\n");
  result = img.save(NULL, NULL, imagePath);
  if (not result)
    printf("save image failed\n");
  // load image before meta
  result = img2.load(img.name, NULL, imagePath);
  if (not result)
    printf("load image failed\n");
  result = img2.loadMeta("img0000", "xml", imagePath);
  if (not result)
    printf("load meta failed\n");
  //
  result = img2.saveMeta("img0001", "xml", imagePath);
  if (not result)
    printf("save meta1 failed\n");
  result = img2.save("img0001", "png", imagePath);
  if (not result)
    printf("save image1 failed\n");
  return result;
}

//////////////////////////////////////////////////////////

bool testRawImageLoadSave()
{
  URawImage img;
  URawImage img2;
  bool result;
  //
  result = img.loadImage("log01040907_082835.245_2.bmp", imagePath);
  if (not result)
    printf("Load failed\n");
  //
  img.imageNumber = 12649;
  img.imgTime.Now();
  //
  if (result)
    result = img.saveMeta("rimg0000.ini",  imagePath);
  if (not result)
    printf("save meta failed\n");
  if (result)
    result = img.saveImage("rimg0000.bmp",  imagePath);
  if (not result)
    printf("save image failed\n");
  // load image before meta
  if (result)
    result = img2.loadImage("rimg0000",  imagePath);
  if (not result)
    printf("load image failed\n");
  if (result)
    result = img2.loadMeta("rimg0000",  imagePath);
  if (not result)
    printf("load meta failed\n");
  //
  if (result)
    result = img2.saveMeta("rimg0001",  imagePath);
  if (not result)
    printf("save meta1 failed\n");
  if (result)
    result = img2.saveImage("rimg0001",  imagePath);
  if (not result)
    printf("save image1 failed\n");
  return result;
}

#endif

//////////////////////////////////////////////

bool testMatrixDensity()
{ // test matrix density function (for size 4 matrix/vector)
   bool result;
  UMatrix4 Q(3,3,1.0), Qi; // covariance matrix / inversed
  UMatrix4  v(1, 3, 0.0); // vector position
  UMatrix4 vm(1, 3, 0.0); // mean value
  UMatrix4 vd(1, 3);
  double d, dens, e, ee, p; // density
  const int MAXS = 200;
  char s[MAXS];
  int err, n;
  bool OK;
  //
  // make covariance matrix from this vector
  Q.setDiag(sqr(7.0), 1.0, 1.0);
  d = Q.det(&err);
  v.set(1.0, 1.0, 1.0);
  Q.print("Q");
  printf("Q determinant is %f (err = %d)\n", d, err);
  OK = (d > 1e-10) and (err == 0);
  if (OK)
  {
    Qi = Q.inversed();
    OK = (Qi.err == 0);
  }
  if (OK)
  {
    vd = v - vm;
    e = ((vd * Qi * vd.transposed()) * -0.5).get(0);
    ee = exp(e);
    n = Q.rows();
    p = pow(sqrt(2 * M_PI), n);
    dens = ee / (p * sqrt(d));
    snprintf(s, MAXS, "Density is %f for v=", dens);
    v.print(s);
  }
  if (true)
  {
    d = Q.density(v, vm, &result);
    if (result)
    {
      snprintf(s, MAXS, "Density is %f for v=", d);
      v.print(s);
    }
    else
      printf("Density evaluation failed\n");
  }
  //
  return result;
}

///////////////////////////////////////////

bool testEigenvector2x2()
{
  UMatrix4 mA(2,2);
  UMatrix4 roots(2,1);
  UMatrix4 eigenvectors(2,2);
  bool complex;
  //
  // matrix mA = [1,2;3,4]
  // roots = 5.3723, -0.3723
  printf("Test eigenvectors\n");
  mA.setRow(0,1.0,2.0);
  mA.setRow(1,3.0,4.0);
  roots = mA.eig2x2(&complex, &eigenvectors);
  mA.print("mA");
  roots.print("Roots");
  eigenvectors.print("eigenvectors");
  if (complex)
    printf(" --- complex\n");
  //
  mA.setRow(0,1.0,1.0);
  mA.setRow(1,1.0,1.0);
  roots = mA.eig2x2(&complex, &eigenvectors);
  mA.print("mA");
  roots.print("Roots");
  eigenvectors.print("eigenvectors");
  if (complex)
    printf(" --- complex\n");
  //
  mA.setRow(0,1.0,-2.0);
  mA.setRow(1,2.0,4.0);
  roots = mA.eig2x2(&complex, &eigenvectors);
  mA.print("mA");
  roots.print("Roots");
  eigenvectors.print("eigenvectors");
  if (complex)
    printf(" --- complex\n");
  //
  mA.setRow(0,1.0,0.0);
  mA.setRow(1,0.0,5.0);
  roots = mA.eig2x2(&complex, &eigenvectors);
  mA.print("mA");
  roots.print("Roots");
  eigenvectors.print("eigenvectors");
  if (complex)
    printf(" --- complex\n");
  return true;
}

//////////////////////////////////////////////////////

bool testEigenvector()
{
  UMatrix4 mA(4,4);
  UMatrix4 mB(4,4);
  UMatrix4 mW(4,4);
  UMatrix4 mU(4,4);
  UMatrix4 mV(4,4);
  UMatrix4 mR(4,4);
  UMatrix4 m(4,4);
  UMatrix4 vE;
  bool isComplex;
  //
  mA.setRow(0, 10.0, 3.0,  2.0,  1.0);
  mA.setRow(1, 1.0, 11.0,  4.0,  1.0);
  mA.setRow(2, 0.5,  1.0, 15.0,  2.0);
  mA.setRow(3, 0.25, 4.0,  7.0, 33.0);
  //
  // make symmetric
  mB = mA + mA.transposed();
  mB.mult(0.5);
  mA.print("mB");
  //
  // find eigenvalues
  mB.eig(&mW, &mU, &mV, CV_SVD_MODIFY_A + CV_SVD_V_T);
  //
  mW.print("Eigenvalues mW");
  mU.print("Eigenvector mU");
  mV.print("Eigenvector mV'");
  //
  mR = mU * mW * mV;
  mR.print("mR = mU * mW * mV ~= mA");
  //
  m = mA * mU;
  m.print("mA * mU");
  m = mU * mW;
  m.print("mU * mW");
  m = mA * mV.transposed();
  m.print("mA * mV");
  m = mV.transposed() * mW;
  m.print("mV * mW");
  //
  //
  // now try eigenvalues and eigenvectors for a 2x2 matrix
  mA.setSize(2,2);
  mA.setRow(0, 10.0, -8.0);
  mA.setRow(1, -8.0, 20.0);
  //
  mB = mA;
  mA.print("mA - source 2x2 matrix");
  //
  mR.setSize(2,2);
  vE = mA.eig2x2(&isComplex, &mR);
  if (isComplex)
    printf("Eigenvalues of 2x2 matrix are complex\n");
  vE.print("2x2-vectors");
  mR.print("Eigenvectors");
  //
  mW.setSize(2,2);
  mU.setSize(2,2);
  mV.setSize(2,2);
  mB.eig(&mW, &mU, &mV, CV_SVD_MODIFY_A + CV_SVD_V_T);
  mW.print("Eigenvalues mW 2x2");
  mU.print("Eigenvector mU 2x2");
  mV.print("Eigenvector mV 2x2'");
  //
  return true;
}


//////////////////////////////////////////////////////

// bool testXmlFile()
// {
//   bool result = true;
//   Uxml3D * xf;
//   UMatrixBig mm(3, 11, 1.0);
//   UMatrixBig mn(2, 2);
//   char * data;
//   int v;
//   double dv;
//   UTime t, t2;
//   const char s[] = "Christian Andersen";
//   char name[UxmlFile::XML_NAME_LENGTH];
//   char tagName[UxmlFile::XML_NAME_LENGTH];
//   char value[UxmlFile::XML_NAME_LENGTH];
//   UPosition pos(1.1, 2.2, 3.3), pos2;
//   UPosition pos3(2.1, 3.2, 4.3);
//   ULineSegment line, line2;
//   URotation rot(0.001,0.002,0.003), rot2;
//   //
//   xf = new Uxml3D("xml2.3dxml", 'w');
// //  fxf = xf->getFile();
//   result = (xf != NULL);
//   if (result)
//   { // make an xml-like file
//     xf->saveStartTag("doc", "main block", false);
//     mm.setRC(0,  3, 12345678.901234567);
//     mm.setRC(0,  9, 10.1);
//     mm.setRC(0, 10, 11.1);
//     mm.setRC(1,  9, 10.2);
//     mm.setRC(1, 10, 11.2);
//     mm.setRC(2,  9, 10.3);
//     mm.setRC(2, 10, 11.3);
//     xf->saveString("#!map-lib-test\n");
//     result = xf->saveMatrix(&mm, "test");
//     //
//     t.Now();
//     xf->saveTime(t, "now");
//     //
//     xf->saveStartTag("doc", "nested block", false);
//     xf->saveStringTag("myName", s);
//     xf->saveIntegerTag("test", 27);
//     xf->saveDoubleTag("version", 1.25);
//     xf->save3DposTag(pos, "myPosition");
//     xf->save3DrotTag(rot, "OmegaPhiKappa");
//     //
//     line.setFromPoints(&pos, &pos3);
//     line.show("source myLine");
//     // false end tag
//     //xf->saveEndTag("falseEndTag");
//     // right end tag
//     xf->saveEndTag("doc");
//     xf->saveLineSeg(line, "myLine");
//     //
//     // missing end tag
//     //xf->saveEndTag("doc");
//     //
//     if (result)
//       xf->closeFile();
//   }
//   //
//   // read file back
//   //
//   if (result)
//   { // open for reading
//     result = xf->openFile("xml2.3dxml", 'r');
//   }
//   if (result)
//   { // read matrix back
//     while (true)
//     {
//       data = xf->getNextTag();
//       // stop if no more tags (end of file)
//       if (data == NULL)
//         break;
//       if (xf->isTagA("matrix"))
//       { // buffer is start of matrix
//         xf->setBufferUnused();
//         xf->loadMatrix(&mn, name);
//         mn.print("mn");
//       }
//       else if (xf->isTagA("datetime"))
//       { // buffer is time value
//         xf->setBufferUnused();
//         xf->loadTime(&t2, value);
//         t2.show();
//       }
//       else if (xf->isTagA("myName"))
//       {
//         xf->getNextAttribute(&data, NULL, value,
//                      UxmlFile::XML_NAME_LENGTH);
//         printf("Read name is '%s'\n", value);
//       }
//       else if (xf->isTagA("test"))
//       {
//         xf->setBufferUnused();
//         if (xf->loadIntTag(&v, tagName))
//           printf("Found %s = %d\n", xf->getTagName(), v);
//         else
//           printf("Read Integer for %s failed\n", tagName);
//       }
//       else if (xf->isTagA("version"))
//       {
//         xf->setBufferUnused();
//         if (xf->loadDoubleTag(&dv, tagName))
//           printf("Found %s = %f\n", tagName, dv);
//         else
//           printf("Read double for %s failed\n", tagName);
//       }
//       else if (xf->isTagA("pos3D"))
//       {
//         xf->setBufferUnused();
//         xf->load3DposTag(&pos2, name);
//         pos2.show(name);
//       }
//       else if (xf->isTagA("rot3D"))
//       {
//         xf->setBufferUnused();
//         xf->load3DrotTag(&rot2, name);
//         rot2.show(name);
//       }
//       else if (xf->isTagA("lineSegment"))
//       {
//         xf->setBufferUnused();
//         xf->loadLineSeg(&line2, name);
//         line2.show(name);
//       }
//       else if (xf->isTagA("doc"))
//       {
//         xf->skipThisTag();
//       }
//       else if (xf->isTagAnEnd("doc"))
//         // that is OK - just skip
//         ;
//       else
//         xf->errRemark("Do not know this tag!", tagName);
//     }
//   }
//   //
//   return result;
// }

//////////////////////////////////////////////////////

#ifndef NO_OPENCV_HIGHGUI

bool harris()
{
  bool result;
  UImage640 i1;
  UImage640 i2;
  const int mask = 5;
  //const char imgn[] = "img10014f";
  const char imgn[] = "vision/snapshot-20040818-091843";
  char fn[100];
  //const float ss[mask] = {0.1, 0.2, 0.5, 0.9, 0.5, 0.2, 0.1};
  double ss[mask]; // = {0.2, 0.5, 1.0, 0.5, 0.2};
  UMatrix4 m(2,2);
  double m11, m12, m22;
  double Rx, Ry, Gx, Gy, Bx, By, d, h, t, hl;
  int i, j, x, y;
//  UPixel * lpxi;
  UPixel ipx1, ipx2, ipx3, ipx4, ipx5;
  UPixel hpx;
  UPixel * lpx;
  int err, hi, sh, sr, himax = 0;
  //
  t = 0.0;
  for (i = 0; i < mask; i++)
  {
    h = double(mask-1) / 2.0;
    d = (double(i) - h)*2.0/h; // (x value)
    ss[i] = 1.0/sqrt(2.0 * PI) * exp(-sqr(d)/2.0);
    t += ss[i];
  }

  printf("Testing generation of Harris-points\n");
  result = i1.load(imgn, "bmp", imagePath);
  if (not result)
    printf("File not found: '%s/%s'\n",imagePath, fn);
  if (result)
  {
    i2.copy(&i1);
    for (x = 0; x < int(i1.height()) - mask - 2; x++)
    {
      for (y = 0; y < int(i1.width()) - mask - 2; y++)
      {
        m11 = 0.0;
        m12 = 0.0;
        m22 = 0.0;
        for (i = 0; i< mask; i++)
        {
          for (j = 0; j< mask; j++)
          {
            ipx1 = i1.getPix(x+i,y+j);
            ipx2 = i1.getPix(x+i+1,y+j);
            Rx = ipx1.r - ipx2.r;
            Gx = ipx1.r - ipx2.r;
            Bx = ipx1.r - ipx2.r;
            ipx2 = i1.getPix(x+i,y+j+1);
            Ry = ipx1.r - ipx2.r;
            Gy = ipx1.r - ipx2.r;
            By = ipx1.r - ipx2.r;
            d = ss[i]*ss[j];
            m11 += (sqr(Rx) + sqr(Gx) + sqr(Bx)) * d;
            m12 += (Rx*Ry + Gx*Gy + Bx*By) * d;
            m22 += (sqr(Ry) + sqr(Gy) + sqr(By)) * d;
          }
        }
        m.setRow(0, m11, m12);
        m.setRow(1, m12, m22);
        d = m.det(&err);
        t = m.trace();
        h = (d - 0.04*sqr(t));
        hi = maxi(0,roundi(h / sqr(255.0)));
        if ((h > hl) or (y == 0))
          sh = ipx1.g/2 + 127;
        else
          sh = ipx1.g/2;
        hl = h;
        ipx2 = i2.getPix(x+mask/2-1, y+mask/2);
        if ((ipx2.r <= hi) or (x == 0))
          sr = ipx1.b/2 + 127;
        else
          sr = ipx1.b/2;
        if (hi > himax)
        {
          himax = hi;
          printf("Max = %d\n", himax);
        }
        if (err == 0)
          ipx1.set(hi, sh, sr);
        else
          ipx1.clear();
        i2.setPix(x+mask/2, y+mask/2, ipx1);
      }
    }
    j = 0;
    ipx1.set(0,0,0);
    i1.tone(&ipx1, 50);
    for (x = mask/2; x < int(i1.height()) - mask/2 - 3; x++)
    {
      for (y = mask/2; y < int(i1.width()) - mask/2 - 3; y++)
      {
        lpx = i2.getLine(x);
        ipx1 = lpx[y];
        //ipx2 = lpx[y-1];
        ipx3 = lpx[y+1];
        //ipx4 = i2.GetPix(x-1,y);
        ipx5 = i2.getPix(x+1,y);
        if ((ipx1.g > 127) and (ipx3.g < 128) and
            (ipx1.b > 127) and (ipx5.b < 128) and
            (ipx1.r > 1))
        {
           ipx1.set(255,ipx1.g,255);
           i2.setPix(x,y, ipx1);
           i1.setPix(x,y, ipx1);
           j++;
        }
      }
    }
    printf("Found %d harris points\n", j);
    snprintf(fn, 100, "%s-harris", imgn);
    i2.save(fn, "png", imagePath);
    snprintf(fn, 100, "%s-harris-tone", imgn);
    i1.save(fn, "png", imagePath);
  }
  return result;
}

/////////////////////////////////////////////////////////////

bool harris2()
{
  bool result;
  UImage640 is;
  UImage320 i1;
  UImage320 i2;
  const int mask = 5;
  //const char imgn[] = "img10014f";
  //const char imgn[] = "vision/snapshot-20040818-091843";
  // const char imgn[] = "vision/snapshot-20040818-124936";
  const char imgn[] = "log01040908_153837.328_108";
  char fn[100];
  double ss[mask]; // = {0.2, 0.5, 1.0, 0.5, 0.2};
//  UMatrix4 m(2,2);
  double m11, m12, m22;
  double d, h, t;
  int i, j, x, y, v, vmin;
  const int line_width = 320;
  double hval[line_width];
  bool hxinc[line_width];
  bool hyinc[line_width];
  bool hxi, hyi;
  int Rx[mask][line_width]; int Gx[mask][line_width]; int Bx[mask][line_width];
  int Ry[mask][line_width]; int Gy[mask][line_width]; int By[mask][line_width];
  int gi, gii; // gradient index
  int * Rxi;
  int * Gxi;
  int * Bxi;
  int * Ryi;
  int * Gyi;
  int * Byi;
  // resulting points
  const int max_pkts = 200;
  int hpx[max_pkts+1];
  int hpy[max_pkts+1];
  int hpv[max_pkts+1];
  int hpCnt = 0;
  UPixel ipx;
  UPixel * lpx;
  UPixel * lpy;
  int hi, himax = 0;
  //
  // debug
  int n, k;
  int line1r[12];
  int line2r[12];
  double hmask[mask * mask];
  UPixel iph;
  UImage320 ihx;
  UImage320 ihy;
  UImage320 ihxy;
  // debug end
  //
  t = 0.0;
  for (i = 0; i < mask; i++)
  {
    h = double(mask-1) / 2.0;
    d = (double(i) - h)*2.0/h; // (x value)
    ss[i] = 1.0/sqrt(2.0 * PI) * exp(-sqr(d)/2.0);
    t += ss[i];
  }

  for (i = 0; i < line_width; i++)
  { // zero harris value and gradient sign
    hval[i] = 0.0;
    hxinc[i] = false; // decreasing x gradient
    hyinc[i] = false; // decreasing y gradient
  }

  printf("Testing generation of Harris-points\n");
  result = is.load(imgn, "bmp", imagePath);
  i1.copyScaleDown(&is, 2);
  if (not result)
    printf("File not found: '%s/%s'\n",imagePath, fn);
  if (result)
  {
    i2.copy(&i1);
    ipx.set(0,0,0);
    i2.tone(&ipx, 80);
    ipx.set(255,255,255);
    // debug
    ihx.copy(&i2);
    ihy.copy(&i2);
    ihxy.copy(&i2);
    // debug end
    //
    gi = 0;
    Rxi = Rx[0]; Gxi = Gx[0]; Bxi = Bx[0]; Ryi = Ry[0]; Gyi = Gy[0]; Byi = By[0];
    while (true)
    { // fill gradient values up to l less than needed
      lpx = i1.getLine(gi);
      lpy = i1.getLine(gi+1);
      // debug
      for (n = 0; n < 12; n++)
      {
        line1r[n] = lpx[n].r;
        line2r[n] = lpy[n].r;
      }
      // debug end
      gii = gi % mask;
      Rxi = Rx[gii]; Gxi = Gx[gii]; Bxi = Bx[gii];
      Ryi = Ry[gii]; Gyi = Gy[gii]; Byi = By[gii];
      for (x = 0; x < int(i1.width()); x++)
      { // save values
        *Rxi = -lpx->r; *Gxi = -lpx->g;  *Bxi = -lpx->b;
        *Ryi = -lpx->r; *Gyi = -lpx->g;  *Byi = -lpx->b;
        lpx++; // advance to next pixel on line
        *Rxi += lpx->r; *Gxi += lpx->g;  *Bxi += lpx->b;
        *Ryi += lpy->r; *Gyi += lpy->g;  *Byi += lpy->b;
        lpy++; // advance to next pixel on following line
        Rxi++; Gxi++; Bxi++; Ryi++; Gyi++; Byi++;
      }
      if (gi < (mask - 2))
        gi++;
      else
        break;
    }
    //
    for (y = mask/2; y < int(i1.height()) - mask/2 - 1; y++)
    { // for all lines - except near edge
      // expand gradient list with one line
      gi++;
      // get lines
      lpx = i1.getLine(gi);
      lpy = i1.getLine(gi+1);
      // debug
      for (n = 0; n < 12; n++)
      {
        line1r[n] = lpx[n].r;
        line2r[n] = lpy[n].r;
      }
      // debug end
      gii = gi % mask;
      Rxi = Rx[gii]; Gxi = Gx[gii]; Bxi = Bx[gii];
      Ryi = Ry[gii]; Gyi = Gy[gii]; Byi = By[gii];
      for (x = 0; x < int(i1.width()); x++)
      { // save gradient values in x and y
        *Rxi = -lpx->r; *Gxi = -lpx->g;  *Bxi = -lpx->b;
        *Ryi = -lpx->r; *Gyi = -lpx->g;  *Byi = -lpx->b;
        lpx++; // advance to next pixel on line
        *Rxi += lpx->r; *Gxi += lpx->g;  *Bxi += lpx->b;
        *Ryi += lpy->r; *Gyi += lpy->g;  *Byi += lpy->b;
        lpy++; // advance to next pixel on following line
        Rxi++; Gxi++; Bxi++; Ryi++; Gyi++; Byi++;
      }
      // now summ values over mask for each x,y pixel in line
      for (x = mask/2; x < int(i1.width()) - mask/2 - 1; x++)
      { // for all pixels - except near edge
        // zero result
        m11 = 0.0;
        m12 = 0.0;
        m22 = 0.0;
        for (i = 0; i < mask; i++)
        { // i is row, j is column
          gii = (gi + i + 1) % mask;
          j = x - mask/2; // first column to use
          Rxi = &Rx[gii][j]; Gxi = &Gx[gii][j]; Bxi = &Bx[gii][j];
          Ryi = &Ry[gii][j]; Gyi = &Gy[gii][j]; Byi = &By[gii][j];
          for (j = 0; j< mask; j++)
          {
            d = ss[i] * ss[j]; // gauss weight
            m11 += (sqr(*Rxi) + sqr(*Gxi) + sqr(*Bxi)) * d;
            m12 += (*Rxi * *Ryi + *Gxi * *Gyi + *Bxi * *Byi) * d;
            m22 += (sqr(*Ryi) + sqr(*Gyi) + sqr(*Byi)) * d;
            Rxi++; Gxi++; Bxi++; Ryi++; Gyi++; Byi++;
          }
        }
        // determinant
        d = m11 * m22 - sqr(m12);
        t = m11 + m22;
        h = (d - 0.04*sqr(t));
        hi = maxi(0,roundi(h / 256.0));
        hxi = (h >= hval[x-1]); // x gradient increasing
        hyi = (h >= hval[x]); // y gradient increasing
        // debug
        k = mini(255, maxi(0, hi / 1000));
        iph.set(k,k,0);
        i2.setPix(y,x,iph);
        // and gradient
        if (hxi) k = 200; else k = 100;
        iph.set(k,0,0);
        ihx.setPix(y,x,iph);
        if (hyi) n = 200; else n = 100;
        iph.set(0,0,n);
        ihy.setPix(y,x,iph);
        iph.set(k,0,n);
        ihxy.setPix(y,x,iph);
        // debug end
        if (hi > himax)
        { // print max value
          himax = hi;
          printf("Max = %d\n", himax);
        }
        if (hxinc[x] and hyinc[x] and not (hyi or hxinc[x+1]) and
            (hval[x] > 1000000.0))
        {
          v = roundi(hval[x] / 256.0);
          if (hpCnt < (max_pkts))
          {
            k = hpCnt;
            hpCnt++;
          }
          else
          { // find min
            k = -1;
            vmin = v;
            for (n = 0; n < hpCnt; n++)
              if (hpv[n] < vmin)
              {
                vmin = hpv[n];
                k = n;
              }
          }
          if (k >= 0)
          {
            hpx[k] = x;
            hpy[k] = y-1;
            hpv[k] = v;
          }
        }
        // save newest value
        //hg = hxinc[x] and hyinc[x]; // potential peak point
        hval[x] = h;
        hxinc[x] = hxi;
        hyinc[x] = hyi;
      }
      // debug
      // move rows up one
      for (n = 0; n < mask-1; n++)
        for (k = 0; k < mask; k++)
          hmask[n * mask + k] = hmask[(n+1)*mask + k];
      // fill last row
      for (k = 0; k < mask; k++)
        hmask[(mask-1)*mask + k + mask/2] = hval[k+mask/2];
      // debug end
    }
    for (j = 0; j < hpCnt; j++)
    {
      i1.setPix(hpy[j], hpx[j], ipx);
    }
    printf("Found %d harris points\n", hpCnt);
    snprintf(fn, 100, "%s-harris%d", imgn, mask);
    i2.save(fn, "png", imagePath);
    snprintf(fn, 100, "%s-harris%d-tone", imgn, mask);
    i1.save(fn, "png", imagePath);
    snprintf(fn, 100, "%s-harris%d-xgrad", imgn, mask);
    ihx.save(fn, "png", imagePath);
    snprintf(fn, 100, "%s-harris%d-ygrad", imgn, mask);
    ihy.save(fn, "png", imagePath);
    snprintf(fn, 100, "%s-harris%d-xygrad", imgn, mask);
    ihxy.save(fn, "png", imagePath);
  }
  return result;
}

#endif

/////////////////////////////////////////////////////////////

bool testLinearAlgebra()
{
  UMatrix4 a(2,4);
  UMatrix4 a2(2,4);
  UMatrix4 b(4,4,1.0);
  UMatrix4 c;
  UMatrix4 d;
  double dt = -1;
  int err = 7;
  //
  printf("Linear algebra test\n");
  //
  a.setRow(0, 1.0, 2.0, 3.0, 4.0);
  a.setRow(1, -4.0, -3.0, -2.0, -1.0);
  a2.setRow(0, 10.0, 20.0, 30.0, 40.0);
  a2.setRow(1, -40.0, -30.0, -20.0, -10.0);
  //
  a.print("a (2x4)");
  a2.print("a (2x4)");
  //
  c.add(&a, &a2);
  c.print("c = a + a2");
  c.sub(&a, &a2);
  c.print("c = a - a2");
  //
  c.clear();
  c.print("cleared to zero");
  c = a;
  c.init(4,2,7.0);
  c.print("cleared to 4x2, I * 7.0");
  //a.add(0.1);
  //a.print("a (2x4) + 0.1");
  //a.mult(0.1);
  //a.print("a (2x4) * 0.1");

  c = a * a.transposed();
  c.print("c = a * a'");
  c = a.transposed() * a;
  c.print("c = a' * a");
  c.transpose(&a);
  c.print("c.transpose(a)");
  c = a.transposed();
  c.print("c = a'ed");
  d = a + c; // error not equal sized
  d.print("d in error");
  c.expand(4, 4, 1.0);
  a.expand(4, 4, 1.0);
  /*
  a.set(2,0,3.1);
  a.set(2,1,3.2);
  a.set(2,2,3.3);
  */
  a.print("a");
  c.print("c");
  d = a + c; // OK
  d.print("d = a + a'");
  d = a * b; // same as a;
  d.print("d (=a * I)");
  d = a * b.inversed(); // same as a;
  d.print("d (=a * I')");
  a.setSize(3,3);
  a.setRow(0, 1.0, 2.0, 3.0);
  a.setRow(1, 4.0, 3.0, 2.0);
  a.setRow(2, 1.0, 0.0, 1.0);
  a.print("a (3x3)");
  dt = a.det(&err);
  printf("determinant of a is %f (err = %d)\n", dt, err);
  c = a.inversed();
  c.print("c = a inversed");
  d = c.inversed();
  d.print("d = c inversed (=a)");
  //
  return true;
}

////////////////////////////////////

bool testMatrix()
{ // test rotation and translation functions
  UPosition pos1(1.0,2.0,3.0);
  URotation rot1(0.1, 0.2, 0.3);
  UMatrix4 R, Ri, mI;
  //
  R = rot1.asMatrix3x3CtoW();
  R.print("Rotation matrix C->W");
  Ri = rot1.asMatrix3x3WtoC();
  Ri.print("Rotation W->C");
  mI = R * Ri;
  mI.print("mI (C->W * W->C = I)");
  //
  return true;
}

////////////////////////////////////

bool testMatrixVec()
{ // test rotation and translation functions
  UMatrix4 mA(4,4,1.0);
  UMatrix4 vX(4, 1);
  UMatrix4 vB(1, 4);
  UMatrix4 vX2(4, 1, 0.0);
  bool result = true;
  //
  // make matrix
  mA.setRow(0,  1.0, -2.0,  4.0,  0.0);
  mA.setRow(1, -1.0,  0.0,  2.0,  2.0);
  mA.setRow(2,  2.0, -1.0,  2.5, -1.0);
  mA.setRow(3, -1.0,  4.0, -3.0,  4.0);
  mA.print("mA");
  //
  vX.set(1.0, 2.0, -1.0, -2.0);
  vX.print("vX");
  //
  // calc result
  vB = mA * vX;
  vB.print("vB = mA * vX");
  result = mA.solve(&vB, &vX2);
  vX2.print("vX2 : mA * vX2 = vB");
  // test
  vB = mA * vX2;
  vB.print("test = vB");
  //
  return result;
}

////////////////////////////////////

bool testMatrixResize()
{ // test rotation and translation functions
  UMatrix4 mA(3,3);
  bool result = true;
  //
  // make matrix
  mA.setRow(0,  1.1, -1.2, 1.3);
  mA.setRow(1, -2.1, -2.2, 2.3);
  mA.setRow(2, -3.1, -3.2, 3.3);
  mA.print("mA 3x3");
  mA.expand(4, 4, 4.0);
  mA.print("mA");
  mA.expand(1, 4, 14.0);
  mA.print("mA");
  //
  return result;
}
////////////////////////////////////

bool testMatrixVecBig()
{ // test rotation and translation functions
  double buf[6*5];
  UMatrixBig mA(6,5,buf, 6*5);
  UMatrix4 vX(5, 1);
  UMatrix4 vB(15, 1);
  UMatrix4 vX2(1, 16, 0.0);
  bool result;
  const int S_SIZE = 200;
  char s[S_SIZE];
  FILE * f = NULL;
  //
  snprintf(s, S_SIZE, "%s/matrix.m", dataPath);
  f = fopen(s, "w");
  // make matrix
  mA.setSize(5,5);
  mA.setRow(0,  1.0, -2.0,  4.0,  0.0);
  mA.setRC(0, 4, -0.1);
  mA.setRow(1, -1.0,  0.0,  2.0,  2.0);
  mA.setRC(1, 4,  0.2);
  mA.setRow(2,  2.0, -1.0,  2.5, -1.0);
  mA.setRC(2, 4,  1.1);
  mA.setRow(3, -1.0,  4.0, -3.0,  4.0);
  mA.setRC(3, 4, -8.0);
  mA.setRow(4, -1.0,  4.0, -3.0,  4.0);
  mA.setRC(4, 4, 80.0);
  mA.print("mA");
  mA.save(f, "mA5");
  //
  vX.set(1.0, 2.0, -1.0, -2.0);
  vX.setAt(4,  0.1);
  vX.print("vX");
  vX.save(f, "xX5");
  //
  // calc result
  vB.mult(&mA, &vX);
  vB.print("vB = mA * vX");
  vB.save(f, "vB5");
  vX2.setSize(5, 1);
  result = mA.solve(&vB, &vX2);
  vX2.print("X(mA=5x5)");
  if (not mA.expand(6,5))
    printf("Failed to expand\n");
  mA.setRow(5, -1.1,  3.9, -3.0,  4.0);
  mA.setRC(5, 4, 80.5);
  mA.save(f, "mA6");
  mA.print("mA6");
  vB.expand(6,1);
  vB.set(5, 1.0);
  vB.save(f, "vB6");
  vB.print("mB6");


  result = mA.solve(&vB, &vX2);
  vX2.print("vX2(mA=6x5)");
  vX2.save(f, "vX6");
  // test
  vB.mult(&mA, &vX2);
  vB.print("test = vB");

  //
  if (f != NULL)
    fclose(f);
  //
  return result;
}

////////////////////////////////////


///////////////////////////////////////////////////////
#ifndef NO_OPENCV_HIGHGUI

bool testImageHandling()
{
  UImage640 img;
  UImage320 img2;
  UImage320 imgSplit;
  bool result;
  const int FN_SIZE = 200;
  char fn[FN_SIZE];
  UPixel pix(0,0,0);
  // points
  CvPoint pts[10];
  // pointes to start of each contur
  CvPoint * ppts[2] = {pts, &pts[4]};
  // size of each contur
  int cnts[2] = {4,6};
  int n;
  //
  CvFont font;
  //
  printf("Test image handling\n");
  //
  // result = img.load("mmr2/log01040908_153837.328_108", "bmp", imagePath);
  snprintf(fn, FN_SIZE, "%s/mmr2/log01040908_153837.328_108.bmp", imagePath);
  result = img.loadBMP(fn);
  if (not result)
    printf("Load failed");
  //
  img.imageNumber = 12;
  img.imgTime.Now();
  img.pos.set(1.0, 2.0, 3.0);
  img.rot.set(0.01, 0.02, 0.03);
  //
  if (result)
  {
    result = img.save("mmr2/imh", "png", imagePath);
    if (not result)
      printf("save image1 failed\n");
    snprintf(fn, FN_SIZE, "%s/mmr2.bmp", imagePath);
    img.saveBMP(fn);
  }
  //
  if (result)
  {
    printf("Toning black 50%%\n");
    img.tone(&pix, 50);
    printf("Diagonal purpel\n");
    cvLine( img.cvArr(), cvPoint(0,0), cvPoint(640,480), CV_RGB(255,0,255), 2, 4);
    printf("Rectangle blue\n");
    cvRectangle(img.cvArr(), cvPoint(320,240), cvPoint(160,120), CV_RGB(0,0,255), 2);
    printf("Circle cyan\n");
    cvCircle(img.cvArr(), cvPoint(320,240), 160, CV_RGB(0,255,255), 2);
    printf("Circle yellow\n");
    cvCircle(img.cvArr(), cvPoint(320,240), 20, CV_RGB(255,255,0), 4);
    printf("Ellipse yellow\n");
    cvEllipse(img.cvArr(), cvPoint(320,240), cvSize(160,20),
                    -20.0, -100.0, 100.0, CV_RGB(255,255,0), 4);
    printf("Fille polygon red\n");
    n = 5;
    pts[0].x = 20; pts[0].y = 20;
    pts[1].x = 50; pts[1].y = 20;
    pts[2].x = 50; pts[2].y = 70;
    pts[3].x = 100; pts[3].y = 80;
    pts[4].x = 10; pts[4].y = 200;
    pts[5].x = 20; pts[5].y = 230;
    pts[6].x = 140; pts[6].y = 460;
    pts[7].x = 350; pts[7].y = 400;
    pts[8].x = 370; pts[8].y = 380;
    pts[9].x = 180; pts[9].y = 300;
    cvFillConvexPoly( img.cvArr(), pts, n, CV_RGB(255,0,0), 8);
    printf("Filld polygon pink\n");
    cvFillPoly( img.cvArr(), ppts, cnts, 2, CV_RGB(255,100,100), 8);
    printf("Filld polyline white\n");
    cvPolyLine( img.cvArr(), ppts, cnts, 2, true, CV_RGB(255,255,250), 8, 2);
    // text
    cvInitFont( &font, CV_FONT_HERSHEY_TRIPLEX,
                    1.0, 1.0, // scale
                    0.0,    // tilt
                    1,       // thick
                    8 );
    printf("Text in pail yellow\n");
    cvPutText(img.cvArr(), "This is a text - Line 2",
              cvPoint(200, 50), &font, CV_RGB(255, 255, 100));
    //
    if (true)
    { // copy color to max res
      result = img2.copyToMaxRes(&img);
      if (not result)
        printf("Could not copy to half size\n");
      if (result)
        result = imgSplit.setSize(240, 320, 1, 8);
      if (result)
      {
        result = img2.save("mmr2/imh_half", NULL, imagePath);
        // YUV
        img2.toYUV();
        img2.save("mmr2/imh_YUV", NULL, imagePath);
        // into channels
        cvSplit(img2.cvArr(), imgSplit.cvArr(), NULL, NULL, NULL);
        imgSplit.save("mmr2/imh_YUV_Y", NULL, imagePath);
        cvSplit(img2.cvArr(), NULL, imgSplit.cvArr(), NULL, NULL);
        imgSplit.save("mmr2/imh_YUV_U", NULL, imagePath);
        cvSplit(img2.cvArr(), NULL, NULL, imgSplit.cvArr(), NULL);
        imgSplit.save("mmr2/imh_YUV_V", NULL, imagePath);
      }
    }
    if (true)
    { // copy to BW
      result = img2.setSize(240, 320, 1, 8);
      if (not result)
        printf("Gould not set size of img2\n");
      if (result)
      {
        cvCvtColor(img.cvArr(), img2.cvArr(), CV_BGR2GRAY);
        result = img2.save("mmr2/imh_BGR2GRAY", NULL, imagePath);
      }
    }
    if (true)
    { // copy to other formats
      result = img2.setSize(240, 320, 3, 8);
      if (result)
        result = imgSplit.setSize(240, 320, 1, 8);
      if (not result)
        printf("Gould not set size of img2\n");
      if (result)
      { // BGR
        cvCvtColor(img.cvArr(), img2.cvArr(), CV_BGR2RGB);
        result = img2.save("mmr2/imh_CV_BGR2RGB", NULL, imagePath);
        // XYZ
        cvCvtColor(img.cvArr(), img2.cvArr(), CV_BGR2XYZ);
        result = img2.save("mmr2/imh_CV_BGR2XYZ", NULL, imagePath);
        // split into channels
        cvSplit(img2.cvArr(), imgSplit.cvArr(), NULL, NULL, NULL);
        imgSplit.save("mmr2/imh_CV_BGR2XYZ_X", NULL, imagePath);
        cvSplit(img2.cvArr(), NULL, imgSplit.cvArr(), NULL, NULL);
        imgSplit.save("mmr2/imh_CV_BGR2XYZ_Y", NULL, imagePath);
        cvSplit(img2.cvArr(), NULL, NULL, imgSplit.cvArr(), NULL);
        imgSplit.save("mmr2/imh_CV_BGR2XYZ_Z", NULL, imagePath);
        // YCrCb
        cvCvtColor(img.cvArr(), img2.cvArr(), CV_BGR2YCrCb);
        result = img2.save("mmr2/imh_CV_BGR2YCrCb", NULL, imagePath);
        // into channels
        cvSplit(img2.cvArr(), imgSplit.cvArr(), NULL, NULL, NULL);
        imgSplit.save("mmr2/imh_CV_BGR2YCrCb_Y", NULL, imagePath);
        cvSplit(img2.cvArr(), NULL, imgSplit.cvArr(), NULL, NULL);
        imgSplit.save("mmr2/imh_CV_BGR2YCrCb_Cr", NULL, imagePath);
        cvSplit(img2.cvArr(), NULL, NULL, imgSplit.cvArr(), NULL);
        imgSplit.save("mmr2/imh_CV_BGR2YCrCb_Cb", NULL, imagePath);
        // HSV
        cvCvtColor(img.cvArr(), img2.cvArr(), CV_BGR2HSV);
        result = img2.save("mmr2/imh_CV_BGR2HSV", NULL, imagePath);
        // into channels
        cvSplit(img2.cvArr(), imgSplit.cvArr(), NULL, NULL, NULL);
        imgSplit.save("mmr2/imh_CV_BGR2HSV_H", NULL, imagePath);
        cvSplit(img2.cvArr(), NULL, imgSplit.cvArr(), NULL, NULL);
        imgSplit.save("mmr2/imh_CV_BGR2HSV_S", NULL, imagePath);
        cvSplit(img2.cvArr(), NULL, NULL, imgSplit.cvArr(), NULL);
        imgSplit.save("mmr2/imh_CV_BGR2HSV_V", NULL, imagePath);
        // YUV

      }
    }
  }
  //
  return result;
}

#endif

/////////////////////////////////////////////////

#ifndef NO_OPENCV_HIGHGUI

bool testImageHandlingRaw()
{
  bool result;
  UImage640 img;
  URawImage raw;
  //const char path[] = "/home/jca/jca/images/mmr2";
  UPixel pix(0,0,0);
  //
  printf("Test image handling RAW\n");
  result = raw.loadImage("mmr2/log01040908_153837.328_108.bmp", imagePath);
  if (not result)
    printf("Did not find image\n");
  if (result)
  { // convert YUV to BGR during copy
    result = raw.moveToRGB(&img, true);
    if (not result)
      printf("Could not copy from YUV420 to RGB\n");
    if (result)
    {
      result = img.save("mmr2/imhr_YUV420toBGR", NULL, imagePath);
      if (not result)
        printf("Could not save img2\n");
    }
  }
  if (result)
  { // convert from YUV to BGR in full image
    result = raw.moveToYUV(&img);
    if (not result)
      printf("Could not copy from YUV420 to RGB\n");
    if (result)
    {
      img.toBGR();
      result = img.save("mmr2/imhr_YUV420toYUVtoBGR", NULL, imagePath);
      if (not result)
        printf("Could not save img2\n");
    }
  }
  return result;
}

//////////////////////////////////////////////////////


UImage320 img;
UImage320 imgL; // for image load in full color
UImage320 imgAg;
UImage320 imgBg;
UImage320 imP1; // pyramid 1
UImage320 imP2; // pyramid 2
UImage640 imT1;
UImage640 imT2;

bool testOpticalFlow()
{
  bool result;
  const int MAX_COUNT = 100;
  CvPoint2D32f pnts1[MAX_COUNT];
  CvPoint2D32f pnts2[MAX_COUNT];
  CvPoint2D32f * pa1;
  CvPoint2D32f * pa2;
  //UImage * sourceImg;
  UImage * imgA; // image in gray with known points
  UImage * imgB; // image in gray, where to find corresponding points
  UImage * imgPA; // pyramin info for A image
  UImage * imgPB; // pyramin info for B image
  UImage * imgt; // temp pointer for swap
  int count;
  int i, k, n;
  /*
  const char in1[] = "mmr2/log01040908_154114.720_400";
  const char in2[] = "mmr2/log01040908_154115.120_401";
  const char in3[] = "mmr2/log01040908_154115.720_402";
  */
  const int INN_CNT = 5;
  const char * inn[INN_CNT];
  // find features to track parameters
  const double quality = 0.01;
  const double min_distance = 25;
  int win_size = 13;
  // flow
  char status[MAX_COUNT];
  int flags = 0;
  //CvArr * cvS1;
  //CvArr * cvS2;
  //CvArr * cvpA;
  //CvArr * cvpB;
  // fundamental matrix est
  UMatrix4 mF(3,3,1.0); // fundamental
  UMatrixBig mP1(MAX_COUNT, 2); // points in img 1
  UMatrixBig mP2(MAX_COUNT, 2); // points in img 2
  UMatrixBig vStatus(MAX_COUNT); // valid points in match
  UMatrix4 v1(1, 3), v2(3, 1);
  double d;
  //const char res[] = "flow246to260";
  const char res[] = "flow300to304";
  const int MAX_S = 100;
  char s[MAX_S]; // string buffer
  CvScalar rgb;
  /* assign filenames
  inn[0] = "mmr2/log01040908_153839.528_112";
  inn[1] = "mmr2/log01040908_153839.928_113";
  inn[2] = "mmr2/log01040908_153840.528_114";
  inn[3] = "mmr2/log01040908_153841.128_115";
  inn[4] = "mmr2/log01040908_153841.527_116";
  */
  inn[0] = "mmr-outdoor/raw20041117_154741.039_0_246";
  inn[1] = "mmr-outdoor/raw20041117_154741.639_0_247";
  inn[2] = "mmr-outdoor/raw20041117_154742.239_0_248";
  inn[3] = "mmr-outdoor/raw20041117_154742.839_0_249";
  inn[4] = "mmr-outdoor/raw20041117_154743.439_0_250";
  //
  inn[0] = "mmr-outdoor/raw20041117_154813.433_0_300";
  inn[1] = "mmr-outdoor/raw20041117_154814.043_0_301";
  inn[2] = "mmr-outdoor/raw20041117_154814.643_0_302";
  inn[3] = "mmr-outdoor/raw20041117_154815.243_0_303";
  inn[4] = "mmr-outdoor/raw20041117_154815.843_0_304";
  //
  printf("Testing mage flow\n");
  // load last image and convert to BW
  result = img.load(inn[INN_CNT-1], "bmp", imagePath);
  if (not result)
    printf("Did dot find image %s/%s.bmp\n", imagePath, inn[INN_CNT-1]);
  if (result)
  { // gray-level 8-bit images
    imgAg.copyMeta(&img, false);
    imgAg.setSize(240, 320, 1, 8);
    imgBg.setSize(240, 320, 1, 8);
  }
  if (result)
    cvCvtColor(img.cvArr(), imgAg.cvArr(), CV_BGR2GRAY);
  if (result)
  { // find features to track
    // size images for eig and temp for 'cvGoodFeaturesToTrack(...)'
    imT1.setSize(240, 320, 1, 32);
    result = imT2.setSize(240, 320, 1, 32);
    if (result)
    {
      count = MAX_COUNT;
      //IplImage* eig = cvCreateImage( cvGetSize(sourceImg->cvArr()), 32, 1 );
      //IplImage* temp = cvCreateImage( cvGetSize(sourceImg->cvArr()), 32, 1 );

      cvGoodFeaturesToTrack( imgAg.cvArr(),
                             imT1.cvArr(), imT2.cvArr(),
                             pnts1, &count,
                             quality, min_distance, 0 );
      cvFindCornerSubPix( imgAg.cvArr(), pnts1, count,
                  cvSize(win_size,win_size), cvSize(-1,-1),
                  cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
      result = count > 0;
      // paint initian points in result image
      for (i = 0; i < count; i++)
        cvCircle( img.cvArr(), cvPointFrom32f(pnts1[i]), 4, CV_RGB(255, 0, 0), 1, 8,0);

    }
    printf("Found %d interesting points\n", count);
  }
  //
  if (result)
  {
    imP1.setSize(240, 320, 3, 8);
    imP2.setSize(240, 320, 3, 8);
    /*
    cvS1  = imgCg.cvArr(); // source image
    cvS2  = imgBg.cvArr(); // end image
    cvpA = imP1.cvArr(); // pyramid for A
    cvpB = imP2.cvArr(); // pyramin for B
    */
    flags = 0; // no pyramids are valid
    pa1 = pnts1;
    pa2 = pnts2;
    imgA = &imgAg;
    imgB = &imgBg;
    imgPA = &imP1;
    imgPB = &imP2;
    for (n = INN_CNT - 2; n >= 0; n--)
    {
      printf("compare image %d<->%d for flow using %d points\n", n+1, n, count);
      // get new image
      result = imgL.load(inn[n], "bmp", imagePath);
      if (result)
      { // save also name and position
        imgB->copyMeta(&imgL, false);
        // convert to BW
        cvCvtColor(imgL.cvArr(), imgB->cvArr(), CV_BGR2GRAY);
      }
      // calculate flow
      cvCalcOpticalFlowPyrLK( imgA->cvArr(), imgB->cvArr(),
                    imgPA->cvArr(), imgPB->cvArr(),
                    pa1, pa2, count,
                    cvSize(win_size,win_size), 3, status, 0,
                    cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags );
      k = 0;
      rgb = CV_RGB(0, n * 255/INN_CNT, (INN_CNT - n) * 255/INN_CNT);
      for (i = 0; i < count; i++)
      { // paint points in image
        if (status[i] > 0)
        {
          //cvCircle( img.cvArr(), cvPointFrom32f(pa2[i]), 3, CV_RGB(0, 255, 0), 1, 8,0);
          cvLine( img.cvArr(), cvPointFrom32f(pa1[i]), cvPointFrom32f(pa2[i]), rgb, 1, 8, 0);
          // put into related points matrix
          mP1.setRow(k, pa1[i].x, pa1[i].y);
          mP2.setRow(k, pa2[i].x, pa2[i].y);
          // move surveived points back to pa1
          pa1[k++] = pa2[i];
        }
      }
      count = k;
      // resize related points list
      mP1.expand(k, 2);
      mP2.expand(k, 2);
      vStatus.expand(1, k);
      //
      printf(" ... found %d flow end points\n", k);
      // move to image, pyramd to A image set
      imgt = imgA;
      imgA = imgB;
      imgB = imgt;
      // reuse pyramid for last image
      imgt = imgPA;
      imgPA = imgPB;
      imgPB = imgt;
      flags |= CV_LKFLOW_PYR_A_READY; // pyramid A is (now) OK
      //
      // Find fundamental matrix between image  S1 and S2
      result = cvFindFundamentalMat(mP1.cvMat(),mP2.cvMat(),
                         mF.cvMat(),
                         CV_FM_RANSAC, 1.0, 0.99,
                         vStatus.cvMat());
      if (result)
      {
        printf("Fundamental matrix was found:\n");
        mF.print("F= ");
      }
      k = 0;
      for (i = 0; i < count; i++)
      {
        if (vStatus.get(i) > 0.5)
        { // point is valid
          cvCircle( img.cvArr(), cvPointFrom32f(pa1[i]), 2, rgb, 1, 4,0);
          k++;
          v1.set(mP1.get(i,0), mP1.get(i, 1), 1.0);
          v2.set(mP2.get(i,0), mP2.get(i, 1), 1.0);
          d = (v1 * mF * v2).get(0);
          printf("tried point %d and x1 * F * x2' = %f\n", i, d);
        }
      }
      printf("Used %d points for fundamental matrix calculation\n", k);
      // save result image
      snprintf(s, MAX_S, "%s_%d", res, n);
      result = img.save(s, "png", imagePath);
      if (result)
        printf("Saved flow image to %s/%s.png\n", imagePath,s);
    }
  }
  printf("finished\n");
  //
  return result;
}

/////////////////////////////////////////////////////////////

bool testOpticalFlowHarris()
{
  bool result;
  const int MAX_COUNT = 100;
  CvPoint2D32f pnts1[MAX_COUNT];
  CvPoint2D32f pnts2[MAX_COUNT];
  CvPoint2D32f * pa1;
  CvPoint2D32f * pa2;
  //UImage * sourceImg;
  UImage * imgA; // image in gray with known points
  UImage * imgB; // image in gray, where to find corresponding points
  UImage * imgPA; // pyramin info for A image
  UImage * imgPB; // pyramin info for B image
  UImage * imgt; // temp pointer for swap
  int count;
  int i, k, n;
  /*
  const char in1[] = "mmr2/log01040908_154114.720_400";
  const char in2[] = "mmr2/log01040908_154115.120_401";
  const char in3[] = "mmr2/log01040908_154115.720_402";
  */
  const int INN_CNT = 5;
  const char * inn[INN_CNT];
  // find features to track parameters
  const double quality = 0.01;
  const double min_distance = 25;
  int win_size = 13;
  // flow
  char status[MAX_COUNT];
  int flags = 0;
  //CvArr * cvS1;
  //CvArr * cvS2;
  //CvArr * cvpA;
  //CvArr * cvpB;
  // fundamental matrix est
  UMatrix4 mF(3,3,1.0); // fundamental
  UMatrixBig mP1(MAX_COUNT, 2); // points in img 1
  UMatrixBig mP2(MAX_COUNT, 2); // points in img 2
  UMatrixBig vStatus(MAX_COUNT); // valid points in match
  UMatrix4 v1(1, 3), v2(3, 1);
  double d;
  const char res[] = "flow112to116";
  const int MAX_S = 100;
  char s[MAX_S]; // string buffer
  CvScalar rgb;
  // assign filenames
  inn[0] = "mmr2/log01040908_153839.528_112";
  inn[1] = "mmr2/log01040908_153839.928_113";
  inn[2] = "mmr2/log01040908_153840.528_114";
  inn[3] = "mmr2/log01040908_153841.128_115";
  inn[4] = "mmr2/log01040908_153841.527_116";
  //
  printf("Testing mage flow\n");
  // load last image and convert to BW
  result = img.load(inn[INN_CNT-1], "bmp", imagePath);
  if (not result)
    printf("Did dot find image %s/%s.bmp\n", imagePath, inn[INN_CNT-1]);
  if (result)
  { // gray-level 8-bit images
    imgAg.copyMeta(&img, false);
    imgAg.setSize(240, 320, 1, 8);
    imgBg.setSize(240, 320, 1, 8);
  }
  if (result)
    cvCvtColor(img.cvArr(), imgAg.cvArr(), CV_BGR2GRAY);
  if (result)
  { // find features to track
    // size images for eig and temp for 'cvGoodFeaturesToTrack(...)'
    imT1.setSize(240, 320, 1, 32);
    result = imT2.setSize(240, 320, 1, 32);
    if (result)
    {
      count = MAX_COUNT;
      //IplImage* eig = cvCreateImage( cvGetSize(sourceImg->cvArr()), 32, 1 );
      //IplImage* temp = cvCreateImage( cvGetSize(sourceImg->cvArr()), 32, 1 );

      cvGoodFeaturesToTrack( imgAg.cvArr(),
                             imT1.cvArr(), imT2.cvArr(),
                             pnts1, &count,
                             quality, min_distance, 0 );
      cvFindCornerSubPix( imgAg.cvArr(), pnts1, count,
                  cvSize(win_size,win_size), cvSize(-1,-1),
                  cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
      result = count > 0;
      // paint initian points in result image
      for (i = 0; i < count; i++)
        cvCircle( img.cvArr(), cvPointFrom32f(pnts1[i]), 4, CV_RGB(255, 0, 0), 1, 8,0);

    }
    printf("Found %d interesting points\n", count);
  }
  //
  if (result)
  {
    imP1.setSize(240, 320, 3, 8);
    imP2.setSize(240, 320, 3, 8);
    /*
    cvS1  = imgCg.cvArr(); // source image
    cvS2  = imgBg.cvArr(); // end image
    cvpA = imP1.cvArr(); // pyramid for A
    cvpB = imP2.cvArr(); // pyramin for B
    */
    flags = 0; // no pyramids are valid
    pa1 = pnts1;
    pa2 = pnts2;
    imgA = &imgAg;
    imgB = &imgBg;
    imgPA = &imP1;
    imgPB = &imP2;
    for (n = INN_CNT - 2; n >= 0; n--)
    {
      printf("compare image %d<->%d for flow using %d points\n", n+1, n, count);
      // get new image
      result = imgL.load(inn[n], "bmp", imagePath);
      if (result)
      { // save also name and position
        imgB->copyMeta(&imgL, false);
        // convert to BW
        cvCvtColor(imgL.cvArr(), imgB->cvArr(), CV_BGR2GRAY);
      }
      // calculate flow
      cvCalcOpticalFlowPyrLK( imgA->cvArr(), imgB->cvArr(),
                    imgPA->cvArr(), imgPB->cvArr(),
                    pa1, pa2, count,
                    cvSize(win_size,win_size), 3, status, 0,
                    cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags );
      k = 0;
      rgb = CV_RGB(0, n * 255/INN_CNT, (INN_CNT - n) * 255/INN_CNT);
      for (i = 0; i < count; i++)
      { // paint points in image
        if (status[i] > 0)
        {
          //cvCircle( img.cvArr(), cvPointFrom32f(pa2[i]), 3, CV_RGB(0, 255, 0), 1, 8,0);
          cvLine( img.cvArr(), cvPointFrom32f(pa1[i]), cvPointFrom32f(pa2[i]), rgb, 1, 8, 0);
          // put into related points matrix
          mP1.setRow(k, pa1[i].x, pa1[i].y);
          mP2.setRow(k, pa2[i].x, pa2[i].y);
          // move surveived points back to pa1
          pa1[k++] = pa2[i];
        }
      }
      count = k;
      // resize related points list
      mP1.expand(k, 2);
      mP2.expand(k, 2);
      vStatus.expand(k, 1);
      //
      printf(" ... found %d flow end points\n", k);
      // move to image, pyramd to A image set
      imgt = imgA;
      imgA = imgB;
      imgB = imgt;
      // reuse pyramid for last image
      imgt = imgPA;
      imgPA = imgPB;
      imgPB = imgt;
      flags |= CV_LKFLOW_PYR_A_READY; // pyramid A is (now) OK
      //
      // Find fundamental matrix between image  S1 and S2
      result = cvFindFundamentalMat(mP1.cvMat(),mP2.cvMat(),
                         mF.cvMat(),
                         CV_FM_RANSAC, 1.0, 0.99,
                         vStatus.cvMat());
      if (result)
      {
        printf("Fundamental matrix was found:\n");
        mF.print("F= ");
      }
      k = 0;
      for (i = 0; i < count; i++)
      {
        if (vStatus.get(i) > 0.5)
        { // point is valid
          cvCircle( img.cvArr(), cvPointFrom32f(pa1[i]), 2, rgb, 1, 4,0);
          k++;
          v1.set(mP1.get(i,0), mP1.get(i, 1), 1.0);
          v2.set(mP2.get(i,0), mP2.get(i, 1), 1.0);
          d = (v1 * mF * v2).get(0);
          printf("tried point %d and x1' * F * x2 = %f\n", i, d);
        }
      }
      printf("Used %d points for fundamental matrix calculation\n", k);
      // save result image
      snprintf(s, MAX_S, "%s_%d", res, n);
      result = img.save(s, "png", imagePath);
      if (result)
        printf("Saved flow image to %s/%s.png\n", imagePath,s);
    }
  }
  printf("finished\n");
  //
  return result;
}

//////////////////////////////////////////////////////////////////

bool harris3()
{
  bool result;
  UImage640 is;
  UImage320 i1;
  UImage320 i2;
  const int mask = 5;
  //const char imgn[] = "img10014f";
  //const char imgn[] = "vision/snapshot-20040818-091843";
  // const char imgn[] = "vision/snapshot-20040818-124936";
  const char imgn[] = "mmr2/log01040908_153841.527_116";
  char fn[100];
  double ss[mask]; // = {0.2, 0.5, 1.0, 0.5, 0.2};
//  UMatrix4 m(2,2);
  double m11, m12, m22;
  double d, h, t;
  int i, j, x, y, v, vmin;
  const int line_width = 320;
  double hval[line_width];
  bool hxinc[line_width];
  bool hyinc[line_width];
  bool hxi, hyi;
  int Rx[mask][line_width]; int Gx[mask][line_width]; int Bx[mask][line_width];
  int Ry[mask][line_width]; int Gy[mask][line_width]; int By[mask][line_width];
  int gi, gii; // gradient index
  int * Rxi;
  int * Gxi;
  int * Bxi;
  int * Ryi;
  int * Gyi;
  int * Byi;
  // resulting points
  const int max_pkts = 100;
  int hpx[max_pkts+1];
  int hpy[max_pkts+1];
  int hpv[max_pkts+1];
  int hpCnt = 0;
  UPixel ipx;
  UPixel * lpx;
  UPixel * lpy;
  int hi, himax = 0;
  //
  // debug
  int n, k;
  int line1r[12];
  int line2r[12];
  double hmask[mask * mask];
  UPixel iph;
  UImage320 ihx;
  UImage320 ihy;
  UImage320 ihxy;
  // debug end
  //
  t = 0.0;
  for (i = 0; i < mask; i++)
  { // make gaussian values
    h = double(mask-1) / 2.0;
    d = (double(i) - h)*2.0/h; // (x value)
    ss[i] = 1.0/sqrt(2.0 * PI) * exp(-sqr(d)/2.0);
    t += ss[i];
  }

  for (i = 0; i < line_width; i++)
  { // zero harris value and gradient sign
    hval[i] = 0.0;
    hxinc[i] = false; // decreasing x gradient
    hyinc[i] = false; // decreasing y gradient
  }

  printf("Testing generation of Harris-points\n");
  result = is.load(imgn, "bmp", imagePath);
  //i1.copyScaleDown(&is, 2);
  i1.copy(&is);
  if (not result)
    printf("File not found: '%s/%s'\n",imagePath, fn);
  if (result)
  {
    i2.copy(&i1);
    ipx.set(0,0,0);
    i2.tone(&ipx, 80);
    ipx.set(255,255,255);
    // debug
    ihx.copy(&i2);
    ihy.copy(&i2);
    ihxy.copy(&i2);
    // debug end
    //
    gi = 0;
    Rxi = Rx[0]; Gxi = Gx[0]; Bxi = Bx[0]; Ryi = Ry[0]; Gyi = Gy[0]; Byi = By[0];
    while (true)
    { // fill gradient values up to l less than needed
      lpx = i1.getLine(gi);
      lpy = i1.getLine(gi+1);
      // debug
      for (n = 0; n < 12; n++)
      {
        line1r[n] = lpx[n].r;
        line2r[n] = lpy[n].r;
      }
      // debug end
      gii = gi % mask;
      Rxi = Rx[gii]; Gxi = Gx[gii]; Bxi = Bx[gii];
      Ryi = Ry[gii]; Gyi = Gy[gii]; Byi = By[gii];
      for (x = 0; x < int(i1.width()); x++)
      { // save values
        *Rxi = -lpx->r; *Gxi = -lpx->g;  *Bxi = -lpx->b;
        *Ryi = -lpx->r; *Gyi = -lpx->g;  *Byi = -lpx->b;
        lpx++; // advance to next pixel on line
        *Rxi += lpx->r; *Gxi += lpx->g;  *Bxi += lpx->b;
        *Ryi += lpy->r; *Gyi += lpy->g;  *Byi += lpy->b;
        lpy++; // advance to next pixel on following line
        Rxi++; Gxi++; Bxi++; Ryi++; Gyi++; Byi++;
      }
      if (gi < (mask - 2))
        gi++;
      else
        break;
    }
    //
    for (y = mask/2; y < int(i1.height()) - mask/2 - 1; y++)
    { // for all lines - except near edge
      // expand gradient list with one line
      gi++;
      // get lines
      lpx = i1.getLine(gi);
      lpy = i1.getLine(gi+1);
      // debug
      for (n = 0; n < 12; n++)
      {
        line1r[n] = lpx[n].r;
        line2r[n] = lpy[n].r;
      }
      // debug end
      gii = gi % mask;
      Rxi = Rx[gii]; Gxi = Gx[gii]; Bxi = Bx[gii];
      Ryi = Ry[gii]; Gyi = Gy[gii]; Byi = By[gii];
      for (x = 0; x < int(i1.width()); x++)
      { // save gradient values in x and y
        *Rxi = -lpx->r; *Gxi = -lpx->g;  *Bxi = -lpx->b;
        *Ryi = -lpx->r; *Gyi = -lpx->g;  *Byi = -lpx->b;
        lpx++; // advance to next pixel on line
        *Rxi += lpx->r; *Gxi += lpx->g;  *Bxi += lpx->b;
        *Ryi += lpy->r; *Gyi += lpy->g;  *Byi += lpy->b;
        lpy++; // advance to next pixel on following line
        Rxi++; Gxi++; Bxi++; Ryi++; Gyi++; Byi++;
      }
      // now summ values over mask for each x,y pixel in line
      for (x = mask/2; x < int(i1.width()) - mask/2 - 1; x++)
      { // for all pixels - except near edge
        // zero result
        m11 = 0.0;
        m12 = 0.0;
        m22 = 0.0;
        for (i = 0; i < mask; i++)
        { // i is row, j is column
          gii = (gi + i + 1) % mask;
          j = x - mask/2; // first column to use
          Rxi = &Rx[gii][j]; Gxi = &Gx[gii][j]; Bxi = &Bx[gii][j];
          Ryi = &Ry[gii][j]; Gyi = &Gy[gii][j]; Byi = &By[gii][j];
          for (j = 0; j< mask; j++)
          {
            d = ss[i] * ss[j]; // gauss weight
            m11 += (sqr(*Rxi) + sqr(*Gxi) + sqr(*Bxi)) * d;
            m12 += (*Rxi * *Ryi + *Gxi * *Gyi + *Bxi * *Byi) * d;
            m22 += (sqr(*Ryi) + sqr(*Gyi) + sqr(*Byi)) * d;
            Rxi++; Gxi++; Bxi++; Ryi++; Gyi++; Byi++;
          }
        }
        // determinant
        d = m11 * m22 - sqr(m12);
        t = m11 + m22;
        h = (d - 0.04*sqr(t));
        hi = maxi(0,roundi(h / 256.0));
        hxi = (h >= hval[x-1]); // x gradient increasing
        hyi = (h >= hval[x]); // y gradient increasing
        // debug
        k = mini(255, maxi(0, hi / 1000));
        iph.set(k,k,0);
        i2.setPix(y,x,iph);
        // and gradient
        if (hxi) k = 200; else k = 100;
        iph.set(k,0,0);
        ihx.setPix(y,x,iph);
        if (hyi) n = 200; else n = 100;
        iph.set(0,0,n);
        ihy.setPix(y,x,iph);
        iph.set(k,0,n);
        ihxy.setPix(y,x,iph);
        // debug end
        if (hi > himax)
        { // print max value
          himax = hi;
          printf("Max = %d\n", himax);
        }
        if (hxinc[x] and hyinc[x] and not (hyi or hxinc[x+1]) and
            (hval[x] > 1000000.0))
        {
          v = roundi(hval[x] / 256.0);
          if (hpCnt < (max_pkts))
          {
            k = hpCnt;
            hpCnt++;
          }
          else
          { // find min
            k = -1;
            vmin = v;
            for (n = 0; n < hpCnt; n++)
              if (hpv[n] < vmin)
              {
                vmin = hpv[n];
                k = n;
              }
          }
          if (k >= 0)
          {
            hpx[k] = x;
            hpy[k] = y-1;
            hpv[k] = v;
          }
        }
        // save newest value
        //hg = hxinc[x] and hyinc[x]; // potential peak point
        hval[x] = h;
        hxinc[x] = hxi;
        hyinc[x] = hyi;
      }
      // debug
      // move rows up one
      for (n = 0; n < mask-1; n++)
        for (k = 0; k < mask; k++)
          hmask[n * mask + k] = hmask[(n+1)*mask + k];
      // fill last row
      for (k = 0; k < mask; k++)
        hmask[(mask-1)*mask + k + mask/2] = hval[k+mask/2];
      // debug end
    }
    for (j = 0; j < hpCnt; j++)
    {
      //cvCircle(i1.cvArr(), cvPoint(pa1[i]), 2, rgb, 1, 4,0);
      cvCircle(i1.cvArr(),  cvPoint(hpx[j], hpy[j]), 3, CV_RGB(255,0,0), 1, 4, 0);
    }
    printf("Found %d harris points\n", hpCnt);
    snprintf(fn, 100, "%s-3harris%d", imgn, mask);
    i2.save(fn, "png", imagePath);
    snprintf(fn, 100, "%s-3harris%d-tone", imgn, mask);
    i1.save(fn, "png", imagePath);
    /*
    snprintf(fn, 100, "%s-harris%d-xgrad", imgn, mask);
    ihx.save(fn, "png", imagePath);
    snprintf(fn, 100, "%s-harris%d-ygrad", imgn, mask);
    ihy.save(fn, "png", imagePath);
    snprintf(fn, 100, "%s-harris%d-xygrad", imgn, mask);
    ihxy.save(fn, "png", imagePath);
    */
  }
  return result;
}

///////////////////////////////////////////////////////

bool testCameraSettings()
{
  bool result = true;
  URawImage raw; // distorted image
  //URawImage raw2; // other image
//  UImage640 imT1;     // used to save result
//  UImage640 imT2;     // used to save result
  UCamRad cam;
  const int SL_MAX = 200;
  char s[SL_MAX];
  char * cn;
  const char * ws = "none";
  int wbRed;
  int wbBlue;
  int wbMode;
  int gn, sh;
  //bool isOK = false;
  //const char path[] = "/home/jca/jca/images/";
  int i, n;
  //
  cam.setDeviceNumber(0);
  cn = cam.getCameraName();
  result = (cn != NULL);
  if (not result)
    printf("No camera found\n!");
  if (result)
  {
    printf("Camera 0 is %s\n", cn);
    result = cam.openAndSetDevice(320, 240, 10);
    if (not result)
      printf("Could not set device\n");
  }
  if (result)
  { // camera is open,
    cam.setImageNumber(100);
    // set led to blink to framerate
    cam.setLedFrameRate();
    // automatic gain
    cam.setGain(-1);
    // set automatic shutter
    cam.setShutter(-1);
    // prefer uncompressed images
    cam.setCompPref(0);
    // set contour value
    cam.setContour(0x3000);
    // gamma off
    cam.setGamma(0x8000);
    // now take some images
  }
  // compression
  if (false and result)
  {
    for (n = 0; n < 4; n++)
    { // test compression
      cam.setCompPref(n);
      for (i = 0; i < 8; i++)
      { // wait for next image at 5 frames/sec
        //Wait(0.2);
        result = cam.getImageSnapshot(&raw, false);
        if (not result)
        {
          printf("Failed to get image\n");
          break;
        }
        if (result)
        {
          result = raw.moveToRGB(&imT1, true);
          if (not result)
            printf("Could not copy from YUV442 to RGB\n");
          if (result)
          { // save image copy
            snprintf(s, SL_MAX, "imCamSet%luComp%d", imT1.imageNumber, cam.getCompPref(true));
            imT1.save(s, NULL, imagePath);
            printf("Saved image %s.png\n", s);
          }
        }
      }
    }
    // reset
    cam.setCompPref(0);
  }
  // contour
  if (false and result)
  {
    for (n = 0; n < 6; n++)
    { // test contour
      cam.setContour(0x800 * n);
      for (i = 0; i < 5; i++)
      { // get image
        result = cam.getImageSnapshot(&raw, false);
        if (not result)
        {
          printf("Failed to get image\n");
          break;
        }
        if (result)
        {
          result = raw.moveToRGB(&imT1, true);
          if (not result)
            printf("Could not copy from YUV420 to RGB\n");
          if (result)
          { // save image copy
            snprintf(s, SL_MAX, "cont/imCamSet%luCont%x", imT1.imageNumber, cam.getContour(true));
            imT1.save(s, NULL, imagePath);
            printf("Saved image %s.png\n", s);
          }
        }
      }
    }
    cam.setContour(0x1000);
  }
  // test white ballance settings
  if (false and result)
  { // set initial white ballance
    wbMode = PWC_WB_AUTO;
    wbRed = 0x4000; // not used in auto
    wbBlue = 0x4000;     // not used in auto
    for (n = 0; n < 6; n++)
    { // set new white ballance
      cam.setWhiteBalance(wbMode, wbRed, wbBlue);
      // try some white ballance modes
      cam.getWhiteBalance(true, &wbRed, &wbBlue, &wbMode);
      switch (wbMode)
      {
        case PWC_WB_AUTO:
          printf("White balance is set to auto (=%d) red:%d, blue:%d\n", wbMode, wbRed, wbBlue);
          ws = "WBauto";
          wbMode = PWC_WB_MANUAL;
          wbRed = 0x4000;
          wbBlue = 0x4000;
          break;
        case PWC_WB_MANUAL:
          printf("White balance is set to manual (=%d) red:%d, blue:%d\n", wbMode, wbRed, wbBlue);
          ws = "WBmanual";
          wbMode = PWC_WB_INDOOR;
          break;
        case PWC_WB_INDOOR:
          printf("White balance is set to indoor (=%d) red:%d, blue:%d\n", wbMode, wbRed, wbBlue);
          ws = "WBindoor";
          wbMode = PWC_WB_OUTDOOR;
          break;
        case PWC_WB_OUTDOOR:
          printf("White balance is set to outdoor (=%d) red:%d, blue:%d\n", wbMode, wbRed, wbBlue);
          ws = "WBoutdoor";
          wbMode = PWC_WB_FL;
          break;
        case PWC_WB_FL:
          printf("White balance is set to flurocent (=%d) red:%d, blue:%d\n", wbMode, wbRed, wbBlue);
          ws = "WBfl";
          wbMode = PWC_WB_AUTO;
          break;
        default:
          printf("White balance is set to unknown (=%d) red:%d, blue:%d\n", wbMode, wbRed, wbBlue);
          break;
      }
      for (i = 0; i < 10; i++)
      { // wait for next image at 5 frames/sec
        //Wait(0.2);
        result = cam.getImageSnapshot(&raw, false);
        if (not result)
        {
          printf("Failed to get image\n");
          break;
        }
        // save image
        raw.moveToRGB(&imT1, true);
        snprintf(s, SL_MAX, "imCamSet%lu%s", imT1.imageNumber, ws);
        printf("Saved image %s\n", s);
        imT1.save(s, NULL, imagePath);
      }
    }
  }
  // test gain and shutter
  if (false and result)
  { // set white ballance to manual - nutral
    wbMode = PWC_WB_AUTO;
    wbRed = 0x4000; // not used in auto
    wbBlue = 0x4000;     // not used in auto
    cam.setWhiteBalance(wbMode, wbRed, wbBlue);
    //
    gn = 2000;
    sh = 20000;
    for (n = 0; n < 64; n++)
    { // test gain and shutter
      switch(n)
      {
        case 0: // auto gain and shutter
          cam.setGain(-1);
          cam.setShutter(-1);
          ws = "AsAg";
          break;
        case 1: // auto gain and shutter
          cam.setGain(gn);
          cam.setShutter(-1);
          ws = "AsMg";
          break;
        case 2: // auto gain and shutter
          cam.setGain(-1);
          cam.setShutter(sh);
          ws = "MsAg";
          break;
        case 3: // auto gain and shutter
          gn = 0;
          sh = 0x3000;
          cam.setGain(gn);
          cam.setShutter(sh);
          ws = "MsMg";
          break;
        default:
          gn = gn + 0x2000;
          if (gn > 0xFFF0)
          {
            gn = 0x0000;
            sh = sh + 0x1000;
          }
          cam.setGain(gn);
          cam.setShutter(sh);
          ws = "MsMg";
          break;
      }
      for (i = 0; i < 10; i++)
      { // wait for next image at 5 frames/sec
        //Wait(0.2);
        result = cam.getImageSnapshot(&raw, false);
        if (not result)
        {
          printf("Failed to get image\n");
          break;
        }
        // save image
        raw.moveToRGB(&imT1, true);
        snprintf(s, SL_MAX, "imCamSet%lu%s-s%x-g%x",
             imT1.imageNumber, ws,
             cam.getShutter(true),
             cam.getGain(true));
        printf("Saved image %s\n", s);
        imT1.save(s, NULL, imagePath);
      }
    }
  }
  // test gamma correction
  if (result)
  { // set white ballance to manual - nutral
    wbMode = PWC_WB_AUTO;
    wbRed = 0x4000; // not used in auto
    wbBlue = 0x4000;     // not used in auto
    cam.setWhiteBalance(wbMode, wbRed, wbBlue);
    //
    n = 0x0000;
    // test gamma
    for (i = 0; i <= 16; i++)
    { // wait for next image at 5 frames/sec
      cam.setGamma(n);
      // get 3 images to ensure effect is implemented
      result = cam.getImageSnapshot(&raw, false);
      result = cam.getImageSnapshot(&raw, false);
      Wait(0.3);
      result = cam.getImageSnapshot(&raw, false);
      if (not result)
      {
        printf("Failed to get image\n");
        //break;
      }
      // save image
      if (true)
        raw.moveToRGB(&imT1, true);
      else
      {
        raw.moveToYUV(&imT1);
        imT1.toRGB();
      }
      snprintf(s, SL_MAX, "gamma/imGamma_%xh", cam.getGamma(true));
      imT1.save(s, NULL, imagePath);
      raw.saveImage(s, imagePath);
      printf("Saved image %s\n", s);
      n += 0x0FFF;
    }
  }
  // close
  if (result)
    cam.setDeviceClosed();
  return result;
}

////////////////////////////////////////////////////
///////////////////////////////////////////////////

const int RAW_MAX = 12;
URawImage rawHDR[RAW_MAX]; // distorted image

bool testCameraHDR()
{
  bool result = true;
  URawImage * raw;
//  URawImage raw2; // other image
//  UImage640 imT1;     // used to save result
//  UImage640 imT2;     // used to save result
  UCamRad cam;
  const int SL_MAX = 200;
  char s[SL_MAX];
  char * cn;
  int i, n, j;
  const int MAX_RNG = 8;
  URawImage * rawa[MAX_RNG];
  URawImage * rawr;
  URawImage * rawHD;
  int gna[MAX_RNG] = {0x5000, 0x5000, 0x5000, 0x5000, 0x5000, 0x5000, 0x5000, 0x5000};
  int sha[MAX_RNG] = {0x5c00, 0x5e00, 0x6000, 0x7000, 0x8000, 0xa000, 0xc000, 0xfff0};
  int gn, sh;
  int min, max;
  float avg, var;
  float oldAvg;
  unsigned char * ps[MAX_RNG];
  unsigned char y11, y12, y21, y22, u, v;
  unsigned char sy11, sy12, sy21, sy22, su, sv;
//  int iy11, iy12, iy21, iy22;
  unsigned int r, c;
  const int TH = 200; // saturation treshold
//  const int BK = 30; // buttom cut
  const int shutZero = 0x5a00;
  const int shutMax = 0x10000;
  const int shutGain = shutMax - shutZero;
  UPixel px;
  UPixel * ppix1;
  UPixel * ppix2;
  int pxr, pxg, pxb;
  int pbr, pbg, pbb;
  int fac;
  float lmax, lr, lg, lb;
  //
  //
  cam.setDeviceNumber(0);
  cn = cam.getCameraName();
  result = (cn != NULL);
  if (not result)
    printf("No camera found\n!");
  if (result)
  {
    printf("Camera 0 is %s\n", cn);
    result = cam.openAndSetDevice(640, 480, 10);
    if (not result)
      printf("Could not set device\n");
  }
  if (result)
  { // camera is open,
    cam.setImageNumber(1000);
    // set led to blink to framerate
    cam.setLedFrameRate();
    // prefer uncompressed images
    cam.setCompPref(0);
    // set contour value
    cam.setContour(0x1000);
  }
  if (result)
  { // automatic gain
    gn = gna[0]; //0x4000;
    sh = shutZero; // 0x3000;
    cam.setGain(gn);
    cam.setShutter(sh);
    //Wait(1.0);
    oldAvg = 0.0;
    n = 0;
    raw = &rawHDR[n];
    // remove green image
    result = cam.getImageSnapshot(raw, false);
    // get a reference image (average)
    result = cam.getImageSnapshot(raw, false);
    raw->getStatistics(&min, &max, &oldAvg, &var);
    printf("Stat sh:%x i=%d, min:%3d, max:%3d, avg:%6.3f, SD:%6.3f\n",
            sh, -1, min, max, oldAvg, sqrt(var));
    for (i = 0; i < 16; i++)
    { // save image
      raw = &rawHDR[n % RAW_MAX];
      // advance shutter and gain
      if (i < MAX_RNG)
      { // increase sensitivity
        gn = gna[i];
        sh = sha[i];
      }
      else
      { // shut down
        sh = 0x1000;
        gn = gna[0];
      }
      cam.setShutter(sh);
      cam.setGain(gn);
      Wait(0.011);
      result = cam.getImageSnapshot(raw, false);
      if (result)
      { // test image end
        raw->getStatistics(&min, &max, &avg, &var);
        printf("Stat sh:%x i=%d, gn:%x, max:%3d, avg:%6.3f, SD:%6.3f\n",
            sh, i, gn, max, avg, sqrt(var));
        raw->imgTime.show("imgTime");
        if ((avg - oldAvg) > 15 or true)
        {
          if ((oldAvg > 100) and ((oldAvg * 0.9) > avg) and (i > 4))
            // last image were the darkest
            break;
          n++;
          printf("Advanced n to %d\n", n);
        }
        oldAvg = avg;
      }
      if (not result)
      { //
        printf("get image or something failed\n");
        break;
      }
    }
   //save images
    for (j = 0; j < mini(n + 1, RAW_MAX); j++)
    { // image has gone dark
      raw = &rawHDR[j];
      raw->moveToRGB(&imT1, true);
      snprintf(s, SL_MAX, "hdr/imHDR%2d", j);
      printf("Saved image %s\n", s);
      imT1.save(s, NULL, imagePath);
    }
    if (n > 4)
    { // make HDR image in rawHDR[0]
      for (i = 0; i < MAX_RNG; i++)
        rawa[i] = &rawHDR[n-MAX_RNG+i];
      // black-level image
      raw = &rawHDR[n];
      // result image ring format
      rawr = &rawHDR[0];
      // result image HDR
      rawHD = &rawHDR[1];
      // prepare HDR image
      imT1.setSize(raw->getHeight(), raw->getWidth(), 3, 8);
      imT2.setSize(raw->getHeight()/2, raw->getWidth()/2, 3, 8);
      // make images
      for (r = 0; r < rawHDR->getHeight()-1; r += 2)
      {
        for (i = 0; i < MAX_RNG; i++)
          ps[i] = rawa[i]->getYline(r);
        ppix1 = imT1.getLine(r);
        ppix2 = imT1.getLine(r+1);
        for (c = 0; c < rawHDR->getWidth()-1; c += 2)
        { // get best pixel
          for (i = MAX_RNG-1; i > 0; i--)
            if (*ps[i] < TH)
              break;
          // get black-level image for 2 shortest shutter times
          rawa[0]->getQuadPix(r, c, &y11, &y12, &y21, &y22, &u, &v);
          raw->getQuadPix(r, c, &sy11, &sy12, &sy21, &sy22, &su, &sv);

          // debug
          if (i == 0)
            j = 0;
          else if (i == 1)
            j = 1;
          else if (i == 2)
            j = 2;
          else
            j = i;
          // debug end;

          // make HDR image.
          // - calculate black value
          // -- convert some intensity pixel to RGB
          px.setYUVto((y11 + y22)/2, u, v, PIX_PLANES_RGB);
          pxr = px.r;
          pxg = px.g;
          pxb = px.b;
          // -- convert no intensity pixel to RGB
          px.setYUVto((sy11 + sy22)/2, su, sv, PIX_PLANES_RGB);
          fac = -(sha[i] - shutZero);
          // calculate black level (image read-out exposure)
          pbr = maxi(0, (shutZero * pxr - sha[i] * px.r)/fac);
          pbg = maxi(0, (shutZero * pxg - sha[i] * px.g)/fac);
          pbb = maxi(0, (shutZero * pxb - sha[i] * px.b)/fac);
          //
          imT2.setPix(r/2, c/2, imT2.pixRGB(pbr, pbg, pbb));
          // - get pixel from usable image
          rawa[i]->getQuadPix(r, c, &y11, &y12, &y21, &y22, &u, &v);
          // - set result - in bands raw.
          rawr->setQuadPix(r, c, y11, y12, y21, y22, u, v);
          // - make 4 HDR pixels
          // - find max expanded pix value
          max = shutGain * 256 / (sha[0] - shutZero);
          lmax = log(float(max));
          fac = sha[i] - shutZero;
          // -- first value
          px.setYUVto(y11, u, v, PIX_PLANES_RGB);
          // --- subtract background and extend relative to shutter value
          pxr = (maxi(1,(px.r - pbr)) * shutGain)/fac;
          pxg = (maxi(1,(px.g - pbg)) * shutGain)/fac;
          pxb = (maxi(1,(px.b - pbb)) * shutGain)/fac;
          lr = log(float(pxr));
          lg = log(float(pxg));
          lb = log(float(pxb));
          px.setRGBto((unsigned char)((lr * 256.0) / lmax),
                      (unsigned char)((lg * 256.0) / lmax),
                      (unsigned char)((lb * 256.0) / lmax), PIX_PLANES_RGB);
          // --- save
          *ppix1++ = px;
          // -- second value first row
          px.setYUVto(y12, u, v, PIX_PLANES_RGB);
          pxr = (maxi(1, px.r - pbr) * shutGain)/fac;
          pxg = (maxi(1, px.g - pbg) * shutGain)/fac;
          pxb = (maxi(1, px.b - pbb) * shutGain)/fac;
          lr = log(float(pxr));
          lg = log(float(pxg));
          lb = log(float(pxb));
          px.setRGBto((unsigned char)((lr * 256.0) / lmax),
                      (unsigned char)((lg * 256.0) / lmax),
                      (unsigned char)((lb * 256.0) / lmax), PIX_PLANES_RGB);
//          px.setRGBto((pxr * 256) / max,
//                      (pxg * 256) / max,
//                      (pxb * 256) / max, PIX_PLANES_RGB);
          *ppix1++ = px;
          // -- first value second row
          px.setYUVto(y21, u, v, PIX_PLANES_RGB);
          pxr = (maxi(1, px.r - pbr) * shutGain)/fac;
          pxg = (maxi(1, px.g - pbg) * shutGain)/fac;
          pxb = (maxi(1, px.b - pbb) * shutGain)/fac;
          lr = log(float(pxr));
          lg = log(float(pxg));
          lb = log(float(pxb));
          px.setRGBto((unsigned char)((lr * 256.0) / lmax),
                      (unsigned char)((lg * 256.0) / lmax),
                      (unsigned char)((lb * 256.0) / lmax), PIX_PLANES_RGB);
//          px.setRGBto((pxr * 256) / max,
//                      (pxg * 256) / max,
//                      (pxb * 256) / max, PIX_PLANES_RGB);
          *ppix2++ = px;
          // -- second value second row
          px.setYUVto(y22, u, v, PIX_PLANES_RGB);
          pxr = (maxi(1, px.r - pbr) * shutGain)/fac;
          pxg = (maxi(1, px.g - pbg) * shutGain)/fac;
          pxb = (maxi(1, px.b - pbb) * shutGain)/fac;
          // debug
          if ((((pxr * 256) / max) > 255) or
                      (((pxg * 256) / max) > 255) or
                      (((pxb * 256) / max) > 255))
            // overflow
            j = max;
          // debug end
          lr = log(float(pxr));
          lg = log(float(pxg));
          lb = log(float(pxb));
          px.setRGBto((unsigned char)((lr * 256.0) / lmax),
                      (unsigned char)((lg * 256.0) / lmax),
                      (unsigned char)((lb * 256.0) / lmax), PIX_PLANES_RGB);
//          px.setRGBto((pxr * 256) / max,
//                      (pxg * 256) / max,
//                      (pxb * 256) / max, PIX_PLANES_RGB);
          *ppix2++ = px;
          //
          // Advance source intensity pointer
          for (i = 0; i < MAX_RNG; i++)
            ps[i] += 2;
        }
      }
      // save new image - HDR
      snprintf(s, SL_MAX, "hdr/imHDR-HDR");
      printf("Saved image %s\n", s);
      imT1.save(s, NULL, imagePath);
      // save background
      snprintf(s, SL_MAX, "hdr/imHDR-BGR");
      printf("Saved image %s\n", s);
      imT2.save(s, NULL, imagePath);
      // save new image - band
      rawr->moveToRGB(&imT1, true);
      snprintf(s, SL_MAX, "hdr/imHDR-Band");
      printf("Saved image %s\n", s);
      imT1.save(s, NULL, imagePath);
    }
  }
  return result;
}

#endif

///////////////////////////////////////////////////////

bool testSegmentation1()
{
  return true;
}

///////////////////////////////////////////////////////

bool testSobel()
{
  bool result;
  UImage640 img;
  UImage640 img2;
  const int FN_SIZE = 200;
  char s[FN_SIZE];
  //const char imgn[] = "log01040908_153841.527_116";
  const char imgn[] = "log01040908_153758.526_36";
  //
  snprintf(s, FN_SIZE, "%s/mmr2/%s.bmp", imagePath, imgn);
  result = img.loadBMP(s);
  //
  if (result)
  {
    img.edgeSobel(&img2);
    snprintf(s, FN_SIZE, "%s/%s-Sobel.bmp", imagePath, imgn);
    img2.saveBMP(s);
    printf("Saved result to %s\n", s);
  }
  // testimage
  if (result)
  {
    img.clear(128);
    cvRectangle( img.cvArr(), cvPoint(0,0), cvPoint(4, 10), CV_RGB(200,128,128), -1, 8, 0 );
    cvRectangle( img.cvArr(), cvPoint(1,100), cvPoint(100, 110), CV_RGB(100,150,100), -1, 8, 0 );
    cvRectangle( img.cvArr(), cvPoint(10,140), cvPoint(100, 145), CV_RGB(100,100,150), -1, 8, 0 );
    cvCircle(img.cvArr(), cvPoint(200,30), 30, CV_RGB(160,160,90), -1, 8, 0);
    img.edgeSobel(&img2, false, 4);
    snprintf(s, FN_SIZE, "%s/%s-Sobel_source.bmp", imagePath, imgn);
    img.saveBMP(s);
    snprintf(s, FN_SIZE, "%s/%s-Sobel_dest.bmp", imagePath, imgn);
    img2.saveBMP(s);
    printf("Saved result to %s\n", s);
  }
  //
  return result;
}

///////////////////////////////////////////////////////////

bool testCamPar()
{
  UCamPar par;
  UPosition pos1(3.3, -0.2, 0.1);
  URPos pixpos;
  UPosition pos2;
  //
  printf("Testing some camera parameter functions, convert to image and back.\n");
  par.setCameraParameters(320.0, 240.0, 1e-7, 1e-12, 1055.0, 1.0);
  par.setPixelSize(1.0); // 640x480
  par.setPixelSize(2.0); // 320x240
  // show some matrices
  par.print("par");
  pos1.show("3D position");
  pixpos = par.getCtoPRob(pos1);
  pixpos.show("Pix position");
  pos2 = par.getPtoCRob(roundi(pixpos.x) , roundi(pixpos.y), pos1.x);
  pos2.show("Back to 3D (rounded)");
  //
  return true;
}

//////////////////////////////////////////////////////////////

bool testLinePlane()
{
  ULine plane;
  ULine line;
  UPosition p,v;
  bool result;
  //
  p.set(1.0, 2.0, 3.0);
  v.set(0.1, 0.1, 1.0);
  plane.set(p, v);
  plane.show("Plane");
  //
  p.set(3.0, 4.0, 5.0);
  v.set(2.0, 2.0, 2.0);
  line.set(p, v);
  line.show("Line");
  // now get crossing position
  p = plane.getPlaneLineCrossing(line, &result);
  // show result
  p.show("Crossing at - should be (1,2,3)");
  //
  return result;
}

//////////////////////////////////////////////////////////////

bool testGetKey()
{
  int nkey = 0;
  int ckey = 0;
  int n;
  //
  printf("Hit keys to see code, and 'q' to quit\n");
  while (nkey != 'q')
  {
    n = getKey(&nkey, &ckey);
    if (n == 1)
      printf("Normal  key 0x%02x (%d) char: '%c'\n", nkey, nkey, char(nkey));
    else
      printf("Control key 0x%02x (%d) char: '%c'\n", ckey, ckey, char(ckey));
  }
  //
  return true;
}

///////////////////////////////////////////////////////////////

bool testLineSegment()
{
  UPosition pos1(1.0, 1.0, 1.0);
  UPosition pos2(1.0, 1.0, 2.0);
  UPosition pos3(0.0, 0.0, 0.0);
  ULineSegment seg;
  double d;
  //
  seg.setFromPoints(&pos1, &pos2);
  seg.show("Line (from (1,1,1) pointing up length 1");
  pos3.show("Point");
  d = seg.getDistanceFromSeg(pos3);
  printf("Distnace is %f m (should be sqrt(3.0) (%f)\n", d, sqrt(3.0));
  //
  pos3.set(0.0, 0.0, 1.0);
  pos3.show("Point");
  d = seg.getDistanceFromSeg(pos3);
  printf("Distnace is %f m (should be sqrt(2.0) (%f)\n", d, sqrt(2.0));
  //
  pos3.set(0.0, 0.0, 1.5);
  pos3.show("Point");
  d = seg.getDistanceFromSeg(pos3);
  printf("Distnace is %f m (should be sqrt(2.0) (%f)\n", d, sqrt(2.0));
  //
  pos3.set(0.0, 0.0, 2.0);
  pos3.show("Point");
  d = seg.getDistanceFromSeg(pos3);
  printf("Distnace is %f m (should be sqrt(2.0) (%f)\n", d, sqrt(2.0));
  //
  pos3.set(0.0, 0.0, 3.0);
  pos3.show("Point");
  d = seg.getDistanceFromSeg(pos3);
  printf("Distnace is %f m (should be sqrt(3.0) (%f)\n", d, sqrt(3.0));
  //
  return true;
}

/////////////////////////////////////////////////////////////////

void testHex()
{
  int i, h;
  char val[4];
  printf("Test hex to int conversion ...\n");
  for (i = 0; i < 255; i++)
  {
    snprintf(val, 4, "%02x", i);
    h = hex2int(val[0], val[1]);
    if (h != i)
      printf("error, as %d == %s == %d is false\n", i, val, h);
  }
  printf("Test hex to int conversion finished\n");
}

/////////////////////////////////////////////////////////////////

void testPath()
{
  printf("Image    path '%s'\n", imagePath);
  printf("Data     path '%s'\n", dataPath);
  printf("Host '%s' port %d\n", serverName, serverPort);
}

/////////////////////////////////////////////////////////////////

void testImageShift()
{
  UImage640 imgData;
  UImage * img = &imgData;
  //
  printf("Testing image-transplation horizontal and vertical\n");
  printf("Loading image 'atest.bmp' in %s ...\n", imagePath);
  //
  img->loadBMP(imagePath,"atest", -1, "");
  if (img->valid)
  {
    img->shiftRows(50, 255);
    img->saveBMP(imagePath, "atest", 50, "row");
    img->shiftCols(50, 180);
    img->saveBMP(imagePath, "atest", 50, "com");
    img->shiftRows(-50, 100);
    img->saveBMP(imagePath, "atest", 50, "mrow");
    img->shiftCols(-50, 10);
    img->saveBMP(imagePath, "atest", 50, "mcol");
    printf("... saved some result images in image-path\n");
  }
  else
    printf("... image 'atest.bmp' not found\n");
}

/////////////////////////////////////////////////////////////

void   testCylinderCrossing()
{
  ULine l;
  UPosition p1, p2, c;
  double r, t1, t2;
  int n;
  //
  p1.set(0.0, -2.0, 4.0);
  p2.set(2.0, -1.0, 5.0);
  l.setFromPoints(&p1, &p2);
  c.set(-1.0, -1.0, 0.0);
  r = 3.0;
  n = l.getCylinderCrossings(c, r, &t1, &t2);
  if (n == 2)
  {
    printf("The 2 crossings are at %f and %f\n", t1, t2);
    p1 = l.getPositionOnLine(t1);
    p1.show("p1");
    p2 = l.getPositionOnLine(t2);
    p2.show("p2");
  }
  else
    printf("There are no crossings with cylinder\n");
  // and using a sphere
  n = l.getSphereCrossings(c, r, &t1, &t2);
  if (n == 2)
  {
    printf("The 2 crossings are at %f and %f\n", t1, t2);
    p1 = l.getPositionOnLine(t1);
    p1.show("p1");
    p2 = l.getPositionOnLine(t2);
    p2.show("p2");
  }
  else
    printf("There are no crossings with sphere\n");
}

///////////////////////////////////////////////////////////////////////

void testXmlEscape()
{
  const int SL = 40;
  char s1[SL] = "This \"shit\" is & <bad> for 'XML'.";
  char s2[SL];
  const int SLL = 100;
  char s3[SLL];
  char * destXml;
  char * destStr;
  //
  destXml = str2xml(s2, SL, s1);
  printf("Original text. '%s'\n", s1);
  printf("XML size[40] . '%s'\n", destXml);
  destStr = xml2str(s3, SLL, s2, -1);
  printf("and back text. '%s'\n", destStr);
  // using one string buffer only
  strcpy(s3, s1);
  printf("Original text. '%s'\n", s3);
  destXml = str2xml(s3, SLL, s3);
  printf("XML size[100]. '%s'\n", s3);
  destStr = xml2str(s3, SLL, s3, -1);
  printf("and back text. '%s'\n", s3);
}

//////////////////////////////////////////////////////////////

void testTime2()
{
  UTime t;
  //
  t.Now();
  t.print("Time is now");
  t += 59.5;
  t.print("Time + 59.5s");
  t -= 59.5;
  t.print("Time - 59.5s");
  t -= 59.5;
  t.print("Time - 59.5s");
}

//////////////////////////////////////////////////////////////

void testTangetPoint()
{
  UPosition a(10.0, 10.0, 0.0); // outside point
  UPosition c(-5.0, 0.0, 0.0);  // circle centre
  double radius = 30;
  bool isOK;
  UPosition b; // result;
  //
  b = a.getTangentPointXY(c, radius, true, &isOK);
  a.print("A");
  c.print("Center");
  printf("Radius %f, valid %s\n", radius, bool2str(isOK));
  b.print("B");
}

////////////////////////////////////////////////

void testPlane()
{
  UOriPlane pl1, pl2;
  UPosition pos;
  URotation rot;
  //
  pl1.print("zero (floor) plane");
  pos.set(22.0, 33.0, 44.0);
  pl1.set(pos, rot);
  pl1.print("shifted (floor) plane");
  // turned along x axis only
  rot.set(M_PI /2.0, 0.0, 0.0);
  pl1.set(pos, rot);
  pl1.print("upright plane");
  // turned along z axis (45deg) and then x axis (90deg)
  // i.e. upright but rotated 45 deg
  rot.set(M_PI /2.0, 0.0, M_PI * 0.25);
  pl1.set(pos, rot);
  pl1.print("upright plane turned 45 deg");
}

////////////////////////////////////////////////

void testPng()
{
#ifndef IMAGE_NO_PNG
  UImage800 img;
  bool result;
  //result = img.loadPNG("/home/chr/chr/images/Laser-20050819_184050.596-00002.png");
  result = img.loadPNG("/home/jca/chr/images/YUV420_d0_s-1_Top_-20070507_163438.339.png");
  if (result)
    result = img.saveBMP("test.bmp");
  if (result)
    result = img.savePNG("test.png");
#endif
}

///////////////////////////////////////////////////////

void testZlib()
{
  UImage800 img;
  UImage800 img2;
  char * buff;
  int buffSize;
  int bufUsed;
  bool result;
  UTime t;
  //
  //result = img.loadPNG("/home/chr/chr/images/Laser-20050819_184050.596-00002.png");
  result = img.loadBMP("/home/chr/chr/images/mmr/log01040906_151154.956_233.bmp");
  buffSize = 800 * 600 * 3;
  buff = (char*)malloc(buffSize);
  result = (buff != NULL);
  if (result)
  {
    t.Now();
    bufUsed = img.packZLIB(buff, buffSize);
    printf("Packing %d bytes to %d took %.2f ms\n",
           img.imgBytes(), bufUsed, t.getTimePassed() * 1000.0);
    img.clear(0);
    t.Now();
    result = img.unpackZLIB(img.height(), img.width(), "BGR", buff, bufUsed);
    printf("Unpacking %d bytes to %d took %.2f ms\n",
           bufUsed, img.imgBytes(), t.getTimePassed() * 1000.0);
  }
  if (result)
  {
    img.saveBMP("testZlib.bmp");
    img.savePNG("testZlib.png");
  }
}

/////////////////////////////////////////////////////

// void testZlibRawImg()
// {
//   URawImage img;
//   char * buff;
//   int buffSize;
//   int bufUsed;
//   bool result;
//   UTime t;
//   bool diff = true;
//   //
//   //result = img.loadPNG("/home/chr/chr/images/Laser-20050819_184050.596-00002.png");
//   //result = img.loadBMP("/home/chr/chr/images/mmr/log01040906_151154.956_233.bmp");
//   result = img.loadBMP("/home/chr/chr/images/mmr/log01040906_150418.348_34.bmp");
//   img.saveBMP("testDiffZ-org.bmp");
//   buffSize = (640 * 480 * 3);
//   buff = (char*)malloc(buffSize);
//   result = (buff != NULL);
//   if (result)
//   {
//     t.Now();
//     if (diff)
//       bufUsed = img.packDiffZ(buff, buffSize);
//     else
//       bufUsed = img.packZ(buff, buffSize);
//     printf("Packing %d bytes to %d took %.2f ms\n",
//            img.getDataSize(), bufUsed, t.getTimePassed() * 1000.0);
//     img.clear();
//     t.Now();
//     if (diff)
//       result = img.unpackDiffZ(img.getHeight(), img.getWidth(), buff, bufUsed);
//     else
//       result = img.unpackZ(img.getHeight(), img.getWidth(), buff, bufUsed);
//     printf("Unpacking %d bytes to %d took %.2f ms\n",
//            bufUsed, img.getDataSize(), t.getTimePassed() * 1000.0);
//   }
//   if (result)
//   {
//     img.saveBMP("testDiffZ.bmp");
//   }
// }

///////////////////////////////////////////////////////

void testPolygon()
{
  UPolygon40 p40;
  UPolygon40 p41;
  UPolygon40 conv40;
  UPolygon40 conv41;
  bool overlap;
  bool isIn;
  //
  p40.add(-2.0, 8.0);
  p40.add(-1.0, 4.0);
  p40.add(-1.0, 5.0);
  p40.add(-2.0, 0.0);
  p40.add(0.0, 0.0);
  p40.add(0.0, 3.0);
  p40.add(1.0, 3.0);

  p41.add(0.0, 0.0);
  p41.add(0.0, 2.0);
  //p41.add(0.0, 2.0);
  p41.add(2.0, 1.0);
  p41.add(2.0, 4.0);
  p41.add(4.0, 1.0);
  p41.add(4.0, 4.0);
  //
  p40.print("points40");
  p40.extractConvexTo(&conv40);
  conv40.print("conv hull 40");
  p41.print("points41");
  p41.extractConvexTo(&conv41);
  conv41.print("conv hull 41");
  //
  // test for overlap
  overlap = conv40.isOverlappingXY(&conv41);
  if (overlap)
    printf("conv40 and conv41 is overlapping\n");
  else
    printf("conv40 and conv41 is not overlapping\n");
  //
  isIn = conv40.isInsideConvex(-1.0, 1.0); // in
  isIn = conv40.isInsideConvex(-2.0, 1.0);
  isIn = conv40.isInsideConvex(-3.0, 1.0);
  isIn = conv40.isInsideConvex( 1.0, 1.0);
  isIn = conv40.isInsideConvex(-2.0, 0.0);
  isIn = conv40.isInsideConvex(-2.0, -1.0);
  isIn = conv40.isInsideConvex(-2.0, 8.0);
  isIn = conv40.isInsideConvex(-2.0, 0.0);
  isIn = conv40.isInsideConvex(-2.0, 8.0);
  isIn = conv40.isInsideConvex( 0.0, 0.0);
  isIn = conv40.isInsideConvex(-3.0, -1.0);
  // 1144 bytes
  printf("Size of an UPolygon40 is %d bytes\n", sizeof(UPolygon40));
}

////////////////////////////////////////////

void testSignedDistance()
{
  UPosition p1(0.0,  0.0, 0.0);
  UPosition p2(-1.0, -1.0, 0.0);
  UPosition pt(1.5, 3.0, 0.0);
  ULineSegment seg;
  double dist;
  int w;
  //
  seg.setFromPoints(p1, p2);
  dist = seg.getDistanceXYSigned(pt, &w);
  printf("Distance is %f (w=%d)\n", dist, w);
}

////////////////////////////////////////////////

void testUPosRot()
{ // test new class UPosRot and the additions to URotation
  // setFromYZ
  UPosition g0, Y, Z, r0, rZ, rY, rrY, rrZ;
  URotation R, rOrg, rotG;
  UMatrix4 mR, mRR;
  UPosRot prG, prC, prB, prR, pr0;
  //
  printf("Two axis vectors Y and Z:\n");
  Z.set(0.0, 0.0, 1.0);
  Y.set(0.0, 1.0, 0.0);
  Z.print("Z-vec");
  Y.print("Y-vec");
  rOrg.set(10.0 * M_PI / 180.0, -9.0 * M_PI / 180.0, -33.0 * M_PI / 180.0);
  mR = rOrg.asMatrix4x4RtoM();
  // get the 2 unit vectors rotatet
  printf("Is seen from a rotated coordinate system\n");
  rOrg.print("rot");
  rZ = mR * Z;
  rY = mR * Y;
  printf("The vectors are here:\n");
  Z.print("Zrot");
  Y.print("Yrot");
  // get rotation from the rotated Y-Z plane
  R.setFromYZ(rY, rZ);
  printf("The rotation is recovered from the vectors using setFromYZ(Yrot, Zrot) to\n");
  R.print("rec");
  // use the obtained rotation to generate a inverse rotation matrix
  mRR = R.asMatrix4x4MtoR();
  rrZ = mRR * rZ;
  rrY = mRR * rY;
  // back to unit vectors in Y and Z direction
  printf("The 2 vectors are then rotated back using the recovered rotation\n");
  rrZ.print("Z-back");
  rrY.print("Y-back");
  // ------------------------------------------ GMK -------------------
  // now try a guidemark seen from a rotated camera
  printf("\n");
  printf("Set a guidemark at this (position/rotation):\n");
  // set guidemark position
  // ahead facing robot (5.0, 0.0, 0.0;
  prG.set(5.0, 1.0, 0.33,
          0.0 * M_PI / 180.0, -90.0 * M_PI / 180.0, 0.0 * M_PI / 180.0);
  prG.print("Org");
  // get guidemark position in map coordinates
  g0 = *prG.getPos();
  mR = prG.getRot()->asMatrix4x4RtoM();
  rZ = g0 + mR * Z;
  rY = g0 + mR * Y;
  printf("Seen from a camera at this position:\n");
  // set camera position
  prC.set(0.45, 0.3, 1.0,
          7.0 * M_PI / 180.0,  8.5 * M_PI / 180.0, -14.0 * M_PI / 180.0);
  prC.print("Cam");
  // convert to camera coordinates
  mR = prC.getMtoRMatrix();
  rZ = mR * rZ;
  rY = mR * rY;
  r0 = mR * g0;
  //r0.print("GMK pos");
  //rY.print("GMK Yax");
  //rZ.print("GMK Zax");
  // set guidemark position/rotation, as seen from camera.
  rotG.setFromYZ(rY - r0, rZ - r0);
  prG.set(r0, rotG);
  printf("From the camera the guidemark is at this (position/rotation)\n");
  prG.print("gmk");
      //
  // now convert guidemark position (back to robot coordinates
  prB.setCtoR(&prC, &prG, true);
  printf("Using setConverted(camPosRot, gmkPosRot) the guidemark is back to:\n");
  prB.print("cnv");
  // show robot position in guidemark coordinates
  prR.setRtoC(&prB, &pr0);
  printf("Show robot coordinates (origo) in guidemark reference system\n");
  prR.print("Rob");
}


////////////////////////////////////////////////////////

void testOpenCv()
{
  UImage * img1 = new UImage320();
  UImage * img2 = new UImage320();
  UImage * img3 = new UImage320();
  //
  img1->loadBMP("/home/jca/chr/img.bmp");
  img1->toBW(NULL);
  img2->setSize(img1->height(), img1->width(), 1, IPL_DEPTH_16S, "BW16S");
  img3->setSize(img1->height(), img1->width(), 1, IPL_DEPTH_16S, "BW16S");
  cvSobel(img1->cvArr(), img2->cvArr(), 0, 1, 3);
  cvSobel(img1->cvArr(), img3->cvArr(), 1, 0, 3);
  img2->toBW(NULL);
  img2->savePNG("/home/jca/chr/imgSobel1.png");
  img3->toBW(NULL);
  img3->savePNG("/home/jca/chr/imgSobel2.png");
  img1->loadBMP("/home/jca/chr/img.bmp");
  //void cvCanny( const CvArr* image, CvArr* edges, double threshold1,
  //            double threshold2, int aperture_size=3 );  delete img1;
  img1->toBW(NULL);
  img2->setSize(img1->height(), img1->width(), 1, 8, "BW");
  cvCanny(img1->cvArr(), img2->cvArr(), 0.9, 0.1, 3);
  img2->savePNG("/home/jca/chr/imgCanny.png");
  delete img2;
  delete img3;
}

////////////////////////////////////////////////////
#include <sys/ioctl.h>
#include <stropts.h>
#include <sys/mman.h>
#include <unistd.h>
#include "ucampwc.h"

void testVideo()
{
  bool result = true;
//  URawImage raw; // distorted image
  UImage800 * img8;
  UCamDevBase dev;
  //UCamRad cam;
  UCamRad cam(&dev);
//  const int MSL = 50;
//  char s[MSL];
  struct video_channel vch;
//  int err;
  struct video_mbuf vmbuf;
  struct video_mmap vmmap;
  char * memp;
  char * imgBuff;
//  struct video_buffer framebuffer;
  int n;
  //
  img8 = new UImage800();
  //
  printf("Testing video image\n");
  //
  dev.setDeviceNumber(0);
  printf("Getting camera name ... \n");
  // open device
  result  = dev.setDevice(320, 240, 5);
  result &= dev.openDevice();
  if (result)
    printf("Camera %s open in (%d x %d) \n",
           dev.getCameraName(), dev.getWidth(), dev.getHeight());
  else
    printf("Camera set size and framerate failed\n");
  // try code from videoDog
  //grab_init(&videodog);
/*  if ((vd->grab_fd = open(vd->v_device,O_RDWR)) == -1 ) {
    fprintf(stderr,"%s\n", vd->v_device);
    perror("open videodev");
    exit(1);
  }
  if (ioctl(vd->grab_fd,VIDIOCGCAP,&(vd->grab_cap)) == -1) {
    fprintf(stderr,"wrong device\n");
    exit(1);
  } */
  vch.channel = 0;
  if (ioctl(dev.getCamFd(),  VIDIOCGCHAN, &vch) == -1)
    perror("init: VIDIOCGCHAN");
  // default settings

  //vd->grab_vid.norm=VIDEO_MODE_PAL;
  //vd->grab_vid.channel=0;
  vch.channel = 0;
  if (ioctl(dev.getCamFd(),  VIDIOCSCHAN, &vch) == -1)
    perror("init: VIDIOCGCHAN");
  /* Query the actual buffers available */
  //if(ioctl(vd->grab_fd, VIDIOCGMBUF, &(vd->grab_vm)) < 0)
  if (ioctl(dev.getCamFd(), VIDIOCGMBUF, &vmbuf) < 0)
    perror("VIDIOCGMBUF");
  //get memmap data
  memp = (char *)mmap(0, vmbuf.size, PROT_READ /*|PROT_WRITE*/, MAP_SHARED, dev.getCamFd(), 0);

  // grab one
  vmmap.frame = 0;
  vmmap.format = VIDEO_PALETTE_RGB24;
  vmmap.width  = dev.getWidth();
  vmmap.height = dev.getHeight();

  if (-1 == ioctl(dev.getCamFd(), VIDIOCMCAPTURE, &vmmap))
    perror("ioctl VIDIOCMCAPTURE");
  if (-1 == ioctl(dev.getCamFd(), VIDIOCSYNC, &vmmap))
    perror("ioctl VIDIOCSYNC");


  //set_picture(&videodog);
  //if (videodog.channel != 99) set_channel(&videodog);
  //set_channel(&videodog);
  //set_mode(&videodog);

  //update_cap(&videodog);
  // grapOne
        // copy image data to img8;
  img8->setSize(dev.getHeight(), dev.getWidth(), 3, 8, "RGB");
  imgBuff = (char *)img8->getData();
  n = img8->getDataSize();
  memmove(imgBuff, memp + vmbuf.offsets[vmmap.frame], n);
  img8->toRGB(NULL);
  img8->saveBMP("frame0dog.bmp");
  img8->savePNG("frame0dog.png");

}

////////////////////////////////////////////////////////

void testVideo2()
{
  bool result = true;
//  URawImage raw; // distorted image
  UImage800 img8;
  //UCamRad cam;
  UCamDevBase dev;
  UCamRad cam(&dev);
  int i, n;
  const int MSL = 50;
  char s[MSL];
  struct video_channel vch;
  int err;
  struct video_mbuf vmbuf;
  char * mem;
  char * imgBuff;
  struct video_buffer framebuffer;
  //
  printf("Testing video image\n");
  //
  dev.setDeviceNumber(0);
  printf("Getting camera name ... \n");
  // open device
  result = dev.setDevice(320, 240, 5);
  if (result)
      printf("Camera %s open in (%d x %d) \n",
             dev.getCameraName(), dev.getWidth(), dev.getHeight());
  else
    printf("Camera set size and framerate failed\n");
  if (false)
  { //start capture
    i = 1;
    err = ioctl(dev.getCamFd(), VIDIOCCAPTURE, &i);
    if (err != 0)
      perror("VIDIOCCAPTURE 1");
    else
      printf("Capture started\n");
    printf("Trying a read\n");
    result = cam.getImageSnapshot(&img8); //, false);
    Wait(0.2);
    result = cam.getImageSnapshot(&img8); //, false);
    if (result)
    {
      snprintf(s, MSL, "dev%dCap%d.png", dev.getDeviceNumber(), i);
      img8.savePNG(s);
    }
    else
      printf("No imge (%d)\n", i);
  }
  if (true)
  { // set to TV channel
    vch.channel = 0;
    printf("Trying source %d (0=TV 1=? 2=s-video, 4=?)\n", vch.channel);
    err = ioctl(dev.getCamFd(), VIDIOCGCHAN, &vch);
    if (err != 0)
      printf("Request channel %d info failed\n", vch.channel);
    if (vch.flags != 0)
      printf("Channel set to vch.name=%s\n", vch.name);
    // get buffer structure
    err = ioctl(dev.getCamFd(), VIDIOCGMBUF, &vmbuf);
    if (err != 0)
      perror("ioctl VIDIOCGMBUF");
    else
      printf("Buffer with %d frames available (size=%d)\n", vmbuf.frames, vmbuf.size);
    // now setup access to memory buffers
/*    mem = (char *)mmap(0, vmbuf.size, PROT_READ|PROT_WRITE,MAP_SHARED,
    cam.getCamFd(),0);*/
    mem = (char *)mmap(0, vmbuf.size,
    PROT_READ /*| PROT_WRITE*/, MAP_SHARED,
                    dev.getCamFd(),0);
    if (int(mem) < 0)
      perror("mmap");
    else
      printf("Got pointer to shared memory (%ld)\n", (long)mem);

    if (false)
    { // set framebuffer to not-used (not allowed)
      framebuffer.base = NULL;
      err = ioctl(dev.getCamFd(), VIDIOCSFBUF, &framebuffer);
      if (err != 0)
        perror("framebuffer");
      else
        printf("framebuffer disabled\n");
    }

    if (false)
    { // start capture to framebuffer - shows image directly using framebuffer
      i = 1;
      err = ioctl(dev.getCamFd(), VIDIOCCAPTURE, &i);
      if (err != 0)
        perror("VIDIOCCAPTURE 1");
      else
        printf("Capture started\n");
    }
    if (false)
    {  // capture to buffer 0 - fails for bttv - wrong argument
      i = 1;                    //VIDIOCMCAPTURE
      err = ioctl(dev.getCamFd(), VIDIOCMCAPTURE, &i);
      if (err != 0)
        perror("capture 1");
      else
        printf("Instructed to capture to buffer %d", i);
      err = ioctl(dev.getCamFd(), VIDIOCSYNC, &i);
      if (err != 0)
        perror("Sync 1");
      else
        printf("Data in buffer %d is now avaiilable", i);
      if ((err == 0))
      {
        // copy image data to img8;
        img8.setSize(dev.getHeight(), dev.getWidth(), 3, 8, "RGB");
        imgBuff = (char *)img8.getData();
        n = img8.getDataSize();
        memmove(imgBuff, mem + vmbuf.offsets[i], n);
        snprintf(s, MSL, "dev%dframe0.png", dev.getDeviceNumber());
        img8.savePNG(s);
      }
    }
    if (false)
    { // get image using read
      img8.setSize(dev.getHeight(), dev.getWidth(), 3, 8, "RGB");
      imgBuff = (char *)img8.getData();
      n = img8.getDataSize();
      i = read(dev.getCamFd(), imgBuff, n);
      if (i < 0)
        perror("bttv read");
      else
        printf("Got %d byted from TV card\n", i);
      img8.savePNG("devimageByRead.png");
    }
/*      channel The channel number
        name  The input name - preferably reflecting the label on the card input itself
        tuners  Number of tuners for this input
        flags Properties the tuner has
        type  Input type (if known)
        norm  The norm for this channel*/
    /* map grab buffer */
  }
  if (true)
  {
    /* ------------ */
    for (i = 0; i <= 4; i++)
    {
      vch.channel = i;
      printf("Trying source %d\n", vch.channel);
      err = ioctl(dev.getCamFd(), VIDIOCGCHAN, &vch);
      if (err != 0)
        printf("Request channel %d info failed\n", vch.channel);
      if (vch.flags != 0)
      {
        err = ioctl(dev.getCamFd(), VIDIOCSCHAN, &vch.channel);
        if (err != 0)
          printf("Set channel %d error\n", vch.channel);
        result = cam.getImageSnapshot(&img8); //, false);
        if (result)
        {
          snprintf(s, MSL, "dev%dbttv%d.png", dev.getDeviceNumber(), i);
          img8.savePNG(s);
        }
        else
          printf("No imge (%d)\n", i);
      }
    }
  }
  if (true)
  {
    for (i = 0; i < 4; i++)
    {
      result = cam.getImageSnapshot(&img8); //, false);
      if (result)
      {
        snprintf(s, MSL, "dev%dbttv%d.png", dev.getDeviceNumber(), i);
        img8.savePNG(s);
      }
      else
        printf("No imge (%d)\n", i);
    }
  }
  dev.closeDevice();
  //printf("Hit return to finish\n");
  //fgets(s, MSL, stdin);
  printf("finished (%s)\n", s);
}

//////////////////////////////////////////////////////

void test2DlineFit()
{
  U2Dlined l1(-1.0, -1.0, 5.0); // y = -x + 5
  U2Dlined l2;
  U2Dlined l3(0.0, 1.0, 0.0); // y = 0
  //double x[10] = {1.0, 3.0, 5.0, 6.0, 8.0, 9.0, 10.0, 10.0, 11.0, 12.0};
  //double y[10] = {1.0, 2.0, 2.0, 3.0, 3.0, 5.0,  4.0,  3.0,  4.0,  5.0};
  double x[10] = {1.0, 1.5, 2.0, 2.5, 3.0+1.0, 3.5+1.0,  4.0,  4.5,  5.0,   5.5};
  double y[10] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0,  7.0,  8.0,  9.0,  10.0};
  int i;
  double E; // mean
  double V, V2; // variance
  //
  // set from series of points - best fit line
  l2.fit(x,y,10);
  V = l2.variance(x, y, 10, &E); // E should be ~3
  l2.fitV(x,y,10, &V2);
  printf("Variance SDtrue=%f V2=%f\n", sqrt(V), sqrt(V2));
  l2.print("1:");
  for (i = 0; i < 10; i++)
  {
    x[i] *= 0.02;
  }
  l2.fit(x,y,10);
  l2.fitV(x,y,10, &V2);
  V = l2.variance(x, y, 10, &E); // E should be ~3
  printf("Variance SDtrue=%f V2=%f\n", sqrt(V), sqrt(V2));
  l2.print("1:");
  for (i = 0; i < 10; i++)
  {
    x[i] /= 0.02;
    y[i] *= 0.02;
  }
  l2.fit(x,y,10);
  l2.fitV(x,y,10, &V2);
  V = l2.variance(x, y, 10, &E); // E should be ~3
  printf("Variance SDtrue=%f V2=%f\n", sqrt(V), sqrt(V2));
  l2.print("1:");
}

/////////////////////////////////////////////////////////

void testFuzzySplit()
{
  const int MPC = 20;
  UPixel pixx[MPC];
  UPixel * pix;
  UFuzzyPixel fp[MPC];
  UFuzzyPixel * pfp;
  UFuzzySplit split;
  int i;
  const int clustCnt = 3;
  //
  pix = pixx;
  pix++->set(10, 200, 50);
  pix++->set(11, 220, 51);
  pix++->set(2, 210, 57);
  pix++->set(100, 110, 54);
  pix++->set(90,  130, 53);
  pix++->set(110, 100, 52);
  pix++->set(26, 210, 140);
  pix++->set(17, 205, 155);
  pix++->set(18, 230, 152);
  pix++->set(19, 190, 151);
  pix++->set(100, 0, 50);
  pix++->set(51, 20, 51);
  pix++->set(2, 210, 220);
  pix++->set(10, 10, 14);
  pix++->set(90,  130, 3);
  pix++->set(110, 10, 52);
  pix++->set(190, 210, 140);
  pix++->set(17, 160, 15);
  pix++->set(180, 23, 52);
  pix++->set(190, 190, 51);
  pfp = fp;
  pix = pixx;
  for (i = 0; i < MPC; i++)
  {
    pfp->setPixel(pix++);
    split.addElement(pfp);
    pfp++;
  }
  split.classify(clustCnt, 1e-4, 30, true);
  //split.classifyOld(clustCnt, 1e-4, 30);
  pfp = fp;
  split.countMembers();
  for (i = 0; i < clustCnt; i++)
    printf("Cluster %d has %d members\n", i, split.getMembCount(i));
  printf("Finished\n");
}

///////////////////////////////////////////////////////////

void testFuzzySplit2()
{
  const int MPC = 20;
  UPixel pixx[MPC];
  UPixel * pix;
  UFuzzyPixel fp[MPC];
  UFuzzyPixel * pfp;
  UFuzzySplit split;
  int i;
  const int clustCnt = 3;
  //
  pix = pixx;
  pix++->set(10, 200, 50);
  pix++->set(11, 220, 51);
  pix++->set(2, 210, 57);
  pix++->set(100, 110, 54);
  pix++->set(90,  130, 53);
  pix++->set(110, 100, 52);
  pix++->set(26, 210, 140);
  pix++->set(17, 205, 155);
  pix++->set(18, 230, 152);
  pix++->set(19, 190, 151);
  pix++->set(100, 0, 50);
  pix++->set(51, 20, 51);
  pix++->set(2, 210, 220);
  pix++->set(10, 10, 14);
  pix++->set(90,  130, 3);
  pix++->set(110, 10, 52);
  pix++->set(190, 210, 140);
  pix++->set(17, 160, 15);
  pix++->set(180, 23, 52);
  pix++->set(190, 190, 51);
  pfp = fp;
  pix = pixx;
  for (i = 0; i < MPC-5; i++)
  {
    pfp->setPixel(pix++);
    split.addElement(pfp);
    pfp++;
  }
  split.initFromValues(0, 0, 3);
  split.initFromValues(1, 3, 3);
  split.initFromValues(2, 6, 3);
  //
  split.classify(clustCnt, 1e-5, 10, false);
  pfp = fp;
  split.countMembers();
  for (i = 0; i < clustCnt; i++)
    printf("Cluster %d has %d members\n", i, split.getMembCount(i));
  printf("Finished\n");
}

///////////////////////////////////////////////////////////

void testCromaImage()
{
  UImage800 img8;
  //
  printf("converting some images to cromaticity only ...\n");
  img8.loadBMP("/home/chr/chr/images/k.bmp");
  img8.toCromaBGR(NULL);
  img8.saveBMP("/home/chr/chr/images/kCroma.bmp");
  //
  img8.loadBMP("/home/chr/chr/images/k1.bmp");
  img8.toCromaBGR(NULL);
  img8.saveBMP("/home/chr/chr/images/k1Croma.bmp");
  //
  img8.loadBMP("/home/chr/chr/images/k2.bmp");
  img8.toCromaBGR(NULL);
  img8.saveBMP("/home/chr/chr/images/k2Croma.bmp");
  //
  img8.loadBMP("/home/chr/chr/images/k3.bmp");
  img8.toCromaBGR(NULL);
  img8.saveBMP("/home/chr/chr/images/k3Croma.bmp");
  //
  printf("...[OK]\n");
}

////////////////////////////////////////////////

void test3DtoPix()
{
  UCamDevBase dev;
  UCamMounted cam(&dev);
  UPosition pos;
//  Uconfig ini;
  UPosition cPos(0.44, 0.0, 0.86);
  URotation cRot(0.0, 0.43, 0.0);
  float x,y;
  // prepare camera position
/*  ini.readConfig("/home/chr/cpp/robcam.conf");
  cam.loadCamSettings(&ini, "mmrCam0");*/
  cam.setPosOnRobot(&cPos, &cRot);
  // test a 3D position
  pos.set(2.6, 0.0, 0.0);
  cam.getMtoPix(pos, true, &x, &y);
  printf("(%gx, %gy, %gz) -> (%gx, %gy) in image\n",
         pos.x, pos.y, pos.z, x, y);
  pos.set(2.6, -1.0, 0.0);
  cam.getMtoPix(pos, true, &x, &y);
  printf("(%gx, %gy, %gz) -> (%gx, %gy) in image\n",
         pos.x, pos.y, pos.z, x, y);
  pos.set(2.6, 1.0, 0.0);
  cam.getMtoPix(pos, true, &x, &y);
  printf("(%gx, %gy, %gz) -> (%gx, %gy) in image\n",
         pos.x, pos.y, pos.z, x, y);
  pos.set(1.6, 0.1, 0.0);
  cam.getMtoPix(pos, true, &x, &y);
  printf("(%gx, %gy, %gz) -> (%gx, %gy) in image\n",
         pos.x, pos.y, pos.z, x, y);
  pos.set(1.6, -0.1, 0.5);
  cam.getMtoPix(pos, true, &x, &y);
  printf("(%gx, %gy, %gz) -> (%gx, %gy) in image\n",
         pos.x, pos.y, pos.z, x, y);
}

/////////////////////////////////////////////

void testPolygonCog()
{
  UPolygon40 poly1, poly2;
  UPosition p1, p2, p3, p4, p5, p6;
  UPosition cog;
  double a;
  //
  printf("Testing polygion area etc\n");
  p1.set(0.0, 0.0, 0.0);
  p2.set(4.0, 3.0, 7.0);
  p3.set(8.0, 4.0, 0.0);
  p4.set(6.0, 6.0, 0.0);
  p5.set(2.0, 6.0, 3.0);
  p6.set(-2.0, 2.0, 0.0);
  // add to polygon
  poly1.add(p1);
  poly1.add(p2);
  poly1.add(p3);
  poly1.add(p4);
  poly1.add(p5);
  poly1.add(p6);
  poly1.extractConvexTo(&poly2);
  //
  cog = poly1.getCogXY();
  a = poly1.getXYarea();
  //
  printf("Concave polygon COG is at %.4fx, %.4fy, area is %.3f m^2\n", cog.x, cog.y, a);
  //
  cog = poly2.getCogXY();
  a = poly2.getXYarea();
  //
  printf("Convex hul      COG is at %.4fx, %.4fy, area is %.3f m^2\n", cog.x, cog.y, a);

}

////////////////////////////////////////////

void testUImage()
{ // test virtual function imgUpdated()
  UImage * img;
  //
  img = new UImage640();
  img->imgUpdated();
  img->resize(800, 600);
  img->resize(752, 511, 4, 8);
  img->resize(800, 600, 1, 16);
  img->setColorType("BW16S");
  if (img->isBW16s())
    printf("img = %s\n", img->name);
  delete img;
}

/////////////////////////////////////////

void testColourConvert()
{ // test virtual function imgUpdated()
  UPixel a, b, c;
  //
  a.p1=50;
  a.p2=100;
  a.p3 = 150;
  printf("RGB p1=%3d p2=%3d p3=%3d\n", a.p1, a.p2, a.p3);
  b = a.asRGB(PIX_PLANES_BGR);
  b = a.asBGR(PIX_PLANES_RGB);
  b = a.asYUV(PIX_PLANES_BGR);
  b = a.asYUV(PIX_PLANES_RGB);
  printf("YUV p1=%3d p2=%3d p3=%3d\n", b.p1, b.p2, b.p3);
  c = b.asBGR(PIX_PLANES_YUV);
  c = b.asRGB(PIX_PLANES_YUV);
  printf("RGB p1=%3d p2=%3d p3=%3d\n", b.p1, b.p2, b.p3);
}

//////////////////////////////////////

void testPolyOverlap()
{
  UPolygon40 p1, p2;
  bool overlap;
  double margin = 0.5;
  UPosition x;
  // flat square
  p1.add(0.0, 0.0, 0.0);
  p1.add(5.0, 0.0, 0.0);
  p1.add(5.0, 1.0, 0.0);
  p1.add(0.0, 1.0, 0.0);
  // crossing triangle
  p2.add(2.0, -2.0, 0.0);
  p2.add(3.0, -2.0, 0.0);
  p2.add(3.0,  4.0, 0.0);
  //
  overlap = p1.isOverlappingXYconvex2(&p2, margin, &x);
  //
  // passing triangle
  p2.clear();
  p2.add(2.0, -2.0, 0.0);
  p2.add(3.0, -2.0, 0.0);
  p2.add(6.0,  0.0, 0.0);
  //
  overlap = p1.isOverlappingXYconvex2(&p2, margin, &x);
  //
  printf("finished\n");
}

//////////////////////////////////

void testPolyDistance()
{
  UPolygon40 p1, p2;
  double d;
  UPosition x;
  // flat square
  p1.add(0.0, 0.0, 0.0);
  p1.add(5.0, 0.0, 0.0);
  p1.add(5.0, 1.0, 0.0);
  p1.add(0.0, 1.0, 0.0);
  // crossing triangle
  p2.add(2.0, -2.0, 0.0);
  p2.add(3.0, -2.0, 0.0);
  p2.add(3.0,  4.0, 0.0);
  //
  d = p1.getClosestDistance(0.5, -0.5, 1.0, &x);
  printf("d = %f closest is %fx, %fy\n", d, x.x, x.y);
  d = p1.getClosestDistance(-0.5, -0.5, 1.0, &x);
  printf("d = %f closest is %fx, %fy\n", d, x.x, x.y);
  d = p1.getClosestDistance(-0.5, 0.5, 1.0, &x);
  printf("d = %f closest is %fx, %fy\n", d, x.x, x.y);
  d = p1.getClosestDistance(0.5, -0.5, 1.0, &x);
  printf("d = %f closest is %fx, %fy\n", d, x.x, x.y);
  d = p1.getClosestDistance(-1.5, 0.5, 1.0, &x);
  printf("d = %f closest is %fx, %fy\n", d, x.x, x.y);
  //
  printf("finished\n");
}

//////////////////////////////////////////////////////////////////////

void testVideo3()
{
  bool result = true;
  UImage800 * img8;
  UCamPwc * devPwc;
  UCamDevBase * dev;
  UCamRad * cam;
  UImage * img;
//  int n;
  //
  printf("Testing camera device\n");
  //
  img8 = new UImage800();
  devPwc = new UCamPwc();
  devPwc->setDeviceNumber(0);
  cam = new UCamRad(devPwc);
  //
  printf("Getting camera name ... \n");
  // open device
  dev = cam->getDev();
  result  = dev->setDevice(320, 240, 5);
  result &= dev->openDevice();
  if (result)
    printf("Camera %s open in (%d x %d) \n",
           dev->getCameraName(), dev->getWidth(), dev->getHeight());
  else
    printf("Camera set size and framerate failed\n");
  // try code from videoDog
  if (dev->getLockedNewImage(&img))
  {
    img->valid = false;
    img->unlock();
  }
  if (dev->getLockedNewImage(&img))
  {
    img->valid = false;
    img->unlock();
  }
  if (cam->getImageSnapshot(img8)) //, false))
  {
    //img8->toRGB();
    if (img8->saveBMP("/home/chr/Desktop/frameCamPWC.bmp"))
      printf("Saved OK\n");
    else
      printf("failed to save image\n");
  }
  else
    printf("Failed to get an image\n");
  // dispose used memory
  delete img8;
  delete cam;
  delete dev;
}

//////////////////////////////////////////////////////////////////////
#ifdef USE_IEEE1394

void testVideo1394()
{
  UImage800 * img8;
  UCamDevIeee1394 * dev1394;
  UCamRad * cam = NULL;
  UImage * img = NULL;
  int n, i, c, j, k;
  bool isOK;
  const int MSL = 100;
  char s[MSL];
  UTime t;
  //
  printf("Testing camera device\n");
  //
  img8 = new UImage800();
  dev1394 = new UCamDevIeee1394();
  dev1394->setDeviceNumber(10);
  cam = new UCamRad(dev1394);
  //
  n = dev1394->getIeee1394PortCnt();
  printf("Found %d IEEE1394 interface cards (ports)\n", n);
  for (i =0; i < n; i++)
  {
    c = dev1394->getIeee1394CamCnt(i);
    printf(" - port %d has %d camera node(s)\n", i, c);
    for (j = 0; j < c; j++)
    {
      dev1394->setCamDeviceNode(i, j);
      //dev1394->getCamInfo();
      dev1394->getCamFeatures();
      printf("Start --- '%s'\n", dev1394->getName());
      // set auto gain
      dev1394->setGain(-1);
      dev1394->setShutter(-1);
      // prepare image streaming
      if (false)
      {
        if (false)
        { // format 7
          isOK = dev1394->setFormat7();
          printf("Format 7 started ok=%s\n", bool2str(isOK));
        }
        else
        { // format 0 mode 5 (VGA)
          isOK = dev1394->setFormat0mode5();
          printf("Format 0 mode 5 (VGA) started ok=%s\n", bool2str(isOK));
        }
        if (isOK)
          isOK = dev1394->startIsoTransmission();
        if (isOK)
          printf("Iso transmission started ok=%s\n", bool2str(isOK));
      }
      else
      {
        isOK = dev1394->setDMAcapture();
        printf("Iso DMA transmission started ok=%s\n", bool2str(isOK));
      }
      // get one image
      if (isOK)
      {
        for (k=0; k < 10; k++)
        {
          img8->valid = false;
          t.now();
          isOK = dev1394->getSingleImage(img8);
          // debug
          printf("Got Image (%s) %d took %f to capture\n", bool2str(isOK), k, t.getTimePassed());
          //
          if (img8->valid)
          {
            t.now();
            switch (k % 5)
            {
              case 0:
                img8->toBGR();
                printf("Image %d took %f to convert to BGR\n", k, t.getTimePassed());
                snprintf(s, MSL, "%s/ieee1394-%d-BGR-%d.png", imagePath, j, k);
                break;
              case 1:
                img8->toHalf();
                printf("Image %d took %f to convert to half (bgr)\n", k, t.getTimePassed());
                snprintf(s, MSL, "%s/ieee1394-%d-half-%d.png", imagePath, j, k);
                break;
              case 2:
                img8->toBW();
                printf("Image %d took %f to convert to BW\n", k, t.getTimePassed());
                snprintf(s, MSL, "%s/ieee1394-%d-BW-%d.png", imagePath, j, k);
                break;
              case 3:
                img8->toBW();
                img8->toHalf();
                printf("Image %d took %f to convert to BW and then half\n", k, t.getTimePassed());
                snprintf(s, MSL, "%s/ieee1394-%d-BW-half-%d.png", imagePath, j, k);
              case 4:
                img8->toBGR();
                img8->toBW();
                printf("Image %d took %f to convert to BGR and then BW\n", k, t.getTimePassed());
                snprintf(s, MSL, "%s/ieee1394-%d-BGR-BW-%d.png", imagePath, j, k);
                break;
                break;
            }
            img8->savePNG(s);
          }
        }
      }
      //
      dev1394->stopIsoTransmission();
      printf("Finished --- '%s'\n", dev1394->getName());

    }
  }
  //
//  printf("Get latest cam-info\n");
//  dev1394->getCamInfo();
//  printf("%s\n", dev1394->getName());
  //
  // prepare image streaming
  //dev1394->setFormat7();
  //
  if (cam != NULL)
    delete cam;
  if (dev1394 != NULL)
    delete dev1394;
  if (img != NULL)
    delete img;
  if (img8 != NULL)
    delete img8;
}

#endif

///////////////////////////////////////////////////////////

void testSSS()
{
  const char * val = "12.123456789012345";
  unsigned long sec, usec;

  sscanf(val, "%lu.%6lu", &sec, &usec);
  printf("%lu.%06lu\n", sec, usec);
}

///////////////////////////////////////////////////////////

void testGuppy()
{
  UCamDevGuppy * cg;
  UImage * img;
  //
  img = new UImage800();
  cg = new UCamDevGuppy();
  if (cg != NULL)
  {
#ifdef USE_GUPPY
    int n;
    bool isOK;
    //
    n = cg->getDc1394CamCnt();
    printf("Found %d ieee1394 camera(s)\n", n);
    if (n > 0)
    {
      isOK = cg->setCamDeviceNode(0);
      printf("Set to first device (%s)\n", bool2str(isOK));
      if (isOK)
      {
        isOK = cg->openDevice();
        printf("Started image flow (%s)\n", bool2str(isOK));
      }
      if (isOK)
      {
        isOK = cg->getSingleImage(img);
        printf("Got an image (%s)\n", bool2str(isOK));
        if (isOK)
          img->savePNG("guppy-image.png");
      }
      if (isOK)
      {
        cg->closeDevice();
        printf("Stopped image flow\n");
      }
    }
#endif
    delete cg;
  }
}


