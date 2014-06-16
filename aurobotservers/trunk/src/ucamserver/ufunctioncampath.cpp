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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <urob4/uimagepool.h>
#include <umap4/uprobpoly.h>
#include <urob4/usmltag.h>

#include "ufunctioncampath.h"

///////////////////////////////////////////////////

UFunctionCamPath::UFunctionCamPath()
{ // new version tag
  setCommand("pathget", "path", "imagePathFinder ($Id: ufunctioncampath.cpp 59 2012-10-21 06:25:02Z jcan $)");
  imp = NULL;
  grid = NULL;
  ana = NULL;
  cvStorage = NULL;
  initialized = false;
  imgFileLog[0] = '\0';
//  odoFileLog[0] = '\0';
  imgFileSubdir[0] = '\0';
  ballance = 0.43;
  limit = 0.65; // 32.0;
  imgFileNum = -1;
  logPath = NULL;
}

///////////////////////////////////////////////////

UFunctionCamPath::~UFunctionCamPath()
{
  if (logPath != NULL)
    fclose(logPath);
}

///////////////////////////////////////////////////

// const char * UFunctionCamPath::name()
// {
//   return "imagePathFinder V1.80 (by Christian Andersen)";
// }

///////////////////////////////////////////////////

// const char * UFunctionCamPath::commandList()
// {
//   return "pathGet";
// }

///////////////////////////////////////////////////

bool UFunctionCamPath::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  //
  if (msg->tag.isTagA("pathGet"))
    result = handlePathGetCommand(msg, (URawImage *)extra);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFunctionCamPath::handlePathGetCommand(UServerInMsg * msg, URawImage * imgBase)
{ // extract parameters
  bool result = false;
  // decode vars
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 100;
  char attValue[VAL_BUFF_LNG];
  const int MRL = 200;
  char reply[MRL];
  // arameters
  int imgDevice = -1;
  char posName[MAX_MOUNT_NAME_SIZE] = "";
  UCamPush * cam = NULL;
  //URawImage * imgRaw;
  UImage * imgRgb = NULL;
  //bool imgRemRad = false;
  bool pathClear = false;
  bool testBalLim = false;
  char imgFileName[MAX_LIST_FILENAME_SIZE];
  char imgFullPath[MAX_PATH_LENGTH];
  UTime imgTime;
  UPose robPose;
  bool push = false;
  unsigned long imageNumber = 0;
  int imgNumNext = 0;
  int n;
  bool ask4help = false;
  int edgePoolImg = -1;
  double lookingLeft = 0.0;
  bool colorCorrect = false;
  int x1 = -1, y1 = -1, x2 = -1, y2 = -1; // seed area (-1 = default)
  UPosition p1, p2;
  bool gotPos1Pos2 = false;
  float fx, fy;
//  int imgHgt = -1;
  int w,h;
  //
  if (not initialized)
    initialized = initializeFindPath();
  result = initialized;
  //
  if (not result)
    sendWarning(msg, "Could not initialize needed memory");
  else
  { // extract parameters
    while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
    {
      if (strcasecmp(attName, "device") == 0)
        sscanf(attValue, "%d", &imgDevice);
      else if (strcasecmp(attName, "posName") == 0)
        strncpy(posName, attValue, MAX_MOUNT_NAME_SIZE);
      else if (strcasecmp(attName, "help") == 0)
        ask4help = true;
      else if (strcasecmp(attName, "clear") == 0)
        pathClear = true;
      else if (strcasecmp(attName, "imgNum") == 0)
        sscanf(attValue, "%d", &imgFileNum);
      else if (strcasecmp(attName, "next") == 0)
      { // advance M images
        n = sscanf(attValue, "%d", &imgNumNext);
        if (n != 1)
          imgNumNext = 1;
      }
      else if (strcasecmp(attName, "imgLog") == 0)
        strncpy(imgFileLog, attValue, MAX_LIST_FILENAME_SIZE);
      else if (strcasecmp(attName, "subdir") == 0)
        strncpy(imgFileSubdir, attValue, MAX_LIST_FILENAME_SIZE);
//      else if (strcasecmp(attName, "odoLog") == 0)
//        strncpy(odoFileLog, attValue, MAX_LIST_FILENAME_SIZE);
      else if (strcasecmp(attName, "push") == 0)
        push = true;
      else if (strcasecmp(attName, "bal") == 0)
        sscanf(attValue, "%lf", &ballance);
      else if (strcasecmp(attName, "lim") == 0)
        sscanf(attValue, "%lf", &limit);
      else if (strcasecmp(attName, "poolimg") == 0)
        edgePoolImg = strtol(attValue, NULL, 10);
      else if (strcasecmp(attName, "lookingLeft") == 0)
        lookingLeft = strtof(attValue, NULL);
      else if (strcasecmp(attName, "x1") == 0)
        x1 = strtol(attValue, NULL, 10);
      else if (strcasecmp(attName, "y1") == 0)
        y1 = strtol(attValue, NULL, 10);
      else if (strcasecmp(attName, "x2") == 0)
        x2 = strtol(attValue, NULL, 10);
      else if (strcasecmp(attName, "y2") == 0)
        y2 = strtol(attValue, NULL, 10);
      else if (strcasecmp(attName, "irRed") == 0)
        colorCorrect = str2bool(attValue);
      else if (strcasecmp(attName, "px1") == 0)
        p1.x = strtod(attValue, NULL);
      else if (strcasecmp(attName, "py1") == 0)
        p1.y = strtod(attValue, NULL);
      else if (strcasecmp(attName, "pz1") == 0)
        p1.z = strtod(attValue, NULL);
      else if (strcasecmp(attName, "px2") == 0)
        p2.x = strtod(attValue, NULL);
      else if (strcasecmp(attName, "py2") == 0)
      {
        p2.y = strtod(attValue, NULL);
        gotPos1Pos2 = true;
      }
      else if (strcasecmp(attName, "pz2") == 0)
      {
        p2.z = strtod(attValue, NULL);
        gotPos1Pos2 = true;
      }
      else if (strcasecmp(attName, "testBL") == 0)
      {
        if (strlen(attValue) > 0)
          testBalLim = str2bool(attValue);
        else
          testBalLim = true;
      }
/*      else if (strcasecmp(attName, "imgHgt") == 0)
        imgHgt = strtol(attValue, NULL, 0);*/
    }
    // get camera from push-image
  }
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"pathGet\">\n");
    sendText(msg, "---- Available PATHGET options:\n");
    sendText(msg,  "device=N         From device 'N', i.e. /dev/videoN\n");
    sendText(msg,  "posName=name     From camera 'name', e.g. left for left camera\n");
    sendText(msg,  "imgNum=N         From file list (number in list)\n");
    sendText(msg,  "next=N           From file list, advance N images\n");
    sendText(msg,  "clear            Clear all stored path solutions\n");
    sendText(msg,  "imgLog=\"logfile\" Name of filelist (set once)\n");
    snprintf(reply, MRL,  "subdir=\"subpath\" Subpath relative to imagePath (%s)\n", imgFileSubdir);
    sendText(msg, reply);
    sendText(msg,   "push             Send full path image when completed\n");
    snprintf(reply, MRL,  "bal=VAL          Ballance 0=croma, 1=edge, now %g [0..1]\n", ballance);
    sendText(msg, reply);
    snprintf(reply, MRL,  "lim=VAL          limit for load 0=nothing 2=all, now %g [0..2]\n", limit);
    sendText(msg, reply);
    sendText(msg,  "lookingLeft=L    Range from 1.0 (left) to -1.0 (right)\n");
    sendText(msg,  "poolimg=N        Use image N in image pool as source\n");
    sendText(msg,  "x1,y1,x2,y2      Seed sample road xy1=top-left, xy2 = bot-right\n");
    sendText(msg,  "px1,py1,pz1,px2,py2,pz2  Seed segment 3D position\n");
    sendText(msg,  "irRed=true/false Correct for too red image (no IR filter)\n");
    sendText(msg,  "testBL[=false]   Test different values of balance and limit\n");
//    sendText(msg,  "imgHgt=lines     Limit image to top 'Lines' only (disabled)\n");
    sendText(msg,  "---------------------\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else
  {
    if (pathClear)
    { // reset
      imgFileNum = -1;
      sendInfo(msg, "Settings cleared");
    }
    else if (imgFileNum >= 0)
    { // assume image is from file
      if (imgNumNext != 0)
        imgFileNum += imgNumNext;
      //
      // debug
      //const char * odoLog = "odo_20041115_184623.004.log";
      //const char * imageLog = "imagelog20041115_184616.773.log";
      //const char * subdir = "mmr-indoor";
      //
      //const char * odoLog = "odo_20041117_155058.975.log";
      //const char * imageLog = "imagelog20041117_155054.094.log";
      //const char * subdir = "mmr-outdoor";
      //
      //  const char * odoLog = "odo_20041117_154518.807.log";
      // const char * imageLog = "imagelog20041117_154512.962.log";
      // const char * subdir = "mmr-outdoor";
      // debug end
      //
      if (strlen(imgFileSubdir) == 0)
      { // set a default subdir
        strcpy(imgFileSubdir, "mmr-outdoor");
      }
      // debug end
      //
      if (strlen(imgFileLog) == 0)
      { // set a default image list file
        strcpy(imgFileLog, "imagelog20041117_154512.962.log");
      }
  //    if (strlen(odoFileLog) == 0)
  //    { // set a default odometry logfile
  //      strcpy(odoFileLog, "odo_20041117_154518.807.log");
  //    }
      if (result)
      {
        result = getImageName(
                imgFileNum,      // image number in listfile
                imgFileSubdir,     // subdir for images
                imgFileLog, // filenaem of image file list
                imgFileName,          // image filename
                MAX_LIST_FILENAME_SIZE,          // image filename buffer length
                &imgTime,         // image time (from image name)
                &imgDevice,      // alternative to posName
                posName,           // alternative to camDeviceNum
                &imageNumber
                );
      }
      if (result)
      {
        imgRgb = imgPool->getImage(8, true);
        result = (imgRgb != NULL);
      }
      if (result)
      {
        snprintf(imgFullPath, MAX_PATH_LENGTH,
                "%s/%s", imagePath, imgFileSubdir);
        result = imgRgb->loadBMP(imgFullPath, imgFileName, -1, "");
      }
      if (result)
      {
        cam = getCam(imgDevice, posName);
        result = (cam != NULL);
      }
      if (result)
      {
        imgRgb->imgTime = imgTime;
        imgRgb->camDevice = cam->getDev()->getDeviceNumber();
        imgRgb->imageNumber = imageNumber;
      }
    }
    // use image-pool as source
    else if (edgePoolImg >= 0)
    { // get related camera
      cam = getCam(imgDevice, posName);
      // get image
      imgRgb = imgPool->getImage(edgePoolImg, false);
      result = imgRgb != NULL;
      if (result)
      { // set camera and its device number
        imgRgb->cam = (void *)cam;
        imgRgb->camDevice = cam->getDev()->getDeviceNumber();
      }
    }
    else
    { // get image from camera
      imgRgb = imgPool->getImage(8, true);
      result = getCamAndRawImage(&cam,  // result camera                  out
                        &imgRgb,       // result image                   out
                        &imgDevice,    // camera device number           in-out
                        //&imgRemRad,    // should/is radial error remoced in-out
                        imgBase,       // pushed image (YUV)             in
                        posName, imgDevice + 3);      // camera position name           in
      if (result)
        result = (imgRgb != NULL);
      if (result)
        result = imgRgb->toRGB(NULL);
    }
    if (result)
    { // ensure image is in RGB format
      imgRgb->toRGB(NULL);
      imgRgb->imgUpdated();
      // set default values for seed area
      if (x1 < 0)
        x1 = imgRgb->width() / 3;
      if (x2 < 0)
        x2 = (imgRgb->width() / 3) * 2;
      if (y1 < 0)
        y1 = (imgRgb->height() / 3) * 2;
      if (y2 < 0)
        y2 = y1 + 14;
      if (gotPos1Pos2)
      { // cnvert 3D to 2D position in image
        // get deformed pixel position (real image position)
        p1.print("seed pos1");
        p2.print("seed pos2");
        cam->getMtoPix(p1, true, &fx, &fy);
        printf("seed x1,y1: (%.1fx,%.1fy)", fx, fy);
        x1 = roundi(fx);
        y1 = roundi(fy);
        // round to pixel (inside image)
        cam->getMtoPix(p2, true, &fx, &fy);
        printf("x2,y2: (%.1fx,%.1fy)\n", fx, fy);
        x2 = roundi(fx);
        y2 = roundi(fy);
        // take center part only
        w = (x2 - x1)/3; // remove this much each side
        x1 = x1 + w;
        x2 = x2 - w;
        // ensure inside image
        x1 = mini(int(imgRgb->width())-1, maxi(0, x1));
        y1 = mini(int(imgRgb->height())-1, maxi(0, y1));
        x2 = mini(int(imgRgb->width())-1, maxi(0, x2));
        y2 = mini(int(imgRgb->height())-1, maxi(0, y2));
        //
        h = absi(y2 - y1);
        if ((h < 5) or (h > 15))
        { // ensure proper y limits
          h = (y1 + y2) / 2;
          y1 = h - 7;
          y2 = h + 7;
          y1 = mini(int(imgRgb->height())-1, maxi(0, y1));
          y2 = mini(int(imgRgb->height())-1, maxi(0, y2));
        }
      }
      result = absi(x1 - x2) > 10;
      if (not result)
        printf("Seed area not valid (%d,%d) to (%d,%d)\n", x1, y1, x2, y2);
    }
    if (result)
    { // open logfile if needed
      if (testBalLim and (logPath == NULL))
      {
        snprintf(reply, MRL, "%s/pathana.log", imagePath);
        logPath = fopen(reply, "w");
      }
      // find path
      result = findPath(msg, imgRgb,
                        cam, robPose,
                        push, lookingLeft,
                        x1, y1, x2, y2, colorCorrect,
                        testBalLim);
    }
    if (not result)
      // send result to client
      sendWarning(msg, "No solution found");
  }
  //
  return result;
}

/////////////////////////////////////////////////////

bool UFunctionCamPath::findPath(UServerInMsg * msg,
                                UImage * img,
                                UCamPush * cam,
                                UPose robPose, // pose is unknown
                                bool push,
                                double lookingLeft,
                                int x1, int y1, int x2, int y2,
                                bool colorCorrect,
                                bool testBalLim)
{
  //UImage * imgd;
  //UImage * imgo;
  char * imgN = NULL;
  bool result;
  int dev = 0;
  UTime imgTime;
  //double mapCellSize;
  //int seedTopLeft;
  bool found;
  bool debug = false;
  UProbPoly * floorPoly;
  UPose zeroPose(0.0, 0.0, 0.0);
  USmlTag tag;
  UTime t;
  const int MRL = 100;
  char reply[MRL];
  int b,l;
  double ba, lm;
  UImage * imgp;
  //
  // uses poolimg 1, 2, 3 and 9 (9 is result polygon)
  //
  result = (img != NULL);
  //
  // All ready to go
  result = (img != NULL);
  if (result)
  {
    result = false;
    t.Now();
    ana->setImage(img);
    if (ana->imageUsable(img))
    { // should be OK
      dev = img->camDevice;
      //
      if (not testBalLim)
      {
        found = ana->findRoadPoly(debug, dev, imgN, ballance, limit,
                                  lookingLeft, x1, y1, x2, y2, colorCorrect);
      }
      else
      { // test some values around bal and lim
        cam->getCamPar()->setPixelSize(640 / img->width());
        // set camera parameters
        imp->setCamPar(cam->getCamPar());
        imp->setCamPos(cam->getPos(), cam->getRot());
        ba = 0.0;
        floorPoly = imp->getFloorPoly();
        for (b = 0; b < 11; b++)
        {
          lm = maxf(minf(1.5, limit - 0.2), 0.0);
          for (l=0; l < 16; l++)
          {
            found = ana->findRoadPoly(debug, dev, imgN, ba, lm,
                                      lookingLeft, x1, y1, x2, y2, colorCorrect);
            // get image with result
            imgp = imgPool->getImage(9, false);
            if (logPath != NULL)
            { // save image and data
              snprintf(reply, MRL, "%s/bal%g-lim%g.png", imagePath,
                      ba, lm);
              imgp->savePNG( reply);
              floorPoly->clear();
              floorPoly->copy(ana->getFloorPolygon(), ana->getFloorPolygonCnt());
              fprintf(logPath, "%g %g %d %g %d\n",
                      ba, lm,
                      floorPoly->getPointsCnt(),
                      ana->getCromaTraceSD(),
                      roundi(floorPoly->getXYarea()));
            }
            lm += 0.4/15.0;
            printf(".");
          }
          ba += 1.0/10.0;
          printf("\n%g", ba);
        }
      }
      printf("\n");
      //
      if (push)
      { // send image of polygon in image to client (fake as imageGet request)
        sendImage(msg, "imageGet", imgPool->getImage(9, false), 0, 0, 640, 480, 9);
      }
      if (not found)
        printf("Discarded image %lu - no good solution found\n", img->imageNumber);
      if (found)
      { // project to floor and path images
        // set image size for camera parameter to be right
        // (assuming the right camera)
        cam->getCamPar()->setPixelSize(640 / img->width());
        // set camera parameters
        imp->setCamPar(cam->getCamPar());
        imp->setCamPos(cam->getPos(), cam->getRot());
        //cam->getPos().print("Cam Pos");
        //cam->getRot().print("Cam Rot");
        /*
        if (dev == 0)
          // right camera
          imp->setCamPos(0.60, -0.175, 0.86,
                3.0 * PI / 180.0, // rotated on x axis (a bit cv)
                13.0 * PI / 180.0, // looking down (y axis)
                16.0 * PI / 180.0); // looking left
        else
          // left camera - looking right
          imp->setCamPos(0.60,  0.175, 0.86,
                -3.0 * PI / 180.0, // rotated on forward (x) axis (a bit ccv)
                13.0 * PI / 180.0, // looking down
                -16.0 * PI / 180.0); // looking -left (right)
        */
        // project using this color image and path mask
        //imp.doProject(img, imgd);
        //
        // Allocate new polygon slot and set pose and time
        //grid->setNewPolygon(robPose, img->imgTime);
        floorPoly = imp->getFloorPoly();
        floorPoly->clear();
        floorPoly->setPoly(zeroPose, img->imgTime);
        // convert to grid-scale floor-aligned polygon
        //imp->setMapCellSize(grid->getCellSize());
        imp->doProjectPolygonToFloorReal(ana->getFloorPolygon(),
                  ana->getObstacleFlags(),
                  ana->getFloorPolygonCnt(),
                  floorPoly);
        // print timing
        printf("UFunctionCamPath::findPath pathfind analysis time=%.4f sec\n", t.getTimePassed());
        // now a polygon in floor coordinates is found
        // report to client
        //snprintf(reply, MRL, "seedSD=\"%g\"", ana->getCromaTraceSD());
        floorPoly->setCromaSD(ana->getCromaTraceSD());
        tag.sendProbPoly(floorPoly, msg, "floor", "", cmdHandler);
        sendInfo(msg, "done");
        result = true;

        // make probability grid

        //grid->makeProbGrid2(imgN);
        //
        /*
        // now convert using display map cell size
        // convert inage polygon to local robot floor polygon
        imp->setMapCellSize(mapCellSize);
        imp->doProjectPolygonToMapPix(ana->getFloorPolygon(),
                  ana->getFloorPolygonCntP(), ana->getObstacleFlags());
        // paint color map using this polygon
        imp->doPolygonMap(imgo, imgd, ana->getFloorPolygon(),
                  ana->getFloorPolygonCnt(), ana->getObstacleFlags());
        //
        // paint robot position
        if (result)
        {
          imp->paintRobot(imp->getMapColImg(), img->imageNumber);
          imp->paintText(imp->getMapColImg(), 100, 18, msg->message);
        }
        */
        // send to on-line monitoring
        //camSockServ.sendImagePoolUpdated(1, 10, true);
      }
    }
    else
      printf("Discarded image %lu - too bright or too dark\n", img->imageNumber);
    // get next image
/*    if (result)
    { // debug
      printf("Finished analysis of %lu dev:%d, %s result is %s\n",
            img->imageNumber, dev, img->name, bool2str(result));
    }*/
  }
  //
  return result;
}

////////////////////////////////////////////////////////////

bool UFunctionCamPath::initializeFindPath()
{ //
  bool result = true;
  //
  if (cvStorage == NULL)
    cvStorage = cvCreateMemStorage(0);
  if (ana == NULL)
  { // initialize analysis object
    ana = new UImageAna();
    result = (ana != NULL);
    if (result)
    {
      ana->setImagePool(imgPool);
      ana->setStorage(cvStorage);
    }
  }
  if (result and (imp == NULL))
  {
    imp = new UImgProj();
    result = (imp != NULL);
    if (false and result)
    {
      result = imp->setImages(imgPool->getImage(7, true), imgPool->getImage(10, true));
      // clear images (map)
      imp->clearColMap();
      imp->clearPathMap();
      // make 1m grid
      imp->paintGrid(imp->getMapColImg());
    }
  }
  // create gridmap
  if (false and result and (grid == NULL))
  {
    grid = new UProbGrid();
    result = (grid != NULL);
    if (result)
    {
      grid->setGrid(imgPool->getImage(6, true));
      grid->setTempImg(imgPool->getImage(11, true));
    }
  }
  return result;
}

////////////////////////////////////////////////////////////////

bool UFunctionCamPath::getImageName(
               int imgNumber,      // image number in listfile
               const char * subdir,     // subdir for images
               const char * imgFileLog, // filenaem of image file list
               char * imgFile,          // image filename
               int imgFileLng,          // image filename buffer length
               UTime * imgTime,         // image time (from image name)
               int * camDeviceNum,      // alternative to posName
               char * posName,           // alternative to camDeviceNum
               unsigned long * imageNumber // image number
               )
{
  bool result;
  FILE * imgList;
  char fn[MAX_PATH_LENGTH];
  const int MLL = 200;
  char sLine[MLL];
  char s[MLL];
  int i, n;
  unsigned long serial;
  int dev;
  int hdr;
  unsigned long ts, tm;
  char * res;
  //
  // open image listfile
  snprintf(fn, MAX_PATH_LENGTH, "%s/%s/%s",
      imagePath, subdir, imgFileLog);
  imgList = fopen(fn, "r");
  result = (imgList != NULL);
  if (result)
  { // find image name
    i = 0;
    result = false;
    while (true)
    {
      res = fgets(sLine, MLL, imgList);
      if (res == NULL)
        break;
      if (i >= imgNumber)
      {
        n = sscanf(sLine, "%lu %d %d %lu.%lu %s",
          &serial, &dev, &hdr, &ts, &tm, s);
        if (n >= 6)
        { // set image time
          imgTime->setTime(ts, tm * 1000);
          *camDeviceNum = dev;
          // remove ".bmp"
          res = strstr(s, ".bmp");
          if (res != NULL)
            *res = '\0';
          strncpy(imgFile, s, imgFileLng);
          result = true;
          break;
        }
      }
      if (sLine[0] != '#')
        // do not count comments
        i++;
    }
    fclose(imgList);
  }
  return result;
}

////////////////////////////////////////////////////////////////

bool UFunctionCamPath::getRobotPose(
               const char * subdir,     // subdir for images
               const char * odoFileLog, // filenaem of image file list
               UTime imgTime,         // image time (from image name)
               UPose * robPose
               )
{
  bool result;
  FILE * odoLog = NULL;
  char fn[MAX_PATH_LENGTH];
  const int MLL = 200;
  char sLine[MLL];
  int n;
  unsigned long ts, tm;
  UTime t2;
  const double IMAGE_TIME_LAG = 0.2; // lag time for images
  double x, y, h;
  char * res;
  //
  // open odo - logfile
  snprintf(fn, MAX_PATH_LENGTH, "%s/%s/%s",
        imagePath, subdir, odoFileLog);
  odoLog = fopen(fn, "r");
  result = (odoLog != NULL);
  //
  if (result)
  { // now find robot pose at image time
    result = false;
    while (true)
    {
      res = fgets(sLine, MLL, odoLog);
      if (res == NULL)
        break;
      n = sscanf(sLine, "%lu.%lu %lf %lf %lf",
        &ts, &tm, &x, &y, &h);
      if (n == 5)
      {
        t2.setTime(ts, tm);
        if ((t2 - imgTime - IMAGE_TIME_LAG) > 0.0)
        { // use first pose that is about 0.2 sec before imageTime
          robPose->set(x, y, h);
          result = true;
          break;
        }
      }
    }
  }
  if (odoLog != NULL)
    fclose(odoLog);
  return result;
}
//////////////////////////////////////////////////////////////////
