/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#ifdef OPENCV2
#include <legacy/compat.hpp>
#endif

#include <cstdlib>
#include <stdlib.h>

#include <urob4/usmltag.h>
#include <ugen4/ufuzzypixel.h>
#include <ugen4/ufuzzysplit.h>
#include <ucam4/ufunctioncambase.h>


/**
Example plugin to find balls in camera image
@author Christian Andersen
*/
class UFuncFz : public UFunctionCamBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunc followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncFz()
  {
    setCommand("fz", "fz", "Fuzzy image classifier - road finder (compiled " __DATE__ " " __TIME__ ")");
    // create global variables
    createBaseVar();
    // initialize local variables
    fuzzy = NULL;
    imgBuf = NULL;
    for (int n = 0; n < MES; n++)
    { // init pointer array
      ems1[n] = NULL;
      ems2[n] = NULL;
      ems3[n] = NULL;
      ems4[n] = NULL;
    }
    seed = NULL;
    seedCnt = 0;
    seedCntMax = 0;
    debug = true;
    classesCnt = 4;
  };
  /**
  Destructor - to delete the resource (etc) when finished */
  virtual ~UFuncFz()
  { // possibly remove allocated variables here - if needed
    int n;
    for (n = 0; n < MES; n++)
    {
      if (ems1[n] != NULL)
        delete ems1[n];
      else
        break;
    }
    for (n = 0; n < MES; n++)
    {
      if (ems2[n] != NULL)
        delete ems2[n];
      else
        break;
    }
    for (n = 0; n < MES; n++)
    {
      if (ems3[n] != NULL)
        delete ems3[n];
      else
        break;
    }
    for (n = 0; n < MES; n++)
    {
      if (ems4[n] != NULL)
        delete ems4[n];
      else
        break;
    }
    if (seed != NULL)
    {
      for (int n = 0; n < seedCntMax; n++)
      {
        if (seed[n] != NULL)
          delete seed[n];
      }
    }
  }
  /**
  Handle incomming command
  Must return true if the function is handled -
  otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra)
  { // handle command(s) send to this plug-in
    const int MRL = 2000;
    char reply[MRL];
    bool ask4help;
    const int MVL = 50;
    char val[MVL];
    int camDevice = -1;
    int imgPoolNum = -1;
    UImage * img = (UImage *)extra;
    USmlTag tag;
    UCamPush * cam = NULL;
    bool result;
    bool smrcl = false; // default is <ball ...> reply
    bool gotBlue = false;
    // check for parameters - one parameter is tested for - 'help'
    // the help value is ignored, e.g. if help="bark", then
    // the value "bark" will be in the 'helpValue' string.
    ask4help = msg->tag.getAttValue("help", val, MVL);
    if (not ask4help)
    { // get all other parameters
      msg->tag.getAttValueInt("device", &camDevice);
      msg->tag.getAttValueInt("img", &imgPoolNum);
      msg->tag.getAttValueBool("debug", &debug, true);
      msg->tag.getAttValueBool("smrcl", &smrcl, true);
      msg->tag.getAttValueBool("blue", &gotBlue, true);
    }
    // ask4help = false, if no 'help' option were available.
    if (ask4help)
    { // create the reply in XML-like (html - like) format
      sendHelpStart("FZ");
      sendText("--- available FuzzyClassifier (FZ) options\n");
      
      sendText("device=X          Use this camera - for position and parameters\n");
      sendText("img=X             Get image from this image pool image\n");
      sendText("init              Initialize new classes from seed-line\n");
      sendText("continue          Reuse classes from last solution (default)\n");
      sendText("iterate           Iterate on last solution (do not initialize)\n");
      sendText("debug=false       More images and print on server console (def=true)\n");
      sendText("help              This message\n");
      sendHelpDone();
      sendInfo("done");
      result = true;
    }
    else
    { // resource is available, so make a reply
      if (imgPoolNum < 0)
        imgPoolNum = varSrcImg->getInt();
      if (img == NULL)
      { // get source image from image pool
        img = imgPool->getImage(imgPoolNum, false);
      }
      result = (img != NULL);
      if (result and camDevice < 0)
        //take image device from image
        camDevice = img->camDevice;
      if (result)
      {
        if (camDevice < 0)
          camDevice = img->camDevice;
        cam = camPool->getCam(camDevice);
        result = (cam != NULL);
      }
      // camera and image is now available
      if (result)
      { // there is an image, make the required classifier
        int iter=-1;
        bool init = msg->tag.getAttValue("init", NULL, 0); // reseed all classes from line
        bool cont = msg->tag.getAttValue("continue", NULL, 0); // continue with new image
        bool gotiter = msg->tag.getAttValueInt("iterate", &iter); // just iterate old data
        if (iter <= 0)
          iter = varFzIter->getInt();
        doClassify(img, init or not (gotiter or cont), cont and not gotiter, iter);
      }
      else
      {
        snprintf(reply, MRL, "failed, got image %s, got camera %d %s\n",
                bool2str(img != NULL), camDevice, bool2str(cam != NULL));
        sendWarning(msg, reply);
      }
    }
    // return true if the function is handled with a positive result
    return result;
  }

protected:

/////////////////////////////////////////////////////////////

/**
 * Classify into 4 classes - road, not road, dark and bright.
 * \param imgsrc is source image in any format
 * \param init should classes be initialized from line-column marking of road.
 * \param newdata should new data be loaded into classifier
 * \param iter should result be iterated - this number of times
 * */
void doClassify(UImage * imgsrc, bool init, bool newdata, int iter)
{
  UImage * imgfz = NULL;
  if (imgsrc != NULL)
  { // zero histogram
    imgsrc->lock();
    // is image in a usable format
    if (not (imgsrc->isBGR() or imgsrc->isRGB() or imgsrc->isYUV()))
    { // a conversion is needed
      if (imgBuf == NULL)
        imgBuf = new UImage();
      imgsrc->toRGB(imgBuf);
      imgsrc->unlock();
      imgsrc = imgBuf;
      imgsrc->lock();
    }
    // make YUV image copy
    imgfz = imgPool->getImage(varTmpImg->getInt(), true);
    //imgfz->copy(imgsrc);
    switch (varFzFormat->getInt())
    {
      case 0:
        imgsrc->toCromaBGR(imgfz);
        break;
      case 1:
        imgsrc->toYUV(imgfz);
        break;
      default:
        imgsrc->toRGB(imgfz);
    }
    imgfz->updated();
    // make sure we have a classifier
    if (fuzzy == NULL)
      fuzzy = new UFuzzySplit();
    // do fuzzy classify on yuv
    if (init)
    {
      classesCnt = 4;
      initializeClassesFromLine(imgfz);
    }
    if (newdata or init)
    {
      if (not init)
        fuzzy->clear();
      // load sample data from 
      loadNewImage(imgfz);
    }
    if (iter > 0)
    {
      // do the required iterations
      fuzzy->classify(classesCnt, 0.01, iter, false);
      // show result count
      fuzzy->countMembers();
      for (int n = 0; n < classesCnt; n++)
        printf("Cluster %d has %d members\n",
              n, fuzzy->getMembCount(n));
    }
    if (debug)
    { // get debug image
      UImage * imgDbg = imgPool->getImage(varTmpImg->getInt() + 1, true);
      paintColoredImage(imgsrc, imgfz, imgDbg);
      if (init)
      { // paint also initialization line
        CvPoint p1, p2;
        CvScalar greenYUV = CV_RGB(255, 10, 10);
        CvScalar greenRGB = CV_RGB(30, 255, 30);
        CvScalar * green = &greenRGB;
        if (imgsrc->isYUV())
          green = &greenYUV;
        p1.x = 0;
        p1.y = varFzRoadLine->getInt();
        p2.x = imgDbg->getWidth() - 1;
        p2.y = p1.y;
        cvLine(imgDbg->cvArr(), p1, p2, *green, 1, 8,0);
        p1.x = varFzRoadCol->getInt();
        p1.y = mini(maxi(0, varFzRoadLine->getInt() - 10), imgDbg->height() - 1);
        p2.x = p1.x;
        p2.y = mini(maxi(0, p1.y + 20), imgDbg->height() - 1);
        cvLine(imgDbg->cvArr(), p1, p2, *green, 1, 8,0);
      }
    }
    imgsrc->unlock();
  }
}

/**
 * Paint the result in a debug image of same size as source image, with two main 
 * classes as red (road) and blue (blue) and all other pixels as the original image
 * \param imgsrc is original image
 * \param imgfz is image used for classify
 * \param imgDbg is debug image to paint. */
void paintColoredImage(UImage * imgsrc, UImage * imgfz, UImage * imgDbg)
{
  if (imgfz != NULL)
  { // paint the result
    UPixel *pfz, *pcd, *psrc, *ppix;
    UPixel pixBlue(255, 0, 0);
    UPixel pixRed(0, 0, 255);
    UPixel pixBlueD(155, 0, 0);
    UPixel pixRedD(0, 0, 155);
    int cl;
    //
    imgDbg->lock();
    imgDbg->copyMeta(imgsrc, true);
    //
    for (int r = 0; r < (int)imgsrc->height(); r++)
    {
      pfz = imgfz->getLine(r);
      pcd = imgDbg->getLine(r);
      psrc = imgsrc->getLine(r);
      //
      for (int c = 0; c < (int)imgsrc->width(); c++)
      { // paint small copy in top-right corner
        cl = getPixelClass(*pfz, classesCnt);
        // paint class 0 and 1 only
        switch (cl)
        {
          case 0: ppix = &pixBlueD; break;
          case 1: ppix = &pixRedD; break;
          // paint CROMA image as small image
          default: ppix = psrc; break;
        }
        // copy pixel to UV image
        *pcd++ = *ppix;
        pfz++;
        psrc++;
      }
    }
//     // paint circle around mouse location
//     pos.x = globalPixPoint2.x / redFac + MWH - 180 - 10;
//     pos.y = globalPixPoint2.y / redFac + 10;
//     cvCircle(imgUV->cvArr(), pos, 5, white, 2+lw, 8, 0);
//     cvCircle(imgUV->cvArr(), pos, 5, red, lw, 8, 0);
//     pos.x = mini(imgsrc->width()-1, maxi(0, globalPixPoint2.x));
//     pos.y = mini(imgsrc->height() - 1, maxi(0, globalPixPoint2.y));
//     bgr = imgsrc->getPixRef(pos.y, pos.x);
//     if (bgr != NULL)
//     {
//       pix = bgr->asYUV(imgsrc->getColorType());
//       if (not asCROMA)
//       {
//         pos.x = mini(255, maxi(0, (pix.u - 128) * 1 + 128)) * 2;
//         pos.y = (255 - mini(255, maxi(0, (pix.v - 128) * 1 + 128))) * 2;
//       }
//       else
//       {
//         d = float(bgr->p1 + bgr->p2 + bgr->p3);
//         x = float(bgr->getRed(imgsrc->getColorType())) / d;
//         y = float(bgr->getGreen(imgsrc->getColorType())) / d;
//         pos.x = mini(MWH - 2, maxi(0, roundi(x * MWH)));
//         pos.y = MWH - 2 - mini(MWH - 2, maxi(0, roundi(y * MWH)));
//       }
//       hyc = roundi((bgr->p1 + bgr->p2 + bgr->p3) / 3.0);
//       cvCircle(imgUV->cvArr(), pos, 5, white, 1+lw, 8, 0);
//       cvCircle(imgUV->cvArr(), pos, 5, red, lw, 8, 0);
//     }

    // fuzzy
//     if (fuzzy != NULL)
//     { // paint result of fuzzy
//       paintClustPixels(true,
//                         imgfz,
//                         imgUV);
//       for (cl = 0; cl < classesCnt; cl++)
//       {
//         paintClustEllipse(true, fuzzy->getV(cl),
//                           fuzzy->getF(cl),
//                           imgUV, cl);
//       }
//     }
    imgDbg->updated();
    imgDbg->unlock();
    imgDbg->used=0;
  }
}

/////////////////////////////////////////////////////

bool initializeClassesFromLine(UImage * imgfz)
{
  int r, c, n, sr, sc;
  int ems1Cnt = 0;
  int ems2Cnt = 0;
  int ems3Cnt = 0;
  int ems4Cnt = 0;
  //
  UPixel *fzpix;
  UPixel * p;
  //
  fuzzy->clear();
  // allocate elements
  sr = mini(maxi(1, varFzRoadLine->getInt()), imgfz->height()-1);
  sc = mini(maxi(0, varFzRoadCol->getInt()) , imgfz->width());
  for (r = -1; r < 2; r++)
  {
    p = &imgfz->getLine(sr+r)[sc];
    //
    fzpix = imgfz->getLine(sr + r);
    for (c = 0; c < (int)imgfz->width(); c++)
    {
      if ((absi(fzpix->p1 - p->p1) < 20) and
          (absi(fzpix->p2 - p->p2) < 20) and 
          (absi(fzpix->p3 - p->p3) < 20))
      { // road pixel
        if (ems1Cnt < MES)
        {
          if (ems1[ems1Cnt] == NULL)
            ems1[ems1Cnt] = new UFuzzyPixel();
          ems1[ems1Cnt++]->setPixel(fzpix);
        }
      }
      else
      {
        int intens;
        if (imgfz->isYUV())
          intens = fzpix->p1;
        else if (imgfz->isBGR()) 
          // is actually cromaticity
          intens = fabs(fzpix->p1 - fzpix->p2) + fabs(fzpix->p2 - fzpix->p3);
        else
          // is RGB
          intens = fzpix->getSum() / 3;
        if (intens < 30)
        { // dark group
          if (ems3Cnt < MES)
          {
            if (ems3[ems3Cnt] == NULL)
              ems3[ems3Cnt] = new UFuzzyPixel();
            ems3[ems3Cnt++]->setPixel(fzpix);
          }
        }
        else if (intens > 235)
        { // bright
          if (ems4Cnt < MES)
          {
            if (ems4[ems4Cnt] == NULL)
              ems4[ems4Cnt] = new UFuzzyPixel(); 
            ems4[ems4Cnt++]->setPixel(fzpix);
          }
        }
        else
        { 
          if (ems2Cnt < MES)
          {
            if (ems2[ems2Cnt] == NULL)
              ems2[ems2Cnt] = new UFuzzyPixel();
            ems2[ems2Cnt++]->setPixel(fzpix);
          }
        }
      }
      fzpix++;
    }
  }
  for (int i = ems2Cnt; i < 4; i++)
  {
    UPixel pix1(maxi(0, p->p1 - 20 -i), maxi(0, p->p2 - 20 - i/3), maxi(0, p->p3 - 20+i/2));    
    UPixel pix2(mini(255, p->p1 + 20), mini(255, p->p2 + 20 + i), mini(255, p->p3 + 20 - i/2));    
    if (ems2[ems2Cnt] == NULL)
      ems2[ems2Cnt] = new UFuzzyPixel();
    ems2[ems2Cnt++]->setPixel(&pix1);
    if (ems2[ems2Cnt] == NULL)
      ems2[ems2Cnt] = new UFuzzyPixel();
    ems2[ems2Cnt++]->setPixel(&pix2);
  }
  for (int i = ems3Cnt; i < 6; i++)
  {
    UPixel pix(i / 3, i / 2, 6 - i);    
    if (ems3[ems3Cnt] == NULL)
      ems3[ems3Cnt] = new UFuzzyPixel();
    ems3[ems3Cnt++]->setPixel(&pix);
  }
  if (ems4Cnt == 0)
    classesCnt = 3;
  else
    for (int i = ems4Cnt; i < 6; i++)
    {
      UPixel pix(250 + i/3, 255-i/2, 255-i);    
      if (ems4[ems4Cnt] == NULL)
        ems4[ems4Cnt] = new UFuzzyPixel();
      ems4[ems4Cnt++]->setPixel(&pix);
    }
  // any values available?
  printf("Class 0 is based on %d pixels (road)\n", ems1Cnt);
  printf("Class 1 is based on %d pixels (not road)\n", ems2Cnt);
  printf("Class 2 is based on %d pixels (dark)\n", ems3Cnt);
  if (classesCnt > 3)
    printf("Class 3 is based on %d pixels (bright)\n", ems4Cnt);
  //
  if (ems1Cnt > 5) // ((ems1Cnt + ems3Cnt) > 5) and ((ems2Cnt + ems4Cnt) > 10))
  {
    // add elements to fuzzy splitter
    for (n = 0; n < ems1Cnt; n++)
      fuzzy->addElement(ems1[n]);
    for (n = 0; n < ems2Cnt; n++)
      fuzzy->addElement(ems2[n]);
    for (n = 0; n < ems3Cnt; n++)
      fuzzy->addElement(ems3[n]);
    for (n = 0; n < ems4Cnt; n++)
      fuzzy->addElement(ems4[n]);
    // initialize classes from these values
    // road
    fuzzy->initFromValues(0, 0, ems1Cnt);
    // not road
    fuzzy->initFromValues(1, ems1Cnt, ems2Cnt);
    // (dark)
    fuzzy->initFromValues(2, ems1Cnt + ems2Cnt, ems3Cnt);
    if (ems4Cnt > 0)
      // (bright)
      fuzzy->initFromValues(3, ems1Cnt + ems2Cnt + ems3Cnt, ems4Cnt);
  }
  else
    printf("Not enough valid values\n");
  return ems1Cnt > 5;
}


/**
 * Load new data into fuzzy classifier from this source image */
void loadNewImage(UImage * imgfz)
{
  int scale = varRedFac->getInt(); // scaled down image
  int r,c;
  int cnt = imgfz->height() / scale * imgfz->width() / scale;
  //
  if (cnt > seedCntMax)
  {
    int bytes = cnt * sizeof(UFuzzyPixel*);
    if (seed == NULL)
      seed = (UFuzzyPixel**)malloc(bytes);
    else    
      seed = (UFuzzyPixel**)realloc(&seed, bytes);
    for (int i=seedCntMax; i < cnt; i++)
      seed[i] = NULL;
    seedCntMax = cnt;
  }
  fuzzy->clear();
  seedCnt = 0;
  // create element list
  for (r = scale/2; r < (int)imgfz->height(); r += scale)
  {
    for (c = scale/2; c < (int)imgfz->width(); c += scale) 
    {
      if (seed[seedCnt] == NULL)
        seed[seedCnt] = new UFuzzyPixel();
      seed[seedCnt++]->setPixel(imgfz->getPixRef(r, c));
    }
  }
  // load element list to fuzzy classifier
  for (int n = 0; n < seedCnt; n++)
    fuzzy->addElement(seed[n]);
}


//////////////////////////////////////////////////

int getPixelClass(UPixel src, int clustCnt)
{
  UFuzzyPixel pe;
  int cl;
  UPixel pix;
  //
  pix = src;
  // make classifier element from pixel
  pe.setPixel(&pix);
  // get class
  cl = fuzzy->updateElement(&pe, clustCnt);
  //
  return cl;
}

///////////////////////////////////////////////////

UPixel getCroma(UPixel * src, int sourceFormat)
{
  UPixel bgr;
  UPixel pix;
  int sum;
  //
  if (sourceFormat != PIX_PLANES_BGR)
    bgr = src->asBGR(sourceFormat);
  else
    bgr = *src;
  //
  sum = bgr.p1 + bgr.p2 + bgr.p3;
  pix.p1 = (255 * bgr.p3) / sum; // red
  pix.p2 = (255 * bgr.p2) / sum; // green
  pix.p3 = (255 * bgr.p1) / sum; // blue
  //
  return pix;
}

/////////////////////////////////////////////////////////////

void paintClustPixels(bool isUV,
                                UImage * imgsrc,
                                UImage * imgUV
                              )
{
  UPixel * yuv;
  UFuzzyPixel pe;
  int cl, r, c;
  UPixel pixBlue(255, 0, 0);
  UPixel pixRed(0, 0, 255);
  UPixel pixBlueD(155, 0, 0);
  UPixel pixRedD(0, 0, 155);
  CvPoint pos = {0,0};;
  float x,y;
  const int MWH = 512;
  //
  for (r = 0; r < int(imgsrc->height()); r ++)
  {
    yuv = imgsrc->getLine(r);
    for (c = 0; c < int(imgsrc->width()); c++)
    {
      cl = getPixelClass(*yuv, classesCnt);
      if ((cl != 2) or (classesCnt > 3))
      { // paint all pixels that belong to cluster 0 and 1
        if (isUV)
        { // get position directly from U and V
          pos.x = mini(255, maxi(0, (yuv->u - 128) * 1 + 128)) * 2;
          pos.y = (255 - mini(255, maxi(0, (yuv->v - 128) * 1 + 128))) * 2;
        }
        else
        { // convert to cromaticity
          x = float(yuv->p3) / 256.0;
          y = float(yuv->p2) / 256.0;
          pos.x = mini(MWH - 2, maxi(0, roundi(x * MWH)));
          pos.y = MWH - 2 - mini(MWH - 2, maxi(0, roundi(y * MWH)));
        }
        switch (cl)
        {
          case 0: imgUV->setPix(pos.y, pos.x, pixBlueD); break;
          case 1: imgUV->setPix(pos.y+1, pos.x, pixRedD); break;
          case 2: imgUV->setPix(pos.y, pos.x+1, pixBlue); break;
          case 3: imgUV->setPix(pos.y+1, pos.x+1, pixRed); break;
          default: imgUV->setPix(pos.y+1, pos.x+1, pixRed); break;
        }
      }
      yuv++;
    }
  }
}

////////////////////////////////////////////////////////////

void paintClustEllipse(bool isUV,
                                 UMatrix * mV,
                                 UMatrix * mQ,
                                 UImage * img,
                                 int clust
                                         )
{ // paint error ellipse with center point
  // x,y if within [0-w, 0-h]
  UMatrix4 Q(2,2);
  UMatrix4 E(2,1);
  UMatrix4 eig(2, 1); // eigenvalues
  UMatrix4 eigVec(2,2);// eigenvectors (columns)
  bool isComplex; // matrix has complex eigenvalues
  double p, a, b;
  int r, c;
  // light blue
  CvScalar col = CV_RGB(50, 0, 0);
  //CvScalar red = CV_RGB(100, 100, 255);
  CvPoint pos;
  CvSize sz;
  //
  switch (clust)
  { // paint in light red
    case 0: col = CV_RGB(50, 0, 0); break;
    case 1: col = CV_RGB(0, 0, 50); break;
    case 2: col = CV_RGB(100, 0, 0); break;
    case 3: col = CV_RGB(0, 0, 100); break;
    default: col = CV_RGB(0, 0, 0); break;
  }
  if (isUV or true)
  { // move to reduced matrix/vector.
    for (r = 1; r < int(mV->rows()); r++)
      E.setRC(r-1, 0, mV->get(r,0));
    for (r = 1; r < int(mQ->rows()); r++)
      for (c = 1; c < int(mQ->cols()); c++)
        Q.setRC(r-1, c-1, mQ->get(r,c));
  }
  // get center position
  pos.x = mini(255, maxi(0, (roundi(E.get(0,0)) - 128) * 1 + 128)) * 2;
  pos.y = (255 - mini(255, maxi(0, (roundi(E.get(1,0)) - 128) * 1 + 128))) * 2;
  // get ellipse data
  eig = Q.eig2x2(&isComplex, &eigVec);
  if (fabs(eig.get(1)) < fabs(eig.get(0)))
  { // first eigenvalue is the larger
    p = atan2(eigVec.get(1,0),eigVec.get(0,0));
    a = sqrt(eig.get(0)); // first eigenvalue
    b = sqrt(eig.get(1)); // second eigenvalue
  }
  else
  { // second eigenvalue is the larger
    p = atan2(eigVec.get(1,1),eigVec.get(0,1));
    a = sqrt(eig.get(1));
    b = sqrt(eig.get(0));
  }
  sz.width = roundi(a*3.0);
  sz.height = roundi(b*3.0);
  cvEllipse( img->cvArr(), pos, sz, p * 180.0 / M_PI,
             0.0, 360.0, col, 1, 4, 0);
}

///////////////////////////////////////////


  /**
  Make the variables that will be available to other plugins */
  void createBaseVar()
  {
    varSrcImg = addVar("srcImg", 18.0, "d", "(rw) source image for classifier");
    varTmpImg = addVar("tmpImg", 25.0, "d", "First tmporary image pool image to use");
    varRedFac = addVar("redFac", 3.0, "d", "Use every this many pixels in row and column");
    varFzFormat = addVar("format", 0.0, "d", "(rw) color format to use 0=croma, 1=yuv, 2=RGB");
    //varFzSize = addVar("size", 2.0, "d", "(rw) half size of seed area (2=>5x5 area)");
    //varFzClasses = addVar("fzClasses", 3.0, "d", "(rw) Number of classes (if 3, then 2 from seed and one background)");
    varFzIter = addVar("fzIterations", 2.0, "d", "(rw) Number if iterations to get best clusters (0=use seed only)");
    varFzRoad = addVar("fzRoad", 1.0, "d", "(rw) (rw) look for road using a line in image (0=use mouse)");
    varFzRoadLine = addVar("fzRoadLine", 200.0, "d", "(rw) Line in image (from top) that intersects road");
    varFzRoadCol = addVar("fzRoadCol", 200.0, "d", "(rw) Column on line (from left) that is assumed to be road");
  }
  //
private:
  /**
  Fuzzy classifier */
  UFuzzySplit * fuzzy;

  /// fuzzy seed size
  //UVariable * varFzSize;
  /// number of classes
//  UVariable * varFzClasses;
  /// fuzzy iterations
  UVariable * varFzIter;
  /// looking for road drom line in image
  UVariable * varFzRoad;
  /// looking for road using this line
  UVariable * varFzRoadLine;
  /// road is in this column (on line)
  UVariable * varFzRoadCol;
  /// source image
  UVariable * varSrcImg;
  /// temporary image pool image to use
  UVariable * varTmpImg;
  /// format to classify in
  UVariable * varFzFormat;
  /// format to classify in
  UVariable * varRedFac;
  /// 
  int classesCnt;
  UImage * imgBuf;
  static const int MES = 1000;
  ///values for classification seed
  /// road
  UFuzzyPixel * ems1[MES];
  /// not road
  UFuzzyPixel * ems2[MES];
  /// darkish
  UFuzzyPixel * ems3[MES];
  /// brightish
  UFuzzyPixel * ems4[MES];
  /// brightish
  UFuzzyPixel ** seed;
  int seedCnt; // number of used seed values
  int seedCntMax;
  /// debug flag - for more printout
  bool debug; // default is debug on
};


#ifdef LIBRARY_OPEN_NEEDED
UFunctionBase * createFunc()
{ // called by server to create an object of this type
  /** replace 'UFuncFz' with your classname, as used in the headerfile */
  return new UFuncFz();
}
#endif
