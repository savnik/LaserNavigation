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
#include "uclientcams.h"

UClientCamData::UClientCamData()
{
  device = -1;
  width = 0;
  height = 0;
  fps = 0;
  focalLength = 1000;
  name[0] = '\0';
}

///////////////////////////////////

UClientCamData::~UClientCamData()
{
}

///////////////////////////////////

bool UClientCamData::handleCamGet(USmlTag * tag)
{
  bool result;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  int n = 0, m = 0;
  //int w = width, h = height;
  double f = focalLength;
  //double k1 = radialK1, k2 = radialK2;
  double hX = headX, hY = headY;
  //
  tag->reset();
  lock();
  while (tag->getNextAttribute(att, val, MVL))
  {
    n++;
    if (strcasecmp(att, "posx") == 0)
      pos.x = strtod(val, NULL);
    else if (strcasecmp(att, "posy") == 0)
      pos.y = strtod(val, NULL);
    else if (strcasecmp(att, "posz") == 0)
      pos.z = strtod(val, NULL);
    else if (strcasecmp(att, "rotOmega") == 0)
      rot.Omega = strtod(val, NULL);
    else if (strcasecmp(att, "rotPhi") == 0)
      rot.Phi = strtod(val, NULL);
    else if (strcasecmp(att, "rotKappa") == 0)
      rot.Kappa = strtod(val, NULL);
    else if (strcasecmp(att, "width") == 0)
      width = strtol(val, NULL, 0);
    else if (strcasecmp(att, "height") == 0)
      height = strtol(val, NULL, 0);
    else if (strcasecmp(att, "focalLength") == 0)
      focalLength = strtod(val, NULL);
    else if (strcasecmp(att, "fps") == 0)
      fps = strtod(val, NULL);
    else if (strcasecmp(att, "k1") == 0)
      radialK1 = strtod(val, NULL);
    else if (strcasecmp(att, "k2") == 0)
      radialK2 = strtod(val, NULL);
    else if (strcasecmp(att, "name") == 0)
      strncpy(name, val, MAX_NAME_LENGTH);
    else if (strcasecmp(att, "headX") == 0)
      headX = strtod(val, NULL);
    else if (strcasecmp(att, "headY") == 0)
      headY = strtod(val, NULL);
    // else just ignore the attribute
    else
      m++;
  }
  unlock();
  result = (n > m);
  if (result)
  { // some values updated
    updTime.Now();
    if (width > 0) // and (width != w or height != h or k1 != radialK1 or k2 != radialK2 or focalLength != f))
    {
      resFactor = 640.0 / float(width);
      if (focalLength != f)
        // the new focal length should be stored as full resolution focal length
        focalLength *= resFactor;
      if (hX != headX)
        headX *= resFactor;
      if (hY != headY)
        headY *= resFactor;
      setMatrices();
      parValid = true;
    }
  }
  return (result);
}

///////////////////////////////////

char * UClientCamData::snprint(const char * preString, char * buff, const int buffCnt)
{
  char * p1 = buff;
  int n = 0;
  const int MSL = 30;
  char s[MSL];
  //
  snprintf(p1, buffCnt, "%s %s\n", preString, name);
  n += strlen(p1);
  p1 = &buff[n];
  UCamPar::snprint("  -  ", p1, buffCnt - n);
  n += strlen(p1);
  p1 = &buff[n];
  pos.snprint("  -   pos ", p1, buffCnt - n);
  n += strlen(p1);
  p1 = &buff[n];
  rot.snprint("  -   rot ", true, p1, buffCnt - n);
  n += strlen(p1);
  p1 = &buff[n];
  updTime.getTimeAsString(s, true);
  snprintf(p1, buffCnt - n, "  -   size %dx%d at %g fps (updated %s)\n", height, width, fps, s);
  //
  return buff;
}

///////////////////////////////////
///////////////////////////////////
///////////////////////////////////


UClientCams::UClientCams()
{
  int i;
  //
  camsCnt = 0;
  for (i = 0; i < MAX_CLIENT_CAMS; i++)
    cams[i] = NULL;
}

/////////////////////////////////////

UClientCams::~UClientCams()
{
}

////////////////////////////////////

UClientCamData * UClientCams::getCamData(int device, bool mayCreate)
{
  UClientCamData ** camd = NULL;
  UClientCamData * result = NULL;
  int i;
  //
  camd = cams;
  for (i = 0 ; i < camsCnt; i++)
  {
    if ((*camd)->device == device)
    {
      result = *camd;
      break;
    }
    camd++;
  }
  if ((*camd == NULL) and mayCreate and
        (camsCnt < MAX_CLIENT_CAMS))
  {
    *camd = new UClientCamData();
    (*camd)->device = device;
    result = *camd;
    camsCnt++;
  }
  return result;
}

