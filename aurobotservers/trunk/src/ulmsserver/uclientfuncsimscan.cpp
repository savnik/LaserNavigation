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

#include <urob4/usmltag.h>

#include "ulaserdata.h"
#include "uclientfuncsimscan.h"

UClientFuncSimScan::UClientFuncSimScan()
 : UClientFuncBase()
{
  scan = NULL;
}

////////////////////////////////////////////

UClientFuncSimScan::~UClientFuncSimScan()
{
}

////////////////////////////////////////////

const char * UClientFuncSimScan::name()
{
  return "laser_scanner_sim (" __DATE__ " jca@oersted.dtu.dk)";
}

////////////////////////////////////////////

const char * UClientFuncSimScan::commandList()
{
  return "scanget";
}

////////////////////////////////////////////

void UClientFuncSimScan::handleNewData(USmlTag * tag)
{ // distribute to sub-functions
  if (tag->isTagA("scanGet"))
    handleLaserScan(tag);
  else
    printReply(tag, "UClientFuncSimScan::handleNewData: not mine");
}

//////////////////////////////////////////

bool UClientFuncSimScan::handleLaserScan(USmlTag * tag)
{
  enum Units {CM, MM, DM};
  enum Format {TAG, HEX, BIN};
  bool result = true;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  long sec = 0, usec = 0;
  USmlTag binTag;
  int dataFirst = 0;
  int dataCount = 0;
  int dataInterval = 1;
  Units dataUnit = DM;
  Format dataCodex = HEX;
  double dataMin;
  double dataMax;
  UTime t;
  //double intAng = 0.0;
  int n = 0, m, h;
  int binSize, valCnt, got;
  unsigned char lsb, msb;
  unsigned char * binDataSource;
  unsigned long serial = 0;
  //double laserTilt = 0.0;
  // debug
  // <scanget interval="1.0" count="180" tod="63.90000000000064"
  //   unit="mm" min="-90.0" max="90.0" >
  // <bin size="720" codex="HEX">
  // 008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080d805d805d405d405d405d005d005d005d405d405d405d805d805dc05e005e405e805ec05f105f905fd05008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080008000800080
  // </bin>
  // </scanget>

  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "serial") == 0)
      sscanf(val, "%lu", &serial);
    else if (strcasecmp(att, "first") == 0)
    { // from request
      sscanf(val, "%d", &dataFirst);
    }
    else if (strcasecmp(att, "interval") == 0)
    { // used to show that some measurements are skipped
      // interval 1,2,3,...
      // NB! IKKE measurement resolution
      sscanf(val, "%d", &dataInterval);
      dataInterval = maxi(1, dataInterval);
    }
    else if (strcasecmp(att, "count") == 0)
      sscanf(val, "%d", &dataCount);
    else if (strcasecmp(att, "tod") == 0)
      sscanf(val, "%ld.%6ld", &sec, &usec);
    else if (strcasecmp(att, "unit") == 0)
    {
      if (strcasecmp(val, "mm") == 0)
        dataUnit = MM;
      else if (strcasecmp(val, "cm") == 0)
        dataUnit = CM;
      else if (strcasecmp(val, "10cm") == 0)
        dataUnit = DM;
    }
    else if (strcasecmp(att, "min") == 0)
      sscanf(val, "%lf", &dataMin);
    else if (strcasecmp(att, "max") == 0)
      sscanf(val, "%lf", &dataMax);
    else if ((strcasecmp(att, "codex") == 0) or (strcasecmp(att, "codec") == 0))
    {
      if (strcasecmp(val, "TAG") == 0)
        dataCodex = TAG;
      else if (strcasecmp(val, "HEX") == 0)
        dataCodex = HEX;
      else if (strcasecmp(val, "BIN") == 0)
        dataCodex = BIN;
    }
//     else if (strcasecmp(att, "laserTilt") == 0)
//       laserTilt = strtod(val, NULL);
    else if ((strcasecmp(att, "warning") == 0) or
              (strcasecmp(att, "debug") == 0) or
              (strcasecmp(att, "info") == 0))
      result = false;
  }
  //
  if (result)
  { // put data in buffer
    if (scan == NULL)
      printf("UClientSimScan::handleLaserScan: missing data buffer\n");
    else
    {
      t.setTime(sec, usec);
      if (dataCount > 1)
        scan->setAngleResAndStart(dataMin, (dataMax - dataMin)/(dataCount - 1));
      scan->setRangeCnt(dataCount);
      scan->setScanTime(t);
      scan->setUnit(dataUnit);
    }
    // calculate angle interval
//     if (dataCount > 1)
//       intAng = (dataMax - dataMin)/((double)dataCount - 1.0);
//     else
//       intAng = 1.0;
  }
  if ((scan != NULL) and tag->isAStartTag())
  { // there is more - get the rest
    // get next tag - either a measurement or a binary start tag
    result = tag->getNextTag(&binTag, 300);
    scan->lock();
    if (result)
      scan->setSerial(scan->getSerial() + 1);
    n = 0;
    if (result)
    {
      if (binTag.isTagA("bin"))
      { // data format is BIN or HEX
        binSize = 0;
        while (binTag.getNextAttribute(att, val, MVL))
        {
          if (strcasecmp(att, "size") == 0)
            sscanf(val, "%d", &binSize); // size in bytes of 'binary' data
          else if ((strcasecmp(att, "codec") == 0) or (strcasecmp(att, "codex") == 0))
            ; // known already
        }
        if (binSize > 0)
        { // get data
          m = binSize;
          valCnt = 0;
          n = 0;
          while (m > 0)
          {
            got = tag->getNBytes(&val[valCnt], mini(MVL, m), 200);
            if (got == 0)
              break;
            m -= got;
            valCnt += got;
            binDataSource = (unsigned char *)val;
            if (dataCodex == HEX)
            {
              for (h = 0; h < valCnt/4; h++)
              {
                lsb = hex2int(binDataSource[0], binDataSource[1]);
                binDataSource += 2;
                msb = hex2int(binDataSource[0], binDataSource[1]);
                binDataSource += 2;
                scan->setValue(n, lsb, msb);
                n++;
              }
              // any remaining odd data is saved
              if (((valCnt % 4) != 0) and (valCnt > 3))
                // move remaining part to start of buffer
                memcpy(val, &val[valCnt - (valCnt % 4)], valCnt % 4);
              valCnt = valCnt % 4;
            }
            else if (dataCodex == BIN)
            {
              for (h = 0; h < valCnt/2; h++)
              {
                lsb = binDataSource[0];
                msb = binDataSource[1];
                binDataSource += 2;
                scan->setValue(n, lsb, msb);
                n++;
              }
              // any remaining odd data is saved
              if (((valCnt % 2) != 0) and (valCnt > 1))
                // move remaining part to start of buffer
                memcpy(val, &val[valCnt - (valCnt % 2)], valCnt % 2);
              valCnt = valCnt % 2;
            }
          }
        }
        result = tag->getNextTag(&binTag, 300);
        if (result)
        {
          if (not binTag.isTagAnEnd("bin"))
            printf("expected a </bin> tag, got %s\n",
                   binTag.getTagName());
          // fetch also the scanget end tag
          result = tag->getNextTag(&binTag, 300);
        }
      }
      else
        printf("UClientSimScan::handleLaserScan: no recognized data???\n");
      // now is the scanGet end tag the only thing missing
      if (not binTag.isTagAnEnd(tag->getTagName()))
        printf("expected a %s tag, got %s\n",
               tag->getTagName(), binTag.getTagName());
    }
    scan->unlock();
  }
  return result;
}


