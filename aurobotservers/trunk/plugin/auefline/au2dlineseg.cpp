/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@oersted.dtu.dk
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "au2dlineseg.h"

AU2DLineSeg::AU2DLineSeg()
{
}

//////////////////////////////////////////////////

AU2DLineSeg::~AU2DLineSeg()
{
}

//////////////////////////////////////////////////

char * AU2DLineSeg::toXMLString(char *target, const int targetCnt, const char * extra)
{
  if (extra == NULL)
    snprintf(target, targetCnt, "<line x=\"%g\" y=\"%g\" th=\"%g\" l=\"%g\" msq=\"%g\"/>\n",
            x, y, th, length, resSQ);
  else
    snprintf(target, targetCnt, "<line x=\"%g\" y=\"%g\" th=\"%g\" l=\"%g\" msq=\"%g\" %s/>\n",
             x, y, th, length, resSQ, extra);
  return target;
}

////////////////////////////////////////////////

void AU2DLineSeg::setLine(double vx, double vy, double vth, double vlen, double vresSQ)
{
  x = vx;
  y = vy;
  th = vth;
  length = vlen;
  resSQ = vresSQ;
}

///////////////////////////////////////////////

void AU2DLineSeg::getOtherEnd(double * x2, double * y2)
{
  if (x2 != NULL)
    *x2 = x + length * cos(th);
  if (y2 != NULL)
    *y2 = y + length * sin(th);
}

