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

#include "ufuzzyelement.h"

UFuzzyElement::UFuzzyElement()
{}


UFuzzyElement::~UFuzzyElement()
{}

//////////////////////////////////////////////

double UFuzzyElement::getMembValue(int clust)
{
  printf("UFuzzyElement::no virtual 4\n");
  return 0.0;
}

//////////////////////////////////////////////

bool UFuzzyElement::addPosition(int clust, double m, UMatrix * mAvg, double * membSum)
{
  printf("UFuzzyElement::no virtual 3\n");
  return false;
}

//////////////////////////////////////////////

bool UFuzzyElement::addCovariance(int clust,
                                  double m,
                                  UMatrix * mF,
                                  UMatrix * mV,
                                  double * membSum)
{
  printf("UFuzzyElement::no virtual 3\n");
  return false;
}

//////////////////////////////////////////////

bool UFuzzyElement::setDistance(int clust, UMatrix * mAi, UMatrix * mV)
{
  printf("UFuzzyElement::no virtual 2\n");
  return false;
}

//////////////////////////////////////////////

void UFuzzyElement::updateMembValue(double m, int clustCnt)
{
  printf("UFuzzyElement::no virtual 1\n");
}

///////////////////////////////////////////////

int UFuzzyElement::getVectorSize()
{
  printf("UFuzzyElement::no virtual 5\n");
  return 0;
}

//////////////////////////////////////////////

double UFuzzyElement::getMembValueChange(int clustCnt)
{
  printf("UFuzzyElement::no virtual 6\n");
  return 0.0;
}

//////////////////////////////////////////////

void UFuzzyElement::setMembValue(double value, int clust)
{
  printf("UFuzzyElement::no virtual 7\n");
}

//////////////////////////////////////////////

void UFuzzyElement::seedRandom(int clustCnt)
{
  printf("UFuzzyElement::no virtual 8\n");
}

//////////////////////////////////////////////

int UFuzzyElement::getClass(int clustCnt)
{
  printf("UFuzzyElement::no virtual 9\n");
  return 0;
}

///////////////////////////////////////////////

int UFuzzyElement::getMaxClusterCnt()
{
  printf("UFuzzyElement::no virtual 10\n");
  return 0;
}

////////////////////////////////////////////////

void UFuzzyElement::print(const char * prestring, int clustCnt)
{
  printf("%s, (virtual only)\n", prestring);
}


