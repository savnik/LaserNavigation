/***************************************************************************
 *   Copyright (C) 2011 by Mikkel Viager and DTU                           *
 *   s072103@student.dtu.dk                                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 ***************************************************************************/

// Defines the Leg-Object

#include <list>

#include "Leg.h"
#include "string.h"


Leg::Leg(){}

//Given a vector of Upositions with points defining this leg, create with the
// given values.
Leg::Leg(int ID, long scanID, double time, std::vector<UPosition> p, bool DEBUG)
{
    double tempX = 0.0;
    double tempY = 0.0;
    double count = 0.0;

    //take mean of x and y values
    for (std::vector<UPosition>::size_type i = 0; i !=p.size(); i++)
    {
        tempX += p[i].x;
        tempY += p[i].y;
        count++;
    }

    Xmean = tempX/count;
    Ymean = tempY/count;
    UPosition pos(Xmean, Ymean, 0);
    posHist.push_back(pos);
    timestamp = time;
    points = (int) count;
    setColor();
    certainty = 1;
    this->ID = ID;
    this->scanID = scanID;


    //DEBUG - Print points and mean values to console
    if(DEBUG)
    {
        printf("\nP Leg no. %d\n",ID);
        for (std::vector<UPosition>::size_type j = 0; j != p.size(); j++){
            p[j].print("Point:");
        }
        printf("Mean    x: %f y:  %f\n", Xmean, Ymean);
        printf("Points in leg: %d\n", (int) count);
        printf("Color of leg: %s\n",color);
    }
    //END DEBUG

}
void Leg::setColor()
{
    color[0] = 'r';
    color[1] = '9';
    color[2] = 'o';
    color[3] = 'd';
    color[4] = '"';
    color[5] = '"';
    color[6] = '"';
    color[7] = '\0';
}

void Leg::setColor(char * color)
{
    if(strlen(color) >= 4)
    {
        this->color[0] = color[0];
        this->color[1] = color[1];
        this->color[2] = color[2];
        this->color[3] = color[3];
    }
    else
        printf("ERROR: setColor in Leg had less than 4 characters\n");
}

void Leg::setColorChar(char c)
{
    color[0] = c;
}

char * Leg::getColor()
{
    
    return this->color;
}

char * Leg::getColorChar(int i)
{
    if (i >= 0 && i < 8)
        return &color[i];
    else
        return 0;
}

double Leg::getTimeStamp()
{
    return timestamp;
}

void Leg::setTimeStamp(double time)
{
    timestamp = time;
}

double Leg::getXmean()
{
    return Xmean;
}
double Leg::getYmean()
{
    return Ymean;
}

int Leg::getCertainty()
{
    return certainty;
}

void Leg::addCertainty(int x, int max)
{
    if(certainty + x <= max)
    {
        certainty += x;
    }else
        certainty = max;

}

//update current position, and store previous in history vector
void Leg::setPosHist(std::list<UPosition> * l)
{
    std::list<UPosition>::iterator iter = l->begin();
    while(iter != l->end())
    {
        UPosition pos = *iter;
        posHist.push_back(pos);
        iter++;
    }
}

std::list<UPosition> * Leg::getPosHist()
{
    return &posHist;
}

int Leg::getID()
{
    return ID;
}

void Leg::setID(int ID)
{
    this->ID = ID;
}

long Leg::getScanID()
{
    return scanID;
}
