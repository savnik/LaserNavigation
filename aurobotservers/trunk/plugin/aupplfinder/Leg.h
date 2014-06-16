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

//Header file for Leg.cpp
//Defines the Leg-object
 
#ifndef PPL_LEG_H
#define PPL_LEG_H

#include <vector>
#include <list>
#include <ugen4/u3d.h>

class Leg {

    public:
        Leg();
        Leg(int ID, long scanID, double time, std::vector<UPosition> p, bool DEBUG);
        void setColor();
        void setColor(char * color);
        void setColorChar(char c);
        char * getColor();
        char * getColorChar(int i);
        void setTimeStamp(double time);
        double getTimeStamp();
        double getXmean();
        double getYmean();
        int getCertainty();
        void addCertainty(int x, int max);
        void setPosHist(std::list<UPosition> * l);
        std::list<UPosition> * getPosHist();
        int getID();
        void setID(int ID);
        long getScanID();

        char color[8];

    private:
        int ID;
        long scanID;
        double timestamp;
        double Xmean;
        double Ymean;
        std::list<UPosition> posHist;

        int points;
        int certainty;

};

#endif
