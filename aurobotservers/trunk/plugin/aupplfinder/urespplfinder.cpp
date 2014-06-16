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

#include <math.h>

void anyCooPhytagoras(double x1, double x2, double y1, double y2, double * c){
    
    double a, b;
    
    //find a
    if (x1 < 0)
    {
        if(x2 < 0)
        {
            a = fabs(fabs(x1)-fabs(x2));
        }else
        {
            a = fabs(x1-x2);
        }
    }else
        a = fabs(x2-x1);


    //find b
    if (y1 < 0)
    {
        if(y2 < 0)
        {
            b = fabs(fabs(y1)-fabs(y2));
        }else
        {
            b = fabs(y1-y2);
        }
    }else
        b = fabs(y2-y1);

    (*c) = sqrt(a*a + b*b);
}
