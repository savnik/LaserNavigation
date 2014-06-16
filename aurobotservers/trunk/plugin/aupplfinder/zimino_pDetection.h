/***************************************************************************
 *   DMZ - Headerfile til plugin til menneske-genkendelse                  *
 *   s011359@student.dtu.dk                                                *
 *                                                                         *
 ***************************************************************************/

 
#ifndef UFUNC_COG_H
#define UFUNC_COG_H

#include <cstdlib>
#include <urob4/ufunctionbase.h>
#include <ulms4/ulaserpool.h>
#include <vector>
#include "Leg.h"

  /**
  Function to get closest distance */
  bool handlePeopleDetection(ULaserData * pushData, std::vector<std::vector<UPosition> > * zLegsAsPoints);
  
#endif
