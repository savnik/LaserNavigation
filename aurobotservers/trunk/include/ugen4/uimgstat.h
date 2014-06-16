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
#ifndef UIMGSTAT_H
#define UIMGSTAT_H

class UImage;

/**
Hold image statistics for evaluation of image properties.

@author Christian Andersen
*/
class UImgStat
{
public:
  /**
  Constructor */
  UImgStat();
  /**
  Destructor */
  ~UImgStat();
  /**
  Make statistics invalid. */
  inline void clear() { valid = false;};
  /**
  Evaluate statistics */
  bool evaluateStatistics(UImage * rawImg, int iStep = 4);
  /**
  Is statistical data valid. */
  inline bool isValid() { return valid;};
  /**
  Get average intensity */
  inline int getAvgY() { return avgY;};
  /**
  Get maximum intensity */
  inline int getMaxY() { return maxY;};
  /**
  Get average intensity for pixels above yHigh treshold */
  inline int getAvgHighY() { return avgHighY;};
  /**
  Get count of analyzed high intencity pixels */
  inline int getCntHighY() { return cntHigh;};
  /**
  Get average intensity for pixels below yHigh treshold */
  inline int getAvgLowY() { return avgLowY;};
  /**
  Get count of analyzed low intencity pixels */
  inline int getCntLowY() { return cntLow;};
  /**
  Get relation between low and high intensity count values */
  inline int getLowHighRel()
      { return cntLow/(cntHigh + 1); };
private:
  /**
  Highest intensity value */
  int maxY;
  /**
  Is statistics data valid */
  bool valid;
  /**
  Limit between high and low intensity pixels. */
  int yHigh;
  /**
  When calculation statistics, not all pixels
  are used. this 'step' value determins distance
  between pixels both horizontal and vertical. */
  int step;
  /**
  Average intensity (Y) for
  about 1/10 of image pixels */
  int avgY;
  /**
  Average of yntensity pixels above top limit */
  int avgHighY;
  /**
  Count of pxels above limit */
  int cntHigh;
  /**
  Average of intensity for pixels below limit, */
  int avgLowY;
  /**
  Count of used pixels below limit */
  int cntLow;
};

#endif
