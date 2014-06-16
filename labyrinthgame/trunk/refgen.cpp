
#include <math.h>
#include <stdlib.h>
#include <ugen4/uline.h>
#include "refgen.h"
#include "tiltcontrol.h"

/// is last position (carrot point) valid
bool lastPosInit = false;
/// segment index of last point
int lastPosIdx;
/// position on segment
double lastPosM;
/// last line segment
U2Dseg lastSeg;


/* ****************************************************************
 * Navn       : InitRefGen
 * Funktion   : Initialiserer variablene i referencegenerator-
 *              struct
 * Kald       : -
 * Ind        : \param *rg    - pointer til struct med referencegenerator-
 *                       variable
 *              \param xstart - x-koordinat paa foerste punkt i ruten
 *              \param ystart - y-koordinat paa foerste punkt i ruten
 * Ud         : -
 * Reference  : -
 * Skrevet af : Allan Theill Sorensen
 * Dato       : 9/7 1990
 ********************************************************************/
void InitRefGen(refgen_parm *rg, float xstart, float ystart, float hast)
{
  rg->tstep = 1.0;
  rg->xref = xstart;
  rg->yref = ystart;
  rg->xlp = xstart;
  rg->ylp = ystart;
  rg->forward = true;
  rg->hast = hast;
}

/* *****************************************************************
 * Navn       : RefGen
 * Funktion   : Genererer referencevaerdier udfra array af knaek-
 *              punkter i referencen og udfra struct med parametre
 *              for referencegenereringen
 * Kald       : -
 * Ind        : \param *rg   - pointer til struct med parametre til refe-
 *                      rencegenereringen
 *              \param ref[] - array med knaekpunkter for referencen
 *              \param i     - indeks til oejeblikkelige placering i ref[]
 *                      saaledes at referencen er paa vej fra
 *                      punkt i-1 til punkt i
 *              \param itot  - antal punkter i alt
 * \returns     true hvis referencen har naaet sidste punkt i ref[]
 *              ellers false
 * Reference  : -
 * Skrevet af : Allan Theill Sorensen
 * Dato       : 9/7 1990
 ********************************************************************/
int RefGen(refgen_parm *rg, int ref[][2], int *i, int itot)
{
  int returnValue;

  returnValue = false; // true if finished

  rg->tstep--;
  if (rg->tstep <= 0)
  { // no more steps left - advance to next segment
    rg->xref = (int)ref[*i][0];
    rg->yref = (int)ref[*i][1];
    if( rg->forward == true )
    { // normal situation
      (*i)++;
      if(*i > itot)
      { // no more ponts
        (*i)--;
        rg->forward = false;
      }
      else
      { // we are not finished - find number of steps and x,y distance per step
        // find distance vector to next point
        rg->delx=(int)(ref[*i][0]-ref[*i-1][0]);
        rg->dely=(int)(ref[*i][1]-ref[*i-1][1]);
/*        if(abs(rg->delx)>abs(rg->dely))
          rg->tstep=abs(rg->delx)/rg->hast;
        else
          rg->tstep=abs(rg->dely)/(rg->hast); */
        // find time (in steps) to traverse this segment  dist/vel
        rg->tstep = (int)(sqrt(pow(rg->delx,2)+pow(rg->dely,2)) / rg->hast);
        // find distance to go in one step
        rg->xstep = rg->delx / rg->tstep;
        rg->ystep = rg->dely / rg->tstep;
      }
    }
    if( rg->forward == false )
    { // not normal
      (*i)--;
      if(*i < 0)
      { // less than segment 1? - not good
        (*i)++;
        rg->xstep = 0;
        rg->ystep = 0;
        rg->tstep++;
        returnValue = true;
      }
      else
      { // running backwards - why? / chr
        rg->delx=(int)(ref[*i][0]-ref[*i+1][0]);
        rg->dely=(int)(ref[*i][1]-ref[*i+1][1]);
/*        if(abs(rg->delx)>abs(rg->dely))
          rg->tstep=abs(rg->delx)/rg->hast;
        else
          rg->tstep=abs(rg->dely)/rg->hast;
*/      rg->tstep=(int)(sqrt(pow(rg->delx,2)+pow(rg->dely,2))/rg->hast);
        rg->xstep = rg->delx / rg->tstep;
        rg->ystep = rg->dely / rg->tstep;
      }
    }

  }
  // more steps left on same (or new) segment
  // advance reference point
  rg->xref += rg->xstep;
  rg->yref += rg->ystep;

  /* lavpasfiltrering af referencen                       */
  rg->xlp = (1+rg->tau)*rg->xref - rg->tau*rg->xlp;
  rg->ylp = (1+rg->tau)*rg->yref - rg->tau*rg->ylp;

  lastPosInit = false;
  return returnValue;
}

/// /////////////////////////////////////////////////////////////////////

/**
 * go forward along route at konstant speed 
 * \param rg is struct with result (refX, refY)
 * \param ref is track
 * \param idx is current segment number (starting with 1 (point 0 to 1))
 * \param refCnt is number of points in track
 * \param lastX is current ball position in frame coordinates (pixels)
 * \param lastY is current ball position in frame coordinates (pixels)
 * \param advanceDist is distance to ref-position on track - in pixels.
 * \returns true if no more points (game finished successfully)
 * \returns reference position in rg structure and current track index in idx */
int RefGen2(refgen_parm *rg, int ref[][2], int *idx, int refCnt, double lastX, double lastY, double advanceDist)
{
  int returnValue;
  U2Dseg seg;
  double m; // distance along line in pixels
  U2Dpos pos;
  double t1, t2;
  int tCnt;
  //
  returnValue = false; // true if finished
  if (*idx == 0)
    *idx = 1;
  // find carrot point
  while (true)
  { // get next segment
    seg.setFromPoints(ref[*idx-1][0], ref[*idx-1][1], ref[*idx][0], ref[*idx][1]);
    // get crossing on this line
    tCnt = seg.getCircleCrossings(lastX, lastY, advanceDist, &t1, &t2);
    if (tCnt == 2)
    { // use the forward most point
      if (t1 > t2)
        m = t1;
      else
        m = t2;
    }
    else
    {
      printf("refgen2: failed - found %d crossings\n", tCnt);
      returnValue = true;
      // failed to find next point
      break;
    }
    if (m < 0)
      m = 0;
    if (m < seg.length)
      // new carrot point is found on this segment
      break;
    if (*idx < refCnt - 1)
    { // advance to next segment
      *idx = *idx + 1;
    }
    else
    { // reached the end
      returnValue = true;
      break;
    }
  }
  if (not returnValue)
  { // not at the end - so set new carrot position
//     if (lastPosInit)
//     {
//       if (*idx < lastPosIdx or 
//           (*idx == lastPosIdx and m < lastPosM))
//       { // ball rolled back, do not follow
//         *idx = lastPosIdx;
//         m = lastPosM;
//         seg = lastSeg;
//       }
//     }
    pos = seg.getPositionOnLine(m);
    // save last position
    // lastPosInit = true;
    lastPosIdx = *idx;
    lastPosM = m;
    lastSeg = seg;
    rg->xref = pos.x;
    rg->yref = pos.y;
    rg->xlp = pos.x;
    rg->ylp = pos.y;
  }
  return returnValue;
}

/**
 * Find closest point on route 
 * \param rg is where the closest point is returned (xref, yref)
 * \param ref is track
 * \param idx is segment index with closest point.
 * \param refCnt is number of elements in track.
 * \param ballX is ball X position in pixels.
 * \param ballY is ball Y posiition in pixels.
 * \returns distance to closest point in meters. */
double findOnRoutePoint(refgen_parm *rg, int ref[][2], int *idx, int refCnt, double ballX, double ballY)
{
  U2Dseg seg;
  double m; // distance along line in pixels
  U2Dpos pos;
  double minDist = 200;
  U2Dseg minSeg;
  double minM;
  double minSegIdx;
  //
  int i;
  for (i = 1; i < refCnt; i++)
  { // get next segment
    seg.setFromPoints(ref[i-1][0], ref[i-1][1], ref[i][0], ref[i][1]);
    // get crossing on this line
    m = seg.getPositionOnLine(ballX, ballY);
    if (m >= 0 and m <= seg.length)
    {
      int h;
      double d = seg.getDistanceSigned(ballX, ballY, &h);
      if (fabs(d) < minDist)
      {
        minM = m;
        minSeg = seg;
        minSegIdx = i;
        minDist = fabs(d);
      }
    }
  }
  pos = minSeg.getPositionOnLine(minM);
  rg->xref = pos.x;
  rg->yref = pos.y;
  *idx = minSegIdx;
  return  minDist/pixelsPerMeter;
}

/**
  * not used static distance method
  * */
int RefGen3(refgen_parm *rg, int ref[][2], int *idx, int refCnt, double ballX, double ballY, double carrotDist)
{
  int returnValue;
  U2Dseg seg;
  double m; // distance along line in pixels
  U2Dpos pos;
  double t1, t2;
  int tCnt;
  //
  returnValue = false; // true if finished
  if (*idx == 0)
    *idx = 1;
  // find carrot point
  while (true)
  { // get next segment
    seg.setFromPoints(ref[*idx-1][0], ref[*idx-1][1], ref[*idx][0], ref[*idx][1]);
    // get crossing on this line
    tCnt = seg.getCircleCrossings(ballX, ballY, carrotDist, &t1, &t2);
    if (tCnt == 0)
    {  // too far away, go towards line directly
      m = seg.getPositionOnLine(ballX, ballY);
      if (m > seg.length)
        m = seg.length;
    }
    else
    { // use the forward most point
      if (t1 > t2)
        m = t1;
      else
        m = t2;
    }
    if (m < 0)
      m = 0;
    if (m < seg.length)
      // new carrot point is found on this segment
      break;
    if (*idx < refCnt - 1)
    { // advance to next segment
      *idx = *idx + 1;
    }
    else
    { // reached the end
      returnValue = true;
      break;
    }
  }
  if (not returnValue)
  { // not at the end - so set new carrot position
    if (lastPosInit)
    {
      if (*idx < lastPosIdx or 
          (*idx == lastPosIdx and m < lastPosM))
      { // ball rolled back, do not follow
        *idx = lastPosIdx;
        m = lastPosM;
        seg = lastSeg;
      }
    }
    pos = seg.getPositionOnLine(m);
    // save last position
    lastPosInit = true;
    lastPosIdx = *idx;
    lastPosM = m;
    lastSeg = seg;
    rg->xref = pos.x;
    rg->yref = pos.y;
    rg->xlp = pos.x;
    rg->ylp = pos.y;
  }
  return returnValue;
}

