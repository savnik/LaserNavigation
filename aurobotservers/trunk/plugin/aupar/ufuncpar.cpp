/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#include <ugen4/ucommon.h>
#include <urob4/uvariable.h>
#include <urob4/uresposehist.h>
#include <urob4/uvarcalc.h>

#include "ufuncpar.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object
#include <urob4/uvariable.h>

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncPar' with your classname, as used in the headerfile */
  return new UFuncPar();
}

#endif

UFuncPar::~UFuncPar()
{
  if (rollRaw != NULL)
    free(rollRaw);
  if (pitchRaw != NULL)
    free(pitchRaw);
  if (yawRaw != NULL)
    free(yawRaw);
  if (rollFilt != NULL)
    free(rollFilt);
  if (pitchFilt != NULL)
    free(pitchFilt);
  if (w2gDrive != NULL)
    free(w2gDrive);
  if (timeRaw != NULL)
    free(timeRaw);
}

void UFuncPar::init()
{
  rollRaw = NULL;
  rollFilt = NULL;
  pitchRaw = NULL;
  yawRaw = NULL;
  pitchFilt = NULL;
  w2gDrive = NULL;
  timeRaw = NULL;
  imuSampleMaxCnt = 0;
  imuSampleCnt = 0;
  sweepStep = 0;
  sweepAngle = 0;
  sweepTime.Now();
}

///////////////////////////////////////////////////

bool UFuncPar::handleCommand(UServerInMsg * msg, void * extra)
{
  const int MRL = 500;
  char reply[MRL];
//  bool aLog, doLog, aDummy, aSilent = false;
//  bool aFake;
  // check for parameter 'help'
  if (msg->tag.getAttValue("help", NULL, 0))
  { // create the reply in XML-like (html - like) format
    sendHelpStart("Par");
    sendText("--- PAR is for roll and pitch calculations\n");
    snprintf(reply, MRL, "log=true|false      Opens or closes the logfile %s (open=%s)\n", logf.getLogFileName(), bool2str(logf.isOpen()));
    sendText(reply);
    sendText("eval       evaluate parametric roll state\n");
    sendText("update     update parametric roll state with new sensor data\n");
    //sendText("dummy      update dummy data generator (output values for marg)\n");
    sendText("fake=F     fake some roll/pitch values F=roll frq\n");
    sendText("sweep      fake a frequenct sweep for the roll\n");
    sendText("sweepStep  fake a frequenct sweep step\n");
    sendText("pitch=F    with fake F=pitch frq\n");
    sendText("silent     do not make an info reply\n");
    sendText("help       This message\n");
    sendHelpDone();
  }
  else
  { // get any command attributes (when not a help request)
    bool doLog, aSilent, aSweep = false, aSweepStep = false, anEval = false;
    int aSweepSteps = 1;
    double rollFrq, pitchFrq=1.0/10.0; // 10Hz
    bool aLog = msg->tag.getAttBool("log", &doLog, true);
    bool aDummy = msg->tag.getAttBool("dummy", NULL);
    msg->tag.getAttBool("silent", &aSilent, true);
    bool aFake = msg->tag.getAttDouble("fake", &rollFrq, 1.0/20.0);
    msg->tag.getAttDouble("pitch", &pitchFrq, 1.0/10.0);
    msg->tag.getAttBool("sweep", &aSweep, true);
    aSweepStep = msg->tag.getAttInteger("sweepStep", &aSweepSteps, 1);
    msg->tag.getAttBool("eval", &anEval, true);
    bool anUpdate = msg->tag.getAttBool("update", NULL);
    //
    // implement command
    if (aLog)
    { // open or close the log
      aLog = logf.openLog(doLog);
      if (doLog)
        snprintf(reply, MRL, "Opened logfile %s (%s)", logf.getLogFileName(), bool2str(aLog));
      else
        snprintf(reply, MRL, "closed logfile %s", logf.getLogFileName());
      // send an information tag back to client
      sendInfo(reply);
    }
    else if (aDummy)
    {
      //dummy();
      bool isOK = setGlobalVar("imu.aaa[1]", 33.0, true);
      printf("Set ima.aaa=33 %s\n", bool2str(isOK));
      isOK = setGlobalVar("imu.bbb", 34.0, false);
      printf("Set ima.bbb=34 %s\n", bool2str(isOK));
      isOK = getVarPool()->getGlobalVariable("imu.aaa")->setDouble(33.7, 1, true);
      printf("Set ima.aaa=33.7 %s\n", bool2str(isOK));
      if (not aSilent)
        sendInfo("done");
    }
    else if (aFake)
    {
      fakeImuValues(rollFrq, pitchFrq);
      // fake sweep
    }
    else if (aSweep)
    {
      double startFrq=0.001; //1.0; // oldest time
      double sampleRate=10.0;
      double endFrq = 0.3; // 0.0002; // newest time
      int samples=30000;
      UVariable * var = getVarPool()->getGlobalVariable("imu.rot");
      fakeFrqSweep(var, 0.5, startFrq, endFrq, samples, sampleRate);
      // fake sweep
    }
    else if (anEval)
    {
      rollDetectBatch();
      if (not aSilent)
        sendInfo("Done roll detect - not fully implemented yet");
    }
    else if (anUpdate)
    {
      bool done = newImuData();
      if (not aSilent)
      {
        if (done)
          sendInfo("updated sensor values");
        else
          sendInfo("redundant info");
      }
    }
    else if (not aSweepStep)
      sendWarning("No action see 'par help'");
    if (aSweepStep)
    {
      UVariable * var = getVarPool()->getGlobalVariable("imu.rot");
      double startFrq = 0.0003; // Hz
      double sampleRate = 3; // Hz
      double endFrq = sampleRate / 3.0;
      if (not setSweepMeasurement(var, -0.5, aSweepSteps, log(startFrq), log(endFrq), 20000, sampleRate))
      { // last sample
        sweepStep = 0;
        sweepAngle = 0.0;
      }
    }
  }
  return true;
}

/////////////////////////////////////////////////////////////////

void UFuncPar::createResources()
{
  UVarPool * pa;
  varInitialized = addVar("initialized", 0.0, "b", "(r) is block initialized OK");
  varImuSource = addVar("imuSource", "imu.rot", "s", "(r) source variable for calculation (typically imu.rot or ifVar.imu.rot)");
  pa = addStruct("state", "Detection state");
  if (pa != NULL)
  {
    vars.scFlag = pa->addVarA("scFlag", "0 0", "b", "(r) Spectral correlation alarm detected and count");
    vars.scFlag->makeTimeSeries(10000, 1.0);
    vars.wg2Flag = pa->addVarA("w2gFlag", "0 0", "b", "(r) Double Weibull GLRT alarm detection and count");
    vars.wg2Flag->makeTimeSeries(10000, 1.0);
    vars.scIndex = pa->addVar("scIndex", 0.4, "d", "(r) Spectral correlation detection value");
    vars.scIndex->makeTimeSeries(10000, 1.0);
    vars.wg2Index = pa->addVar("w2gIndex", 0.4, "d", "(r) Double Weibull GLRT detection value");
    vars.wg2Index->makeTimeSeries(10000, 1.0);
    vars.alertFlag = pa->addVarA("alertFlag", "0 0", "d", "(r) combined alert index flag and count");
    vars.alertFlag->makeTimeSeries(10000, 1.0);
    vars.alertIndex = pa->addVar("alertIndex", 0.4, "d", "(r) combined alert index");
    vars.alertIndex->makeTimeSeries(10000, 1.0);
    vars.roll6h = pa->addVar("roll6h", 0.0, "d", "(r) max roll value the last 6 hours");
    vars.pitch6h = pa->addVar("pitch6h", 0.0, "d", "(r) max pitch value the last '6 hour'");
    vars.alarmCnt6h = pa->addVar("alartCnt6h", 0.0, "d", "(r) number of alarms last '6 hour'");
    vars.roll1min = pa->addVar("roll1min", 0.0, "d", "(r) max roll value the last minute");
    vars.roll1min->makeTimeSeries(3600, 1.0);
    vars.pitch1min = pa->addVar("pitch1min", 0.0, "d", "(r) max pitch value the last minute");
    vars.pitch1min->makeTimeSeries(3600, 1.0);
    vars.alarmCnt1min = pa->addVar("alartCnt1min", 0.0, "d", "(r) number of alarms last minute");
    vars.alarmCnt1min->makeTimeSeries(3600, 1.0);
    vars.cnt = pa->addVar("cnt", 0.0, "d", "(r) Update count");
    vars.time = pa->addVar("time", 0.0, "t", "(r) Time of last update");
    vars.timeLong = pa->addVar("time", 0.0, "t", "(r) Time of last update of long term variables");
    vars.calcTime = pa->addVar("calcTime", 0.0, "t", "(r) time to make last roll calculation (in ms)");
    vars.w2gShape = pa->addVar("w2gShape", 0.75, "d", "(r) Weibull shape parameter in last estimate");
    vars.w2gScale = pa->addVar("w2gScale", 1.00, "d", "(r) Weibull scale value in last estimate");
  }
  pa = addStruct("param", "Detection parameters");
  if (pa != NULL)
  {
      varp.rollT = pa->addVar("rollT", 22.5, "d", "(rw) roll period time (sec)");
      varp.rollW = pa->addVar("rollW", 0.279, "d", "(rw) roll natural frequency rad/sec");
      //varp.updFrq = pa->addVar("updFrq", 5.0, "d", "(rw) update frequency in Hz");
      varp.filterOrder = pa->addVar("filterOrder", 5.0, "d", "(r) filter order");
      varp.fileterBW = pa->addVar("fileterBW", 1.2, "d", "(r) filter bandwidth - factor");
      varp.filterType = pa->addVar("filterType", 1.0, "d", "(r) filter type 1=butterworth");
      varp.scRollPeriods = pa->addVar("scRollPeriods", 3.0, "d", "(rw) number of used roll periods - Spectral analysis");
      varp.wg2RollPeriods = pa->addVar("w2gRollPeriods", 3.0, "d", "(rw) number of used roll periods - Weibull method");
      varp.initRollPeriods = pa->addVar("initRollPeriods", 30.0, "d", "(rw) initial estimate after this number of periods");
      varp.scThreshold = pa->addVar("scThreshold", 0.5, "d", "(rw) spectral corr alarm threshold [0..1]");
      varp.wg2Threshold = pa->addVar("w2gThreshold", 0.5, "d", "(rw) Weibull alarm threshold [0..1]");
      varp.alarmWindLen = pa->addVar("alertWindLen", 2.0, "d", "(rw) size of detection window in roll periods");
      varp.alarmWindOverlap = pa->addVar("alertWindOverlap", 0.0, "d", "(rw) is ???");
      varp.rollMinVar = pa->addVar("rollMinVar", 0.1, "d", "(rw) minimum variance for roll (rad^2)");
      varp.pitchMinVar = pa->addVar("pitchMinVar", 0.1, "d", "(rw) minimum variance for pitch (rad^2)");
      varp.horizon6h = pa->addVar("horizon6h", 360.0, "d", "(rw) number of minutes in '6 hour' period");
      varp.shapeUpdTime = pa->addVar("shapeUpdTime", 1.0, "d", "(rw) time between updates  of long term variables (e.g. shape0)");
      varp.filteredBufferTime = pa->addVar("filteredBufferTime", 300, "d", "(rw) length of buffer (in seconds) for band-pass filtered values.");
  }
}

////////////////////////////////////////////////////////////////

void UFuncPar::fakeImuValues(double rollFrq, double pitchFrq)
{
  double r = 0.0, p = 0.0;
  double dt = 0.05;
  double ampR = 4.0, ampP = 1.0;
  const char * varSrc = varImuSource->getValues();
  UVariable * rotHist;
  rotHist = getVarPool()->getGlobalVariable(varSrc);
  UTime t;
  double dr, dp;
  const int SMPLS = 5000;
  //
  t.now();
  t = t - double(SMPLS) * dt;
  dr = 2.0 * M_PI * rollFrq * dt;
  dp = 2.0 * M_PI * pitchFrq * dt;
  for (int i = 0; i < SMPLS; i++)
  {
    double a[3];
    a[0] = ampR * sin(r);
    a[1] = ampP * sin(p);
    a[2] = 45.0;
    rotHist->setDouble(a, 3, &t);
    r = limitToPi(r + dr);
    p = limitToPi(p + dp);
    t.add(dt);
  }
}


/////////////////////////////////////////////////////////////////

bool UFuncPar::newImuData()
{
  UVarPool * vp = getVarPool();
  const char * varSrc = varImuSource->getValues();
  UVariable * rotHist = vp->getGlobalVariable(varSrc);
  bool isOK = rotHist != NULL;
  bool initialize = rollRaw == NULL;
  const int oversamplingRate = 40;
  const int firCnt = 51;
  int bufSize = firCnt * 2 + 1;
  // bandpass filter assuming sample rate is 40 times natural roll frq.
  // center frequency is 1/20 = 0.05
  // MATLAB: >> filter51 = fir1(51, [0.045 0.06])
  const double fir51[] = {
  -0.003218328227846,  -0.004057818065448,  -0.005242771040626,  -0.006854855724642,  -0.008903243900917,  -0.011317902752974,
  -0.013950493386618,  -0.016583013848246,  -0.018943644085201,  -0.020728605105972,  -0.021628290934674,  -0.021355518618346,
  -0.019673505879184,  -0.016421150568180,  -0.011533356575684,  -0.005054515489408,   0.002856216387663,   0.011928552849292,
   0.021793609182402,   0.032004583207385,   0.042064159327846,   0.051456638607989,   0.059682385435555,   0.066291962326425,
   0.070917318410768,   0.073297605871962,   0.073297605871962,   0.070917318410768,   0.066291962326425,   0.059682385435555,
   0.051456638607989,   0.042064159327846,   0.032004583207385,   0.021793609182402,   0.011928552849292,   0.002856216387663,
  -0.005054515489408,  -0.011533356575684,  -0.016421150568180,  -0.019673505879184,  -0.021355518618346,  -0.021628290934674,
  -0.020728605105972,  -0.018943644085201,  -0.016583013848246,  -0.013950493386618,  -0.011317902752974,  -0.008903243900917,
  -0.006854855724642,  -0.005242771040626,  -0.004057818065448,  -0.003218328227846};
  // current sample time for this filter and roll resonance
  double dt = varp.rollT->getDouble() / double(oversamplingRate * 2);
  UTime t1;
  //
  if (isOK)
  {
    t1 = rotHist->getUpdTime();
    isOK = (t1 - tLastSample) > dt;
  }
  if (isOK and not initialize)
  { // set update time
    if ((t1 - tLastSample) > 10.0)
      // uninitialized - or reconnected
      initialize = true;
    else
      // just update
      tLastSample += dt;
    // save value
  }
  if (initialize)
  {
    rollRaw = (double *) realloc(rollRaw, bufSize * sizeof(double));
    pitchRaw = (double *) realloc(pitchRaw, bufSize * sizeof(double));
    rollRawCnt = 0;
    tLastSample = t1;
    isOK = rollRaw != NULL and pitchRaw != NULL;
    printf("(re) initialized raw data buffer (%d elements) before filter (OK=%s)\n", bufSize, bool2str(isOK));
  }
  if (isOK)
  {
    if (rollRawCnt < bufSize)
      rollRawCnt++;
    memmove(&rollRaw[1], rollRaw, sizeof(double) * (rollRawCnt - 1));
    memmove(&pitchRaw[1], pitchRaw, sizeof(double) * (rollRawCnt - 1));
    rollRaw[0] = rotHist->getDouble(0);
    pitchRaw[0] = rotHist->getDouble(1);
    rawUpdTime = t1;
    isOK = rollRawCnt == bufSize;
    if (not isOK)
      printf("filling filter buffer, now %d/%d\n", rollRawCnt, bufSize);
  }
  if (isOK)
  {
    int filterBuffer = varp.filteredBufferTime->getDouble() / (dt * 2.0);
    if (rollFilt == NULL or rollFiltMaxCnt < rollFiltMaxCnt)
    {
      rollFilt = (double *) realloc(rollFilt, filterBuffer * sizeof(double));
      pitchFilt = (double *) realloc(pitchFilt, filterBuffer * sizeof(double));
      rollFiltMaxCnt = filterBuffer;
      rollFiltIdx = -1; // newest data
      rollFiltCnt = 0; // number of available elements
      printf("PAR: initialized filter buffer (to %d elements, spanning %.1f seconds)\n", filterBuffer,
             varp.filteredBufferTime->getDouble());
    }
    rollFiltIdx = (rollFiltIdx + 1) % filterBuffer;
    if (rollFiltCnt < filterBuffer)
      rollFiltCnt++;
/*    rollFiltCnt  = filter(&rollFilt[firCnt],  rollRaw,  imuSampleCnt, fir51, firCnt, 2);
    pitchFiltCnt = filter(&pitchFilt[firCnt], pitchRaw, imuSampleCnt, fir51, firCnt, 1);*/
    // filter roll with every other sample
    rollFilt[rollFiltIdx] = fold(rollRaw, fir51, firCnt, 2);
    // filter pitch with every sample - twice the center frequency
    pitchFilt[rollFiltIdx] = fold(&pitchRaw[firCnt/2], fir51, firCnt, 1);
    // save update time of newly filtered value -
    filtUpdTime = t1 - (firCnt * dt);
    // debug
    printf("filter %d: roll:%g to %g, pitch %g to %g at time %.3f\n",
           rollFiltIdx, rollRaw[firCnt], rollFilt[rollFiltIdx],
           pitchRaw[firCnt], pitchFilt[rollFiltIdx], filtUpdTime.GetDecSec());
    // debug end
  }
  return isOK;
}


///////////////////////////////////////////

void UFuncPar::rollDetectBatch()
{ // filter camculated with this oversample rate (filter frequency compared to sample frequency)
  const int oversamplingRate = 40;
  // need double amount of samples to filter around pitch frequency
  int sn = round(varp.scRollPeriods->getDouble() * oversamplingRate) * 2;
  // pointer to variable with raw data
  UVariable * rotHist;
  bool isOK;
  // obtained samples
  int rollRawCnt, pitchRawCnt;
  // obtained samples when filtered
  int rollFiltCnt, pitchFiltCnt;
  UTime t, t0, toTime;
  // estimated values
  double rollMuHat, rollSigmaHat, pitchMuHat, pitchSigmaHat;
  // update rate
  double updr = -1.0;
  // bandpass filter assuming sample rate is 40 times natural roll frq.
  // center frequency is 1/20 = 0.05
  // MATLAB filter51 = fir1(51, [0.045 0.06])
  const int firCnt = 51;
  const double fir51[] = {
  -0.003218328227846,  -0.004057818065448,  -0.005242771040626,  -0.006854855724642,  -0.008903243900917,  -0.011317902752974,
  -0.013950493386618,  -0.016583013848246,  -0.018943644085201,  -0.020728605105972,  -0.021628290934674,  -0.021355518618346,
  -0.019673505879184,  -0.016421150568180,  -0.011533356575684,  -0.005054515489408,   0.002856216387663,   0.011928552849292,
   0.021793609182402,   0.032004583207385,   0.042064159327846,   0.051456638607989,   0.059682385435555,   0.066291962326425,
   0.070917318410768,   0.073297605871962,   0.073297605871962,   0.070917318410768,   0.066291962326425,   0.059682385435555,
   0.051456638607989,   0.042064159327846,   0.032004583207385,   0.021793609182402,   0.011928552849292,   0.002856216387663,
  -0.005054515489408,  -0.011533356575684,  -0.016421150568180,  -0.019673505879184,  -0.021355518618346,  -0.021628290934674,
  -0.020728605105972,  -0.018943644085201,  -0.016583013848246,  -0.013950493386618,  -0.011317902752974,  -0.008903243900917,
  -0.006854855724642,  -0.005242771040626,  -0.004057818065448,  -0.003218328227846
};
  //
  if (sn > imuSampleMaxCnt)
  { // need for more space in sample buffer
//    rollRaw = (double *) realloc(rollRaw, sn * sizeof(double));
    rollFilt = (double *) realloc(rollFilt, sn * sizeof(double));
//    pitchRaw = (double *) realloc(pitchRaw, sn * sizeof(double));
    pitchFilt = (double *) realloc(pitchFilt, sn * sizeof(double));
    yawRaw = (double *) realloc(yawRaw, sn * sizeof(double));
    timeRaw = (double *) realloc(timeRaw, sn * sizeof(double));
    w2gDrive = (double *) realloc(w2gDrive, sn * sizeof(double));
    imuSampleMaxCnt = sn;
    printf("Allocated space for %d samples\n", sn);
  }
  t.now();
  isOK = sn <= imuSampleMaxCnt;
  if (isOK)
  {
    UVarPool * vp = getVarPool();
    const char * varSrc = varImuSource->getValues();
    rotHist = vp->getGlobalVariable(varSrc);
    isOK = rotHist != NULL;
    if (isOK)
      isOK = rotHist->hasHist() and rotHist->getHistoryRows() > oversamplingRate * 4;
    printf("PAR: imu.rot has %d samples, need %d (OK = %s)\n", rotHist->getHistoryRows(), oversamplingRate * 4, bool2str(isOK));
  }
  if (isOK)
  { // get sample rate for data vector
    updr = rotHist->hist->getSampleRate(50);
    // get step size of data for pitch (highest sample rate)
    int step = maxi(1, round(updr * varp.rollT->getDouble() / double(oversamplingRate * 2))); // / varp.updFrq->getDouble());
    // actual sample time for roll and pitch
    double dtR = 0.1, dtP = 0.1;
    //
    printf("PAR: uses every %d step from imu.rot history buffer\n", step);
    //
    t0 = rotHist->getUpdTime();
    if (not t0.valid)
      t0.now();
    toTime = t0 - varp.scRollPeriods->getDouble() * varp.rollT->getDouble();
    // debug
    // do not limit using sample time
    toTime.SetTime(0, 0);
    // debug end
    rollRawCnt  = rotHist->hist->getVectorToTime(rollRaw, sn, 0, toTime, step, &dtR);
    pitchRawCnt = rotHist->hist->getVectorToTime(pitchRaw, sn, 1, toTime, step, &dtP);
    //
    imuSampleCnt = mini(rollRawCnt, pitchRawCnt);
    // debug
    // debug save of values
    // get also yaw info - used for debug only (loaded with sweep frequency
    // NB! index 0 is the newest
    rotHist->hist->getVectorToTime(yawRaw, sn, 2, toTime, step, NULL);
    rotHist->hist->getVectorToTime(timeRaw, sn, -2, toTime, step, NULL);
    // debug print
    printf("IMU sensor raw data %g Hz update rate\n", updr);
    printf(" - pitch: uses every %d measurement, got dtP=%gsec (sfP=%gHz optimal %gHz)\n",
           step, dtP, 1.0/dtP, 2.0*oversamplingRate/varp.rollT->getDouble());
    printf(" - roll:  uses every %d measurement, got dtR=%gsec (sfR=%gHz optimal %gHz)\n",
           step * 2, dtR * 2, 0.5/dtR, oversamplingRate/varp.rollT->getDouble());
    printf(" - rollcnt=%d, pitchcnt=%d/%d, time span max = %g sec\n",
           rollRawCnt, pitchRawCnt, sn, timeRaw[0] - timeRaw[pitchRawCnt-1]);
    // debug end
    // do we have enough samples to try an estimate
    isOK = imuSampleCnt > oversamplingRate * 2;
    if (not isOK)
      printf(" - sample count too low, needs %d got %d\n", oversamplingRate * 2, imuSampleCnt);
  }
  // debug
  //printf("ParRollDetect took %g msec to get roll and pitch data\n", t.getTimePassed()*1000.0);
  // debug end
  if (isOK)
  { // filter roll by half frequency
    memset(rollFilt, 0, sizeof(double)*imuSampleCnt); // firCnt should be enough
    memset(pitchFilt, 0, sizeof(double)*imuSampleCnt);
    rollFiltCnt  = filter(&rollFilt[firCnt],  rollRaw,  imuSampleCnt, fir51, firCnt, 2);
    pitchFiltCnt = filter(&pitchFilt[firCnt], pitchRaw, imuSampleCnt, fir51, firCnt, 1);
    //
  }
  // debug
  //printf("ParRollDetect took %g msec including filter\n", t.getTimePassed() * 1000.0);
  // debug end
  if (isOK)
  { // variance estimate of filtered values
    FILE * toLog;
    toLog = fopen("medianRoll.txt", "w");
    rollMuHat = median(&rollFilt[firCnt], rollFiltCnt, 2, toLog);
    fclose(toLog);
    toLog = fopen("medianPitch.txt", "w");
    pitchMuHat = median(&pitchFilt[firCnt], pitchFiltCnt, 2, toLog);
    fclose(toLog);
    //
    rollSigmaHat = sqr(medianAbsMean(&rollFilt[firCnt], rollFiltCnt, rollMuHat, 2)/0.6745);
    pitchSigmaHat = sqr(medianAbsMean(&pitchFilt[firCnt], pitchFiltCnt, pitchMuHat, 1)/0.6745);
    printf("PAR mean_roll_hat=%.5f, mean_pitch_hat=%.5f, roll_sigma_hat=%.5f, pitch_sigma_hat=%.5f\n",
           rollMuHat, pitchMuHat, rollSigmaHat, pitchSigmaHat);
  }
  // scaled signal combined of filtered pitch squared roll.
  if (isOK)
  { // roll rollFiltCnt is the number of usable samples in the analysis sample set.
    double driveScale = 1.0/(1.0 + sqrt(rollSigmaHat * pitchSigmaHat));
    printf("PAR: drive scale=%.5f\n", driveScale);
    for (int i = 0; i < rollFiltCnt; i++)
    {  // roll has the lowest sample rate, so use i<<1 for pitch index
      w2gDrive[i] = sqr(rollFilt[i] - rollMuHat) * (pitchFilt[i] - pitchMuHat) * driveScale;
    }
    // debug save of values
    if (isOK) // and rollRawCnt > sn - 10)
    {
      double * pData[7];
      pData[0] = timeRaw;
      pData[1] = rollRaw;
      pData[2] = pitchRaw;
      pData[3] = rollFilt;
      pData[4] = pitchFilt;
      pData[5] = yawRaw;
      pData[6] = w2gDrive;
      toFile("rollPitchFilt.txt",pData, 7, rollFiltCnt);
    }
  }
  if (isOK)
  { // find shape of Weibull distribution
    double w2gShape;
    double w2gScale, sm = 0.0;
    //
    w2gShape = findW2gScale(w2gDrive, rollFiltCnt);
    for (int i = 0; i < rollFiltCnt; i++)
      sm += pow(w2gDrive[i], w2gShape);
    w2gScale = pow(sm/double(rollFiltCnt), 1.0/w2gShape);
    //
    vars.w2gShape->setDouble(w2gShape);
    vars.w2gScale->setDouble(w2gScale);
  }
  // debug
  //printf("ParRollDetect took %g msec including median\n", t.getTimePassed() * 1000.0);
  // debug end
  update6hMax(varp.horizon6h->getDouble() * 60.0);
  // debug
  //printf("ParRollDetect took %g msec including 6h update\n", t.getTimePassed() * 1000.0);
  vars.calcTime->setDouble(t.getTimePassed() * 1000.0);
  // debug end
}

//////////////////////////////////////////////////////////

int UFuncPar::filter(double * dst, double * src, int sCnt, const double fir[], int firCnt, int scale)
{
  int n = sCnt - firCnt * 2;
  for (int j = 0; j < n; j++)
  { // start half the FIR filter width before central sample
    double * s = &src[j + firCnt - firCnt / 2 * scale];
    const double * f = fir;
    double v = 0.0;
    for (int i = 0; i < firCnt; i++)
    { // fold values with filter
      v += *s * *f++;
      s += scale;
    }
    // save result value
    for (int i = 0; i < scale; i++)
      dst[j+i] = v;
  }
  return n;
}

/////////////////////////////////////////

double UFuncPar::fold(double * src, const double fir[], int firCnt, int scale)
{
  double * s = src;
  const double * f = fir;
  double v = 0.0;
  for (int i = 0; i < firCnt; i++)
  { // fold values with filter
    v += *s * *f++;
    s += scale;
  }
  return v;
}


//////////////////////////////////////////////////////////////

// void UFuncPar::dummy()
// {
//   double ds,dw;
//   UTime t;
//   t.now();
//   // advanc SC index with random walk data
//   ds = vars.scIndex->getDouble();
//   ds = fmin(1.0, fmax(0.0, ds + 0.2 * (rand() / double(RAND_MAX) - 0.5)));
//   vars.scIndex->setDouble(ds, 0, false, &t);
//   if (ds > varp.scThreshold->getDouble() and not vars.scFlag->getBool())
//   {
//     vars.scFlag->setBool(true, 0, false, &t);
//     vars.scFlag->add(1.0, 1, &t);
//   }
//   else if (ds <= varp.scThreshold->getDouble() and vars.scFlag->getBool())
//     vars.scFlag->setBool(false, 0, false, &t);
//   //
//   // advanc WB2 index with random walk data
//   dw = vars.wg2Index->getDouble();
//   dw = fmin(1.0, fmax(0.0, dw + 0.2 * (rand() / double(RAND_MAX) - 0.5)));
//   vars.wg2Index->setDouble(dw, 0, false, &t);
//   if (dw > varp.wg2Threshold->getDouble() and not vars.wg2Flag->getBool())
//   {
//     vars.wg2Flag->setBool(true, 0, false, &t);
//     vars.wg2Flag->add(1.0, 1, &t);
//   }
//   else if (dw <= varp.wg2Threshold->getDouble() and vars.wg2Flag->getBool())
//     vars.wg2Flag->setBool(false);
//   //
//   vars.alertIndex->setDouble(dw * ds, 0, false, &t);
//   // update common alarm index
//   if (vars.scFlag->getBool() and vars.wg2Flag->getBool() and not vars.alertFlag->getBool())
//     vars.alertFlag->setBool(true);
//   else if (not (vars.scFlag->getBool() and vars.wg2Flag->getBool()) and vars.alertFlag->getBool())
//     vars.alertFlag->setBool(false);
//   //
//   update6hMax(varp.horizon6h->getDouble() * 60.0);
//   vars.cnt->add(1.0);
//   vars.time->setTime(t);
// }

/////////////////////////////////////////////////////////

void UFuncPar::update6hMax(double horizon)
{
  double mn, mi, mx;
  UVariable * rot;
  int n;
  UTime t0, toTime;
  double roll1mMax = 0.0, pitch1mMax = 0.0;
  int alarm1mCnt = 0;
  //
  rot = getVarPool()->getGlobalVariable("imu.rot");
  if (rot != NULL)
  {
    t0 = rot->getUpdTime();
    n = rot->getMeanMinMax(0, hoz1min, &mn, &mi, &mx);
    if (n > 0 )
      roll1mMax = mx - mi;
    n = rot->getMeanMinMax(1, hoz1min, &mn, &mi, &mx);
    if (n > 0)
      pitch1mMax = mx - mi;
  }
  else
    toTime.now();

  // latest alarm number
  alarm1mCnt = vars.alertFlag->getInt(1);
  // make sure to have a valid time
  if (not t0.valid)
    t0.now();
  mi = t0 - hoz1min;
  if (mi > 60.0)
  { // one minute has passed - save a new 1min value
    vars.roll1min->setDouble(roll1mMax, 0, false, &t0);
    vars.pitch1min->setDouble(pitch1mMax, 0, false, &t0);
    vars.alarmCnt1min->setDouble(alarm1mCnt, 0, false, &t0);
    hoz1min = t0;
  }
  toTime = t0 - (varp.horizon6h->getInt() * 60.0);
  n = vars.roll1min->getMeanMinMax(0, toTime, &mn, &mi, &mx);
  if (n > 0)
    vars.roll6h->setDouble(fmax(roll1mMax, mx), 0, false, &t0);
  n = vars.pitch1min->getMeanMinMax(0, toTime, &mn, &mi, &mx);
  if (n > 0)
    vars.pitch6h->setDouble(fmax(pitch1mMax, mx), 0, false, &t0);
  // find minimum alarm count last 6h period
  n = vars.alarmCnt1min->getMeanMinMax(0, toTime, &mn, &mi, &mx);
  if (n > 0)
    vars.alarmCnt6h->setInt(alarm1mCnt - roundi(mi), 0, false, &t0);
}

////////////////////////////////////////////////////////

void UFuncPar::toFile(const char * filename, double * pData[], const int seriesCnt, const int rowCnt)
{
   FILE * fl;
   const int MSL = 200;
   char s[MSL];
   //
   if (filename[0] == '.' or filename[0] == '/')
     snprintf(s, MSL, "%s", filename);
   else
     snprintf(s, MSL, "%s/%s", dataPath, filename);
   fl = fopen(s, "w");
   if (fl != NULL)
   {
     for (int r = 0; r < rowCnt; r++)
     {
       for (int c = 0; c < seriesCnt; c++)
         fprintf(fl, " %g", pData[c][r]);
       fprintf(fl, "\n");
     }
     fclose(fl);
   }
}

/////////////////////////////////////////////

int compare (const void * a, const void * b)
{
  double aa = **(double**)a;
  double bb = **(double**)b;
  if (aa > bb)
    return 1;
  else
    return -1;
}

/////////////////////////////////////////////

double _mu_hat;
int compareabs (const void * a, const void * b)
{
  double aa = **(double**)a;
  double bb = **(double**)b;
  //return ( fabs(**(double**)a - _mu_hat) - fabs(**(double**)b - _mu_hat) );
  if (fabs(aa - _mu_hat) > fabs(bb - _mu_hat))
    return 1;
  else
    return -1;
}

////////////////////////////////////////////

double UFuncPar::median(const double * src, int srcCnt, int scale, FILE * tolog)
{
  const double **pd, **pdi;
  if (scale < 1)
    scale = 1;
  pd = (const double **) malloc(sizeof(double*) * srcCnt / scale);
  double med = 0.0, avg = 0.0;
  //
  if (pd != NULL)
  {
    pdi = pd;
    for (int i = 0; i < srcCnt; i += scale)
    {
      *pdi = &src[i];
      avg += src[i];
      pdi++;
      if (tolog != NULL)
        fprintf(tolog, "%f\n", src[i]);
/*      printf(", %.2f", src[i]);
      if (i %20 == 0)
        printf("\n");*/
    }
    qsort(pd, srcCnt/scale, sizeof(double *), compare);
    if (tolog != NULL)
      for (int i = 0; i < srcCnt/scale; i++)
        fprintf(tolog, "%f\n", *pd[i]);
    med = *pd[srcCnt/2/scale];
    free(pd);
    avg /= (srcCnt/scale);
    printf("PAR::median - median of %d samples is %.5f, mean is %.5f\n", srcCnt, med, avg);
  }
  return med;
}

/////////////////////////////////////////////

double UFuncPar::medianAbsMean(const double * src, int srcCnt, double mean, int scale)
{
  const double **pd, **pdi;
  double med = 0.0;
  if (scale < 1)
    scale = 1;
  pd = (const double **) malloc(sizeof(double*) * srcCnt/scale);
  //
  if (pd != NULL)
  { // _mu_hat is used when sorting absolute values
    _mu_hat = mean;
    pdi = pd;
    for (int i = 0; i < srcCnt; i += scale)
      *pdi++ = &src[i];
    qsort(pd, srcCnt/scale, sizeof(double *), compareabs);
    med = fabs(*pd[srcCnt/2/scale]);
    free(pd);
  }
  return med;
}

//////////////////////////////////////////

double UFuncPar::findW2gScale(double val[], const int valCnt)
{
  double avgLn = 0.0;
  double est = -1.0, best = 1e8;
  const double eps = 1e-14;
  //
  for (int i = 0; i < valCnt; i++)
    avgLn += log(fabs(val[i]) + eps);
  avgLn /= valCnt;
  //
  // analysis of shape equation for legal values of the shape [1.0 .. 0.5]
  const int eCnt = 40;
  for (int e = 0; e < eCnt; e++)
  {
    double shapeEst = 0.05 + 0.95/eCnt * double(e);
    double sumNum = 0.0, sumDen = 0.0;
    double er;
    for (int i = 0; i < valCnt; i++)
    {
      double iab = fabs(val[i] + eps);
      double ips = pow(iab, shapeEst);
      sumNum += ips * log(iab);
      sumDen += ips;
    }
    er = sumNum/sumDen - avgLn - 1.0/shapeEst;
    if (fabs(er) < best)
    {
      est = shapeEst;
      best = fabs(er);
    }
    //printf("w2g shape=%g: %g = %g - %g (err = %g)\n", shapeEst, 1.0/shapeEst, sumNum/sumDen, avgLn, er);
  }
  printf("Found a shape0 = %g (err = %g)\n", est, best);

  return est;
}


///////////////////////////////////////////////////////

void UFuncPar::fakeFrqSweep(UVariable * var, double dcOffset, double startFrq, double endFrq, int samples, double sampleRate)
{
  double f1 = log(startFrq), f2 = log(endFrq);
  //
  if (var == NULL)
    printf("Var not found\n");
  else
  {
    sweepTime.now();
    sweepTime = sweepTime - samples/sampleRate;
    sweepStep = 0;
    sweepAngle = 0;
    { // find frequency ion log scale
      setSweepMeasurement(var, dcOffset, samples, f1, f2, samples, sampleRate);
/*      double flog = f1 + (f2 - f1) / (double)samples * i;
      // convert to true frequency
      double f = exp(flog);
      // find angle shift in a sample time
      double da = f * 2 * M_PI / sampleRate;
      // advance angle
      a += da;
      // find actual measured amplitude
      double v = sin(a);
      // set value
      var->setDouble(v, idx, false, &t);
      // debug - set pitch as frequency
      var->setDouble(f, idx + 1, false, &t);
      // debug end
      t.add(1.0/sampleRate);*/
    }
    printf("Added %d values as a sweep from %g to %g Hz to %s)\n", samples, startFrq, endFrq, var->name);
  }
}

//////////////////////////////////

bool UFuncPar::setSweepMeasurement(UVariable* var, double dcOffset, int steps, double fl1, double fl2, int samples, double sampleRate)
{
  double flog, f, da, v;
  //
  if (steps < 20)
    printf("PAR: added %d steps =", steps);
  for (int i = 0; i < steps; i++)
  {
    sweepStep++;
    flog = fl1 + (fl2 - fl1) / (double)samples * sweepStep;
    // convert to true frequency
    f = exp(flog);
    // find angle shift in a sample time
    da = f * 2 * M_PI / sampleRate;
    // advance angle
    sweepAngle += da;
    // find actual measured amplitude
    v = sin(sweepAngle);
    if (steps < 20)
      printf(" %.3fV (f=%.4fHz),", v, f);
    // set value - roll
    var->setDouble(v + dcOffset, 0, false, &sweepTime);
    // pitch
    var->setDouble(cos(sweepAngle) + dcOffset, 1, false, &sweepTime);
    // debug - set yaw as frequency
    var->setDouble(f, 2, false, &sweepTime);
    // debug end
    sweepTime.add(1.0/sampleRate);
  }
  if (steps < 20)
    printf("\n");
  //
  return sweepStep >= 0 and sweepStep <= samples;
}
