/***************************************************************************
 *   Copyright (C) 2010 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 ***************************************************************************/
#ifndef UFUNC_PAR_H
#define UFUNC_PAR_H

#include <urob4/ufuncplugbase.h>

/**
 * This is a simple example plugin, that manipulates global variables, both of its own, and those of other modules.
@author Christian Andersen
*/
class UFuncPar : public UFuncPlugBase
{ // NAMING convention recommend that the plugin function class
  // starts with UFunc (as in UFuncPar) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncPar()
  { // command list and version text
    setCommand("par", "par", "Plugin for paroll");
    logf.setLogName("par");
    init();
  }
  /**
   * Destructor */
  ~UFuncPar();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs. */
  virtual void createResources();
  /**
   * Handle incomming commands flagged for this module
   * \param msg pointer to the message and the client issuing the command
   * \return true if the function is handled - otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  //
private:
  /// init local variables */
  void init();
  /** new roll data has arrived, update long term estimates
  \returns true if long term data is updated */
  bool newImuData();
  /// do the parametric roll detect
  void rollDetectBatch();
  /** FIR filter
  \param dst is the destination for the filtered values (2*firCnt shorter than the source)
  NB! first value corresponds to src[firCnt].
  \param src is the data samples.
  \param sCnt is the sample count of values.
  \param fir is the filter values.
  \param firCnt is the number of filter elements
  \param scale 1 use all source values, if 2, then use every second source value - frequency divided by scale
  NB! scale 1 and 2 allowed only
  \returns number of values in result buffer. */
  int filter(double * dst, double * src, int sCnt, const double fir[], int firCnt, int scale);
  /** FIR filter
  \param src is the data samples (must have length of at least firCnt * scale).
  \param fir is the filter values.
  \param firCnt is the number of filter elements
  \param scale 1 use all source values, if 2, then use every second source value - frequency divided by scale
  \returns result of folding filter over source values. */
  double fold(double * src, const double fir[], int firCnt, int scale);
  /**
  Dummy data generator */
  //void dummy();
  /**
  Update values for maximum count within a "6 hour" horizon, then actual horizon time
  is specified in parameter.
  \param horizon time over which to search for maximumvalues.*/
  void update6hMax(double horizon);
  /**
  Save values to a MATLAB file */
  void toFile(const char * filename, double * pData[], const int seriesCnt, const int rowCnt);
  /**
  Get median of a series of double values
  \param src is the source array of values.
  \param srcCnt is the number of values in array.
  \param scale use only values at this interval (1=every, 2=every other ...)*/
  double median(const double * src, int srcCnt, int scale, FILE * toLog = NULL);
  /**
  Get median of the absolute value of a series of double values subtracted the mean.
  \param src is the source array of values.
  \param srcCnt is the number of values in array.
  \param mean is the mean value around which the absolute value is to be folded.
  \param scale use only values at this interval (1=every, 2=every other ...)*/
  double medianAbsMean(const double * src, int srcCnt, double mean, int scale);
  /**
  Estimate Weibull shape parameter by iteration for these values.
  \param val is an array of values for the estimation.
  \param valCnt is the number of valid values in the array.
  \returns the estimated value. */
  double findW2gScale(double val[], const int valCnt);
  /**
  Fake some values of roll and pitch.
  \param rollFrq is the roll frequency to use
  \param pitchFrq is the pitch frequency to use */
  void fakeImuValues(double rollFrq, double pitchFrq);
  /**
   * Fake af frequenct sweep and load the result into a variable
   \param var is the variable to set
   \param dcOffset is a fixed dc offset value added to all values,
   \param startFrq in HZ (this will be the frequency to the oldest history time
   \param endFrq in Hz of the newest sample in var.
   \param samples to generate
   \param sampleRate in Hz for the timestamp - newest time is now().
   */
  void fakeFrqSweep(UVariable * var, double dcOffset, double startFrq, double endFrq, int samples, double sampleRate);
  /**
   * take the next sweep step
   \param var is the variable to set
   \param dcOffset is a fixed dc offset value added to all values,
   \param steps is number of samples to add
   \param fl1 log of start frequency
   \param fl2 log of end frequency
   \param samples is total number of samples to generate
   \param sample rate the sample frequency
   \returns true as long as last sample is not reached. */
   bool setSweepMeasurement(UVariable* var, double dcOffset, int steps, double fl1, double fl2, int samples, double sampleRate);

private:
  /**
  Handles to "own" global variables. */
  UVariable * varInitialized;
  UVariable * varImuSource;
  struct
  {
    UVariable * rollT;  /// roll period time
    UVariable * rollW;  /// roll natural frequency rad/sec
    //UVariable * updFrq; /// update frequency in Hz
    UVariable * filterOrder;
    UVariable * fileterBW;
    UVariable * filterType; ///
    UVariable * scRollPeriods; /// number of used roll periods - Spectral analysis
    UVariable * wg2RollPeriods; /// number of used roll periods - Weibull method
    UVariable * initRollPeriods; /// initial estimate after
    UVariable * scThreshold; /// spectral corr alarm threshold
    UVariable * wg2Threshold; /// Weibull alarm threshold
    UVariable * alarmWindLen; /// size of detection window
    UVariable * alarmWindOverlap; ///
    UVariable * rollMinVar; /// minimum variance for roll
    UVariable * pitchMinVar; /// minimum variance for pitch
    UVariable * horizon6h; /// (rw) number of minutes in the 6h period");
    UVariable * shapeUpdTime; /// (rw) time between update of long term parameters (shape0 and scale0)");
    UVariable * filteredBufferTime; /// length of buffer (in seconds) for band-pass filtered values.
  } varp;
  struct
  {
    UVariable * scFlag; ///(r) Spectral correlation alarm detected
    UVariable * wg2Flag;///(r) Double Weibull GLRT alarm detection
    UVariable * alertFlag;///(r) alert flag
    UVariable * scIndex; ///(r) Spectral correlation detection value
    UVariable * wg2Index; ///(r) Double Weibull GLRT detection value
    UVariable * alertIndex; ///(r) combined alert index
    UVariable * roll6h; /// (r) max roll value the last 6 hours");
    UVariable * pitch6h; /// (r) max pitch value the last 6 hours");
    UVariable * alarmCnt6h; /// (r) number of alarms last 6 hours");
    UVariable * roll1min; /// (r) max roll value the last 1 minute");
    UVariable * pitch1min; /// (r) max pitch value the last 1 minute");
    UVariable * alarmCnt1min; /// (r) number of alarms last 1 minute");
    UVariable * time; /// last update time for alert index
    UVariable * timeLong; /// last update time for long term variables
    UVariable * calcTime; /// last time to calculate - in msec
    UVariable * cnt; /// number of updates
    UVariable * w2gShape; /// estimated Weibull shape parameter in last calculation
    UVariable * w2gScale; /// estimated Weibull scale parameter in last calculation
  } vars;
  /**
  Logfile */
  ULogFile logf;
  /**
  Parametric roll temporary values */
  double * rollRaw;
  double * pitchRaw;
  int rollRawCnt; ///- number of measurements in buffer 
  double * yawRaw;
  double * timeRaw;
  UTime rawUpdTime; /// timestamp for newest raw update
  // filtered values
  double * rollFilt;
  double * pitchFilt;
  int rollFiltMaxCnt; ///- number of elements in band-pass filtered signal
  UTime filtUpdTime; ///- timestamp for newest filtered update (at rollFiltIdx)
  int rollFiltIdx;  ///- index to most recent update
  int rollFiltCnt; ///- number of available updates
  
  double * w2gDrive;
  /**
  size of roll sample buffer */
  int imuSampleMaxCnt;
  int imuSampleCnt;
  /**
  time for last 1min slize */
  UTime hoz1min; /// update time for last 1 minute interval
  /// time for last sample
  UTime tLastSample;
  /**
   * debug variables */
  double sweepAngle;
  int sweepStep;
  UTime sweepTime;

};


#endif

