/***************************************************************************
 *   Copyright (C) 2010 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 ***************************************************************************/
#ifndef UFUNC_MINIVAR_H
#define UFUNC_MINIVAR_H

#include <urob4/ufuncplugbase.h>
#include <ulms4/ufunclaserbase.h>
#include <cstdlib>

#define ROBOB 0.4
#define MINRANGE 0.07
#define ROBOW 0.3

/*
 * This is a simple example plugin, that manipulates global variables, both of its own, and those of other modules.
@author Christian Andersen
*/
class Funcavoid : public UFuncLaserBase
{ // NAMING convention recommend that the plugin function class
  // starts with UFunc (as in Funcavoid) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  Funcavoid()
  { // command list and version text
    setCommand("avoidk", "avoidpath", "Plug-in to avoid obstacles");
    logf.setLogName("avoidk");
    maal_x = 0;
    maal_y = 0;
    nummer=1;
    faerdig=0;
    }
  /**
   * Handle incomming commands flagged for this module
   * \param msg pointer to the message and the client issuing the command
   * \return true if the function is handled - otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  //
private:


  void findSpring(ULaserData * data1, double huller[100][5]);	//tested
  void undersoeg(ULaserData * data1, double huller[100][5], double * drej, double * koer, double * drejEfter);		//tested
  bool direkte(ULaserData * data1,double x,double y);			//tested - virker overbevisende
  void korrigerHuller(double huller[100][5]);					//tested
  void sorterArray(double huller[100][5]);						//tested
  void sorterPotArray(double huller[7000][5]); 					//tested
  void sletHuller(double huller[100][5]);						//teoretisk tested
  void findHuller(ULaserData * data1, double huller[100][5]);	//tested
  double tilPI(double tal);										//tested
  double findDrej(double x, double y);							//tested
  double findDrejEfter(double x, double y);						//tested



  /**
  Handles to "own" global variables. */

  double maal_x, maal_y;
  int nummer, faerdig;
  UPose posi;



  UVariable * varLogName;
  UVariable * varUpdateCnt;
  UVariable * varPose;
  UVariable * varTime;
  UVariable * varAUMatrix;
  UVariable * varCVMatrix;
  /**
  Logfile */
  ULogFile logf;
};


#endif

