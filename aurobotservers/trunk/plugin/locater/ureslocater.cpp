/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 * ----------------------------------------------------------------------- *
 *   Plugin for aurobotservers version 2.206.                              *
 *   Does laserbased localisation for a robot running in an orchard        *
 *   environment.                                                          *
 *   Edited by Peter Tjell (s032041) & Søren Hansen (s021751)              *
 * ----------------------------------------------------------------------- *
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
/**
@author Christian Andersen
@editor Peter Tjell / Søren Hansen
*/

#include <stdio.h>
#include <math.h>
#include <urob4/uvarcalc.h>
#include <urob4/uresposehist.h>
#include <umap4/upose.h>
#include <ugen4/uline.h>
#include <urob4/uresvarpool.h>
#include <ugen4/upolygon.h>

#include "ureslocater.h"




//unsigned int noupdate_cnt = 0;

/* Allocate space for the matrices in the RAM */
void UResLocater::make_matrix()
{
 /* State variables */
  x = mmake(3, 1);
  x_ = mmake(3, 1);
  x_up = mmake(3, 1);
  x_odo = mmake(3, 1);

  /* Measurement variable */
  z = mmake(2, 1);
  z3 = mmake(3, 1);

  /* Covariance matrix */
  P = mmake(3, 3);
  P_ = mmake(3, 3);

  /* Variances control signal */
  Q = mmake(2, 2);

  /* Variances measurement signal */
  R = mmake(2, 2);
  R3 = mmake(3, 3);

  /* Jacobians - and temps */
  df_x = mmake(3, 3);
  df_x_trans = mmake(3, 3);
  df_u = mmake(3, 2);
  df_u_trans = mmake(2, 3);
  P_1 = mmake(3, 3);
  P_2 = mmake(3, 3);
  P_3 = mmake(3, 3);
  P_4 = mmake(3, 3);
  eye3 = mmake(3, 3);
  H = mmake(2, 3);
  H_trans = mmake(3, 2);
  H3 = mmake(3, 3);
  H3_trans = mmake(3, 3);
  S = mmake(2, 2);
  S_inv = mmake(2, 2);
  S3 = mmake(3, 3);
  S3_inv = mmake(3, 3);
  //z_map = mmake(2, 1);

  /* Kalman gain */
  K = mmake(3, 2);
  K3 = mmake(3, 3);
  //KL = mmake(3, 2);
  //K3L = mmake(3, 3);

  /* Innovation */
  y_hat = mmake(2, 1);
  y3_hat = mmake(3, 1);
  // temporary matrices
  tmp = mmake(3, 3);
  tmp33 = mmake(3, 3);
  tmp23 = mmake(2, 3);
  tmp32 = mmake(3, 2);
  tmp22 = mmake(2, 2);
  tmp31 = mmake(3, 1);

}

////////////////////////////////////////////////

/** Initialize the matrices */
void UResLocater::init_matrix()
{

  /* State variable */
  minit(x);
  minit(x_);
  minit(x_up);
  minit(x_odo);

  /* Measurement variable */
  minit(z);

  /* Covariance matrix */
  mdiag(P, 0.001);
  mput(P, 2, 2, 0.01);
  minit(P_);
  mPu.setSize(3, 3);
  mXu.setSize(3, 1);

  /* Variances control signal */
  minit(Q);

  /* Variances measurement signal */
  minit(R);
  mput(R, 0, 0, var_d);
  mput(R, 1, 1, var_phi);
  minit(R3);
  mput(R3, 0, 0, var_d);
  mput(R3, 1, 1, var_phi);
  mput(R3, 2, 2, var_t);

  /* Jacobians - and temps */
  minit(df_x);
  minit(df_x_trans);
  minit(df_u);
  minit(df_u_trans);
  minit(P_1);
  minit(P_2);
  minit(P_3);
  minit(P_4);
  minit(tmp);
  minit(tmp23);
  minit(tmp32);
  minit(tmp33);
  minit(tmp22);
  minit(tmp31);
  mdiag(eye3, 1.0);
  minit(H);
  minit(H_trans);
  minit(H3);
  minit(H3_trans);
  minit(S);
  minit(S_inv);
  minit(S3);
  minit(S3_inv);
  //minit(z_map);

  /* Kalman gain */
  minit(K);
  minit(K3);
  //minit(KL);
  //minit(K3L);

  /* Innovation */
  minit(y_hat);
  minit(y3_hat);
}

//////////////////////////////////////////////

UResLocater::UResLocater()
{ // these first two lines is needed
  // to save the ID and version number
  setResID("locater", 10);
  setDescription("Calculates robot position based on lasermeasurements and map information.", false);
  createBaseVar();
  resVersion = getResVersion();
  // other local initializations
  //reslaser = NULL;
  //poseHist = NULL;
  serial = 0;
  // set log names - in data path
  fp.setLogName("locater");
  testfid.setLogName("locater_lines");
  logUpd.setLogName("locater-updates");
  logEnd.setLogName("locater-rowend");

  /* Load standard mapfile if existing */
  /*mapfile = "map.xml";
  mapload();
  */
  /* Initialize parameters to standard values. The variable can later be altered in the function interface */

  /* Robot parameters */
  axle_dist = 1.597;

  /* Ackermann constants standard deviation on distance, angle and steering */
  E_L = 0.3; //0.1; //0.05;
  E_A = 0.088;
  E_SA = 0.04; //0.01;

  /* Laser measurement variances */
  var_d = 2.25; // across line - should be perimeter dependent
  var_t = 2.25; // along line (line end update) - std 1.5 m - ok - should be perimeter dependent
  var_phi = 0.121846; // angle (7 grader)

  /* Limits used to sort lines */
  min_points = 15;



  //skift til global value
  max_angle_deviation = 6 * M_PI / 180;  // rad

  /* Initialize log counters */
/*  log_cnt = 0;
  log_linecnt = 0;*/

  update_allow = true;

  /* Create and initialize matrices in iau-mat */
  make_matrix();
  init_matrix();
  initPoseOK = false;
  poly400 = NULL;
  poly40 = NULL;
}

////////////////////////////////////////////////

UResLocater::~UResLocater()
{
  /* Close and SAVE the logs to files */
  closelog();
  printf("%c[31m --  UResLocater close  --%c[0m\n", 0x1B, 0x1B);
}

void UResLocater::set_state(char *var, double val)
{ // Sets the state of the EKF (matrix x) manually. Used for initalizing
  // the filter.
  //
  bool changed = false;
  UPose tmp;
  UPoseTVQ tvqtmp;
  UResPoseHist *mapposehist;
  UResPoseHist *odoposehist;

  if(strcasecmp(var, "x") == 0)
  {
    mput(x, 0, 0, val);
    tmp.x = val;
    tmp.y = mget(x, 1, 0);
    tmp.h = mget(x, 2, 0);
    changed = true;
  }
  else if(strcasecmp(var, "y") == 0)
  {
    mput(x, 1, 0, val);
    tmp.x = mget(x, 0, 0);
    tmp.y = val;
    tmp.h = mget(x, 2, 0);
    changed = true;
  }
  else if(strcasecmp(var, "t") == 0)
  { // heading
    mput(x, 2, 0, angle_normalize(val));
    tmp.x = mget(x, 0, 0);
    tmp.y = mget(x, 1, 0);
    tmp.h = angle_normalize(val);
    changed = true;
  }

  if(changed)
  {
    this->lock();
      varkalmpose->setPose(&tmp);
    this->unlock();

    mapposehist = (UResPoseHist *) getStaticResource("mapPose", true);
    odoposehist = (UResPoseHist *) getStaticResource("odoPose", true);
    if(mapposehist != NULL and initPoseOK and varKeepMapPose->getBool())
    { // get odometry update time (works better in especially replay)
      tvqtmp = odoposehist->getNewest();
      tvqtmp.set(tmp.x, tmp.y, tmp.h, tvqtmp.t, 0.0, 1.0);
      if (tvqtmp.t.getSec() < 10)
        // supplement with current time - if no time in odoPoseHist
        tvqtmp.t.now();
      // update map-pose
      mapposehist->addIfNeeded(tvqtmp, URESLOCATER_ID_OK);
    }
  }
  if (poly400 != NULL)
    delete poly400;
  if (poly40 != NULL)
    delete poly40;
}

///////////////////////////////////////////

void UResLocater::set_utmstate()
{ // Sets the state of the EKF (matrix x) and mapPose to the newest gps position stored in utmPose
  //

  UPoseTVQ gpspose;
  UPose tmppose;
  UResPoseHist *gpsposehist;
  UResPoseHist *mapposehist;
  UResMapbase * mapbase;
  UPose mapref;
  UPose mapPose;

  gpsposehist = (UResPoseHist *) getStaticResource("utmPose", true);
  mapposehist = (UResPoseHist *) getStaticResource("mapPose", true);
  mapbase = (UResMapbase *) getStaticResource("mapbase", false, false);

  if((gpsposehist != NULL) && (mapposehist != NULL) and (mapbase != NULL))
  {
    gpspose = gpsposehist->getNewest();
    tmppose = gpspose.getPose();
    // map pose is relative to the map origin node position
    // defined in graph with for the node referenced in the map
    mapref = mapbase->getMapRefPose();
    // convert current gps pose to map coordinates
    mapPose = mapref.getMapToPosePose(&gpspose);
    /* Correct offset values */
    if (gpspose.q > 0.1)
    { // use only if some quality - no utmPose gives quality -1
      tmppose.x = tmppose.x - OX;
      tmppose.y = tmppose.y - OY;
      gpspose.set(tmppose, gpspose.getPoseTime().t, gpspose.vel, gpspose.q);
      if (varKeepMapPose->getBool())
        mapposehist->addIfNeeded(gpspose, URESLOCATER_ID_SET);
      initPoseOK = true;
      mdiag(P, 0.004);
      switch (roundi(gpspose.q))
      { // these values are a guess from seen values, and not from spec.
        // should be updated when GPS values are available
        case 2: // float
          mput(P, 0, 0, 25.0); // 5m
          mput(P, 1, 1, 25.0); // 5m
          mput(P, 2, 2, 0.09); // 18 degrees
          break;
        case 3: // fix
          mput(P, 0, 0, 0.0016); // 4cm
          mput(P, 1, 1, 0.0016); // 4cm
          mput(P, 2, 2, 0.01);   // 6 degrees
          break;
        case 4: // autonomous
          mput(P, 0, 0, 4.0);   // 2m
          mput(P, 1, 1, 4.0);   // 2m
          mput(P, 2, 2, 0.04);  // 12 degrees
          break;
        default: // no fix? - is this different from old-position?
          mput(P, 0, 0, 225.0); // 15m
          mput(P, 1, 1, 225.0); // 15m
          mput(P, 2, 2, 0.14);  // 24 degrees
          break;
      }
      mset(P_, P);
    }

    this->lock();
      mput(x, 0, 0, tmppose.x);
      mput(x, 1, 0, tmppose.y);
      mput(x, 2, 0, tmppose.h);
      varkalmpose->setPose(&tmppose);
      varUpdcnt->setInt(0);
    this->unlock();

    //printf("UTM state set to: x = %f, y = %f, t = %f\n", mget(x, 0, 0), mget(x, 1, 0), mget(x, 2, 0));
  }
  else
  {
    printf("No utmPose or mapPose resource available!\n");
  }
}

void UResLocater::set_poselast()
{ // Sets the old odometry position, such that
  // extract_control works properly
  //
  UPose pose;
  UResPoseHist *posehist;

  posehist = (UResPoseHist *) getStaticResource("odoPose", true);

  if(posehist != NULL)
  {
    pose = posehist->getNewest();
    this->lock();
      pose_last = pose;
    this->unlock();
    //printf("pose_last: x = %f, y = %f, t = %f\n", pose.x, pose.y, pose.h);
  }
  else
  {
    printf("No odoPose resource available!\n");
  }
}


// Find the intersection between row and path - use as target
/*void UResLocater::set_target(int rowno, int pathno)
{

  double x1, y1, x2, y2;
  UResMapbase *mb;

  // Acquire mapbase resource:
  mb = (UResMapbase *) getStaticResource("mapbase", false, false);

  if(mb != NULL)
  {
    if(mb->path_cnt >= pathno && mb->map_cnt > rowno)
    {
      x1 = (mb->path[pathno-1].C * mb->map[rowno].B - mb->map[rowno].C  * mb->path[pathno-1].B) / (mb->map[rowno].A * mb->path[pathno-1].B - mb->path[pathno-1].A * mb->map[rowno].B);
      y1 = - mb->path[pathno-1].A / mb->path[pathno-1].B * x1 - mb->path[pathno-1].C / mb->path[pathno-1].B;

      x2 = (mb->path[pathno-1].C  * mb->map[rowno-1].B - mb->map[rowno-1].C * mb->path[pathno-1].B) / (mb->map[rowno-1].A * mb->path[pathno-1].B - mb->path[pathno-1].A * mb->map[rowno-1].B);
      y2 = - mb->path[pathno-1].A / mb->path[pathno-1].B * x2 - mb->path[pathno-1].C / mb->path[pathno-1].B;

      xtarget = (x1 + x2) / 2.0;
      ytarget = (y1 + y2) / 2.0;
      ttarget = atan2(-mb->map[rowno-1].A, mb->map[rowno-1].B);

      printf("set_target: x = %f, y = %f, t = %f\n", xtarget, ytarget, ttarget);
    }
    else
      printf("Row or path number out of range\n");
  }
  else
    printf("No mapbase available\n");


  printf("WARNING: Set_target depricated\n");
}
*/
bool UResLocater::methodCall(const char *name, const char *paramOrder,
                             char **strings, const double *pars,
                             double *value,
                             UDataBase **returnStruct, int *returnStructCnt)
{
  bool result = true;
/*
  if((strcasecmp(name, "correct") == 0) && (strcmp(paramOrder, "dddd") == 0))
  {
    drive(pars[0], pars[1], pars[2]);
    printf("WARNING: locate.correct used with parameters!!!!!\n");
  }
  if((strcasecmp(name, "correct") == 0) && (strcmp(paramOrder, "d") == 0))
  {
    driveangle(xtarget, ytarget, ttarget, 15);
  }
  else
  {
    result = false;
  }
*/
  if((strcasecmp(name, "resetstate") == 0) && (strcmp(paramOrder, "") == 0))
  {
    set_utmstate();
    set_poselast();
  }
  else if((strcasecmp(name, "utmToPose") == 0) && (strlen(paramOrder) == 0))
  {
    setUtmPoseToMapPose();
  }
  else
  {
    result = false;
  }

  return result;
}

////////////////////////////////////////////////////////////////////

void UResLocater::retrieveData(ULaserData *data, RangeData *rd)
{
  double d;
  bool valid;

  rd->points = data->getRangeCnt();

  for (int i = 0; i < rd->points; i++)
  {
    // Get angle as radians
    rd->point_th[i] = data->getAngleRad(i);

    // Get range and translate to meters
    d = data->getRangeMeter(i, &valid);

    if(!valid)
    {
      d = OUT_OF_RANGE;
    }

    rd->point_r[i] = d;
  }

  rd->polarToCart();
}

void UResLocater::createBaseVar()
{
  varkalmpose = addVar("kalmpose", "0.0 0.0 0.0", "pose", "(r) Pose according to localisation");
  /*varXk = addVar("xk", 0.0, "d", "Kalman filter x variable");
  varYk = addVar("yk", 0.0, "d", "Kalman filter y variable");
  varTk = addVar("tk", 0.0, "d", "Kalman filter t variable");
  varXrel = addVar("xrel", 0.0, "d", "Relative x coordinate (in odo system)");
  varYrel = addVar("yrel", 0.0, "d", "Relative y coordinate (in odo system)");
  varTrel = addVar("trel", 0.0, "d", "Relative t coordinate (in odo system)");
  varDist = addVar("dist", 0.0, "d", "Distance to target/drive (in odo system)");*/
  varUpdcnt = addVar("updcnt", "0.0 0.0 0.0", "d", "(r/w) Counts number of updates, [0]=scans since last update, [1] number of locater updates, [2]=updates since last miss.");
  varKeepMapPose = addVar("keepMapPose", 1.0, "d", "(r/w) Should locater maintain mapPose (1=true) when update is available");
  varUseRowEnd = addVar("useRowEnd", 1.0, "d", "(r/w) determines if row end detections should be used in locater pose update");
  varRowEndIn1 = addVar("rowEndIn1", 4.0, "d", "distance inside coverage before row end detect - when row end is expected out of range");
  varRowEndIn2 = addVar("rowEndIn2", 2.0, "d", "distance inside coverage before row end detect - when row end is expected in range");
  varRowEndMaxGap = addVar("rowEndMaxGap", 4.0, "d", "(rw) Max opening in a detected row to be valid (meter)");
  varUseAvgUpdate = addVar("useAvgUpdate", 0.0, "d", "(rw) Flag if averaged update is to be used rather than update for each line individually");
  varRowToRobotMaxDist = addVar("rowToRobotMaxDist", 4.0, "d", "(rw) maximum this far from robot if usable for row end detection.");
  varRowEndMaxDist = addVar("rowEndMaxDist", 4.5, "d", "(rw) reject row end detections more than this away from expected position (m)");
  varOdoDistScale = addVar("odoDistScale", 1.0, "d", "(rw) multiplied on driven distance in locater update");
  varRowExtraLog = addVar("rowExtraLog", 0.0, "d", "(r/w) Make extra log of correlations to mapped rows - much data");
  varRowWidthFactor = addVar("rowWidthFactor", 1.0, "d", "(r/w) Adjust the row perimeter (width) stated in map with this factor");
  varMaxAngleDeviation = addVar("maxAngleDeviation", 6, "d", "(r/w) accept line detect if withint this angle (deg) of mapped line");
  // Methods available:
  addMethod("resetstate", "", "Set the state to the pose (in utmPose) and resets old odoposes");
  addMethod("utmToMap", "", "Set map pose from the utm pose - when keepMapPose is false only");
}

////////////////////////////////////////////////////////

void UResLocater::openlog()
{
  fp.openLog();
  testfid.openLog();
  logUpd.openLog();
 // logEnd.openLog();
}

/////////////////////////////////////////////

void UResLocater::closelog()
{
  fp.closeLog();
  testfid.closeLog();
  logUpd.closeLog();
  logEnd.closeLog();
}

//////////////////////////////////////////////////////////

void UResLocater::varprint()
{
  double x, y, t;

  this->lock();
    x = varkalmpose->getPose().x;
    y = varkalmpose->getPose().y;
    t = varkalmpose->getPose().h;
  this->unlock();

  printf("\nPoint: x = %f, y = %f, t = %f\n", x, y, t);
}


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void UResLocater::odoPredict(UPose * poseScan)
{
  double delta_theta;
  double th;
  double L, theta_sa;
  /* Extract control signals from pose and pose_last */
  extract_control(poseScan, pose_last, axle_dist, &L, &theta_sa);
  // multiply with scale to improve accuracy - remove known bias
  L *= varOdoDistScale->getValued();
  delta_theta = L * tan(theta_sa) / axle_dist;
  th = angle_normalize(mget(x, 2, 0) + delta_theta / 2.0);
  /*printf("L = %f, th = %f, delta_theta = %f, theta_sa = %f\n", L, th, delta_theta, theta_sa);*/
  /* Calculate Jacobians */
  mput(df_x, 0, 2, -L*sin(th));
  mput(df_x, 1, 2, L*cos(th));
  /*printf("df_x =\n"); mprint(df_x);*/
  mput(df_u, 0, 0, cos(th) - sin(th) * delta_theta / 2.0);
  mput(df_u, 0, 1, -sin(th) / 2.0 * (pow(L, 2) / axle_dist + axle_dist * pow(delta_theta, 2)));
  mput(df_u, 1, 0, sin(th) + cos(th) * delta_theta / 2.0);
  mput(df_u, 1, 1, cos(th) / 2.0 * (pow(L, 2) / axle_dist + axle_dist * pow(delta_theta, 2)));
  mput(df_u, 2, 0, tan(theta_sa) / axle_dist);
  mput(df_u, 2, 1, L / axle_dist + tan(theta_sa) * delta_theta);
  /*printf("df_u =\n"); mprint(df_u);*/
  mput(Q, 0, 0, pow(E_L, 2) * fabs(L));
  mput(Q, 1, 1, pow(tan(E_A + E_SA * fabs(theta_sa)) / axle_dist, 2) * fabs(L));
  /*printf("Q =\n"); mprint(Q);*/

  /* Prediction of state */
  mput(x_odo, 0, 0, L * cos(mget(x, 2, 0) + 0.5 * delta_theta));
  mput(x_odo, 1, 0, L * sin(mget(x, 2, 0) + 0.5 * delta_theta));
  mput(x_odo, 2, 0, delta_theta);
  madd(x_, x, x_odo);
  mput(x_, 2, 0, angle_normalize(mget(x_, 2, 0)));

  /* Prediction of covariance */
  mtrans(df_x_trans, df_x);
  mtrans(df_u_trans, df_u);
  mmul(P_1, P, df_x_trans);
  mmul(P_2, df_x, P);
  mmul(P_3, P_2, df_x_trans);
  mmul(tmp32, df_u, Q);
  mmul(P_4, tmp32, df_u_trans);
  madd(P_, P, P_1);
  madd(P_, P_, P_2);
  madd(P_, P_, P_3);
  madd(P_, P_, P_4);

  //printf("x_ =\n"); mprint(x_);
  //printf("P_ =\n"); mprint(P_);

}

///////////////////////////////////////////////////

void UResLocater::correctionUpdate(double dist_meas, double dist_pred,
                                   double ang_dev,
                                   mapline * line)
{
  double denom_line;
  /* Measurement residual */
  mput(y_hat, 0, 0, dist_meas - dist_pred);
  mput(y_hat, 1, 0, ang_dev);
              //printf("y_hat =\n"); mprint(y_hat);
              //printf("ls_offset = %f\n", ls_offset);

  denom_line = hypot(line->A, line->B);
  /* Measurement Jacobian */
  mput(H, 0, 0, line->A / denom_line);
  mput(H, 0, 1, line->B / denom_line);
  mput(H, 1, 2, -1);
  //printf("H =\n"); mprint(H);

  /* Covariance residual (S): */
  mmul(tmp23, H, P_);
  mtrans(H_trans, H);
  mmul(S, tmp23, H_trans);
  madd(S, S, R);
  //printf("S =\n"); mprint(S);

  /* Kalman gain (K): */
  minvgauss(S_inv, S);
  mmul(tmp32, P_, H_trans);
  mmul(K, tmp32, S_inv);
  //printf("K =\n"); mprint(K);
/*  mset(KL, K);
  mput(KL, 0, 1, 0.0);
  mput(KL, 1, 1, 0.0);
  mput(KL, 2, 0, 0.0);*/
  //printf("KL' =\n"); mprint(KL);

  // debug
  //printf("x-z (y_hat) =\n"); mprint(y_hat);
  mmul(tmp31, H_trans, y_hat);
  //printf("H' * y_hat =\n"); mprint(tmp31);
  // debug end
  /* State update: */
  mmuladd(x_, x_, K, y_hat, (char *) "++");
  mput(x_, 2, 0, angle_normalize(mget(x_, 2, 0)));
  //printf("state (x,y,h) =\n"); mprint(x_);

  /* Covariance update: */
  mmul(tmp, K, H);
  msub(tmp, eye3, tmp);
  mmul(tmp33, tmp, P_);
  mset(P_, tmp33);
  //printf("P =\n"); mprint(P_);
}

///////////////////////////////////////////////////////////

void UResLocater::correctionUpdate3(double dist_meas, double dist_pred,
                                   double ang_dev,
                                   double rowEndDev,
                                   mapline * line)
{
  double denom_line;
  /* Measurement residual */
  mput(y3_hat, 0, 0, dist_meas - dist_pred);
  mput(y3_hat, 1, 0, ang_dev);
  mput(y3_hat, 2, 0, rowEndDev);

              //printf("ls_offset = %f\n", ls_offset);

  denom_line = hypot(line->A, line->B);
  /* Measurement Jacobian */
  mput(H3, 0, 0, line->A / denom_line);
  mput(H3, 0, 1, line->B / denom_line);
  mput(H3, 1, 2, -1);
  mput(H3, 2, 0, -line->B);
  mput(H3, 2, 1, line->A);
  //printf("H3 =\n"); mprint(H3);

  /* Covariance residual (S): */
  mmul(tmp33, H3, P_);
  mtrans(H3_trans, H3);
  mmul(S3, tmp33, H3_trans);
  madd(S3, S3, R3);
  //printf("S3 =\n"); mprint(S3);

  /* Kalman gain (K): */
  minvgauss(S3_inv, S3);
  mmul(tmp33, P_, H3_trans);
  mmul(K3, tmp33, S3_inv);
  //printf("K3 =\n"); mprint(K3);
/*  mset(K3L, K3);
  mput(K3L, 0, 1, 0.0);
  mput(K3L, 1, 1, 0.0);
  mput(K3L, 2, 0, 0.0);
  mput(K3L, 2, 2, 0.0);*/
  //printf("K3L =\n"); mprint(K3L);

  /* State update: */
  // debug
/*  printf("x-z (y3_hat) =\n"); mprint(y3_hat);
  mmul(tmp31, K3L, y3_hat);
  printf("K*(x-z) =\n"); mprint(tmp31);*/
  // debug end
  mmuladd(x_, x_, K3, y3_hat, (char *) "++");
  //printf("new state (x_k)=\n"); mprint(x_);
  mput(x_, 2, 0, angle_normalize(mget(x_, 2, 0)));

  /* Covariance update: */
  mmul(tmp, K3, H3);
  msub(tmp, eye3, tmp);
  mmul(tmp33, tmp, P_);
  mset(P_, tmp33);
  printf("P =\n"); mprint(P_);

}

//////////////////////////////////////////////////

bool UResLocater::rowEndDetect(UPose mPose, UPose lPose, double maxRange,
                               int lineIdx, mapline * line,
                               double * rend_dev,
                               const char * lineName, UTime scanTime)
{
  const int MHL = 100;
  int hit[MHL + 2];
  int i;
  const int MIN_MEASUREMENTS = 7;
  double tLas; // parameter value for robot and laser pose
  double tSta, tEnd; // value for start and end of row line
  double tMaxRange; // max t value within laser range
  double tEst; // estimated position of line end
  double dLine; // distance from robot to line
  int n, n2, h, hi, hs, he;
  bool findStart;
  bool endIsInCoverage;
  double dh = (maxRange * 2.0) / double(MHL);
  int maxHist, mh2; // max histogram length and half length
  //
  /// if line end is out of range, then detected end must at lest 3 m inside coverage
  int safeDetectOut = roundi(varRowEndIn1->getValued() / dh);
  /// if line expected in range, then at least (1m) past end must be visible
  int endIsIn = roundi(varRowEndIn2->getValued() / dh);
  double t, *hx, *hy;
  double sSum, sMax, sMaxh;
  double muRow = 20.0/16.0*dh;  // row hits over histogram (16m)
  double varRow = sqr(muRow / 2.0);  // estimated 1 sigma distribution if row
  double muNoRow = 3.0/16.0*dh; // no-row hits over histogram (16 m)
  bool useResult = false;
  int gap, gapMax;
  double w;
  UPosition rPosS, rPosE, ePos;
  //bool useStart, useEnd;
  //const double BEHIND_ROBOT = -3.0;
  ULogFile rowhitlog;
  //
  n = cnte_array[lineIdx];
  const int MSL = 100;
  char sname[MSL];
  // debug
  if (varRowExtraLog->getBool())
  { // open extra logfile for detections related to this line only
    snprintf(sname, MSL, "hit%s", lineName);
    rowhitlog.setLogName(sname);
    rowhitlog.setLogNameNumber((int)serial);
    rowhitlog.openLog();
    // log start and endpoints of line
    fprintf(rowhitlog.getF(), "-002 %.3f %.3f\n", line->x_s, line->y_s);
    fprintf(rowhitlog.getF(), "-001 %.3f %.3f\n", line->x_e, line->y_e);
    fprintf(rowhitlog.getF(), "%.5f %.3f %.3f\n", lPose.h, lPose.x, lPose.y);
    // log position of measurements associated to this line
    for (i = 0; i < n; i++)
      fprintf(rowhitlog.getF(), "%3d %.3f %.3f\n", i, x_array[lineIdx][i], y_array[lineIdx][i]);
    rowhitlog.closeLog();
  }
  // debug end
  if (n > MIN_MEASUREMENTS)
  { // there sufficient points for a detection
    // get reference points along line
    // t-parameter value along line at point closest to robot
    //tRob = -line->B * mPose.x + line->A * mPose.y;
    // t-parameter value along line at point closest to laser scanner
    tLas = -line->B * lPose.x + line->A * lPose.y;
    // t-parameter value for line start - relative to laser
    tSta = -line->B * line->x_s + line->A * line->y_s - tLas;
    // t-parameter value for line end - relative to laser
    tEnd = -line->B * line->x_e + line->A * line->y_e - tLas;
    // distance from robot to line
    dLine = line->A * lPose.x + line->B * lPose.y + line->C;
    //
    // get relative distance of line start and end in robot coordinates
    ePos.set(line->x_s, line->y_s, 0.0);
    rPosS = lPose.getMapToPose(ePos);
    ePos.set(line->x_e, line->y_e, 0.0);
    rPosE = lPose.getMapToPose(ePos);
    // use end if end is in front of robot (-2m), or other end is further behind
    //useStart = rPosS.x > BEHIND_ROBOT or rPosE.x < rPosS.x;
    //useEnd = rPosE.x > BEHIND_ROBOT or rPosS.x < rPosE.x;
    // max t value within laser range
    tMaxRange = sqrt(sqr(maxRange) - sqr(dLine));
    // is it meaningfull to expect a line end detection at all
//    if ((useStart and fabs(tSta) < maxRange * 1.75) or (useEnd and fabs(tEnd) < maxRange * 1.75))
    if (true) //(useStart and fabs(tSta) < maxRange * 1.75) or (useEnd and fabs(tEnd) < maxRange * 1.75))
    { // there is a line end near robot
      mh2 = roundi(tMaxRange / dh);
      maxHist = 2 * mh2;
      // zero histogram
      for (h=0; h < maxHist; h++)
        hit[h] = 0;
      // get pointer to detections
      hx = xe_array[lineIdx];
      hy = ye_array[lineIdx];
      n2 = 0;
      // debug
      // printf("Scan %lu Line %s had %d hits\n", serial, lineName, n);
      // debug end
      for (i = 0; i < n; i++)
      {
        t = -line->B * *hx + line->A * *hy - tLas;
        h = roundi(t / dh) + mh2;
        if (h >= 0 and h < maxHist)
        {
          hit[h]++;
          n2++;
        }
        hx++;
        hy++;
      }
      // histogram entry for start and end
      hs = roundi(tSta / dh) + mh2;
      he = roundi(tEnd / dh) + mh2;
      // find no-row end
      findStart = (fabs(tSta) < fabs(tEnd));
      if (findStart)
      { // looking for start of line
        if (tSta < tEnd)
          hi = -1; // line end has higher value - start from from high
        else
          hi = 1; // line end has lower value - start from low end
      }
      else
      { // look for end of line
        if (tEnd < tSta)
          hi = -1; // line start has higher value - start from high end
        else
          hi = 1; // line start has lower value - start from low end
      }
      // adapt probability
      w = fmin(1.0, double(n) / 40.0);
      muRow *= w;
      varRow = sqr(muRow / 2.0);
      muNoRow *= w;
      // start value
      if (hi == 1)
        h = 0;
      else
        h = maxHist - 1;
      sSum = 0.0;
      sMax = 0.0;
      sMaxh  = 0;
      gap = -1;
      gapMax = 0;
      if (logEnd.isOpen())
      {
        if (hi == 1)
          // increase
          h = -(MHL/2 - mh2);
        else
          h = MHL - (MHL/2 - mh2) - 1;
        fprintf(logEnd.getF(), "%lu.%06lu %lu \"%12s\" %d %g %g %3d %3d %d   ",
                scanTime.getSec(), scanTime.getMicrosec(), serial, lineName, MHL, MHL/2.0 * dh,  dh, hi, h, maxHist-1);
        for (i = 0; i < MHL ; i++)
        { // log histogram values
          if (h < 0 or h >= maxHist)
            fprintf(logEnd.getF(), " 0");
          else
            fprintf(logEnd.getF(), " %d", hit[h]);
          h += hi;
        }
        fprintf(logEnd.getF(), "\n");
      }
      // init h to start at right end of histogram
      if (hi == 1)
        h = 0;
      else
        h = maxHist - 1;
      if (false and logEnd.isOpen())
      {
        fprintf(logEnd.getF(), "%lu.%06lu %.3f %d   %g %g %g   ",
                scanTime.getSec(), scanTime.getMicrosec(), dh, maxHist, muRow, muNoRow, varRow);
      }
      for (i = 0; i < maxHist ; i++)
      { // calculate most likely split - from MBlanke change detection page 208 (6.65) (1st issue)
        sSum += (muRow - muNoRow)/varRow * (hit[h] - (muRow + muNoRow)/2.0);
        if (false and logEnd.isOpen())
          fprintf(logEnd.getF(), " %g", sSum);
        if (sSum > sMax)
        { // find best intersection
          sMax = sSum;
          sMaxh = h;
        }
        if (hit[h] > 0)
        {
          if (gap > gapMax)
          {
            gapMax = gap;
            // debug
            // printf("row-end-detect: gap of %d or %gm detected\n", gapMax, gapMax * dh);
            // debug end
          }
          gap = 0;
        }
        else if (gap >= 0)
          gap ++;
        // advance to next histogram value
        h += hi;
      }
      if (false and logEnd.isOpen())
      { // pad with numbers to get full row width
        for (i = maxHist; i < MHL; i++)
          fprintf(logEnd.getF(), " -1");
        fprintf(logEnd.getF(), "\n");
      }
      // evaluate validity of result.
      endIsInCoverage = ((findStart and hs >= endIsIn and hs < maxHist - endIsIn) or
                         (not findStart and he >= endIsIn and he < maxHist - endIsIn));
      if (endIsInCoverage)
        useResult = true;
      else
      { // result must be rather safe to be used
        // safe is if there is at least (~3 m) to end of range
        useResult = ((sMaxh > safeDetectOut and hi == -1) or
                     (sMaxh < maxHist - safeDetectOut and hi == 1));
      }
      if (useResult and gapMax * dh > varRowEndMaxGap->getValued())
      { // too large open gap in row, may be part of another row.
        useResult = false;
        // debug
        // printf("row-end-detect: gap too big - not used (%d or %gm)\n", gapMax, gapMax * dh);
        // debug end
      }
      if (useResult)
      { // find end of row
        tEst = (sMaxh - mh2) * dh;
        // compensate for row thickness
        if (hi == 1)
          // perimeter is assumed to be something like 2 sigma
          tEst -= line->perimeter / 2.0;
        else
          tEst += line->perimeter / 2.0;
        // difference from expected end of row
        if (findStart)
          // found new start position
          *rend_dev = tSta - tEst;
        else
          // found new end position
          *rend_dev = tEnd - tEst;
      }
      // print result
//       printf("%s (%d/%3d dt=%.2f) :", lineName, n2, n, *rend_dev);
//       for (h = 0; h < maxHist; h++)
//       {
//         printf("%2d", hit[h]);
//         if (h == hs)
//           printf("s");
//         if (h == he)
//           printf("e");
//         if (h == sMaxh)
//           // detection
//           printf("*");
//       }
//       printf("\n");
    }
  }
  return useResult;
}

//////////////////////////////////////////////////

/* EKF loop */
void UResLocater::locate(ULaserData * pushData, bool silent)
{
  ULaserPool *reslaser = NULL;  // <-- NOTICE !!
  RangeData range_data;
  ULaserDevice *dev = NULL;
  ULaserData ldata;
  ULaserData *pdata = pushData;
  UResPoseHist *poseHist;
  UResPoseHist *mappose;
  UPoseTVQ tvqtmp;
  bool newData = false;
  int q;
  UPose pose_scan, pose_kalman, mPose, lPose;
  regtype reg;
  bool update_flag = false;
  double ang_dev = -1.0;
  double dist_meas, dist_pred;
  double x_fwd, y_fwd;
  UResMapbase *mapbase;
  bool POINT_LIMIT = true;
  bool ANGLE_LIMIT = true;
  UTime scanTime;
  int bestLine = 0;
  int bestLineCnt = -1;
  int linesOK = 0, linesFailed = 0;
  double rend_dev;
  bool rendOK;
  ULocaterLineResult * ld;
  double * pd;
  int i, r, c = 0; //, y[3];

  // Acquire laser resource:
  reslaser = (ULaserPool *) getStaticResource("lasPool", true);
  if (reslaser != NULL)
    dev = reslaser->getDefDevice();

  // Acquire odopose resource:
  poseHist = (UResPoseHist *) getStaticResource("odoPose", true);

  // Acquire mapbase resource:
  mapbase = (UResMapbase *) getStaticResource("mapbase", false, false);

  // Acquire mappose resource:
  mappose = (UResPoseHist *) getStaticResource("mapPose", true);

  // debug extra log
  if (varRowExtraLog->getBool() and not logEnd.isLogOpen())
    logEnd.openLog();
  else if (not varRowExtraLog->getBool() and logEnd.isLogOpen())
    logEnd.closeLog();


  if(reslaser == NULL || dev == NULL)
  {
    printf("UResLocater::locate() - No laser pool or device available\n");
  }
  else if(poseHist == NULL)
  {
    printf("UResLocater::locate() - No odopose available\n");
  }
  else if(mapbase == NULL)
  {
    printf("UResLocater::locate() - No mapbase available\n");
  }
  else if(mappose == NULL)
  {
    printf("UResLocater::locate() - No mappose available\n");
  }
  else
  {
    if (pdata == NULL)
    { // get data from laser device
      pdata = &ldata;
      newData = dev->getNewestData(pdata, serial, 0);
    }
    else
      newData = true;
  }
  /* If scandata is available run the loop */
  if(newData)
  {
    ls_offset = dev->getDevicePos().x;
    serial = pdata->getSerial();
    //
    retrieveData(pdata, &range_data);
    //
    /* get pose at scantime */
    scanTime = pdata->getScanTime();
    pose_scan = poseHist->getPoseAtTime(scanTime);
    //
    /* update state with robot odometry movement */
    odoPredict(&pose_scan);
    //
    /* Measurement matching and update procedure: */
    if(update_allow)
    { /* Match the laserdata with the map */
      max_angle_deviation = varMaxAngleDeviation->getDouble() * M_PI / 180.0;
      match_points(range_data, x_,
/*                   cnt_array, x_array, y_array,
                   cnte_array, xe_array, ye_array,*/
                   ls_offset, mapbase);
      // x_ is old map coordinates for robot pose
      mset(x_up, x_);
      //printf("current state (x_up)=\n"); mprint(x_up);

      // zero number of detections
      locDecsCnt = 0;
      //
      for(q = 0; q < (int) mapbase->maplines.size(); q++)
      { // for debug - find line with best hit number
        if (cnt_array[q] > bestLineCnt)
        {
          bestLine = q;
          bestLineCnt = cnt_array[q];
        }
        /* If number of points is sufficient calculate regression line: */
        if(cnt_array[q] > min_points / 3)
        {
          POINT_LIMIT = false;
          reg.N = cnt_array[q];
          reg.x_array = x_array[q];
          reg.y_array = y_array[q];
          linear_regression(&reg);
          // debug monitoring of row forming
          makeRowPolygon(reg, mapbase->maplines[q]);
          // estimate line
          // get angle difference compared to mapped angle
          ang_dev = angle_deviation(mapbase->maplines[q].A, mapbase->maplines[q].B, reg.a, reg.b);
          /*printf("reg.a = %f, reg.b = %f, reg.c = %f\n", reg.a, reg.b, reg.c);*/
          if (reg.N > min_points or
              ((reg.dMax - reg.dMin) < 0.25 * (reg.tMax - reg.tMin) and
               (reg.tMax - reg.tMin) > 1.0))
          { // if many points, or favorabel geometry
            //printf("Current angle: %f\n", fabs(ang_dev));
            if(fabs(ang_dev) < max_angle_deviation)
            { /* If line and map angle deviation is within allovable angle use line to update state */
              ANGLE_LIMIT = false;
              //printf("Matching with line %s\n", mapbase->maplines[q].number);
              /* Save line params */
              if (testfid.isLogOpen())
              //if(log_linecnt < LOG_LENGTH)
              {
                fprintf(testfid.getF(), "%lu.%06lu %lu %f %f %f %f %f %f\n",
                        scanTime.getSec(), scanTime.getMicrosec(),
                        serial, (double)reg.a, (double)reg.b, (double)reg.c,
                        mget(x_up, 0, 0), mget(x_up, 1, 0), mget(x_up, 2, 0));
              /* log_lines[log_linecnt][0] = serial;
                log_lines[log_linecnt][1] = reg.a;
                log_lines[log_linecnt][2] = reg.b;
                log_lines[log_linecnt][3] = reg.c;
                log_lines[log_linecnt][4] = mget(x_up, 0, 0);
                log_lines[log_linecnt][5] = mget(x_up, 1, 0);
                log_lines[log_linecnt][6] = mget(x_up, 2, 0);
                log_linecnt++;*/
              }

              x_fwd = mget(x_up, 0, 0);
              y_fwd = mget(x_up, 1, 0);
              // signed distance to estimated line (measured)
              dist_meas = dist2line(reg.a, reg.b, reg.c, x_fwd, y_fwd);
              // signed distance to mapped line (predicted)
              dist_pred = dist2line(mapbase->maplines[q].A, mapbase->maplines[q].B, mapbase->maplines[q].C, x_fwd, y_fwd);
              /* row end detect */
              // current estimate of mapPose
              mPose.set(x_fwd, y_fwd, mget(x_up, 2, 0));
              // laser scanner pose in robot coordinates
              lPose.set(ls_offset, 0.0, 0.0);
              // and in map coordinates
              lPose = mPose.getPoseToMapPose(lPose);
              // detect end of row
              rend_dev = 0.0;
              rendOK = rowEndDetect(mPose, lPose, pdata->getMaxValidRange(),
                                    q, &mapbase->maplines[q],
                                    &rend_dev,
                                    mapbase->maplines[q].number, scanTime);
              // summarize available updates
              if (locDecsCnt < MAX_LOC_DECS)
              {
                ld = &locDecs[locDecsCnt];
                ld->distMap = dist_pred;
                ld->distMeas = dist_meas;
                ld->angleDelta = ang_dev;
                ld->lineEndOK = rendOK;
                ld->lineEndDelta = rend_dev;
                ld->length = reg.tMax - reg.tMin;
                ld->width =reg.dMax - reg.dMin;
                ld->hitCnt = reg.N;
                ld->line = &mapbase->maplines[q];
                locDecsCnt++;
              }
              // individual update
              /* update kalman filter */
              if (not varUseAvgUpdate->getBool())
              {
                if (rendOK and varUseRowEnd->getBool())
                  // update with row-end detection
                  correctionUpdate3(dist_meas, dist_pred, ang_dev,
                                rend_dev,
                                &mapbase->maplines[q]);
                else
                  // update with no end detected
                  correctionUpdate(dist_meas, dist_pred, ang_dev,
                                &mapbase->maplines[q]);
                update_flag = true;
              }
  //            noupdate_cnt = 0;
              if (not silent)
              {
                if (rendOK)
                  printf("line %s, (%.2fx, %.2fy, %.1fdeg) %d pnts, "
                      "dDst=%.2fm, dAng=%.1fdeg, lng=%.1f, w=%.1f dt=%.2f (DAT)\n",
                      mapbase->maplines[q].number,
                      mapbase->maplines[q].x_s, mapbase->maplines[q].y_s,
                      atan2(mapbase->maplines[q].A, mapbase->maplines[q].B)*180/M_PI,
                      cnt_array[q], dist_meas - dist_pred, ang_dev*180.0/M_PI,
                      reg.tMax - reg.tMin, reg.dMax - reg.dMin, rend_dev);
                else
                  printf("line %s, (%.2fx, %.2fy, %.1fdeg) %d pnts, "
                      "dDst=%.2fm, dAng=%.1fdeg, lng=%.1f, w=%.1f (DA-only)\n",
                      mapbase->maplines[q].number,
                      mapbase->maplines[q].x_s, mapbase->maplines[q].y_s,
                      atan2(mapbase->maplines[q].A, mapbase->maplines[q].B)*180/M_PI,
                      cnt_array[q], dist_meas - dist_pred, ang_dev*180.0/M_PI,
                      reg.tMax - reg.tMin, reg.dMax - reg.dMin);
              }
              linesOK++;
            }
            else
            {
              if (not silent)
                printf("line %s, (%.2fx, %.2fy, %.1fdeg) %d pnts, dAng=%.1fdeg, lng=%.1f, w=%.1f (failed on angle)\n",
                     mapbase->maplines[q].number,
                     mapbase->maplines[q].x_s, mapbase->maplines[q].y_s,
                     atan2(mapbase->maplines[q].A, mapbase->maplines[q].B)*180/M_PI,
                           cnt_array[q], ang_dev*180.0/M_PI,
                           reg.tMax - reg.tMin, reg.dMax - reg.dMin);
              linesFailed++;
            }
          }
          else
          {
            if (not silent)
              printf("line %s, (%.2fx, %.2fy, %.1fdeg) %d pnts, dAng=%.1fdeg, lng=%.1f, w=%.1f (failed on on pnts/gmtry)\n",
                 mapbase->maplines[q].number,
                 mapbase->maplines[q].x_s, mapbase->maplines[q].y_s,
                 atan2(mapbase->maplines[q].A, mapbase->maplines[q].B)*180/M_PI,
                       cnt_array[q], ang_dev*180.0/M_PI, reg.tMax - reg.tMin, reg.dMax - reg.dMin);
            linesFailed++;
          }
        }
        else if (cnt_array[q] > 0)
        {
/*          printf("line %s, (%.2fx, %.2fy, %.1fdeg) %d pnts (failed on points)\n",
               mapbase->maplines[q].number,
               mapbase->maplines[q].x_s, mapbase->maplines[q].y_s,
               atan2(mapbase->maplines[q].A, mapbase->maplines[q].B)*180/M_PI,
               cnt_array[q]);*/
          if (cnt_array[q] > 3)
            linesFailed++;
        }
      }
    }
    if (locDecsCnt > 0 and (varUseAvgUpdate->getBool() or logUpd.isOpen()))
    {
      if (logUpd.isLogOpen())
      {
        // y[0] = 0.0; y[1] = 0.0, y[2] = 0.0;
        for (i = 0; i < locDecsCnt; i++)
        {
          ld = &locDecs[i];
          fprintf(logUpd.getF(), "%.lu.%06lu %lu %5.2f %d %.5f %.5f %7.2f  %5.2f %5.2f %5.2f\n",
               scanTime.getSec(), scanTime.getMicrosec(), pdata->getSerial(),
                 ld->distMap, ld->lineEndOK,
                 ld->line->A, ld->line->A, ld->line->C,
                 ld->distMap - ld->distMeas, ld->lineEndDelta, ld->angleDelta * 180.0 / M_PI);
        }
      }
//    printf("P_ before =\n"); mprint(P_);
      doLocatorUpdates(varUseAvgUpdate->getBool(), &logUpd, scanTime, pose_scan, pdata->getSerial());
      // set resulting state
      if (varUseAvgUpdate->getBool())
      {
        pd = mXu.getData();
        for (r = 0; r < 3; r++)
          mput(x_, r, c, *pd++);
        // set resulting covariance matrix
        pd = mPu.getData();
        for (r = 0; r < 3; r++)
          for (c = 0; c < 3; c++)
            mput(P_, r, c, *pd++);
        update_flag = true;
      }
    }
    /* Set the new state */
//    printf("x before =\n"); mprint(x);
//    printf("P before =\n"); mprint(P);
    mset(x, x_);
    mset(P, P_);
//    printf("x after =\n"); mprint(x);
//    printf("P after =\n"); mprint(P);
    pose_kalman.x = mget(x, 0, 0);
    pose_kalman.y = mget(x, 1, 0);
    pose_kalman.h = mget(x, 2, 0);
    tvqtmp.set(pose_kalman, scanTime, 0.0, quality());
    /* Update variables in pool */
    this->lock();
      varkalmpose->setPose(&pose_kalman);
      if (varKeepMapPose->getBool() and initPoseOK)
        mappose->addIfNeeded(tvqtmp, URESLOCATER_ID_OK);
      pose_last = pose_scan;
      // debug
/*      printf("locater pose %.2fx, %.2fy, %.1fdeg  use=%s\n",
            pose_kalman.x, pose_kalman.y, pose_kalman.h * 180.0 / M_PI,
            bool2str(varKeepMapPose->getBool() and initPoseOK));*/
      // debug end
    this->unlock();



    /* Save logdata */
    if (fp.isLogOpen())
    //if((log_cnt < LOG_LENGTH))
    {
      fprintf(fp.getF(), "%lu.%06lu %lu %.3f %.3f %.5f  %.3f %.3f %.5f  %f %f %f\n",
              scanTime.GetSec(), scanTime.GetMicrosec(),
              serial,
              mget(x, 0, 0), mget(x, 1, 0), mget(x, 2, 0),
              pose_scan.x, pose_scan.y, pose_scan.h,
              mget(P, 0, 0), mget(P, 1, 1), mget(P, 2, 2));
      //gettimeofday(&tp, &tz);
      //log_data[log_cnt][0] = tp.tv_sec + tp.tv_usec * 1e-6;
      /* log_data[log_cnt][0] = data.getScanTime().GetSec() + data.getScanTime().GetMicrosec() * 1e-6;
        log_data[log_cnt][1] = serial;
        log_data[log_cnt][2] = mget(x, 0, 0);
        log_data[log_cnt][3] = mget(x, 1, 0);
        log_data[log_cnt][4] = mget(x, 2, 0);
        log_data[log_cnt][5] = pose_scan.x;
        log_data[log_cnt][6] = pose_scan.y;
        log_data[log_cnt][7] = pose_scan.h;
        log_data[log_cnt][8] = mget(P, 0, 0);
        log_data[log_cnt][9] = mget(P, 1, 1);
        log_data[log_cnt][10] = mget(P, 2, 2);
        log_cnt++;*/
    }
    //
    if(!update_flag && update_allow)
    {
      //printf("Warning: No update!\n");
      //noupdate_cnt++;
      // increment no-update count
      varUpdcnt->add(1, 0);
      // clear consecutive update count
      varUpdcnt->setInt(0, 2);
      if (bestLineCnt > 0 and not silent)
      { // not relevant if no correlations
        if(POINT_LIMIT == true)
          printf("Locater:: Too few points, best line %s had %d correlations\n", mapbase->maplines[bestLine].number, bestLineCnt);
        else if(ANGLE_LIMIT == true)
          printf("Locater:: Angle not within limit (fabs(%g) > %g)\n", ang_dev, max_angle_deviation);
      }
    }
    else if(update_flag)
    { // increase locator update flag
      // and set no update to 0
      varUpdcnt->setInt(0);
      // increase update count (total and since last miss)
      varUpdcnt->add(1, 1);
      if (linesOK > linesFailed)
        varUpdcnt->add(1, 2);
      if (not silent)
        printf("Update successfull (maybe)!\n");
    }
  }
  else if (not silent)
    printf("UResLocater::locate() - No new laser data\n");
}

/*
   Extracts control signals from current and last pose using the Ackerman steering model
   Result is stored in *L and *theta_sa
*/
void extract_control(UPose pose_scan, UPose pose_last, double axle_dist, double *L, double *theta_sa)
{
  double dx, dy, dt, dp;

  dx = pose_scan.x - pose_last.x;
  dy = pose_scan.y - pose_last.y;
  dp = dx * cos(pose_scan.h) + dy * sin(pose_scan.h);
  if(dp >= 0)
    *L = sqrt(pow(dx, 2) + pow(dy, 2));
  else
    *L = -sqrt(pow(dx, 2) + pow(dy, 2));
  dt = angle_normalize(pose_scan.h - pose_last.h);

  if(*L == 0)
    *theta_sa = 0;
  else
    *theta_sa = atan(dt * axle_dist / *L);
}

/*
   Matches points to map data and saves the number of matched points
   Result is returned in x_array, y_array and cnt_array
*/
void UResLocater::match_points(RangeData rd, matrix *x_,
/*                  int *cnt_array,
                  double x_array[][POINTS_MAX],
                  double y_array[][POINTS_MAX],
                  int *cnte_array,
                  double xe_array[][POINTS_MAX],
                  double ye_array[][POINTS_MAX],*/
                  double ls_offset,
                  UResMapbase *mapbase)
{
  int j, q;
  double dist, utm_x, utm_y;
  int bestMatch = 0;
  bool bestIsLine;
  double d, bestMatchDist = 1000.0;
  mapline mLine;
  U2Dseg mSeg;
  UPose mPose;
  U2Dpos p1, p2;

  for(j = 0; j < (int) (mapbase->maplines.size() + mapbase->mappoints.size()); j++)
  { // set all counts to zero
    cnt_array[j] = 0;
    cnte_array[j] = 0;
  }

  mPose.set(mget(x_, 0, 0), mget(x_, 1, 0), mget(x_, 2, 0));
  for(j = 0; j < rd.points; j++)
  {
    if(rd.point_r[j] != OUT_OF_RANGE)
    {
      /* Convert points to map coordinates */
      // get measurement position
      p1.x = rd.point_x[j] + ls_offset;
      p1.y = rd.point_y[j];
      // convert to map position
      p2 = mPose.getPoseToMap(p1);
      utm_x = p2.x;
      utm_y = p2.y;
      //
/*      utm_x = (rd.point_x[j] + ls_offset) * cos(mget(x_, 2, 0)) - rd.point_y[j] * sin(mget(x_, 2, 0)) + mget(x_, 0, 0);
      utm_y = rd.point_y[j] * cos(mget(x_, 2, 0)) + (rd.point_x[j] + ls_offset) * sin(mget(x_, 2, 0)) + mget(x_, 1, 0);*/
      //
      // find best matched line - modified to use one line only.
      // this prohibits use of localization use of grid-line maps
      bestMatchDist = 100.0;
      bestMatch = -1;
      bestIsLine = true;
      /* Test against lines in map */
      for(q = 0; q < (int) mapbase->maplines.size(); q++)
      {
        mLine = mapbase->maplines[q];
        mSeg.setFromPoints(mLine.x_s, mLine.y_s, mLine.x_e, mLine.y_e);
        d = mSeg.getDistanceSigned(utm_x, utm_y, NULL);
        //d = dist2line(mLine.A, mLine.B, mLine.C, utm_x, utm_y);
        dist = fabs(d);
        if (dist < bestMatchDist)
        { // find closest match - used if no match at all
          bestMatchDist = dist;
          bestMatch = q;
        }
      }
      /* Test against mapped points */
      for(q = 0; q < (int) mapbase->mappoints.size(); q++)
      {
        // dist = fabs(pointdist(mapbase->mappoints[q].x, mapbase->mappoints[q].y, utm_x, utm_y));
        dist = hypot(mapbase->mappoints[q].y - utm_y, mapbase->mappoints[q].x - utm_x);
        if (dist < bestMatchDist)
        { // find closest match - used if no match at all
          bestMatchDist = dist;
          bestMatch = q;
          bestIsLine = false;
        }
      }
      /* If the point is within the limits: Save it */
      if (bestMatch >= 0)
      {
        if (bestIsLine)
        {
          mLine = mapbase->maplines[bestMatch];
          if(bestMatchDist < mLine.perimeter * varRowWidthFactor->getDouble())
          { // inside line if extended - used by line end estimator
            xe_array[bestMatch][cnte_array[bestMatch]] = utm_x;
            ye_array[bestMatch][cnte_array[bestMatch]] = utm_y;
            cnte_array[bestMatch]++;
            // test distance to within line segment
            mSeg.setFromPoints(mLine.x_s, mLine.y_s, mLine.x_e, mLine.y_e);
            d = fabs(mSeg.getDistanceSigned(utm_x, utm_y, NULL));
            if (d < mLine.perimeter * varRowWidthFactor->getDouble())
            { // is usable to line distance-direction test
              x_array[bestMatch][cnt_array[bestMatch]] = utm_x;
              y_array[bestMatch][cnt_array[bestMatch]] = utm_y;
              cnt_array[bestMatch]++;
            }
          }
        }
        else
        {
          if(bestMatchDist < mapbase->mappoints[bestMatch].perimeter * varRowWidthFactor->getDouble())
          { // placed after line correlations
            int idx = bestMatch + mapbase->maplines.size();
            x_array[idx][cnt_array[idx]] = utm_x;
            y_array[idx][cnt_array[idx]] = utm_y;
            cnt_array[idx]++;
          }
        }
      }
    }
  }
  /*
  for(j = 0; j < (int) (mapbase->maplines.size() + mapbase->mappoints.size()); j++)
    printf("cnt_array[%d] = %d\n", j, cnt_array[j]);
  */
  // debug
  // printf("locater:: best match is %gm away from line %s\n", bestMatchDist, mapbase->maplines[bestMatchLine].number);
  // debug end
}

//////////////////////////////////////////////////////////////

# if(0)
void UResLocater::drive(double xv, double yv, double tv)
{
  UPose odo_pose;
  double xm, ym, tm, to, xo, yo, xk, yk;
  int mutex_error;

  mutex_error = this->lock();
    odo_pose = pose_last;
    xk = varXk->getValued();
    yk = varYk->getValued();
    to = angle_normalize(odo_pose.h - varTk->getValued());
  this->unlock();

  /* Calculate origo for odometric coordinates in kalman coordinates: */
  xo = odo_pose.x - xk * cos(to) + yk * sin(to);
  yo = odo_pose.y - xk * sin(to) - yk * cos(to);

  /* Calculate wanted position in odometric coordinates: */
  xm = xo + xv * cos(to) - yv * sin(to);
  ym = yo + xv * sin(to) + yv * cos(to);
  tm = angle_normalize(to + tv);

  /* Save reults in variable pool */
  varXrel->setValued(xm);
  varYrel->setValued(ym);
  varTrel->setValued(tm * 180/M_PI);
  printf(" Kalman: (%f,%f,%f)\n OdoPose: (%f,%f,%f)\n Rel: (%f,%f,%f)\n",mget(x, 0, 0),mget(x, 1, 0),mget(x, 2, 0),odo_pose.x,odo_pose.y,odo_pose.h,xm,ym,tm);
}

/* Convertion of angle from robot to carrot-point to odometric system */
void UResLocater::driveangle(double xv, double yv, double tv,double dist)
{
  UPose odo_pose;
  double xk, yk, tk;
  double A, B, C, At, Bt, Ct,xx,yy;
  int mutex_error;

  mutex_error = this->lock();
    odo_pose = pose_last;
    xk = varXk->getValued();
    yk = varYk->getValued();
    tk = varTk->getValued();
  this->unlock();

  /* Position carrot-point in front of robot: */
  if(fabs(angle_normalize(tv - tk)) > M_PI/2.0)
    tv -= M_PI;

  printf("Drive: %f rad\n", tv);

  if (tan(tv) == 0)
  {
    A = 1;
    B = 0;
    C = -(xk + dist);

    At = 0;
    Bt = 1;
    Ct = -yv;
  }
  else
  {
    /* Angle => Line through point perpendicular to heading */
    A = -1 / tan(tv);
    B = -1;
    C = (yk + dist * sin(tv)) - A * (xk + dist * cos(tv));

    /* Target line on ABC form */
    At = tan(tv);
    Bt = -1;
    Ct = yv - At * xv;
  }

/* Find intersection between the two lines */
  xx = (Ct * B - C * Bt) / (A * Bt - At * B);
  yy = -At / Bt * xx - Ct / Bt;

  printf("Intersection:(%f,%f), A=%f, B=%f, C=%f, At=%f, Bt=%f, Ct=%f\n", xx, yy, A, B, C, At, Bt, Ct);

  /* If the robot is far of the targetline the carrot-point is moved 1/3 closer */
  if(fabs(dist2line(At, Bt, Ct, xk, yk)) > 0.20)
  {
    printf("WARNING: Large distance to targetline\n");
    if (tan(tv)==0)
    {
      A = 1;
      B = 0;
      C = -(xk + dist / 3.0);
    }
    else
    {
      /* Angle => Line through point perpendicular to heading */
      A = -1/tan(tv);
      B = -1;
      C = (yk + dist / 3.0 * sin(tv)) - A * (xk + dist / 3.0 * cos(tv));
    }
      /* Target line on ABC form */
      xx = (Ct * B - C * Bt) / (A * Bt - At * B);
      yy = -At / Bt * xx - Ct / Bt;
      printf("!!!NEW INTERSECTION!!!:(%f,%f), A=%f, B=%f, C=%f, At=%f, Bt=%f, Ct=%f\n", xx, yy, A, B, C, At, Bt, Ct);
  }

  /* Updates the variables */
  varXrel->setValued(odo_pose.x);
  varYrel->setValued(odo_pose.y);
  varTrel->setValued(angle_normalize(atan2(yy - yk, xx - xk) - tk + odo_pose.h) * 180/M_PI);
  varDist->setValued(sqrt(pow(xv - xk, 2) + pow(yv - yk, 2)));

  printf("dist = %f\n", varDist->getValued());
  printf(" Kalman: (%f,%f,%f)\n OdoPose: (%f,%f,%f)\n Rel: (%f,%f,%f)\n",mget(x, 0, 0),mget(x, 1, 0),mget(x, 2, 0),odo_pose.x,odo_pose.y,odo_pose.h,varXrel->getValued(),varYrel->getValued(),varTrel->getValued());
}
# endif

float UResLocater::quality()
{
  int noupdates = varUpdcnt->getInt(0);

  if(noupdates > 100)
    return(0.0);
  else
   return(1.0 - noupdates / 100.0);
}

///////////////////////////////////////////////////////////////

bool UResLocater::setUtmPoseToMapPose()
{
  UResPoseHist *mapposehist;
  UResPoseHist *utmposehist;
  UResMapbase * mapbase;
  bool result = false;
  UPoseTVQ mapPose, utmPose;
  UPose mapRef;
  //
  if (not varKeepMapPose->getBool())
  { // mapPose are not maintained from locater update, so
    // do it here
    mapposehist = (UResPoseHist *) getStaticResource("mapPose", false);
    utmposehist = (UResPoseHist *) getStaticResource("utmPose", false);
    mapbase = (UResMapbase *) getStaticResource("mapbase", false, false);
    if (mapposehist != NULL and utmposehist != NULL and mapbase != NULL)
    { // all data is available, so do it
      utmPose = utmposehist->getNewest();
      mapRef = mapbase->getMapRefPose();
      // convert current gps pose to map coordinates
      mapPose = mapRef.getMapToPosePose(&utmPose);
      mapPose.t = utmPose.t;
      mapPose.vel = utmPose.vel;
      mapPose.q = utmPose.q;
      /* update map pose */
      mapposehist->addIfNeeded(mapPose, URESLOCATER_ID_SET);
      result = true;
    }
  }
  return result;
}

//////////////////////////////////////////////////////////
bool UResLocater::doLocatorUpdates(bool updateKalman, ULogFile * logUpd, UTime updTime, UPose pose_scan, unsigned int serial)
{
  bool result = false;
  UMatrix4 mYavg(3, 1, 0.0); // best combined average estimate for all measurements
  UMatrix4 mYavg2(2, 1, 0.0); // best combined average estimate for x,y component
  UMatrix4 mY(2, 1, 0.0); // current measurmenet
  UMatrix4 mwY(2, 1, 0.0); // current measurmenet in state space with variance weight
  UMatrix4 mW(2,2); // weight of current measurment
  UMatrix4 mWi(2,2); // weight of current measurment inverted
  UMatrix4 mwYSum(2, 1, 0.0); // waighted sum of measurements
  UMatrix4 mwSum(2, 2, 0.0); // sum of weights
  UMatrix4 mwInv(2,2);
  UMatrix4 mRavg(3, 3); // sum of weights inverted - as is average measurement error
//  UMatrix4 mWVec(2,2), mWeig; // for inversion og sum of weights
  ULocaterLineResult * ld;
  UMatrix4 mH(2, 2); // orientation of result - map from dist-angle-end to states
  UMatrix4 mHT(2, 2); // H transposed
  //UMatrix4 mHi(2, 2); // inverse of orientation of result - map from dist-angle-end to states
  //UMatrix4 mHTi(2, 2); // inverse H transposed
  UMatrix4 mR(3, 3, 0.0); // current
  UMatrix4 mP(3, 3);  // predicted covaiance
  UMatrix4 mS(3, 3);  // kalman sensitivity
  UMatrix4 mK(3, 3);  // kalman gain
  UMatrix4 mX(3, 1); // predicted state
  UMatrix4 mI(3, 3, 1.0); // Identity matrix
  UMatrix4 mIe(2, 2, var_d * 0.01); // small identity matrix to avoid singularity for matrix inversion
  int i, n3 = 0, n2 = 0;
  double poseDist, poseT, ux, uy, uth;
  double yaSum = 0;
  int hitCntMax = 0;
  bool useEnd;
  //

  //printf("locater update hitCnt:");
  for (i = 0; i < locDecsCnt; i++)
  {
    ld = &locDecs[i];
    //printf(" %d", ld->hitCnt);
    if (ld->hitCnt > hitCntMax)
      hitCntMax = ld->hitCnt;
  }
  //printf("\n");
  for (i = 0; i < locDecsCnt; i++)
  {
    //printf("---- measurement %d ----\n", i);
    ld = &locDecs[i];
    mY.setRC(0, 0, ld->distMeas - ld->distMap);
    // angle is taken as independent
    yaSum -= ld->angleDelta;
    // cross and line end measurements
    useEnd = (ld->lineEndOK and varUseRowEnd->getBool() and
      fabs(ld->distMap) < varRowToRobotMaxDist->getValued() and
      fabs(ld->lineEndDelta) < varRowEndMaxDist->getValued());
//     if (not useEnd and ld->lineEndOK and varUseRowEnd->getBool())
//       printf("line %s %.2f away END NOT used\n", ld->line->number, ld->distMap);
    if (useEnd)
    { // use line end, cross distance (and angle)
      mY.setSize(2, 1);
      mY.setRC(1, 0, ld->lineEndDelta);
      n3++;
    }
    else
    { // use cross distance (and angle) only
      mY.setSize(1, 1);
      n2++;
    }
    //mY.print("mY");
    //printf("mYangle %g (%.2fdeg)\n", ld->angleDelta, ld->angleDelta * 180.0 / M_PI);
    //printf("ls_offset = %f\n", ls_offset);

    /* Measurement Jacobian */
    if (ld->lineEndOK and varUseRowEnd->getBool())
    {
      mH.setSize(2, 2);
      mH.setRow(0,  ld->line->A, ld->line->B);
      mH.setRow(1, -ld->line->B, ld->line->A);
    }
    else
    {
      mH.setSize(1, 2);
      mH.setRow(0,  ld->line->A, ld->line->B);
    }
    // and its transposed
    mHT.transpose(&mH);
    //mH.print("mH");
    //mHT.print("mHT");
    // set measurement error
    if (ld->lineEndOK and varUseRowEnd->getBool())
    { // across line direction, angle and along line
      mR.init(2, 2, 0.0);
      mR.setDiag(var_d, var_t);
    }
    else
    { // across line direction and angle only
      mR.init(1, 1, 0.0);
      mR.setDiag(var_d, 0.0);
    }
    //mR.print("mR");
    // weight of this measurement (in state [x,y,th]' coordinates)
    // from the inverse of measurement covariance
    mW = mHT * mR * mH + mIe;
    //mW.print("mW");
    //
    // summarize weighted measurements
    mwY = mW * mHT * mY;
    //mwY.print("mwY");
    mwYSum.add(&mwY);
    //mwYSum.print("mwYSum");
    // summarize waights too
    mwSum.add(&mW);
    //mwSum.print("mwSum");
    //printf("----finished measurement %d ----\n", i);
  }
  mwInv.inverse(&mwSum);
  //mwInv.print("mwInv");
  // reverse effect of weight
  mYavg2 = mwInv * mwYSum;
  //mYavg2.print("mYavg2");
  mYavg.setCol(0, mYavg2.get(0), mYavg2.get(1), yaSum / double(locDecsCnt));
  //mYavg.print("mYavg");
  //
  if (logUpd->isOpen())
  {
    ux = mYavg.get(0,0);
    uy = mYavg.get(1,0);
    uth = mYavg.get(2,0);
    poseT = ux * cos(pose_scan.h) + uy*sin(pose_scan.h);
    poseDist = - ux * sin(pose_scan.h) + uy*cos(pose_scan.h);
    fprintf(logUpd->getF(), "%lu.%06lu %d %d %7.3f %7.3f %7.3f  %6.3f %6.3f %6.3f      %6.3f %6.3f\n", updTime.getSec(), updTime.getMicrosec(),
            n3, n2, pose_scan.x, pose_scan.y, pose_scan.h , ux, uy, uth, poseDist, poseT);
  }
  // now prepare kalman filter update
  if (updateKalman)
  { //implement results in kalman state
    // - first get current covariance
    mP.setRow(0, mget(P_,0,0), mget(P_,0,1), mget(P_,0,2));
    mP.setRow(1, mget(P_,1,0), mget(P_,1,1), mget(P_,1,2));
    mP.setRow(2, mget(P_,2,0), mget(P_,2,1), mget(P_,2,2));
    //mP.print("mP");
    // then the kalman sensitivity
    //create resulting average update error covariance
    mRavg.setRow(0, mwInv.get(0,0) , mwInv.get(0,1), 0.0);
    mRavg.setRow(1, mwInv.get(1,0) , mwInv.get(1,1), 0.0);
    mRavg.setRow(2,              0.0 ,             0.0, var_phi/double(locDecsCnt));
    // (the H matrix related to averaged measurement is now an I matrix)
    mS = mP + mRavg;
    //mS.print("mS");
    mK = mP * mS.inversed();
    //mK.print("mK");
    //
    // current state
    //printf("Updated using %d measurements\n", locDecsCnt);
    //printf("current state (x_up)=\n"); mprint(x_up);
    mX.setCol(0, mget(x_up, 0, 0), mget(x_up, 1, 0), mget(x_up, 2, 0));
    //mX.print("mX");
    // state update
    mXu = mX + mK * mYavg;
    //mXu.print("mXu");
    // covariance update
    mPu = (mI - mK) * mP;
    //mPu.print("mPu");
  }
  //
  return result;
}

//////////////////////////////////////////////////////////////////

void UResLocater::makeRowPolygon(regtype reg, mapline mLine)
{
  int i;
  const int MSL = 30;
  char s[MSL];
  bool isOK;
  U2Dseg seg;
  double t, tMin, tMax;
  U2Dpos po1;
  //
  if (poly400 == NULL)
  { // first time use - allocate memory
    poly400 = new UPolygon400();
    poly40 = new UPolygon40;
  }
  else
  {
    poly400->clear();
    poly40->clear();
  }
  // fill points to polygon - max 400
  seg.clear();
  seg.U2Dlined::set(reg.a, reg.b, reg.c);
  // line length
  seg.length = reg.tMax - reg.tMin;
  // use first point as reference
  seg.getOnLine(reg.x_array[0], reg.y_array[0], &seg.x, &seg.y);
  for (i = 0; i < reg.N; i++)
  {
    po1.x = reg.x_array[i];
    po1.y = reg.y_array[i];
    poly400->add(po1.x, po1.y);
    // find also line ends
    if (i == 0)
    {
      tMin = seg.getPositionOnLine(po1);
      tMax = tMin;
    }
    else
    {
      t = seg.getPositionOnLine(po1);
      if (t > tMax)
        tMax = t;
      if (t < tMin)
        tMin = t;
    }
  }
  // convert to convex
  poly400->extractConvexTo(poly40);
  poly40->setAsPolygon();
  strcpy(poly40->color, "cddd");
  // set polygon coordinate system to odometry (0=odo, 1=utm, 2=map)
  // parameter 0 is polygon name
  snprintf(s, MSL, "treerow.%s", mLine.number);
  isOK = polygonToPolyPlugin(poly40, s, 2);
  //
  if (isOK)
  { // add also fitted line
    poly400->clear();
/*    poly400->add(reg.b * reg.tMin, -reg.a * reg.tMin);
    poly400->add(reg.b * reg.tMax, -reg.a * reg.tMax);*/
    po1 = seg.getPositionOnLine(tMin);
    poly400->add(po1.x, po1.y);
    po1 = seg.getPositionOnLine(tMax);
    poly400->add(po1.x, po1.y);
    poly400->setAsPolyline();
    strcpy(poly400->color, "gddd");
    snprintf(s, MSL, "treerowEst.%s", mLine.number);
    isOK = polygonToPolyPlugin(poly400, s, 2);
  }
}

///////////////////////////////////////////////////////

bool UResLocater::polygonToPolyPlugin(UPolygon * poly, char * name, int coordinateSystem)
{
  bool isOK;
  UVariable * par[3];
  UVariable vs;
  UVariable vr;
  UVariable vCoo;
  UDataBase *db, *dbr;
  UResBase * pres;
  int n;
  //
  pres = getStaticResource("poly", false, false);
  isOK = (pres != NULL);
  if (isOK)
  {
    vs.setValues(name, 0, true);
    par[0] = &vs;
    // parameter 1 is polygon to add
    db = poly;
    par[1] = (UVariable *) db;
    // parameter 3 is coordinate system (0=odo, 1=utm, 2=map)
    vCoo.setDouble(coordinateSystem);
    par[2] = &vCoo;
    // number of parameters
    n = 3;
    // set return value
    dbr = &vr;
    // do the call
    isOK = callGlobalV("poly.setPolygon", "scd", par, &dbr, &n);
    if (not isOK or not vr.getBool())
      printf("UResAvoid::copyFootprintPolys: failed to 'poly.setPoly(%s,c,d)'\n", name);
  }
  return isOK;
}

