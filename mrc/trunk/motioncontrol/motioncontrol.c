#include <stdio.h>
#include <stdlib.h>
#include "motioncontrol.h"
#include <math.h>
#include <string.h>
#include "fresnel.h"
void move_init(motiontype *p);
void move(motiontype *p);
void drive_init(motiontype *p);
void driveon_init(motiontype *p);
void drive_initw(motiontype *p);
void driveon_initw(motiontype *p);
void drive(motiontype *p);
void turn_init(motiontype *p);
void turn(motiontype *p);
void turnR_init(motiontype *p);
void turnR(motiontype *p);
void turnC_init(motiontype *p);
void turnC(motiontype *p);
void stop(motiontype *p);
void follow_line_init(motiontype *P);
void follow_line(motiontype *P);
void follow_wall_init(motiontype *p);
void follow_wall(motiontype *P);
void calcvel(motiontype *p);
void idle(motiontype *p);
void align(motiontype *p);
void align_init(motiontype *p);
void set_motor_param(int motor_number,int par, int val);
void directmotorcmds(motiontype *p);
void wdcontroller(motiontype *p);
void wdcontroller_init(motiontype *p);
void drivetopoint_init(motiontype *p);
void drivetopoint(motiontype *p);
void movetoxyz_init(motiontype *p);
void movetoxyz(motiontype *p);
void funcgen_init(motiontype *p);
void funcgen(motiontype *p);

#define PARAMETERLISTSIZE 10
extern double round(double x);
static parameterlisttype followwallparlist;
parameterelem followwallparlistarray[PARAMETERLISTSIZE];

static parameterlisttype driveparlist;
parameterelem driveparlistarray[PARAMETERLISTSIZE];


static parameterlisttype drivetopointparlist;
parameterelem drivetopointparlistarray[PARAMETERLISTSIZE];

static parameterlisttype movetoxyzparlist;
parameterelem movetoxyzparlistarray[PARAMETERLISTSIZE];


functionlistelem motioncontrolfunclist[]={{"followwall",&followwallparlist},
					{"drivetopoint",&drivetopointparlist},
					{"drive",&driveparlist},
					{"movetoxyz",&movetoxyzparlist},
					{ "",0}
				};


void motion_init(motiontype *p){
  p->cmd=0;
  p->tgt_o.x=0;
  p->tgt_o.y=0;
  p->tgt_o.th=0;
  p->tgt=p->tgt_o;
  p->poseerr.x=0;
  p->poseerr.y=0;
  p->poseerr.th=0;
  p->state=$MC_STOP;
  p->status=MC_OK;
  followwallparlist.Nmax=PARAMETERLISTSIZE;
  followwallparlist.N=0;
  followwallparlist.list=followwallparlistarray;
  addpar(&followwallparlist,"gain",ADMIN_par,ADMIN_double,&p->wall_gain);
  addpar(&followwallparlist,"alfa",ADMIN_par,ADMIN_double,&p->wall_alfa);
  addpar(&followwallparlist,"tau",ADMIN_par,ADMIN_double,&p->wall_tau);
  addpar(&followwallparlist,"anglegain",ADMIN_par,ADMIN_double,&p->wall_anglegain);
  addpar(&followwallparlist,"inputmode",ADMIN_par,ADMIN_double,&p->wallinputmode);
  addpar(&followwallparlist,"distl",ADMIN_inp,ADMIN_double,&p->irdistl);
  addpar(&followwallparlist,"distr",ADMIN_inp,ADMIN_double,&p->irdistr);
  addpar(&followwallparlist,"angler",ADMIN_inp,ADMIN_double,&p->wallangler);
  addpar(&followwallparlist,"anglel",ADMIN_inp,ADMIN_double,&p->wallanglel);
// drive  
  driveparlist.Nmax=PARAMETERLISTSIZE;
  driveparlist.N=0;
  driveparlist.list=driveparlistarray;
  addpar(&driveparlist,"kangle",ADMIN_par,ADMIN_double,&p->kangle_drive);
  addpar(&driveparlist,"kdist",ADMIN_par,ADMIN_double,&p->kdist_drive); 
  addpar(&driveparlist,"refx",ADMIN_inp,ADMIN_double,&p->refx_drive);
  addpar(&driveparlist,"refy",ADMIN_inp,ADMIN_double,&p->refy_drive);
  addpar(&driveparlist,"refth",ADMIN_inp,ADMIN_double,&p->refth_drive);
  p->refx_drive=&p->refpose.x;
  p->refy_drive=&p->refpose.y;
  p->refth_drive=&p->refpose.th;
// drivetopoint  
  drivetopointparlist.Nmax=PARAMETERLISTSIZE;
  drivetopointparlist.N=0;
  drivetopointparlist.list=drivetopointparlistarray;
  addpar(&drivetopointparlist,"evalmethod",ADMIN_par,ADMIN_double,&p->dtp.evalmethod);
  p->dtp.evalmethod=DTP_EVAL_SHORTEST; // Default evalutation method (int)
// drivetoxyz 
  movetoxyzparlist.Nmax=PARAMETERLISTSIZE;
  movetoxyzparlist.N=0;
  movetoxyzparlist.list=movetoxyzparlistarray;
  addpar(&movetoxyzparlist,"con1_b0",ADMIN_par,ADMIN_double,&p->xyzcon1.b0);
  addpar(&movetoxyzparlist,"con1_b1",ADMIN_par,ADMIN_double,&p->xyzcon1.b1);
  addpar(&movetoxyzparlist,"con1_a1",ADMIN_par,ADMIN_double,&p->xyzcon1.a1);
  addpar(&movetoxyzparlist,"con1_ffwd",ADMIN_par,ADMIN_double,&p->xyzcon1.ffwd);
  addpar(&movetoxyzparlist,"con1_dlim",ADMIN_par,ADMIN_double,&p->xyzcon1.dlim);
}



void motion_update(motiontype *p){

  if (p->cmd != 0){
    p->curcmd=p->cmd;
    p->ignore_obstacles_flag=p->ignore_obstacles;
    p->ignore_obstacles=0;
    p->time=0;
    switch (p->state){
    case $MC_ALIGN:
/// fjernet af christian
#if (0)
      set_motor_param(1,2,0);
       set_motor_param(2,2,0);
#endif
    break;
    default:
    ;
    }
  }


  switch (p->cmd){
  case 0:// no new command

  break;
  case MC_FWDCMD:
    p->state=$MC_FWD;
    move_init(p);
  break;

  case MC_DRIVECMD:
    p->state=$MC_DRIVE;
    drive_init(p);
  break;

 case MC_DRIVEONCMD:
    p->state=$MC_DRIVE;
    driveon_init(p);
  break;

case MC_DRIVECMDW:
    p->state=$MC_DRIVE;
    drive_initw(p);
  break;

 case MC_DRIVEONCMDW:
    p->state=$MC_DRIVE;
    driveon_initw(p);
  break;



  case MC_TURNCMD:
    p->state=$MC_TURN;
    turn_init(p);
  break;

  case MC_TURNRCMD:
    p->state=$MC_TURNR;
    turnR_init(p);
  break;

 case MC_TURNCCMD:
    p->state=$MC_TURNC;
    turnC_init(p);
  break;

  case MC_FOLLOW_LINECMD:
    p->state=$MC_FOLLOW_LINE;
    follow_line_init(p);
  break;

  case MC_FOLLOW_WALLCMD:
    p->tgt=p->pose;
    p->state=$MC_FOLLOW_WALL;
    follow_wall_init(p);
  break;

  case MC_STOPCMD:
    p->state=$MC_STOP;
  break;

  case MC_IDLECMD:
    p->state=$MC_IDLE;
  break;

  case MC_FOLLOW_WALLCMD2:
    p->state=$MC_FOLLOW_WALL2;
    wdcontroller_init(p);
  break;

  case MC_ALIGNCMD:
    p->state=$MC_ALIGN;
    align_init(p);
  break;

  case MC_DIRECTMOTORCMDS:
    p->state=$MC_DIRECTMOTORCMDS;
  break;

  case MC_DRIVETOPOINTCMD:
    p->state = $MC_DRIVETOPOINT;
    drivetopoint_init(p);
  break;
  case MC_MOVETOXYZCMD:
    p->state = $MC_MOVETOXYZ;
    movetoxyz_init(p);
  break;
  case MC_FUNCGEN:
    p->state = $MC_FUNCGEN;
    funcgen_init(p);
  break;


  default:
  ;
}

p->status=MC_OK;
switch (p->state) {

case $MC_FWD:
  move(p);
break;
case $MC_TURN:
  turn(p);
break;
case $MC_TURNR:
  turnR(p);
break;
case $MC_TURNC:
  turnC(p);
break;
case $MC_STOP:
  stop(p);
break;

case $MC_FOLLOW_LINE:
  follow_line(p);
break;

case $MC_FOLLOW_WALL:
  follow_wall(p);
break;

case $MC_DRIVE:
  drive(p);
break;

case $MC_IDLE:
  idle(p);
break;

case $MC_FOLLOW_WALL2:
  wdcontroller(p);
break;

case $MC_ALIGN:
  align(p);
break;

case $MC_DIRECTMOTORCMDS:
  directmotorcmds(p);
break;

case $MC_DRIVETOPOINT:
  drivetopoint(p);
break;

case $MC_MOVETOXYZ:
  movetoxyz(p);
break;

case $MC_FUNCGEN:
  funcgen(p);
break;

default:
;
}
p->cmd=0;
p->time++;
}

void ratelim(double * newsig,double sig,double rate){
  if (*newsig > (rate+sig))
    *newsig=sig+rate;
  else if (*newsig < (sig-rate))
    *newsig=sig-rate;
  else
  ;
}


void movetoxyz_init(motiontype *p){
p->movetoxyzstates.d=p->pose3d.z*(p->xyzcon1.b1 - p->xyzcon1.b0*p->xyzcon1.a1)/(1+p->xyzcon1.a1);
}

void movetoxyz(motiontype *p){
 double ez,uz;
 ez=p->pose3d.z;
 uz= p->xyzcon1.b0*ez+p->movetoxyzstates.d;
 if (uz>p->xyzcon1.dlim)
    uz=p->xyzcon1.dlim;
 if (uz<-p->xyzcon1.dlim)
    uz=-p->xyzcon1.dlim;
 p->xyzout1=p->refpose3d.z-uz+p->xyzcon1.ffwd;
 p->movetoxyzstates.d=ez*p->xyzcon1.b1-uz*p->xyzcon1.a1;
}

void move_init(motiontype *p){
  p->velref=fabs(p->vel);
  p->tgt_o=p->tgt;
  p->tgt.x+=p->dist*cos(p->tgt.th);
  p->tgt.y+=p->dist*sin(p->tgt.th);
}

void move(motiontype *p){
double ds,vel,vel1,vel2,signv,dtgt,dob,angleerr,dvel;
// calulate acceleration limited  vehicle velocity
  ds=p->acccmd*p->ts;
  vel=fabs(p->velcmd);
  ratelim(&vel,p->velref,ds);

// calculate distance to new target pose

  if (p->dist > 0) {
    dtgt=(p->tgt.x - p->pose.x)*cos(p->tgt.th)+(p->tgt.y - p->pose.y)*sin(p->tgt.th);
    dob= p->d_obstac_front - p->stopdist;
    signv=1;
  }
  else {
    dtgt=((p->tgt.x - p->pose.x)*cos(p->tgt.th)+(p->tgt.y - p->pose.y)*sin(p->tgt.th));
    dob= p->d_obstac_back - p->stopdist;
    signv=-1;
  }

  if (dob < 0) dob=0;
  if (dob < p->alarmdist)  p->status=MC_OBSTACLE;
  if (fabs(dtgt)>dob && !p->ignore_obstacles_flag){
    vel1 = sqrt(2*dob*p->acccmd);
    vel2 = p->kp*dob;
  }
  else {
    vel1 = sqrt(2*fabs(dtgt)*p->acccmd);
    vel2= p->kp*fabs(dtgt);
    signv=sign(dtgt);
  }
  if (vel1 < vel) vel=vel1;
  if (vel2 < vel) vel=vel2;
  p->velref=vel;


// angle controller
  angleerr = p->tgt_o.th - p->pose.th;
  normalizeangle(&angleerr);
  dvel=limit(p->gain*angleerr,p->lim,-p->lim);
  switch (p->type){
   case VELOMEGA:
  {
    p->rvel=vel*signv;
    p->lvel=vel*signv;
    p->omega=dvel;
  }
  break;

  case ACKERMAN:
  {
    p->rvel=vel*signv;
    p->lvel=vel*signv;
    p->steeringangle=dvel;
    p->curvature=dvel;
  }
  break;
  case DIFFERENTIAL:{
    p->rvel=vel*signv+dvel;
    p->lvel=vel*signv-dvel;
  }
  break;
  default:{
    p->rvel=0;
    p->lvel=0;
    p->steeringangle=0;
    p->omega=0;
    p->curvature=0;
  }  
  }
  p->poseerr=posediff(p->tgt,p->pose);
  p->target_dist=dtgt;
}

void drive_init(motiontype *p){
  p->pd.k=p->gain;
  p->pd.tau=p->tau;
  p->pd.alfa=p->alfa;
  p->pd.ts=p->ts;
  pd_init(&p->pd);
  p->velref=p->vel;
  p->usew2odo=0;
 
}

void driveon_init(motiontype *p){
  p->pd.k=p->gain;
  p->pd.tau=p->tau;
  p->pd.alfa=p->alfa;
  p->pd.ts=p->ts;
  pd_init(&p->pd);
  p->velref=p->velcmd;
  p->usew2odo=0;
 
}

void drive_initw(motiontype *p){
  p->pd.k=p->gain;
  p->pd.tau=p->tau;
  p->pd.alfa=p->alfa;
  p->pd.ts=p->ts;
  pd_init(&p->pd);
  p->velref=p->vel;
  p->usew2odo=1;
 
}

void driveon_initw(motiontype *p){
  p->pd.k=p->gain;
  p->pd.tau=p->tau;
  p->pd.alfa=p->alfa;
  p->pd.ts=p->ts;
  pd_init(&p->pd);
  p->velref=p->velcmd;
  p->usew2odo=1;
 
}


void drive(motiontype *p){
double angleerr,dvel,d,u,dlim,refthl,refxl,refyl,th;
  p->tgt=p->pose;
// calulate acceleration limited  vehicle velocity
  calcvel(p);
   if (p->usew2odo){
     th=p->w2odo.th;
     refthl=*p->refth_drive+th;
     normalizeangle(&refthl);
     refxl=cos(th)*(*p->refx_drive)-sin(th)*(*p->refy_drive)+p->w2odo.x;
     refyl=sin(th)*(*p->refx_drive)+cos(th)*(*p->refy_drive)+p->w2odo.y;
   }
   else{
     refthl=*p->refth_drive;
     refxl=*p->refx_drive;
     refyl=*p->refy_drive;
   }
     
  
   p->l_drive.vnx=-sin(refthl);
   p->l_drive.vny=cos(refthl);
   p->l_drive.c=-(p->l_drive.vnx*(refxl)+p->l_drive.vny*(refyl));


   p->target_dist=(refxl - p->pose.x)*cos(refthl)+(refyl -
   p->pose.y)*sin(refthl);



// controller
  angleerr = refthl - p->pose.th;
  normalizeangle(&angleerr);

  d=(p->pose.x*p->l_drive.vnx+p->pose.y*p->l_drive.vny+p->l_drive.c)+p->dist;

  // steering delay defined
  if (p->steerDelay >= p->ts)
  { // there is a need for steering control calculation
    // number of entriest in comand history
    int n = (int)(p->steerDelay / p->ts + 0.5);
    int i, huntCnt = 0;
    double dda = 0.0;
    double ddr = 0.0;
    double esa = p->steeringangleNow; // estimated steering angle
    double dsa;
    // limit to buffer length
    if (n > MaxSteerHist)
      n = MaxSteerHist;
    // initialize buffer if command renewed or after some break
    if (p->steerHistCnt != n || p->tick - p->steerTick > n)
    { // initialize steering history to current steering angle
      for (i = 0; i < n; i++)
        p->steerHist[i] = p->steeringangleNow;
      p->steerHistCnt = n;
      // debug
      printf("Steering: initialized buffer with %d values, each %.4f\n", n, p->steeringangleNow);
      // debug end
    }
    p->steerTick = p->tick;
    for (i = 0; i < n; i++)
    { // calculate estimated steering angle for each step in command history
      dsa = p->steerHist[i] - esa;
      if (fabs(dsa) < p->steerVel *  p->ts)
        // can reach commanded angle
        esa = p->steerHist[i];
      else
        // is limited by steering speed
        esa += sign(dsa) * p->steerVel * p->ts;
      // sum the movement in the delay period
      dda += tan(esa)/p->robotlength * p->ts * p->vel;
      ddr += sin(p->pose.th + dda - refthl) * p->ts * p->vel;
      if (i > 0 && p->steerHist[i-1] * p->steerHist[i] < 0.0)
        huntCnt++;
    }
    // and a bit more to compensate a bit for cut corners in model
    ddr += sin(refthl + dda - p->pose.th) * p->ts * p->vel;
    // debug
    if (p->tick % 20 == n / 2)
    { // every 2 seconds
      printf("Steering distErr=%.4f angErr=%.4f dda=%.4f ddr=%.4f\n", d, angleerr, dda, ddr);
      printf("Steering hist:");
      for (i = 0; i < n;  i++)
        printf(" %.4f", p->steerHist[i]);
      printf("\n");
    }
    // debug end
    // adjust error values with movement in delay time
    d += ddr;
    angleerr -= dda;
    // result is put into later angle command history further down
    if (huntCnt > 1)
    { // steering command is changing sign more than once
      // hunting while driving straight - reduce gain
      // avoids jittering at high speed > 2m/s on Hako
      d *= 0.5;
      angleerr *= 0.5;
      // debug
      printf("Steering - jitter reduction\n");
      // debug end
    }
  }
  
  if (p->kdist_drive > 0){
    dlim=fabs(p->kangle_drive*M_PI/2/p->kdist_drive);
    if (fabs(d) > dlim){
        d=dlim*sign(d);    
    }
  }
   d=sign(p->velref)*d;
   u=-p->kdist_drive*d+p->kangle_drive*angleerr;

  switch (p->type){
  case VELOMEGA: {
    dvel=limit(u,p->lim,-p->lim);
    p->rvel=p->velref;
    p->lvel=p->velref;
    p->omega=dvel;
  }
  break;
  case ACKERMAN:{
    p->rvel=p->velref; 
    p->lvel=p->velref;
    p->steeringangle= limit(u,p->lim,-p->lim);
    p->curvature=limit(u,p->lim,-p->lim);
  }
  break;
  case DIFFERENTIAL: {
    u=u*p->w;
    dvel=limit(u,p->lim,-p->lim);
    p->rvel=p->velref+dvel;
    p->lvel=p->velref-dvel;
  }
  break;
  default:{
    p->rvel=0;
    p->lvel=0;
    p->steeringangle=0;
    p->omega=0;
    p->curvature=0;
  }  
  }
  if (p->steerHistCnt > 0)
  { // shift steering command buffer and insert new value
    memmove(p->steerHist, &p->steerHist[1], sizeof(double)* (p->steerHistCnt - 1));
    p->steerHist[p->steerHistCnt - 1] = p->steeringangle;
  }
}

void driveold(motiontype *p){
double angleerr,dvel;
  p->tgt.x =p->pose.x;
  p->tgt.y =p->pose.y;
// calculate acceleration limited  vehicle velocity
  calcvel(p);

// angle controller
  angleerr = p->tgt.th - p->pose.th;
  normalizeangle(&angleerr);
  p->pd.e=angleerr;
  pd_out(&p->pd);
  pd_update(&p->pd);
  angleerr=p->pd.o;
  dvel=limit(angleerr,p->lim,-p->lim);
  p->rvel=p->velref+dvel;
  p->lvel=p->velref-dvel;
}


void turn_init(motiontype *p){
  p->velref=(fabs(p->curvell)+fabs(p->curvelr))/2;
  p->tgt_o=p->tgt;
  normalizeangle(&p->turnangle);
  p->tgt.th+=p->turnangle;
  normalizeangle(&p->tgt.th);
}

void turn(motiontype *p){
  double ds,vel,du1,du2,du;
  double temp_th=(p->pose.th - p->tgt_o.th);
// calulate acceleration limited  vehicle velocity
  ds=p->acccmd*p->ts;
  vel=p->velcmd;
  if (vel - p->velref > ds) vel  = p->velref+ds;
  if (vel - p->velref < -ds) vel = p->velref-ds;
  p->velref=vel;

// calculate  to new target pose
  normalizeangle(&temp_th);
  if ( p->turnangle > 0){// leftturn
    if (p->turnangle> M_PI/2 && temp_th <-M_PI/2) temp_th+=2*M_PI;
    du1=p->turnangle-temp_th;
  }
  else {
    if (p->turnangle <- M_PI/2 && temp_th >M_PI/2) temp_th-=2*M_PI;
    du1=p->turnangle-temp_th;
  }

  du = sqrt(fabs(du1)*p->w*2*p->acccmd);
  du2= fabs(du1)*p->kp*p->w;

  if (du1 < 0)
    du=-du;
  if (du > vel) du=vel;
  if (du < -vel) du=-vel;
  if (du > du2) du=du2;
  if (du < -du2) du=-du2;
 
    switch (p->type){
   case VELOMEGA:
  {
    p->rvel=0;
    p->lvel=0;
    p->omega=2*du/p->w;
  }
  break;

  case ACKERMAN:
  {
    p->rvel=0;
    p->lvel=0;
    p->steeringangle=0;
    p->curvature=0;
  }
  break;
  case DIFFERENTIAL:{
   p->rvel=du;
   p->lvel=-du;
  }
  break;
  default:{
    p->rvel=0;
    p->lvel=0;
    p->steeringangle=0;
    p->omega=0;
    p->curvature=0;
  }  
  }

 
  p->poseerr=posediff(p->tgt,p->pose);
}

#if (0)
void turnR_init(motiontype *p){
  double s;
  p->velref_l=p->vel;
  p->velref_r=p->vel;
  p->velref=p->vel;
  p->tgt_o=p->tgt;
  normalizeangle(&p->turnangle);
  p->tgt.th+=p->turnangle;
  normalizeangle(&p->tgt.th);
  s=sign(p->turnangle);
  p->tgtnormal=p->tgt.th-s*M_PI/2;
  normalizeangle(&p->tgtnormal);
  p->tcx=p->tgt_o.x- p->R *sin(p->tgt_o.th)*s;
  p->tcy=p->tgt_o.y+ p->R *cos(p->tgt_o.th)*s;
  p->vr_turn=p->velcmd*(1+s*p->w/2/p->R);
  p->vl_turn=p->velcmd*(1-s*p->w/2/p->R);
//  printf(" vrturn %lf vlturn %lf\n",p->vr_turn,p->vl_turn);
}

void turnR(motiontype *p){
  double s,vr,vr1,vl,vl1,th,dth,ds,angleerr,dvel;
   p->tgt.x=p->pose.x;
   p->tgt.y=p->pose.y;
   s=sign(p->turnangle);
// calulate acceleration limited  vehicle velocity
  calcvel(p);
   ds=p->acccmd*p->ts;
   vr=p->vr_turn;
   ratelim(&vr,p->velref_r,ds);
   vl=p->vl_turn;
   ratelim(&vl,p->velref_l,ds);
   th=atan2(p->pose.y-p->tcy,p->pose.x-p->tcx);
   dth=s*(p->tgtnormal-th);
   normalizeangle(&dth);
  if (p->type==ACKERMAN){
     double Rlocal;
     p->lvel=p->velref
     Rlocal=p->R;
     if (Rlocal <0.1) Rlocal=0.1;
     p->steeringangle=s*atan(p->robotlength/Rlocal);
     if (dth >0 )
       p->target_dist=1;
     else
       p->target_dist=0;

     }
   }
    else {

      if (dth < 0){
        vr=p->velcmd;
        vl=p->velcmd;
        p->target_dist=0;
      }
      else{
        p->target_dist=1;
        vl1=(pow(p->velcmd,2)-s*p->acccmd*dth);
        if (vl1 < 0)
          vl1=0;
        else
          vl1=sqrt(vl1);
        vr1=(pow(p->velcmd,2)+s*p->acccmd*dth);
        if (vr1 < 0)
          vr1=0;
        else
          vr1=sqrt(vr1);
        if (s >= 0){
          if (vl < vl1) vl=vl1;
          if (vr > vr1) vr=vr1;
      }
      else{
        if (vl > vl1) vl=vl1;
        if (vr < vr1) vr=vr1;
      }
    }
    p->velref_l=vl;
    p->velref_r=vr;
    // angle controller
    angleerr = th+s*M_PI/2- p->pose.th;
    normalizeangle(&angleerr);
    dvel=limit(p->gain*angleerr,p->lim,-p->lim);
    p->rvel=vr+dvel;
    p->lvel=vl-dvel;
  }
  p->poseerr=posediff(p->tgt,p->pose);
}

#endif



void turnR_init(motiontype *p){
  double s,tr,sv;
  p->velref_l=p->vel;
  p->velref_r=p->vel;
  p->velref=p->vel;
  p->tgt_o=p->tgt;
  p->thold=p->pose.th;
  p->thturned=0; 
  sv=sign(p->velcmd);
  s=sign(p->turnangle);
  p->tgt.th+=sv*p->turnangle;
  normalizeangle(&p->tgt.th);
  p->tcx=p->tgt_o.x- p->R *sin(p->tgt_o.th)*s;
  p->tcy=p->tgt_o.y+ p->R *cos(p->tgt_o.th)*s;
  if (p->R > p->w/2)
    tr=  fabs(p->velcmd/(p->R));
  else
    tr=  fabs(2*p->velcmd/p->w);
  p->vr_turn=sv*tr*(p->R+s*p->w/2);
  p->vl_turn=sv*tr*(p->R-s*p->w/2);
//  printf(" vrturn %lf vlturn %lf\n",p->vr_turn,p->vl_turn);
}

void turnR(motiontype *p){
  double s,vr,vl,th,dth,ds,angleerr,dvel,sv,tr;
   p->tgt.x=p->pose.x;
   p->tgt.y=p->pose.y;
   dth=p->pose.th-p->thold;
   normalizeangle(&dth);
   p->thturned+=dth;
   p->thold=p->pose.th;
   s=sign(p->turnangle);
// calulate acceleration limited  vehicle velocity
   calcvel(p);
   ds=p->acccmd*p->ts;
   sv=sign(p->velcmd);
 
   if (p->R > p->w/2)
     tr=  fabs(p->velref/(p->R));
   else
    tr=  fabs(2*p->velref/p->w);
   p->vr_turn=sv*tr*(p->R+s*p->w/2);
   p->vl_turn=sv*tr*(p->R-s*p->w/2);
   vr=p->vr_turn;
   ratelim(&vr,p->velref_r,ds);
   vl=p->vl_turn;
   ratelim(&vl,p->velref_l,ds);
   th=atan2(p->pose.y-p->tcy,p->pose.x-p->tcx);
   dth=fabs(p->turnangle)-fabs(p->thturned);
   switch (p->type){
    case VELOMEGA:{
      double Rlocal,omega;
      Rlocal=p->R;
      if (Rlocal <0.1) Rlocal=0.1;
      omega=s*p->velref/Rlocal;
      if (dth < 0){     
        p->target_dist=0;
      }
      else{
        p->target_dist=1;	
    }
    // angle controller
    angleerr = th+s*M_PI/2- p->pose.th;
    normalizeangle(&angleerr);
    dvel=limit(p->gain*angleerr,p->lim,-p->lim);
    //dvel=0;
    p->rvel=p->velref;
    p->lvel=p->velref;
    p->omega=dvel+omega;
  }
  break;
   
   case ACKERMAN:{
     double Rlocal;
     p->lvel=p->velref;
     Rlocal=p->R;
     if (Rlocal <0.1) Rlocal=0.1;
     p->steeringangle=s*atan(p->robotlength/Rlocal);
     p->curvature=s*1/Rlocal;
     if (dth >0 )
       p->target_dist=1;
     else
       p->target_dist=0;   
    p->velref_l=vl;
    p->velref_r=vr;    
    p->rvel=vr;
    p->lvel=vl;   

   }
   break;
   case DIFFERENTIAL:{

      if (dth < 0){
        vr=p->velcmd;
        vl=p->velcmd;
        p->target_dist=0;
      }
      else{
        p->target_dist=1;	
    }

    p->velref_l=vl;
    p->velref_r=vr;
    // angle controller
    angleerr = th+s*M_PI/2- p->pose.th;
    normalizeangle(&angleerr);
    dvel=limit(p->gain*angleerr,p->lim,-p->lim);
    //dvel=0;
    p->rvel=vr+dvel;
    p->lvel=vl-dvel;
  }
  break;
  default:{
    p->rvel=0;
    p->lvel=0;
    p->steeringangle=0;
    p->omega=0;
    p->curvature=0;
  }  
  }
  p->poseerr=posediff(p->tgt,p->pose);
}

void turnC_init(motiontype *p){
  double s,g,cartang,C,S;
  p->velref_l=p->vel;
  p->velref_r=p->vel;
  p->velref=p->vel;
  p->tgt_o=p->tgt;
  p->thold=p->pose.th;
  p->thturned=0;
  p->tgt.th+=p->turnangle;
  normalizeangle(&p->tgt.th);
  s=sign(p->turnangle);
  p->tgtnormal=p->tgt.th-s*M_PI/2;
  normalizeangle(&p->tgtnormal);
  p->tcx=p->tgt_o.x- p->R *sin(p->tgt_o.th)*s;
  p->tcy=p->tgt_o.y+ p->R *cos(p->tgt_o.th)*s;

  /* Finding Alpha, the angle acceleration */
  cartang = fabs(p->turnangle)-M_PI/2;
  g = (pow(p->R*cos(cartang),2) + pow(p->R*sin(cartang)+p->R,2)) /(2*p->R*cos(cartang));
  S = fresnel_s(sqrt(fabs(p->turnangle)/(M_PI)));
  C = fresnel_c(sqrt(fabs(p->turnangle)/(M_PI)));
  p->angacc = pow(1/g,2)*M_PI*pow((C+S*tan(fabs(p->turnangle)/2)),2);

  /*finding the distance to drive the clotoid*/
  p->halfdist = sqrt(fabs(p->turnangle)/p->angacc);

  /* initializing these parameters */
  p->turncgain = 3; //NOTE TODO REMOVE WHEN ANGLE GAIN IS PUT IN FILE
  double maxacceleration = 1; // acceleration limited to 1 m/s
  p->drivendist = 0;
  p->vr_turn=p->velcmd;
  p->vl_turn=p->velcmd;

  /* Couple of warnings */
  if(p->turnangle>M_PI/2)
  {
    printf("Warning angles larger than 90 degrees are not recomended");
  }
  if(p->angacc*p->velcmd>maxacceleration/HALF_ROBOT_WIDTH1)
  {
    printf("Warning to sharp turn");
  }
}

void turnC(motiontype *p){
  double s,vr,vl,th,dth,ds,angleerr,dvel,refangle,spdinc;

   /* Calculate the distance */
   p->drivendist += sqrt(pow(((p->pose.x)-(p->tgt.x)),2) + pow(((p->pose.y)-(p->tgt.y)),2));

   p->tgt.x=p->pose.x;
   p->tgt.y=p->pose.y;
   dth=p->pose.th-p->thold;
   normalizeangle(&dth);
   p->thturned+=dth;
   p->thold=p->pose.th;
   s=sign(p->turnangle);
   ds=p->acccmd*p->ts; // max speed change

   /* increment/decrement the speed of the wheels every sample &
      calculate referance angle */
   spdinc = p->angacc*pow(p->velref,2)*(2*HALF_ROBOT_WIDTH1)/2*0.01;
   if(p->drivendist<p->halfdist)
    {
     p->vr_turn += s*spdinc;
     p->vl_turn += -s*spdinc;
     refangle = s*0.5*p->angacc*pow(p->drivendist,2);
    }
    else
    {
     p->vr_turn -= s*spdinc;
     p->vl_turn -= -s*spdinc;
     refangle = s*(-0.5*p->angacc*pow((p->drivendist-2*p->halfdist),2))+ p->turnangle;
    }

    //printf("%f,%f\n",p->thturned,refangle);

// calulate acceleration limited  vehicle velocity
   calcvel(p);
   vr=p->vr_turn*(p->velref/p->velcmd); //stop for obstacles
   ratelim(&vr,p->velref_r,ds); //limit acceleration
   vl=p->vl_turn*(p->velref/p->velcmd); //stop for obstacles
   ratelim(&vl,p->velref_l,ds); //limit acceleration

   th=atan2(p->pose.y-p->tcy,p->pose.x-p->tcx);
   dth=fabs(p->turnangle)-fabs(p->thturned); //angle left

  //if (p->d_obstac_front < p->alarmdist)  p->status=MC_OBSTACLE;
   //printf("obs: %f, alarm: %f\n",p->d_obstac_front, p->alarmdist);

   if (p->type==ACKERMAN){
     double Rlocal;
     p->lvel=p->velref;
     Rlocal=1/(p->angacc*p->drivendist); //radius = 1/ks
     if (Rlocal <0.1) Rlocal=0.1;
     p->steeringangle=s*atan(p->robotlength/Rlocal);
     if (dth >0 )
       p->target_dist=1;
     else
       p->target_dist=0;
   }
   else {

    if (p->drivendist > 2*p->halfdist){ // stopcondition
      vr=p->velcmd;
      vl=p->velcmd;
      p->target_dist=0;
    }
    else{
      p->target_dist=1;
    }

    p->velref_l=vl;
    p->velref_r=vr;
    // angle controller
    angleerr = refangle - p->thturned; //error
    normalizeangle(&angleerr);
    dvel=limit(p->turncgain*angleerr,p->lim,-p->lim); // calculate controller output


	/* Possible obstacle code */
//     if (p->d_obstac_front < p->alarmdist && !p->ignore_obstacles_flag){
//       p->status=MC_OBSTACLE;
//       vr = 0;
//       vl = 0;
//     }


    p->rvel=vr+dvel;
    p->lvel=vl-dvel;


  }
  p->poseerr=posediff(p->tgt,p->pose);
}




void stop(motiontype *p){
double angleerr,dvel;

// angle controller
  angleerr = p->tgt.th - p->pose.th;
  normalizeangle(&angleerr);
//  dvel=limit(p->gain*angleerr,p->lim,-p->lim);
  dvel=p->gain*angleerr;
  if (p->type==ACKERMAN){
    p->rvel=0;
    p->lvel=0;
    p->steeringangle=0;
    p->curvature=0;
  }
  else {
    p->rvel=dvel;
    p->lvel=-dvel;
  }
  p->xyzout1=0;
}


void funcgen_init(motiontype *p){
  p->tgt_o.th = 0;
  p->runlength = 0;
}


void funcgen(motiontype *p){
  int k, n = p->funcgenp[0];
  double dl, dh, dg, temp;
  switch (n){
   case 0: //Step function
    {
     dl = p->funcgenp[2]/0.01;
     dh = p->funcgenp[3]/0.01 + dl;
      if (p->time > dh)
      {
	p->runlength = 1;
	//printf("Done %f\n", p->runlength);
	break; // END CONDITION
      }
        if (p->time >= dl && p->time < dh){
	   temp = p->tgt_o.th + p->funcgenp[1];
           //printf("High %f %f %f %d\n", dl, dh, temp, p->time);
        }
        else {
	   temp = p->tgt_o.th;
	   //printf("Low %f %f %f %d\n", dl, dh, temp, p->time);
        }
    }
    break;
   case 1: //Square Wave
    {
      if (p->time >= p->funcgenp[3]/0.01)
      {
	//printf("Done\n");
	p->runlength = 1;
	break;// END CONDITION
      }
    dg = 1/p->funcgenp[2]*0.5;
    k = round(p->time*0.01/dg-0.5);
     if (k % 2){
	temp = p->tgt_o.th + p->funcgenp[1];
	//printf("Even %d %f %d\n",k, temp, p->time);
	}
    else {
	temp = p->tgt_o.th - p->funcgenp[1];
	//printf("Odd %d %f %d\n",k, temp, p->time);
	}
    }
    break;
   case 2: // Sine Wave
    {
     if (p->time >= p->funcgenp[3]/0.01)
     {
      //printf("Done\n");
      p->runlength = 1;
      break;// END CONDITION
     }
    dg = 2*M_PI*(p->time*0.01)/(1/(p->funcgenp[2]));
    temp = p->funcgenp[1]*sin(dg);
    //printf("Sine Wave %f %f %d\n", dg, temp, p->time);
    }
    break;
   case 3: // Chirp Signal
    {
     if (p->time >= p->funcgenp[4]/0.01)
     {
      //printf("Done\n");
      p->runlength = 1;
      break;// END CONDITION
     }
    dg = (p->funcgenp[2]*p->time*0.01)+((p->funcgenp[3]-p->funcgenp[2])/2*p->funcgenp[1]*(p->time*0.01*p->time*0.01));
    temp = sin(dg);
    //printf("Sine Wave %f %f %d\n", dg, temp, p->time);
    }
    break;
  }
  p->funcgenout=temp;  
}




void follow_line_init(motiontype *p){
  p->line_pd.k=p->line_gain;
  p->line_pd.tau=p->line_tau;
  p->line_pd.alfa=p->line_alfa;
  p->line_pd.ts=p->ts;
  pd_init(&p->line_pd);
  p->line_pd.state=p->line_gain*(p->linepos);
  p->lastlinepose=p->pose;
  p->tgt=p->pose;
  p->velref=p->vel;
  p->duold=0;
  p->lineposold=p->linepos;
}

void follow_line(motiontype *p){
  double du,dth,dist;
  // Set target pose to current pose
  p->tgt.x=p->pose.x;
  p->tgt.y=p->pose.y;
  dth=p->pose.th-p->tgt.th;
  p->tgt.th=p->pose.th;
  calcvel(p);
  if (p->noline){
    dist= calcdist(p->pose,p->lastlinepose);
    if (dist > p->nolinedist){
      p->velref=0;
      du=0;
    }
    else
      du=p->duold;
  }
  else {
    p->lastlinepose=p->pose;
    du=p->line_pd.k*p->linepos-p->line_pd.tau*dth;
  }

  p->lineposold=p->linepos;
  if (p->type==ACKERMAN){
    double dv;
    if (du > 0.8)  du=0.8;
    if (du < -0.8) du=-0.8;
    p->duold=du;
    p->steeringangle=du;
    p->curvature=tan(du)/p->robotlength;
    dv=p->velref*p->w*p->curvature/2;
    p->lvel=p->velref-dv;
    p->rvel=p->velref+dv;
  }
  else {
    if (du > 0.15)  du=0.15;
    if (du < -0.15) du=-0.15;
    p->duold=du;
    p->rvel=p->velref+du;
    p->lvel=p->velref-du;
  }
}




void follow_wall_init(motiontype *p){
  p->wall_pd.k=p->wall_gain;
  p->wall_pd.tau=p->wall_tau;
  p->wall_pd.alfa=p->wall_alfa;
  p->wall_pd.ts=p->ts;
  pd_init(&p->wall_pd);
  if (p->side == MC_LEFT)
    p->wall_pd.state=p->wall_gain*(- p->walldistref +*(p->irdistl));
  else
    p->wall_pd.state=p->wall_gain*(+ p->walldistref -*(p->irdistr));
  p->tgt=p->pose;
  p->velref=p->vel;
}

void follow_wall(motiontype *p){
  double du,dth;
  // Set target pose to current pose
  p->tgt.x=p->pose.x;
  p->tgt.y=p->pose.y;
  calcvel(p);
  if (p->side == MC_LEFT){
    p->wall_pd.e= ( *(p->irdistl) - p->walldistref);
    if (p->wallinputmode==1)
       p->wall_pd.e+=(*(p->wallanglel)-M_PI/2)*p->wall_anglegain;
  }
  else{
    p->wall_pd.e= (-*(p->irdistr) + p->walldistref);
    if (p->wallinputmode==1)
       p->wall_pd.e-=(*(p->wallangler)+M_PI/2)*p->wall_anglegain;
  }
  pd_out(&p->wall_pd);
  pd_update(&p->wall_pd);
  dth=p->wall_pd.o;
  dth=dth*p->ts;
  p->tgt.th+= dth;
  du= (p->tgt.th - p->pose.th);
  normalizeangle(&du);
  du*=2;
  if (du > 0.2)  du=0.2;
  if (du < -0.2) du=-0.2;
  p->rvel=p->velref+du/2;
  p->lvel=p->velref-du/2;
}


void idle(motiontype *p){
  p->steeringangle=0;
  p->omega=0;
  p->rvel=0;
  p->lvel=0;
   p->xyzout1=0;
}

void align_init(motiontype *p){
/// fjernet af christian - gammel smrd kode
#if (0)
  set_motor_param(1,0,0);
  set_motor_param(1,1,0);
  set_motor_param(2,0,0);
  set_motor_param(2,1,0);
#endif
}

void align(motiontype *p){
  p->tgt=p->pose;
  p->rvel=p->velcmd;
  p->lvel=p->velcmd;
  if (fabs(p->curvell) <0.01)
    p->lvel=4;
  if ( fabs(p->curvelr) <0.01)
    p->rvel=4;
}

void directmotorcmds(motiontype *p){
  p->tgt=p->pose;
  p->rvel=p->velcmdr;
  p->lvel=p->velcmdl;
}





double calcdist(posetype p1, posetype p2){
  return(sqrt(pow(p1.x - p2.x,2)+pow(p1.y - p2.y,2)));
}

void calcvel(motiontype *p){
  double ds,vel, vel1, dob;
  ds=p->acccmd*p->ts;
  vel=p->velcmd;
  ratelim(&vel,p->velref,ds);
// calculate distance to obstacle

  if (vel > 0) {
    dob= p->d_obstac_front - p->stopdist;
  }
  else {
    dob= p->d_obstac_back - p->stopdist;
  }

  if (dob < 0) dob=0;
  if (dob < p->alarmdist)  p->status=MC_OBSTACLE;
  if (!p->ignore_obstacles_flag){
    vel1 = sqrt(2*dob*p->acccmd)*sign(vel);
    if (fabs(vel1) < fabs(vel)) vel=vel1;
  }
  p->velref=vel;
}



void wdcontroller_init(motiontype *p){
  p->irwidth = HALF_ROBOT_WIDTH1;
  p->irlen = ROBOT_LENGTH1;
  p->velmin = MINIMUN_SPEED;
  p->wdsspar2is = WDSSPAR2IS;
  /* guarantie of minimun speed */
  if(p->velcmd < p->velmin) p->velref = p->velmin;
  p->velref=p->vel; //NAA
 // p->sswd; load the parametres
}

void wdcontroller(motiontype *p){

  double angspeed;
  double corrections_left, corrections_right;

	/* preparing input */
  switch(p->method){
    case MC_REGRESSION:
      p->sswalldist = p->walldistregr;
      p->sswallang = p->pose.th - p->wallangregr;
      normalizeangle(& p->sswallang); // NAA
      break;
    case MC_DIRECT:
      switch(p->side){
        case MC_RIGHT:
          p->sswalldist = fabs(p->irlen * sin(p->pose.th) - (p->irwidth + *p->irdistr) * cos(p->pose.th));
          break;
        case MC_LEFT:
          p->sswalldist = fabs(p->irlen * sin(p->pose.th) + (p->irwidth + *p->irdistl) * cos(p->pose.th));
          break;
        default:
	;
      }
      p->sswallang = p->pose.th;
      break;
    default:
    ;
  }
  /* checking if we have already got the comanded speed */
  if( p->velref < p->velcmd ){
    p->velref += p->ts * p->acccmd;
  }

  /*calculating wdsspar_pos,which is the file for this value in the state space parametres file*/
  p->wdsspar_pos = round(p->velref/p->wdsspar2is);

  /* doing the control */
  switch(p->side){
    case MC_RIGHT:
      angspeed = (p->walldistref + p->irwidth) * p->sswd[2][p->wdsspar_pos - 1]
         -(   p->sswalldist * p->sswd[1][p->wdsspar_pos - 1]
            + p->sswallang * p->sswd[0][p->wdsspar_pos - 1]  );
      break;
    case MC_LEFT:
      angspeed = - (p->walldistref + p->irwidth) * p->sswd[2][p->wdsspar_pos - 1]
         -( - p->sswalldist * p->sswd[1][p->wdsspar_pos - 1]
            + p->sswallang * p->sswd[0][p->wdsspar_pos - 1]  );
      break;
    default:
    ;
  }




	/*checking if the data is reliant */
  if(p->method == MC_REGRESSION && p->wallreliant == 0) angspeed = 0.0;

  /* calculating the correction according with w */
  if(angspeed >= 0){
    corrections_left = 0;
    corrections_right = (angspeed * p->w /2.0);
  }else{
    corrections_left = (-angspeed * p->w /2.0);
    corrections_right = 0;
  }

  /* writting the output */
  p->lvel = p->velref + corrections_left;
  p->rvel = p->velref + corrections_right;
  p->tgt=p->pose;
}

void read_wdssparam(motiontype *p){
  FILE *fp;
  float k1, k2, n;
	int i = 0;

  if( (fp = fopen("./calib/wdssparam.dat","r")) == NULL){
    if ( (fp =fopen("/usr/local/smr/calib/wdssparam.dat","r")) == NULL){
      printf("file wdssparam.dat couldn't be opened\n");
      exit(1);
    }
  }
  while((fscanf(fp,"%f %f %f", &k1, &k2, &n) != EOF)){
    p->sswd[0][i] = k1;
    p->sswd[1][i] = k2;
    p->sswd[2][i] = n;
    i++;
  }
  fclose(fp);
}

void drivetopoint_init(motiontype *p){

	p->dtp.TOL = 0.0005;
	p->dtp.state = DTP_INIT;
	p->dtp.oldstate = DTP_INIT;

	if (p->type == ACKERMAN && p->dtp.r < 0.1) {
		p->dtp.r = 0.1; 
	}

	if (p->dtp.coordtype == DTP_COORD_ABS){
		// Use odometry coordinates and angle as starting pose
		p->dtp.xa = p->pose.x;
		p->dtp.ya = p->pose.y;
		p->dtp.thetaa = p->pose.th;
	}
	else if (p->dtp.coordtype == DTP_COORD_REL){
		// Use (0,0,0) as starting pose
		p->dtp.xa = 0;
		p->dtp.ya = 0;
		p->dtp.thetaa = 0;
	} else {
		return; // Error. It should not be possible to get here.
	}

	printf("\ndtp: Generating feasible paths...");

	// Compute feasible paths
	drivetopoint_generate(p->dtp.xa, p->dtp.ya, p->dtp.thetaa, p->dtp.xb, p->dtp.yb, p->dtp.thetab, p->dtp.r, p->dtp.arrpath);

	printf("\ndtp: Choosing the optimal path...");

	// Use predefined path evaluation method to set an optimal path
	// (Pass address of the path array's first element and address of optimal path)
	switch (p->dtp.evalmethod){
		case DTP_EVAL_SHORTEST:
			printf("\ndtp_evalmethod = DTP_EVAL_SHORTEST");
			dtp_eval_shortest( p->dtp.arrpath , &(p->dtp.optpath), p->dtp.r );
		break;
		case DTP_EVAL_SHORTEST_NOBACK:
			printf("\ndtp_evalmethod = DTP_EVAL_SHORTEST_NOBACK");
			dtp_eval_shortest_noback( p->dtp.arrpath , &(p->dtp.optpath), p->dtp.r );
		break;
		default:
			// Use shortest path as default evaluation
			dtp_eval_shortest( p->dtp.arrpath , &(p->dtp.optpath), p->dtp.r );
	}
	// Initialization done.
	printf("\ndtp: Starting movement...");
	p->tgt = p->pose; // Update current pose
	p->dtp.state = DTP_STARTCIRCLE;
}

void drivetopoint(motiontype *p){

	if (p->dtp.state != p->dtp.oldstate){
		p->dtp.statetime = 0;
		p->dtp.oldstate = p->dtp.state;
	} else {
		p->dtp.statetime++;
	}

	switch(p->dtp.state){
		case DTP_INIT:
			// Do nothing before the initialization is done
		break;
		case DTP_STARTCIRCLE:
			if (p->dtp.statetime == 0){
				if (p->dtp.optpath.beta1 == 0){
					p->dtp.state = DTP_STRAIGHTLINE;
					break;
				}
				// Stop if we need to change direction
				if (p->dtp.optpath.dir1 * sign(p->vel) == -1){
					stop(p);
				}
				p->turnangle = p->dtp.optpath.beta1;
				p->R = p->dtp.r;
				p->velcmd = p->dtp.optpath.dir1 * fabs(p->velcmd);
				turnR_init(p);
			}
			turnR(p);
			if ( fabs(p->target_dist) < p->dtp.TOL ){
				p->tgt = p->pose; // Update current pose
				p->dtp.state = DTP_STRAIGHTLINE;
			}
		break;
		case DTP_STRAIGHTLINE:
			if (p->dtp.statetime == 0){
				if (p->dtp.optpath.l == 0){
					p->dtp.state = DTP_ENDCIRCLE;
					break;
				}
				// NB: SIGN(0) == 1 !!!!
				if (p->dtp.optpath.dir2 * p->dtp.optpath.dir3 == -1) {
					// Use fwd here
					p->dist = p->dtp.optpath.dir2 * p->dtp.optpath.l;
					p->velcmd = fabs(p->velcmd);
					move_init(p);
					
				} else {
					// Use drive here
					p->refpose = p->pose;
					p->velcmd = p->dtp.optpath.dir2 * fabs(p->velcmd);
					drive_init(p);
				}
			}
			if (p->dtp.optpath.dir2 * p->dtp.optpath.dir3 == -1) {
				move(p);
				if ( sign(p->dist) * p->target_dist < p->dtp.TOL ){
					p->tgt = p->pose; // Update current pose
					p->dtp.state = DTP_ENDCIRCLE;
				}
			} else {
				drive(p);
				if ( p->dtp.optpath.l - fabs(p->target_dist) < p->dtp.TOL ){
					p->tgt = p->pose; // Update current pose
					p->dtp.state = DTP_ENDCIRCLE;
				}
			}
			
		break;
		case DTP_ENDCIRCLE:
			if (p->dtp.statetime == 0){
				if (p->dtp.optpath.beta3 == 0){
					p->dtp.state = DTP_FINISH;
					break;
				}
				// Stop if we need to change direction
				if (p->dtp.optpath.dir3 * sign(p->vel) == -1){
					stop(p);
				}
				p->turnangle = p->dtp.optpath.beta3;
				p->R = p->dtp.r;
				p->velcmd = p->dtp.optpath.dir3 * fabs(p->velcmd);
				turnR_init(p);
			}
			turnR(p);
			if ( fabs(p->target_dist) < p->dtp.TOL ){
				p->tgt = p->pose; // Update current pose
				p->dtp.state = DTP_FINISH;
			}
		break;
		case DTP_FINISH:
			if (p->dtp.statetime == 0){
			}
			// Do nothing
		break;
		default:
		;
			// Do nothing
	}
}
