//*******************************************************************
// Start of statemachines
//

#define SM_CASE(a,b,c)	case a:{ if(sm->time==0){b} else {c} break;}
#define FWD(a)	{motcon.cmd=MC_FWDCMD;motcon.dist=(a);}
#define TURN(a) {  motcon.cmd=MC_TURNCMD;motcon.turnangle=(a);}
#define TURNR(a,b) {  motcon.cmd=MC_TURNRCMD;motcon.turnangle=(a);motcon.R=(b);}
#define NEXTSTATE(a,b) if (a) sm->state=(b);
#define FWDEND	(fabs(motcon.target_dist)<0.005)
#define TURNEND (fabs(motcon.poseerr.th)<0.01)
#define TURNREND (fabs(motcon.target_dist)<0.005)
#define D2R 	(M_PI/180)

void sm_userupdate(sm_type *sm){
    sm->startdist=odo.dist;
    sm->startpose.x=odo.pose.x;
    sm->startpose.y=odo.pose.y;
    sm->startpose.th=odo.pose.th;
}

double statedist(sm_type *sm){
return(odo.dist - sm->startdist);
}


int sm_square(sm_type * sm){
 sm_update(sm);
 switch(sm->state){
      SM_CASE(0, 
          sm->status=SM_RUNNING;
	  FWD(sm->p[0]);,
          NEXTSTATE(FWDEND,1))

      SM_CASE(1,TURN(sm->p[1]),NEXTSTATE(TURNEND,2))

      SM_CASE(2,FWD(sm->p[0]) ,NEXTSTATE(FWDEND,3))
     
      SM_CASE(3,TURN(sm->p[1]),NEXTSTATE(TURNEND,4))

      SM_CASE(4,FWD(sm->p[0]) ,NEXTSTATE(FWDEND,5))

      SM_CASE(5,TURN(sm->p[1]),NEXTSTATE(TURNEND,6))

      SM_CASE(6,FWD(sm->p[0]) ,NEXTSTATE(FWDEND,7))

      SM_CASE(7,TURN(sm->p[1]),NEXTSTATE(TURNEND,8))

      case 8:
      if(sm->time==0){
        printf(" pose %lf %lf %lf \n",odo.pose.x,odo.pose.y,odo.pose.th/M_PI*180);
        motcon.cmd=MC_STOPCMD;
      }
      else{
        sm->status=SM_FINISHED;
      }
      break;
      default:
       ;

  }
return(sm->status);
}

int sm_doublesquare(sm_type *sm){
  sm_update(sm);
  switch (sm->state) {

  SM_CASE(0,
     sm->status=SM_RUNNING;
     sm->sm1=get_sm_data();
     sm->sm1->p[0]=sm->p[0];
     sm->sm1->p[1]=M_PI/2;
     sm_reset(sm->sm1);,
     if (sm_square(sm->sm1)==SM_FINISHED) {sm->state=1;free_sm_data(sm->sm1);})

  SM_CASE(1,
     sm->status=SM_RUNNING;
     sm->sm1=get_sm_data();
     sm->sm1->p[0]=sm->p[0];
     sm->sm1->p[1]=-M_PI/2;
     sm_reset(sm->sm1);,
     if (sm_square(sm->sm1)==SM_FINISHED) {sm->state=2;free_sm_data(sm->sm1);})

  case 2:
    sm->status=SM_FINISHED;
  break;
  default:
   ;
  }
return(sm->status);
}

//**********************************************************************
//*
//*  sm_n_squares
//*
int sm_n_squares(sm_type *sm){
  sm_update(sm);
  switch (sm->state) {

  SM_CASE(0,
     sm->status=SM_RUNNING;
     sm->sm1=get_sm_data();
     sm->sm1->p[0]=sm->p[0];
     sm->sm1->p[1]=M_PI/2;
     sm_reset(sm->sm1);,
     if (sm_square(sm->sm1)==SM_FINISHED) {sm->state=1;free_sm_data(sm->sm1);})

  SM_CASE(1,
    sm->p[1]--;
    if (sm->p[1]> 0) sm->state=0;
    ,
    sm->state=2;
    )

  case 2:
    sm->status=SM_FINISHED;
  break;
  default:
   ;
  }
return(sm->status);
}
//********************************************************************
//
//  sm_follow_line
//
int sm_follow_line(sm_type *sm){
  sm_update(sm);
  switch (sm->state) {
   SM_CASE(0,
     sm->status=SM_RUNNING;
     linetype=LINE_MIDDLE_B;
     motcon.cmd=MC_FOLLOW_LINECMD;
     motcon.velcmd=0.3;,
     if (odo.dist > sm->p[0]) sm->state=1;)

  SM_CASE(1,TURN(M_PI),NEXTSTATE(TURNEND,2))
  
  SM_CASE( 2,
    sm->status=SM_FINISHED;
    motcon.cmd=MC_STOPCMD;,
     )
  break;
  default:
   ;
  }
return(sm->status);
}
//********************************************************************
//  sm_follow_wall
//

int sm_follow_wall(sm_type *sm){
  sm_update(sm);
  switch (sm->state) {
   SM_CASE(0,
     sm->status=SM_RUNNING;
     motcon.cmd=MC_FOLLOW_WALLCMD;
     motcon.velcmd=sm->p[2];
     motcon.walldistref=sm->p[1];
     sm->p[9]=sm->p[1];
     sm->p[8]=0;
     ,
     {
       double tmp=0.01/0.3;
       sm->p[9]=sm->p[9]*(1-tmp)+tmp*ir_dist[0];
       sm->p[8]=sm->p[8]*(1-tmp)+tmp*odo.pose.th;
       if (statedist(sm)> 1) sm->state=1;
     
      })

  SM_CASE(1,
     motcon.cmd=MC_FOLLOW_WALLCMD;
     motcon.velcmd=sm->p[2];
     motcon.walldistref=sm->p[1];
     ,
     {
       double tmp=0.01/0.3;
       if  ((ir_dist[0]-sm->p[9])>0.08){
         motcon.cmd=MC_DRIVECMD;
	 motcon.tgt.th=sm->p[8];
	 sm->state=2;
       }
       else {
         sm->p[9]=sm->p[9]*(1-tmp)+tmp*ir_dist[0];
	 sm->p[8]=sm->p[8]*(1-tmp)+tmp*odo.pose.th;
       }
      
      if (odo.dist > (sm->p[0])) sm->state=3;
     })
  
   SM_CASE(2,
     
     motcon.cmd=MC_DRIVECMD;
     motcon.velcmd=sm->p[2];
     motcon.walldistref=sm->p[1];
     ,
     {
       if  ((ir_dist[0]-sm->p[9])>0.08 && statedist(sm)<1.2){
   
         motcon.cmd=MC_DRIVECMD;
       }
       else {
         motcon.cmd=MC_FOLLOW_WALLCMD;
         sm->state=0;
       }
      
      if (odo.dist > (sm->p[0])) sm->state=3;
     })
  
  
  
  
  
  SM_CASE( 3,
    sm->status=SM_FINISHED;
    motcon.cmd=MC_STOPCMD;,
     )
  break;
  default:
   ;
  }
return(sm->status);
}

int sm_forward(sm_type *sm){
  sm_update(sm);
  switch (sm->state) {
   SM_CASE(0,
     sm->status=SM_RUNNING;
     motcon.cmd=MC_FWDCMD;
     motcon.velcmd=sm->p[1];
     motcon.dist=sm->p[0];,
     if ((odo.dist > sm->p[0]) ||(sm->time> sm->p[2])) sm->state=1;)


  SM_CASE( 1,
    sm->status=SM_FINISHED;
    motcon.cmd=MC_STOPCMD;,
     )
  break;
  default:
   ;
  }
return(sm->status);
} 

//*******************************************************************  
// sm_drive
//
int sm_drive(sm_type *sm){
  sm_update(sm);
  switch (sm->state) {
   case 0:
    sm->p[9]=0;
    sm->state=1;
   sm->status=SM_RUNNING;
   break;  

   SM_CASE(1,
    
     motcon.velcmd=sm->p[1];
     motcon.cmd=MC_DRIVECMD;
     ,
     NEXTSTATE(statedist(sm)>(fabs(sm->p[0])),2) )
   
   SM_CASE(2,
     motcon.cmd=MC_TURNRCMD;
     motcon.R = 0.25;
     motcon.turnangle=M_PI/2;
     , NEXTSTATE(motcon.target_dist<0.1 ,3))
  
   case 3:
     sm->p[9]++;
     if (sm->p[9]== 4) sm->state=4; else sm->state=1;
   break;  
      
     
  SM_CASE( 4,
    sm->status=SM_FINISHED;
    motcon.cmd=MC_STOPCMD;,
     )
  break;
  default:
   ;
  }
return(sm->status);
} 

//*******************************************************************
//  sm_find_box
//
int sm_find_box(sm_type *sm){
  sm_update(sm);
  switch (sm->state) {
   SM_CASE(0,
     sm->status=SM_RUNNING;
     linetype=LINE_RIGHT_B;
     motcon.cmd=MC_FOLLOW_LINECMD;
     motcon.velcmd=0.4;,
     if (ir_dist[2] < sm->p[1] ) {sm->state=100;sm->p[9]=-odo.pose.y+0.4;}else if (statedist(sm) > sm->p[0]) sm->state=4;)
    
   SM_CASE(100,
   	   motcon.cmd=MC_STOPCMD;
	   sm->p[9]=0;
	   ,
	   if (sm->time >40)
	     sm->p[9]+=ir_dist[2]; 
           if (sm->time== 100){
	     printf("dist= %lf \n",-odo.pose.y/1.02+sm->p[9]/(100-40)+0.31);
	     sm->state=1;
	   })

  case 1:{
     char buf[80];
    if (sm->time==0){
      double tmp,l=1.13;
      tmp=-odo.pose.y-2.00;
      sm->p[9]=atan2(tmp,l);
      sm->p[8]=sqrt(tmp*tmp+l*l);
      TURN(M_PI/2+sm->p[9]);
      sprintf(buf,"Obstacle at %6.3f meters",-odo.pose.y+0.35);
      smrspeak(buf);
    }
    else
      NEXTSTATE(TURNEND,2);
  }
  break;

  
  SM_CASE(2,motcon.velcmd=0.8;FWD(sm->p[8]) ,NEXTSTATE(FWDEND,3))

  SM_CASE(3,TURN(-M_PI/2-sm->p[9]),NEXTSTATE(TURNEND,6))
  
 
  SM_CASE( 6,
    sm->status=SM_FINISHED; ,
     )
  break;
  default:
   ;
  }
return(sm->status);
}    

//*******************************************************************
//  sm_between_obstacles
//

int sm_between_obstacles(sm_type *sm){
  sm_update(sm);
  switch (sm->state) {
   case 0:
     if (sm->time==0){
       sm->status=SM_RUNNING;
       motcon.cmd=MC_STOPCMD;
     }
     else {
       if (sm->time > sm->p[0]*100) sm->state=1;
     }
   break;
   
   SM_CASE(1,
     sm->status=SM_RUNNING;
     linetype=LINE_RIGHT_B;
     motcon.cmd=MC_FOLLOW_LINECMD;
     motcon.velcmd=0.3;,
     if (ir_dist[2] < sm->p[1] ) {sm->state=2;sm->p[9]=odo.dist-sm->startdist;})

  case 2:{
    if (sm->time==0){
      TURN(M_PI);
    }
    else
      NEXTSTATE(TURNEND,1);
  }
  break;

  default:
   ;
  }
 return(sm->status);
}    
//*******************************************************************
//  sm_open_door
//

int sm_open_door(sm_type *sm){
   
  sm_update(sm);
  switch (sm->state) {
   SM_CASE(0,
     motcon.ignore_obstacles=1;
     sm->status=SM_RUNNING;
     motcon.cmd=MC_FOLLOW_LINECMD;
     linetype=LINE_RIGHT_B;
     motcon.velcmd=0.2;,
     if (ir_dist[2] < sm->p[1] ) {sm->state=1;sm->p[9]=odo.dist-sm->startdist;}else if (statedist(sm)> sm->p[0]) sm->state=16;)
  
  case 1:{
    if (sm->time==0){
       motcon.velcmd=0.4;
      TURN(M_PI/2);
    }
    else
      NEXTSTATE(TURNEND,2);
  }
  break;

  SM_CASE(2,FWD(0.65) ,NEXTSTATE(FWDEND,3))
         
  SM_CASE(3,TURN(-M_PI/2);,NEXTSTATE(TURNEND,4))

  SM_CASE(4,FWD(0.65) ,NEXTSTATE(FWDEND,5))
         
  SM_CASE(5,TURN(30*D2R);,NEXTSTATE(TURNEND,6))

  SM_CASE(6,motcon.velcmd=0.35;FWD(-1.3) ,NEXTSTATE(FWDEND,7))
         
  SM_CASE(7,TURN(-30*D2R);,NEXTSTATE(TURNEND,15))

 

  SM_CASE(15,
     linetype=LINE_RIGHT_B;
     motcon.cmd=MC_FOLLOW_LINECMD;
     motcon.velcmd=0.4;
     ,
     if (odo.dist -sm->startdist > 1.1) sm->state=16;)
  

  SM_CASE( 16,
    motcon.ignore_obstacles=0;
    sm->status=SM_FINISHED;
    motcon.cmd=MC_STOPCMD;,
     )
  break;
  default:
   ;
  }
return(sm->status);
} 

//*******************************************************************
//  move_box
//
int sm_move_box(sm_type *sm){
  sm_update(sm);
  switch (sm->state) {

  SM_CASE(0,
     motcon.ignore_obstacles=1;
     linetype=LINE_MIDDLE_B;
     motcon.cmd=MC_FOLLOW_LINECMD;
     motcon.velcmd=0.35;, 
     if (statedist(sm)>0.3 && check_crossing_black(&line_data)) {sm->state=2;})
 
     SM_CASE(2,smrsound("obstacle1");FWD(0.2);motcon.ignore_obstacles=1; ,NEXTSTATE(FWDEND,3))

     SM_CASE(3,motcon.velcmd=0.6;FWD(-1.1) ,NEXTSTATE(FWDEND,4))
   
    SM_CASE(4, TURN(-M_PI/2),NEXTSTATE(TURNEND,5))

    SM_CASE(5,motcon.velcmd=0.3;motcon.cmd=MC_DRIVECMD; ,
         if (statedist(sm)> 0.4) motcon.velcmd=0.2;
         NEXTSTATE(line_found(&line_data) && statedist(sm)>0.2,6))
   

    SM_CASE(6,TURNR(M_PI/2,0.21),NEXTSTATE(TURNEND,7))

    SM_CASE(7,  
     linetype=LINE_MIDDLE_B;
     motcon.cmd=MC_FOLLOW_LINECMD;
     motcon.velcmd=0.25;,
 
     if (check_crossing_black(&line_data) && statedist(sm)> 1.1) {sm->state=8;})
   
    SM_CASE(8,        
       motcon.cmd=MC_DRIVECMD;
       motcon.velcmd=0.4;, 
       if (statedist(sm) > 0.3 ) {sm->state=81;})
       
    SM_CASE(81,TURNR(10*D2R,1.0),NEXTSTATE(TURNREND,82))

    SM_CASE(82,        
       motcon.cmd=MC_DRIVECMD;
       motcon.velcmd=1.0;, 
       if (statedist(sm) > 2.0 ) {motcon.velcmd=0.35;};
       if (statedist (sm) > 2.3 ) motcon.velcmd=0.2;
       if ( statedist(sm)>2.3 && check_crossing_black(&line_data)) {sm->state=83;};
       )   

    SM_CASE(83,TURNR(1.57,0.21);,NEXTSTATE(TURNREND,9))
        
       
        
    SM_CASE(9,  
       linetype=LINE_RIGHT_B;
       motcon.cmd=MC_FOLLOW_LINECMD;
       motcon.velcmd=0.3;, 
       if ( statedist(sm)>0.4 && check_crossing_black(&line_data)) {sm->state=10;})
 

    SM_CASE( 10,
    sm->status=SM_FINISHED;
    ,
     )
  break;
  default:
   ;
  }
return(sm->status);
}    
//*******************************************************************
//  SM_course
//
//  This state machine runs the test course of 31385 s03
//
//



int sm_course(sm_type *sm){
  enum{S_start,S_findbox,S_movebox,S_wall00,S_wall01,S_wallend,
      S_wallgate00,S_wallgate01,S_wallgate02,
      S_prec00,S_prec01,S_prec02,S_prec03,
      S_wline00,S_wline01,
      S_togoal00,S_togoal01,S_opengate,
      S_atgoal,S_end};
      
  sm_update(sm);
  switch (sm->state) {

  SM_CASE(S_start,
      smrsound("norm");
      motcon.cmd=MC_FOLLOW_LINECMD;
      sm->status=SM_RUNNING;
      motcon.velcmd=0.4;//sm->state=S_prec01;
      ,
      NEXTSTATE(statedist(sm)>1.3,S_findbox);
       ) 


  SM_CASE(S_findbox, sm->status=SM_RUNNING;
     sm->sm1=get_sm_data();
     sm->sm1->p[0]=3;
     sm->sm1->p[1]=0.25;
     sm_reset(sm->sm1);,
     if (sm_find_box(sm->sm1)==SM_FINISHED) {sm->state=S_movebox;free_sm_data(sm->sm1);})

  SM_CASE(S_movebox,
     sm->status=SM_RUNNING;
     sm->sm1=get_sm_data();
     sm_reset(sm->sm1);,
     if (sm_move_box(sm->sm1)==SM_FINISHED) {sm->state=S_wall00;free_sm_data(sm->sm1);})
   

  SM_CASE(S_wall00,
         motcon.cmd=MC_TURNRCMD;
         motcon.turnangle=M_PI/2;
         motcon.R=0.21;
	 smrspeak("Looking for wall");
         ,
         NEXTSTATE(motcon.target_dist <0.1,S_wall01))

  SM_CASE(S_wall01,
     motcon.cmd=MC_FOLLOW_WALLCMD;
     motcon.velcmd=0.35; 
     motcon.walldistref=0.25;
     smrsound("wall");
      ,
     if (statedist(sm) > 1 && *motcon.irdistl > 0.5) {sm->state=S_wallend; *motcon.irdistl=0.25;})
 
  SM_CASE(S_wallend,
         motcon.cmd=MC_TURNRCMD;
         motcon.turnangle=83*D2R;
         motcon.R=0.32;
         ,
         NEXTSTATE(motcon.target_dist <0.1,  S_wallgate00))
 

  SM_CASE(S_wallgate00,motcon.cmd=MC_DRIVECMD; ,NEXTSTATE(statedist(sm)>0.4,S_wallgate01))

  SM_CASE(S_wallgate01,TURNR(-M_PI/2,0.25);,NEXTSTATE(TURNREND,S_wallgate02))
 
  SM_CASE(S_wallgate02, 
     motcon.cmd=MC_DRIVECMD;,  
    NEXTSTATE(line_found(&line_data) && statedist(sm)>0.4,S_prec00))  
 

  SM_CASE(S_prec00,TURNR(-M_PI/2,0.21);,NEXTSTATE(TURNREND,S_prec01))

  SM_CASE(S_prec01,
     motcon.ignore_obstacles=0;
     linetype=LINE_MIDDLE_B;
     motcon.cmd=MC_FOLLOW_LINECMD;
     motcon.velcmd=0.35;, 
     if (statedist(sm)>1.0 && check_crossing_black(&line_data)) {sm->state=S_prec02;})

  SM_CASE(S_prec02,TURNR(-1.2,0.3);,NEXTSTATE(TURNREND,S_prec03))

  SM_CASE(S_prec03,
     motcon.ignore_obstacles=0;
     linetype=LINE_MIDDLE_B;
     motcon.cmd=MC_FOLLOW_LINECMD;
     motcon.velcmd=0.35;, 
     if  (statedist(sm)>2.7) linetype=LINE_RIGHT_B;
     if (statedist(sm)>3.00) motcon.velcmd=0.25;
     if (statedist(sm)>3.32) {sm->state=S_wline00;})
 
  SM_CASE(S_wline00,TURNR(-100*D2R,0.22);,NEXTSTATE(TURNREND,S_wline01))


  SM_CASE(S_wline01,
     motcon.ignore_obstacles=0;
     linetype=LINE_MIDDLE_W;
     motcon.cmd=MC_FOLLOW_LINECMD;
     motcon.velcmd=0.3;, 
     if (statedist(sm) >1.0) motcon.velcmd=0.4;
    if (statedist(sm)>2.0 && check_crossing_black(&line_data)) {sm->state=S_togoal00;})
    
  SM_CASE(S_togoal00,FWD(0.21),NEXTSTATE(FWDEND,S_togoal01))  

  SM_CASE(S_togoal01,TURN(-M_PI/2);,NEXTSTATE(TURNEND,S_opengate))
 
  SM_CASE(S_opengate, 
     sm->sm1=get_sm_data();
     sm->sm1->p[0]=0.75;
     sm->sm1->p[1]=0.25;
     sm_reset(sm->sm1);
     smrsound("door");
     ,
     if (sm_open_door(sm->sm1)==SM_FINISHED) {sm->state=S_atgoal;free_sm_data(sm->sm1);})

  SM_CASE(S_atgoal,motcon.cmd=MC_STOPCMD;motcon.velcmd=0; smrsound("final");, NEXTSTATE(sm->time
   >400,S_end) )

  case S_end:
     sm->status=SM_FINISHED;
     
    break;

    default:
   ;
  }
return(sm->status);
}

//*******************************************************************  
// sm_test
//
int sm_test(sm_type *sm){
  sm_update(sm);
  switch (sm->state) {
   SM_CASE(0,
    sm->p[9]=0;
    motcon.cmd=MC_STOPCMD;
    sm->status=SM_RUNNING;
    ,
    NEXTSTATE(1,1)) 
     

   SM_CASE(1,
     
     motcon.velcmd=0.3;
    motcon.cmd=MC_FOLLOW_LINECMD;
//     motcon.cmd=MC_DRIVECMD;
     linetype=LINE_TEST;
//     motcon.acccmd=10;
     ,
    switch (2) {
      case 1:{
       if (sm->time==300) motcon.velcmd=0.2;//motcon.tgt.th+=0.04;
       if (sm->time==600) motcon.velcmd=0.3;//motcon.tgt.th+=0.04;
       if (sm->time==900) motcon.velcmd=0.2;//motcon.tgt.th+=0.04;
      }
      break;
      case 2:{
      //motcon.velcmd=0.05*sin(4*M_PI*sm->time/100.0)+0.3;
     // if (sm->time==100) motcon.tgt.th+=0.04;
    //   if (sm->time==600) motcon.tgt.th+=0.04;
      // if (sm->time==900) motcon.tgt.th+=0.04;
      }
     break;
    }
     
     NEXTSTATE(statedist(sm)>sm->p[0],3) )
   
  SM_CASE(3,TURN(M_PI);,NEXTSTATE(TURNEND,4))
     
  SM_CASE( 4,
    sm->status=SM_FINISHED;
    motcon.cmd=MC_STOPCMD;,
     )
  break;
  default:
   ;
  }
return(sm->status);
} 


//*******************************************************************  
// sm_rectangle
// p0	length of rectangle
// p1   width of rectangle
// p2	turning radius
//  

int sm_rectangle(sm_type *sm){
  enum{s_start,s_turn1,s_fwd2,s_turn2,s_fwd3,s_turn3,s_fwd4,s_done};
 sm_update(sm);
   switch (sm->state) {
    SM_CASE(s_start,
      motcon.cmd=MC_DRIVECMD;
      sm->status=SM_RUNNING;
      motcon.velcmd=0.3;
      ,
      NEXTSTATE(statedist(sm)>sm->p[0] - 2* sm->p[2],s_turn1)) 
     
    SM_CASE(s_turn1,TURNR(-M_PI/2,sm->p[2]), NEXTSTATE(TURNREND,s_fwd2);)
    
    SM_CASE(s_fwd2, motcon.cmd=MC_DRIVECMD;
      ,
      NEXTSTATE(statedist(sm)>sm->p[1]-sm->p[2]*2,s_turn2)) 
    
    SM_CASE(s_turn2,TURNR(-M_PI/2,sm->p[2]), NEXTSTATE(TURNREND,s_fwd3);)
    
    SM_CASE(s_fwd3, motcon.cmd=MC_DRIVECMD;
      ,
      NEXTSTATE(statedist(sm)>sm->p[0]-sm->p[2]*2,s_turn3))
 
    SM_CASE(s_turn3,TURNR(-M_PI/2,sm->p[2]), NEXTSTATE(TURNREND,s_fwd4);)
    
    SM_CASE(s_fwd4, motcon.cmd=MC_DRIVECMD;
      ,
      NEXTSTATE(statedist(sm)>sm->p[1]-sm->p[2]*2,s_done))
    
    SM_CASE(s_done,sm->status=SM_FINISHED;,)
  }        
return(sm->status);
} 


//*******************************************************************  
// sm_search
// p0	length of rectangle
// p1   width of rectangle
// p2	turning radius
// p3	loop distance
// p4   number of loops

int sm_search(sm_type *sm){
  enum{s_start,s_loops,s_fwd,s_turn,s_done};
 
  sm_update(sm);
   switch (sm->state) {
    SM_CASE(s_start,
      motcon.cmd=MC_DRIVECMD;
      sm->status=SM_RUNNING;
      motcon.velcmd=0.3;
      sm->sm1=get_sm_data();
      sm->sm1->p[0]=sm->p[0];
      sm->sm1->p[1]=sm->p[1];
      sm->sm1->p[2]=sm->p[2];
     ,
      NEXTSTATE(statedist(sm) > sm->p[2],s_loops))

     SM_CASE(s_loops,
        sm_reset(sm->sm1);
       ,
       if (sm_rectangle(sm->sm1)==SM_FINISHED) {
         sm->p[4]--;
         if (sm->p[4]< 1) {
           sm->state=s_done;free_sm_data(sm->sm1);
        }
        else
          sm->state=s_fwd;
       }
     )

    SM_CASE(s_fwd, motcon.cmd=MC_DRIVECMD;
      ,
      NEXTSTATE(statedist(sm)>sm->p[3],s_turn))

    SM_CASE(s_turn,TURNR(-M_PI/2,sm->p[2]), NEXTSTATE(TURNEND,s_loops);)


    SM_CASE(s_done,sm->status=SM_FINISHED;motcon.cmd=MC_STOPCMD;,)
  }
  return(sm->status);
}

//*******************************************************************
// sm_interp
//


int sm_interp(sm_type *sm){
  enum{s_getcmd,s_cond,s_smfunc,s_done};
  sm_update(sm);
  switch (sm->state) {
     case s_getcmd:
       {
         int parseline=1,res;
         sm->status=SM_RUNNING;
         while (parseline){
           if (plan.pline!=plan.end){
             lineno++;
             bp=plan.pline;
             plan.pline+=strlen(plan.pline)+1;
             update_internvar(&internvar,sm);
             newcmd();
             if (verbose)
	       printf("Line no %3d:   %s \n",lineno,bp);
             strcpy(siminterface.cmdbuf,bp);
	     //*******************************************************
	     #if(0)
	     if (siminterface.simulating){
	       int n,n1,j;
	       n=strlen(bp);
	       j=0;
	       do {
	         if (n > 29)
	           n1=29;
	         else
	           n1=n;
	         n=n-n1;
                 strncpy(&robot->msg[2],(bp+j*29),n1);
	         robot->msg[31]=0;
                 robot->msg[0]=ENET_TYPE_485| (n1+1);
	         robot->msg[1]= (j<<4)| 15; // cmd 0 address 15
	         j++;
	         robot->write_flags=SMR_FLAG_AM;
                 smr_write(robot);
	       }while (n>0 && j< 15);
	     }
	     #endif
	     //**************************************************************
             siminterface.newcmd=1;
             intp_smfuncptr=NULL;
             if (intp_switch.status == SW_STARTED || intp_switch.status==SW_AFTERCASE){
	       res=1;
	        /* Ignore whitespace, get first nonwhite character.  */
       		while (*bp == ' ' || *bp =='\t' || *bp=='\r' )bp++;
		//printf(" in switch %s \n",bp);
 		if ( strncmp(bp,"case ",5)==0){
		  double val;
		  bp+=5;
		  while (*bp == ' ' || *bp =='\t' || *bp=='\r' )bp++;
                  sscanf (bp,"%lf", &val);
		  //printf("val %lfd %d \n",(val),intp_switch.actualcase);
		  if (fabs(val-intp_switch.actualcase)<0.1)
		    intp_switch.status=SW_INCASE;
		}
		else {
		  if ( strncmp(bp,"endswitch",9)==0)
  		    intp_switch.status=SW_ENDED;
                }
	     }
	     else
	       res=yyparse();

             switch (res){
               case 0: parseline=0;
                       if (intp_smfuncptr==NULL)
                         sm->state=s_cond; // wait for command completion
                       else{
                         sm->state=s_smfunc;
                         sm_reset(intp_smdataptr);
                         printf(" p0 %f \n",intp_smdataptr->p[0]);
                       }
	       break;
               case 1: parseline=1; // parse nextline
                       sm->p[0]=-2;
               break;
               case 2: printf(" syntax error in line : %d \n",lineno);
                       sm->state=s_done;
                       sm->status=SM_FINISHED;
                       parseline=0;
                       sm->p[0]=-1;
               break;
               default:
                       printf("Unknown parser result\n");
                       sm->state=s_done;
                       parseline=0;
                       sm->p[0]=-1;
               break;
           
             }  
           }
           else {
             sm->state=s_done;
             parseline=0;
             sm->status=SM_FINISHED;
           }
         }
       }
    break;
    
    case s_cond:
      update_internvar(&internvar,sm);
      if (defaultcond()) {sm->p[0]=0;sm->state=s_getcmd ;}
      if (getncond()){
        calccond();
        yyparse();
        if ((sm->p[0]=getcondres())) sm->state=s_getcmd;
      }
      break;

    case s_smfunc:
       if (intp_smfuncptr(intp_smdataptr)==SM_FINISHED) sm->state=s_getcmd;
    break;
  
    SM_CASE(s_done,sm->status=SM_FINISHED;,)
  }        
  return(sm->status);
} 

//*******************************************************************  
// sm_server
//
//
#define CMDQUESIZE	400


typedef struct {
         int id;
	 char line[256];
	}cmdqueueelem;

extern double anstab[20];
extern int n_ans;
char svr_buf[2048];
char ansbuf[2048];
char plan_buf[2048];
int curplanno;
cmdqueueelem cmdqueue[CMDQUESIZE];
int planrunning=0,cmdqueuecount=0,cmdid=0,cmdquein=0,cmdqueout=0,curcmdid=0;
int streamcount=0;
char eventtab[100][256];
int  eventin=0,eventout=0,nevent=0,waitingforevent,eventtimeout;
socklen_t addr_len;
int putevent(char * event);
int getevent(char * event);
int sm_server(sm_type *sm){
  enum{s_getcmd,s_waitforconnection ,s_done};
  sm_update(sm);
  if (streamvar.n >0 && (streamcount%streamvar.sampletime)==0){
     int i,n1;
     double time;
     char *ansbufptr;
            ansbufptr=ansbuf;
	    time=smrgettime();
	    if (xmlon){
	      n1=sprintf(ansbufptr,"<data time=\"%lf\" ",time);
              ansbufptr+=n1;
              for (i=0;i<streamvar.n;i++){
                n1=sprintf(ansbufptr,"%s=\"%lf\" ",streamvar.ref[i]->name,streamvar.ref[i]->value.pvar[0]);
                ansbufptr+=n1;
              }
	      n1=sprintf(ansbufptr," />\n");
              ansbufptr+=n1;
	    }
	    else {  
              n1=sprintf(ansbufptr,"stream %lf ",time);
              ansbufptr+=n1;
              for (i=0;i<streamvar.n;i++){
                n1=sprintf(ansbufptr,"%lf ",streamvar.ref[i]->value.pvar[0]);
                ansbufptr+=n1;
              }
              *ansbufptr='\n';
              ansbufptr++;
	    }	
          send(serv.s,ansbuf,(int)(ansbufptr-ansbuf),0);
  }
  streamcount++;
  //if ((sm->time & 127)==0)
    //printf(" state = %d %d\n",sm->state,s2lbuf.Nib);
  switch (sm->state) {
     
    case s_getcmd:{
 
      sm->status=SM_RUNNING;
      s2lbuf.Nib=recv(serv.s,s2lbuf.ib,S2L_IBSIZE,0);
      if (s2lbuf.Nib >0){

    //   printf("\n");
    //   for(i=1;i<s2lbuf.Nib;i++)
    //    printf("%x ",s2lbuf.ib[i]);
   //  printf("\n"); 
        stream2line(&s2lbuf);
      }	
      while (s2lbuf.nlines > 0 ){
        int i;
        getnextline(svr_buf,&s2lbuf);
	
      //  for(i=0;i<n;i++)
      //    printf("Serverbuf %s slut \n",svr_buf);
      //  printf("\n");
        if (strncmp(svr_buf,"exit",4)==0){
	   int n;
           sm->state=s_done;
	   n=sprintf(ansbuf,"ok\n");
	   send(serv.s,ansbuf,n,0);
        }else if (strncmp(svr_buf,"logoff",6)==0){
	   int n;
	   n=sprintf(ansbuf,"ok\n");
	   send(serv.s,ansbuf,n,0);
	  sm->state=s_waitforconnection;
	}	
	else if (strlen(svr_buf)==0){
	}
	else if (strncmp(svr_buf,"beginplan",9)==0){
	  int i=0,n;
	  while (plantable.plans[i].used!=0 && i< plantable.size)
	    i++;
	  if ( i <plantable.size){
	     curplanno=i;
	     strcpy(plantable.plans[curplanno].plan.name,&svr_buf[9]);
	     i=strlen(plantable.plans[curplanno].plan.name);
	     plantable.plans[curplanno].plan.start=plantable.plans[curplanno].plan.name+1+i;
	     plantable.plans[curplanno].plan.end= plantable.plans[curplanno].plan.start;
	     plantable.plans[curplanno].used=1; 
	     plantable.plans[curplanno].plan.nlabels=0;
             printf("beginplan %s\n",plantable.plans[curplanno].plan.name);
	  }
	  n=sprintf(ansbuf,"ok\n");
	  send(serv.s,ansbuf,n,0);
        }
	else if (strncmp(svr_buf,"deleteplan",10)==0){
	  int i,found,n;
	  i=0;
	  found=0;
	  while(i < plantable.size && !found){
	    if (plantable.plans[i].used !=0 && strncmp(&svr_buf[10],plantable.plans[i].plan.name,255)==0){
		found=1;
	        plantable.plans[i].used =0;
	    }
	    i++;
	  }
	  n=sprintf(ansbuf,"ok\n");
	  send(serv.s,ansbuf,n,0);
	}
	else if (strncmp(svr_buf,"endplan",7)==0){
	   char *bp;  
	   int n;
	   plantype *p=&plantable.plans[curplanno].plan;
	   bp=p->start;
	   printf("\n");
	   while (bp < p->end){
	     printf("%c",*bp);
	     bp++;
	   }
          printf("endplan %s\n",&svr_buf[7]);
	  printf("start %x end %x  label %x nlabels %d\n",p->start,p->end,p->labels[0],p->nlabels);
	  n=sprintf(ansbuf,"ok\n");
	  send(serv.s,ansbuf,n,0);
	  
        }
	else if (strncmp(svr_buf,"plan",4)==0){
	  char *bp;  
	  plantype *p=&plantable.plans[curplanno].plan;
	  int i,n;
          bp=&svr_buf[4];
          while (*bp==' ' || *bp=='\t')bp++;
          if (strncmp(bp,"label",5)==0) {
            p->nlabels++;
	    p->labels[p->nlabels-1]=p->end;
          }
          strcpy(p->end,bp);
          p->end+=strlen(bp)+1;
          printf("plan %s\n",&svr_buf[4]);
	  n=sprintf(ansbuf,"ok\n");
	  send(serv.s,ansbuf,n,0);
        }
        else if (strncmp(svr_buf,"getevent",8)==0){
           if (nevent >0) {
             getevent(ansbuf);
             send(serv.s,ansbuf,strlen(ansbuf),0);
           }
           else {
             eventtimeout=atof(svr_buf+8)*100;
             waitingforevent=1;
           }
        }
        else if (strncmp(svr_buf,"eval",4)==0){
          int res,n1; 
          //int start_text = 4;
          int j_text = 6;
          int i_text;
          char text[30];
          char * ansbufptr;   
          saveinterpcontext();	
          newcmd();
         
          bp=svr_buf;
          res=yyparse();

          if (res== 2 ){
            n1=sprintf(ansbuf,"syntaxerror\n");
            ansbufptr=n1+ansbuf;
          }
          else {
            ansbufptr=ansbuf;
            if(xmlon) { // Added by JE  
              n1=sprintf(ansbufptr,"<eval "); // Added by JE
              ansbufptr+=n1; // Added by JE
            } // Added by JE
            for (i=0;i<n_ans;i++){
              if(xmlon) {// Added by JE
                i_text=0;
                while((svr_buf[j_text] != ';') && (j_text < strlen(svr_buf)))
                {
                  text[i_text++] = svr_buf[j_text++];
                }
                text[i_text++] = '\0';
                n1=sprintf(ansbufptr,"%s=\"%lf\" ", text, anstab[i]); // Added by JE
                j_text+=2;
              }
              else // Added by JE
                n1=sprintf(ansbufptr,"%lf ",anstab[i]);
              ansbufptr+=n1;
            }
            if(xmlon) { // Added by JE
              n1=sprintf(ansbufptr,"/>"); // Added by JE
              ansbufptr+=n1; // Added by JE
            }
            *ansbufptr='\n';
             ansbufptr++;
          }
          send(serv.s,ansbuf,(int)(ansbufptr-ansbuf),0);
	  loadinterpcontext(); 
        }
	else if (strncmp(svr_buf,"setvar",6)==0){
          int res,n1; 
          char * ansbufptr;   
          saveinterpcontext();	
          newcmd();
          bp=svr_buf;
	  bp[3]=' ';
	  bp[4]=' ';
	  bp[5]=' ';
          res=yyparse();
          if (res== 2 ){
            n1=sprintf(ansbuf,"syntaxerror\n");
            ansbufptr=n1+ansbuf;
	    
          }
	  loadinterpcontext(); 
        }	
	else if (strncmp(svr_buf,"laserscan",9)==0){
	  send(serv.s,laserscan,361*2,0);
	}
	else if (strncmp(svr_buf,"flushcmds",9)==0){
	   int n;
	   cmdquein=cmdqueout;
	   cmdqueuecount=0;
	   if (sm->sm1!=NULL)
	     free_sm_data(sm->sm1);
	   planrunning=0;
	   intp_switch.status=SW_ENDED;
	   if (xmlon)
	     n=sprintf(ansbuf,"<cmd id=\"%d\" status=\"flushed\" />\n",curcmdid);
	   else
	     n=sprintf(ansbuf,"ID%d flushed\n",curcmdid);
	   send(serv.s,ansbuf,n,0);
	}
        else {
	  int n1;
          strcpy(cmdqueue[cmdquein].line,svr_buf);
	  cmdid++;
	  cmdqueue[cmdquein].id=cmdid;
	  if (xmlon)
	    n1=sprintf(ansbuf,"<cmd id=\"%d\" status=\"queued\" />\n",cmdid);
	  else
	    n1=sprintf(ansbuf,"ID%d queued\n",cmdid);
	  send(serv.s,ansbuf,n1,0);
          cmdquein++;
          if (cmdquein >= CMDQUESIZE) cmdquein=0;
          cmdqueuecount++;
        }
      }//end new command
      {
        int n;
	do {
        if (!planrunning && cmdqueuecount){
           
	    strcpy(plan_buf,cmdqueue[cmdqueout].line); 
	    curcmdid=cmdqueue[cmdqueout].id;
            cmdqueout++;
            if (cmdqueout >=CMDQUESIZE) cmdqueout=0;
 	    cmdqueuecount--;
	    if (strncmp(plan_buf,"runplan",7)==0){ 
	      int i,found,j;
	      plantype *p;
	      i=0;
	      found=0;
	      while(i < plantable.size && !found){
	         if (plantable.plans[i].used !=0 && strncmp(&plan_buf[7],plantable.plans[i].plan.name,256)==0){
		    found=1;
	             p=&plantable.plans[i].plan;
	             plan.name=p->name;
	             plan.start=p->start;
	             plan.end=p->end;
	             plan.pline=plan.start;
	             plan.nlabels=p->nlabels;
	             for (j=0;j<p->nlabels;j++){
	        	plan.labels[j]=p->labels[j];
	      	     }
		 }
		 i++;
	       }
	       if (!found){
	         strcpy(plan_buf,"  ");
	         plan.start=plan_buf;
                 plan.end=plan_buf+strlen(plan_buf)+1;
                 plan.pline=plan.start;
	      }
            }
	    else {
              plan.start=plan_buf;
              plan.end=plan_buf+strlen(plan_buf)+1;
              plan.pline=plan.start;
	    }
            setcurrentplan(&plan);
            sm->sm1=get_sm_data();
            sm_reset(sm->sm1);
            if (sm_interp(sm->sm1)==SM_FINISHED){
	    if (xmlon){
              if (sm->sm1->p[0]==-1)
                n=sprintf(ansbuf,"<cmd id=\"%d\"  status=\"syntaxerror\" />\n",curcmdid);
              else if (sm->sm1->p[0]==-2)
                 n=sprintf(ansbuf,"<cmd id=\"%d\" status=\"assignment\" />\n",curcmdid);
              else
                n=sprintf(ansbuf,"<cmd id=\"%d\" status=\"finished\" stopcond=\"%lf\" />\n",curcmdid,sm->sm1->p[0]);
	    }
	    else {
	     if (sm->sm1->p[0]==-1)
                n=sprintf(ansbuf,"ID%d syntaxerror\n",curcmdid);
              else if (sm->sm1->p[0]==-2)
                 n=sprintf(ansbuf,"ID%d assignment\n",curcmdid);
              else
                n=sprintf(ansbuf,"ID%d stopcond %lf\n",curcmdid,sm->sm1->p[0]);	
	    }	
              planrunning=0;
	      free_sm_data(sm->sm1);
            }
            else {
              planrunning=1;
	      if (xmlon)
                n=sprintf(ansbuf,"<cmd id=\"%d\" status=\"started\" />\n",curcmdid);
	      else
		n=sprintf(ansbuf,"ID%d started\n",curcmdid);
		
            } 
            putevent(ansbuf);
           
        }
        else if (planrunning){
          int n;
          if (sm_interp(sm->sm1)==SM_FINISHED){
            if (sm->sm1->p[0] != -2){
	      if (xmlon)
	        n=sprintf(ansbuf,"<cmd id=\"%d\" status=\"finished\" stopcond=\"%lf\" />\n",curcmdid,sm->sm1->p[0]);
              else
	        n=sprintf(ansbuf,"ID%d stopcond %lf\n",curcmdid,sm->sm1->p[0]);
              putevent(ansbuf);
            }
            free_sm_data(sm->sm1);
            planrunning=0;
          }
        } 
      }while (!planrunning && cmdqueuecount);	
	      
      }
    }
    break;

    SM_CASE(s_waitforconnection,
    
      // Close logfile
      {
   	int j;
	int i;
  	for (i=0;i<=(logptr-1);i++){
    	  for(j=0;j<logvar.n;j++)
      	    fprintf(ftemp,"%lf ",logtable[j][i]);
    	  fprintf(ftemp,"\n");
 	}
      }
      fclose(ftemp);
    
    
      /* Waits on a connection from a client and returns with
      * the address on the client process ("peer") and a new
      * socket decriptor to the new connection  */
       addr_len = sizeof(struct sockaddr_in);
       serv.s = accept(serv.ls, (struct sockaddr*) &serv.from, &addr_len);
       if ( serv.s == -1 ) return 4;

       if (fcntl(serv.s,F_SETFL,O_NONBLOCK) == -1) {
	 fprintf(stderr,"startserver: Unable to set flag O_NONBLOCK on serv.s\n");
//	 fprintf(stderr,"Error: %d (%s)\n",errno,strerror(errno)); 
       }
         cmdid=0;
         sm->state=s_getcmd;
	 ftemp=fopen("log","w");
	 logvar.n=0;
	 logptr=0;
    ,
    
    )
    

    SM_CASE(s_done,
            close(serv.s);
            close(serv.ls);
            sm->status=SM_FINISHED;motcon.cmd=MC_STOPCMD;planrunning=0;,)
  } 
  if (!planrunning)
    update_internvar(&internvar,sm);
  if (waitingforevent){
    if (nevent >0) {
      getevent(ansbuf);
      send(serv.s,ansbuf,strlen(ansbuf),0);
      waitingforevent=0;
    }
    else{
      if (eventtimeout-- < 1){
        int msglen;
        msglen=sprintf(ansbuf,"eventtimeout\n");
        send(serv.s,ansbuf,strlen(ansbuf),0);
        waitingforevent=0;
      }
    } 
  }         
  
  return(sm->status);
} 

int putevent(char * event){
if (xmlon) {
   send(serv.s,event,strlen(event),0);
   return 0;
}
else {

if (nevent <100){
  strcpy(eventtab[eventin],event);
  eventin++;
  nevent++;
  if (eventin == 100) eventin=0;
  return(0);
}
else
  return 1;
}
}
int getevent(char * event){
if (nevent >0){
  strcpy(event,eventtab[eventout]);
  eventout++;
  nevent--;
  if (eventout == 100) eventout=0;
  return(0);
}
else
  return 1;
}


//*******************************************************************  
// sm_resetmotors
//
//

int sm_resetmotors(sm_type *sm){
  enum{s_start,s_done};
  sm_update(sm);
  switch (sm->state) {
     
   SM_CASE(s_start,
      printf("reset state1\n");
      sm->status=SM_RUNNING;
      sm->p[0]=odo.pose.x;
      sm->p[1]=odo.pose.y;
      sm->p[2]=odo.pose.th;
      motcon.cmd=MC_IDLECMD;
      reset_motor(1);
      reset_motor(2);
      ,
      NEXTSTATE(sm->time > 4 ,s_done))
  
    SM_CASE(s_done,
     printf("reset state2 \n");
      odo.pose.x=sm->p[0];            
      odo.pose.y=sm->p[1];
      odo.pose.th=sm->p[2];   
      sm->status=SM_FINISHED;,)
  }        
  return(sm->status);
} 


//*******************************************************************  
// sm_barcode
//
//

int sm_barcode(sm_type *sm){
  enum{s_start,s_done};
  sm_update(sm);
  switch (sm->state) {
     
   SM_CASE(s_start,
      printf("barcode state1\n");
      sm->status=SM_RUNNING;
      sm->p[9]=0;
//      motcon.cmd=MC_FOLLOW_LINECMD;
//      linetype=LINE_MIDDLE_W;
//      motcon.velcmd=0.1;
      ,
      {
	int i;
        double max=0;       
	for (i=0;i<8;i++){
          if (ls_corrected[i]> max) 
	    max=ls_corrected[i];
        }
	tmpbuf[(int)sm->p[9]][0]=max;
     
        sm->p[9]+=1;
        NEXTSTATE(statedist(sm) > sm->p[0] ,s_done)})
    
     case s_done:
    {
      int i,state,locount,hicount,barcount=0;
      double f[10000],thresh=0.4;
      FILE *fp;
      enum{hi,lo};
      fp=fopen("barcode","w");
      f[0]=0.6;
      for (i=0;i< (int)(sm->p[9]);i++)
	f[i+1]=f[i]*0.9+0.1*tmpbuf[i][0];
      state=hi;
      locount=0;
      for (i=0;i< (int)(sm->p[9]);i++){
        switch (state) {
        case hi:{
          if (f[i]<thresh)
            locount++;
          else
            locount=0;
          if (locount > 10){
            state=lo;
            barcount++;
            hicount=0;
            printf("shift to lo \n");
          }
        }
        break;
        case lo:{
          if (f[i]>thresh) 
            hicount++;
          else
            hicount=0;
          if (hicount > 10){
            state=hi;
            locount=0;
            printf("shift to hi \n");
          }
        }
        break;
        default:
	  ;
	}
      }
      for (i=0;i<(int)sm->p[9];i++)
        fprintf(fp,"%f %f \n",tmpbuf[i][1],f[i]);
      fclose(fp);
      printf("barcode count %d \n",barcount);
      intpstatereturn=barcount;
      sm->status=SM_FINISHED;
      //      motcon.cmd=MC_STOPCMD;
    }
    break;
  }        
  return(sm->status);
} 
  


