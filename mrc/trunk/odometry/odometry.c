#include <math.h>
#include "odometry.h"
#include "motioncontrol.h"
#define CNTWIDTH	30

#define CNTSIZE		(1 << CNTWIDTH)
#define CNTMAX		(1 << (CNTWIDTH-1))




void odo_init(odotype *p){
  p->pose.x=0;
  p->pose.y=0;
  p->pose.th=0;
  p->vel=0;
  p->velr=0;
  p->vell=0;
  p->dist=0;
  p->distleft=0;
  p->distright=0;
  p->renc_old=p->renc;
  p->lenc_old=p->lenc;
  p->dthold=p->dth;
  p->enc_error=0;
  p->dencr=0;
  p->dencl=0;
  p->time=-1;
  p->time_old=-1;
}


void odo_set(odotype *p,posetype startpose){
  p->pose=startpose;
}


void odo_update(odotype *p){
  int deltal,deltar;
  double dr,dl,dv,dt,dthlocal;
  if (p->enctype==0)
    deltar=p->renc - p->renc_old;
  else
    deltar=p->renc;
  if (deltar > CNTMAX ) deltar-=CNTSIZE;
  else
    if (deltar < -CNTMAX ) deltar+=CNTSIZE;
  p->dencr=deltar;  
  dr=deltar*p->cr;
  if (p->enctype==0)
    deltal=p->lenc - p->lenc_old;
  else
    deltal=p->lenc;
  if (deltal > CNTMAX ) deltal-=CNTSIZE;
  else
    if (deltal < -CNTMAX ) deltal+=CNTSIZE;
  p->dencl=deltal;
  dl=deltal* p->cl;
  p->renc_old=p->renc;
  p->lenc_old=p->lenc;
   if(fabs(deltar) < p->maxtick && fabs(deltal) < p->maxtick ){  
    dv=(dr+dl)/2;
    p->distleft+=dl;
    p->distright+=dr;

    switch (p->control ){
    
    case 0:{
      if (p->type== ACKERMAN) {
        if (p->encwheel==0){
            dthlocal= dv*tan(p->steeringangle-p->steeringangleoffset)/p->robotlength;   
        }
	else {     
            dthlocal= dv*sin(p->steeringangle-p->steeringangleoffset)/p->robotlength;
        }
      } 
      else {
        dthlocal=(dr-dl)/p->w;
      }
    }
    break;
    case 1:{
      if (p->gyrotype==0){
        dthlocal=(p->dth-p->dthold);
	p->dthold=p->dth;
	normalizeangle(&dthlocal);
      }
      else{	
        dthlocal=p->dth*p->ts;
	if (deltar ==0 && deltal== 0)
           p->dth=0;
      }  
    }
    break;
    case 2:{
      dthlocal= dv*p->curvature;   
    }
    break;
    default : {
    ;
    }
    }
    if (p->encwheel==1){
       dv=dv*cos(p->steeringangle-p->steeringangleoffset);
    }
    p->dist+=fabs(dv);
    p->dthout=dthlocal;
    p->dv=dv;
    p->pose.x+=dv*cos(p->pose.th+dthlocal*0.5);
    p->pose.y+=dv*sin(p->pose.th+dthlocal*0.5);
    p->pose.th+=dthlocal;
    if (p->pose.th < -M_PI) p->pose.th+=2*M_PI;
    if (p->pose.th > M_PI) p->pose.th-=2*M_PI;
    dt=p->ts;
    if (p->time > 0 && p->time_old>0){
       dt=p->time - p->time_old;
       if (dt < p->ts/10) dt=p->ts;
    }
    p->time_old=p->time;
    p->vel=dv/dt;
    p->vell=dl/dt;
    p->velr= dr/dt;
 }
  else {  p->enc_error++; }
}


void UMB_updatexy(double *wheelbase,double *diameter_ratio,double xcw,double ycw,double xccw,double yccw,double L){
  double e1,e2;
  e2=(ycw-yccw+xcw+xccw)/2;
 (*wheelbase)*=1/(1+e2/(2*L*M_PI));
  e1=(ycw+yccw+xcw-xccw)/2;
  (*diameter_ratio)*=(8*L*L-*wheelbase*e1)/(8*L*L+*wheelbase*e1);
}

void UMB_updatey(double *wheelbase,double *diameter_ratio,double ycw,double yccw,double L){
  double e1,e2;
  e2=(ycw-yccw);
 (*wheelbase)*=1/(1+e2/(2*L*M_PI));
  e1=(ycw+yccw);
  (*diameter_ratio)*=(8*L*L-*wheelbase*e1)/(8*L*L+*wheelbase*e1);
}

// By Enis BAYRAMOGLU, enisbayramoglu@gmail.com, 09.06.2008
// Adopted in odometry by N.A. Andersen

void updatecov(covtype * us) {

	double B;

	double lL = us->lTick*us->lK; 	// Distance travelled by the left wheel
	double rL = us->rTick*us->rK; 	// Distance travelled by the right wheel

	double L=(rL+lL)/2;		// Forward distance travelled
	double D=(rL-lL)/(B=us->B);	// Angular change

	double theta=us->theta+D/2;
	us->theta+=D;
		
	double sigmab2,B2=B*B;
	if (D!=0)
		sigmab2= us->A2*B2/(2*M_PI*fabs(D));
	else
		sigmab2=0;
	
	double E2;
		
	double sigmar2= (E2=us->E2)*fabs(rL);
	double sigmal2= E2*fabs(lL);
	
	double p11=us->p11, p12=us->p12, p13=us->p13, p22=us->p22, p23=us->p23, p33=us->p33;

	// Repetitive calculations - "_" means division, "m" minus, "p" plus, "xy" x*y, "x2" x²
	double c=cos(theta), s=sin(theta);
	double c2=c*c, s2=s*s;
	double L2=L*L;
	double D2=D*D;
	double Ls=L*s, Lc=L*c;
	double Ls_B=Ls/B, Lc_B=Lc/B;
	double Ls_B2=Ls/B2, Lc_B2=Lc/B2;
	double c_2 = c/2, s_2 = s/2;
	double c_2B = c_2/B, s_2B= s_2/B;
	double c_2mLs_B = c_2-Ls_B, c_2pLs_B=c_2+Ls_B,s_2mLc_B = s_2-Lc_B, s_2pLc_B=s_2+Lc_B;
	double D2Ls_2B2=D2*Ls_B2/2, D2Lc_2B2=D2*Lc_B2/2;
	
	// These values are the new covariance values before adding the error due to B and K's
	double p_33=p33+2*Lc*p13+L2*c2*p11;
	double p_23=p23+Lc*p12-Ls*p13-Ls*Lc*p11;
	double p_22=p22-2*Ls*p12+L2*s2*p11;
	double p_13=p13+Lc*p11;
	double p_12=p12-Ls*p11;
	// double p_11=p11; No need to make this calculation
	
	// Final covariance values	
	us->p11+=(sigmar2+sigmal2+D2*sigmab2)/B2;
	us->p12=p_12+(c_2B-Ls_B2)*sigmar2-(c_2B+Ls_B2)*sigmal2-D2Ls_2B2*sigmab2;
	us->p13=p_13+(s_2B+Lc_B2)*sigmar2-(s_2B-Lc_B2)*sigmal2+D2Lc_2B2*sigmab2;
	us->p22=p_22+c_2mLs_B*c_2mLs_B*sigmar2+c_2pLs_B*c_2pLs_B*sigmal2+D2Ls_2B2*Ls/2*sigmab2;
	us->p23=p_23+c_2mLs_B*s_2pLc_B*sigmar2+c_2pLs_B*s_2mLc_B*sigmal2-D2Ls_2B2*Lc/2*sigmab2;
	us->p33=p_33+s_2pLc_B*s_2pLc_B*sigmar2+s_2mLc_B*s_2mLc_B*sigmal2+D2Lc_2B2*Lc/2*sigmab2;
}

void resetupdatestruct(covtype * us) {
	us->theta=0;
	us->p11=0;
	us->p12=0;
	us->p13=0;
	us->p22=0;
	us->p23=0;
	us->p33=0;
}

