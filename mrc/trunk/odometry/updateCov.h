// By Enis BAYRAMOGLU, enisbayramoglu@gmail.com, 09.06.2008

#include <math.h>

typedef struct updatestruct{

	int lTick;	//Left wheel encoder ticks
	int rTick;	//Right wheel encoder ticks
	
	double lK;	//Left wheel travelled distance per tick
	double rK;	//Right wheel travelled distance per tick
	double E2;	//The variance of the error introduced by K's only for a 1m translation 
	
	double B;	//The average wheel separation
	double A2;	//The variance of the error introduced by B only for a 2pi rotation
	
	double theta;	//The robot orientation change since last reset, it should start as 0
	
	// The relevant elements of the covariance matrix, 1: Theta, 2: x, 3: y
	double p11, p12, p13, p22, p23, p33;
};

void updatecov(updatestruct * us) {

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
