#include "localizationutils.h"
#include <mhf/MultiHypDist.hpp>
#include <boost/foreach.hpp>
using std::cout;

UPose interpolatePoses(UPoseTime uPoseTime1, UPoseTime uPoseTime2, UTime timeOfPose);

void updateDisplacement(UTime &timeOfPose, MultiHypDist<3> &poseDist, UTime scanTime, UResPoseHist  *poseHist, int silent, double odoB, double KR, double KL) {
	UPoseTVQ uPoseTime1;
	UPoseTVQ uPoseTime2;
	poseHist->getPoseNearTime(timeOfPose,&uPoseTime1,&uPoseTime2);
	if(silent==0)
		printf("Last pose Time:%f\n", timeOfPose.getDecSec());
	if(silent<2)
		printf("Scan Time:%f\n", scanTime.getDecSec());
	if(silent==0) {
		printf("Pose Time1:%f\n",uPoseTime1.t.getDecSec());
		printf("Pose Time2:%f\n",uPoseTime2.t.getDecSec());
	}

	UPose P1;
	if(uPoseTime1.t<timeOfPose && uPoseTime2.t>timeOfPose) {
		P1 = interpolatePoses(uPoseTime1, uPoseTime2, timeOfPose);
	} else {
		P1 = uPoseTime1.getPose();
		timeOfPose = uPoseTime1.t;
	}


	while (1) {
		if(timeOfPose == scanTime)
			break;
		if (poseHist->getNewest(0).t==timeOfPose) {
			if (silent == 0)
				cout << "\tpose list end\n";
			break;
		}
		if (not poseHist->getPoseNearTime(timeOfPose+0.001,&uPoseTime1,&uPoseTime2))
			break;
			UPose P2;
			//cout << "\tPose Time Diff:\n";// << uPoseTime2.t.getDecSec()- scanTime << "\n";
		if (uPoseTime2.t > (scanTime + 0.001)) {
			P2 = interpolatePoses(uPoseTime1, uPoseTime2, scanTime);
			timeOfPose = scanTime;
			if (silent == 0)
				cout << "\tTime at scan reached\n";
		} else {
			P2 = uPoseTime2.getPose();
			timeOfPose = uPoseTime2.t;
		}
		BOOST_FOREACH(GaussianHypothesis<3> &gh, poseDist.GHlist) {
			Matrix<double,3,1> & pose = gh.mean;
			Matrix<double,3,3> & poseCov = gh.cov;
			double D, L, delSr, delSl;
			D = fmod(P2.h - P1.h + 3 * M_PI, 2 * M_PI) - M_PI;
			double xDif = (P2.x - P1.x);
			double yDif = (P2.y - P1.y);
			L = sqrt( xDif * xDif +
					yDif * yDif)*((cos(P1.h)*xDif+sin(P1.h)*yDif>0)*2-1);
			//cout << "\tD:" << D << " L:" << L << "\n";

			delSr = L + odoB*D/2;
			delSl = L - odoB*D/2;

			double th_ = pose(2,0)+D/2;

			double thd = P2.h-P1.h;
			double xd = cos(P1.h)*(P2.x-P1.x)+sin(P1.h)*(P2.y-P1.y);
			double yd = -sin(P1.h)*(P2.x-P1.x)+cos(P1.h)*(P2.y-P1.y);
			double thl = pose(2,0);

			Matrix<double,3,1> poseDif;
			poseDif << cos(thl)*xd-sin(thl)*yd,sin(thl)*xd+cos(thl)*yd,thd;
			pose+=poseDif;

			Matrix<double,3,2> delPnew_delY;
			delPnew_delY<<cos(th_)/2-L*sin(th_)/(2*odoB),    cos(th_)/2+L*sin(th_)/(2*odoB),
					sin(th_)/2+L*cos(th_)/(2*odoB),    sin(th_)/2-L*cos(th_)/(2*odoB),
					1/odoB,                        	-1/odoB;

			Matrix<double,3,3> delPnew_delPold;
			delPnew_delPold<<  1,    0,      -L*sin(th_),
					0,    1,      L*cos(th_),
					0,    0,      1;

			Matrix<double,2,2> covU;

			covU<< sqr(KR) * fabs(delSr),    0,
					0                                    ,    sqr(KL) * fabs(delSl);

			//printf("Robot %gbase l=%g R=%g\b", odoB, varKL->getValued(), varKR->getValued());

			//covOut = delPnew_delPold*covIn*delPnew_delPold'+delPnew_delY*covU*delPnew_delY';*/

			poseCov << delPnew_delY *covU * delPnew_delY.transpose()
							  +delPnew_delPold * poseCov * delPnew_delPold.transpose();
		}
		P1 = P2;
		if(silent==0)
			cout<<"-*";
	}

	if (silent < 2)
		cout << "\tupdated displacement\n";
}


void updateDisplacement(UTime &timeOfPose, Matrix<double,3,1> &pose, Matrix<double,3,3> &poseCov, UTime scanTime, UResPoseHist  *poseHist, int silent, double odoB, double KR, double KL) {
  UPoseTVQ uPoseTime1;
  UPoseTVQ uPoseTime2;
  poseHist->getPoseNearTime(timeOfPose,&uPoseTime1,&uPoseTime2);
// 	if(silent==0)
// 		printf("Last Image Time:%f\n", timeOfPose.getDecSec());
// 	if(silent<2)
// 		printf("Image Time:%f\n", scanTime.getDecSec());
  if(not silent) {
    printf("Pose Time %.3f to %.3f %.3f secs\n",timeOfPose.getDecSec(), scanTime.getDecSec(), scanTime - timeOfPose);
  }

  UPose P1;
  if(uPoseTime1.t <= timeOfPose && uPoseTime2.t >= timeOfPose) {
    P1 = interpolatePoses(uPoseTime1, uPoseTime2, timeOfPose);
  } else {
    P1 = uPoseTime1.getPose();
    timeOfPose = uPoseTime1.t;
  }
 

  while (1) {
    if(timeOfPose == scanTime)
      break;
    if (poseHist->getNewest(0).t==timeOfPose) {
      if (not silent)
        cout << "\tpose list end\n";
      break;
    }
    if (not poseHist->getPoseNearTime(timeOfPose+0.001,&uPoseTime1,&uPoseTime2))
      break;
    UPose P2;
      //cout << "\tPose Time Diff:\n";// << uPoseTime2.t.getDecSec()- scanTime << "\n";
    if (uPoseTime2.t > (scanTime + 0.001)) {
      P2 = interpolatePoses(uPoseTime1, uPoseTime2, scanTime);
      timeOfPose = scanTime;
// 			if (not silent)
// 				cout << "\tTime at scan reached\n";
    } else {
      P2 = uPoseTime2.getPose();
      timeOfPose = uPoseTime2.t;
    }
    double D, L, delSr, delSl;
    D = fmod(P2.h - P1.h + 3 * M_PI, 2 * M_PI) - M_PI;
    double xDif = (P2.x - P1.x);
    double yDif = (P2.y - P1.y);
    // signed distance
    L = sqrt( xDif * xDif +
              yDif * yDif) * ((cos(P1.h) * xDif + sin(P1.h) * yDif) * 2 - 1);
    // this looks like an error - replaced with the one above / chr 8/9/2013
//     L = sqrt( xDif * xDif +
//               yDif * yDif) * ((cos(P1.h) * xDif + sin(P1.h) * yDif>0) * 2 - 1);
    //cout << "\tD:" << D << " L:" << L << "\n";

    delSr = L + odoB * D / 2;
    delSl = L - odoB * D / 2;

    double th_ = pose(2,0) + D / 2;

    double thd = P2.h-P1.h;
    double xd =  cos(P1.h) * (P2.x-P1.x) + sin(P1.h) * (P2.y-P1.y);
    double yd = -sin(P1.h) * (P2.x-P1.x) + cos(P1.h) * (P2.y-P1.y);
    double thl = pose(2,0);
    // save old pose
    Matrix<double,3,1> poseOld = pose;;
    // update to new pose, xd forward and yd left of current pose
    Matrix<double,3,1> poseDif;
    poseDif << cos(thl)*xd - sin(thl)*yd,
               sin(thl)*xd + cos(thl)*yd,
               thd;
    pose += poseDif;
    pose(2,0) = limitToPi(pose(2,0));
    // calculate new covariance
    Matrix<double,3,2> delPnew_delY;
    delPnew_delY << cos(th_)/2-L*sin(th_)/(2*odoB),    cos(th_)/2+L*sin(th_)/(2*odoB),
                    sin(th_)/2+L*cos(th_)/(2*odoB),    sin(th_)/2-L*cos(th_)/(2*odoB),
                    1/odoB,                            -1/odoB;

    Matrix<double,3,3> delPnew_delPold;
    delPnew_delPold<<  1,    0,      -L*sin(th_),
                      0,    1,      L*cos(th_),
                      0,    0,      1;

    Matrix<double,2,2> covU;

    covU << sqr(KR) * fabs(delSr),    0,
            0                   ,    sqr(KL) * fabs(delSl);

    //printf("Robot %gbase l=%g R=%g\b", odoB, varKL->getValued(), varKR->getValued());

    //covOut = delPnew_delPold*covIn*delPnew_delPold'+delPnew_delY*covU*delPnew_delY';*/

    poseCov << delPnew_delY *covU * delPnew_delY.transpose()
            +delPnew_delPold * poseCov * delPnew_delPold.transpose();

    if (not silent)
      printf(" - position update to %.3f: %.3fm %.5frad odo (%.3fx %.3fy %.4fr->%.3fx %.3fy %.4fr moved %.3fm dir %.5fh) "
                "map (%.3fx %.3fy %.4fr->%.3fx %.3fy %.4fr moved %.3fm dir %.5fh)\n",
                 timeOfPose.getDecSec(),
                 L, thd, P1.x, P1.y, P1.h, P2.x, P2.y, P2.h, hypot(P2.y - P1.y, P2.x - P1.x), atan2(P2.y - P1.y, P2.x - P1.x),
                 poseOld(0,0), poseOld(1,0), poseOld(2,0), pose(0,0), pose(1,0), pose(2,0),
                 hypot(pose(1,0) - poseOld(1,0), pose(0,0) - poseOld(0,0)),
                 atan2(pose(1,0) - poseOld(1,0), pose(0,0) - poseOld(0,0))
            );
    P1 = P2;
  }

  if (not silent)
    cout << "updated displacement\n";
}

UPose interpolatePoses(UPoseTime uPoseTime1, UPoseTime uPoseTime2, UTime timeOfPose) {
    double distFromBeginning = (timeOfPose-uPoseTime1.t) / (uPoseTime2.t-uPoseTime1.t);
    UPose P;
    UPose Pb = uPoseTime1.getPose();
    UPose Pe = uPoseTime2.getPose();
    if (fabs(Pb.h - Pe.h) > M_PI/2.0)
    {
      if (Pb.h > Pe.h)
        Pe.h += 2.0 * M_PI;
      else
        Pb.h += 2.0 * M_PI;
    }
    P.x = Pb.x*(1-distFromBeginning) + Pe.x*(distFromBeginning);
    P.y = Pb.y*(1-distFromBeginning) + Pe.y*(distFromBeginning);
    P.h = Pb.h*(1-distFromBeginning) + Pe.h*(distFromBeginning);
    P.h = limitToPi(P.h);
    return P;
}
