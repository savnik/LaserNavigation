#ifndef LOCALIZATIONUTILS_H_
#define LOCALIZATIONUTILS_H_

#include <ugen4/utime.h>
#include <urob4/uresposehist.h>
#include <umap4/upose.h>
#include <eigen3/Eigen/Dense>
#include <mhf/MultiHypDist.h>
using namespace Eigen;

void updateDisplacement(UTime &lastScanTime, MultiHypDist<3> &poseDist, UTime scanTime, UResPoseHist  *poseHist, int silent, double odoB, double KR, double KL);
void updateDisplacement(UTime &lastScanTime, Matrix<double,3,1> &pose, Matrix<double,3,3> &poseCov, UTime scanTime, UResPoseHist  *poseHist, int silent, double odoB, double KR, double KL);

#endif /*LOCALIZATIONUTILS_H_*/
