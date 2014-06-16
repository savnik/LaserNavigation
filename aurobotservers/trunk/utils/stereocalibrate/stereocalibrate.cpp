/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warrenty, support or any guarentee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008

   AVAILABLE AT:
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130

   OTHER OPENCV SITES:
   * The source code is on sourceforge at:
     http://sourceforge.net/projects/opencvlibrary/
   * The OpenCV wiki page (As of Oct 1, 2008 this is down for changing over servers, but should come back):
     http://opencvlibrary.sourceforge.net/
   * An active user group is at:
     http://tech.groups.yahoo.com/group/OpenCV/
   * The minutes of weekly OpenCV development meetings are at:
     http://pr.willowgarage.com/wiki/OpenCV
   ************************************************** */

#ifdef OPENCV2
#include <core/core_c.h>
#include <imgproc/imgproc_c.h>
#include <calib3d/calib3d.hpp>
#include <highgui/highgui_c.h>
#else
#include <opencv/cv.h>
#include <opencv/cxmisc.h>
#include <opencv/highgui.h>
#endif
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>

using namespace std;

void print3dcloud(CvStereoBMState *BMState, int imgNum,  CvSize imageSize, CvMat * disp, CvArr * mx1, CvArr * my1, CvMat * matQ);
//
// Given a list of chessboard images, the number of corners (nx, ny)
// on the chessboards, and a flag: useCalibrated for calibrated (0) or
// uncalibrated (1: use cvStereoCalibrate(), 2: compute fundamental
// matrix separately) stereo. Calibrate the cameras and display the
// rectified results along with the computed disparity images.
//
static void
StereoCalib( const char* imageList, int nx, int ny, double squareSize, int useUncalibrated, int estimateK3)
{
    int displayCorners = 1;
    int showUndistorted = 1;
    bool isVerticalStereo = false;//OpenCV can handle left-right
                                      //or up-down camera arrangements
    const int maxScale = 1;
//    const float squareSize = 0.0705f; //Set this to your actual square size

    printf("%dx x %dy squares each %.4fm in size (uncalibrated=%d)\n", nx, ny, squareSize, useUncalibrated);

    FILE* f = fopen(imageList, "rt");

    int i, j, lr, nframes, n = nx*ny, N = 0;
    vector<string> imageNames[2];
    vector<CvPoint3D32f> objectPoints;
    vector<CvPoint2D32f> points[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<CvPoint2D32f> temp(n);
    CvSize imageSize = {0,0};
    // ARRAY AND VECTOR STORAGE:
    double M1[3][3], M2[3][3], D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3];
    double Q[4][4];
    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
    CvMat _D1;
    CvMat _D2;
    if (estimateK3)
    { // estimate also k3
      _D1 = cvMat(1, 5, CV_64F, D1 );
      _D2 = cvMat(1, 5, CV_64F, D2 );
    }
    else
    { // else estimate only k1 and k2
      _D1 = cvMat(1, 4, CV_64F, D1 );
      _D2 = cvMat(1, 4, CV_64F, D2 );
    }
    CvMat _R = cvMat(3, 3, CV_64F, R );
    CvMat _T = cvMat(3, 1, CV_64F, T );
    CvMat _E = cvMat(3, 3, CV_64F, E );
    CvMat _F = cvMat(3, 3, CV_64F, F );
    CvMat matQ = cvMat(4, 4, CV_64FC1, Q);
    const int MSL = 100;
    char fns[MSL];
    //
    if( displayCorners )
        cvNamedWindow( "corners", 1 );

    // READ IN THE LIST OF CHESSBOARDS:
    if( !f )
    {
        fprintf(stderr, "can not open file %s\n", imageList );
        return;
    }

    printf("Starting main loop\n");
    for(i=0;;i++)
    {
        //printf("Iteration %d\n", i);
        char buf[1024];
        int count = 0, result=0;
        lr = i % 2;
        vector<CvPoint2D32f>& pts = points[lr];
        if( !fgets( buf, sizeof(buf)-3, f ))
            break;
        size_t len = strlen(buf);
        while( len > 0 && isspace(buf[len-1]))
            buf[--len] = '\0';
        if( buf[0] == '#')
            continue;
        IplImage* img = cvLoadImage( buf, 0 );
        if( !img )
            break;
        imageSize = cvGetSize(img);
        imageNames[lr].push_back(buf);
    //FIND CHESSBOARDS AND CORNERS THEREIN:
        for( int s = 1; s <= maxScale; s++ )
        {
            IplImage* timg = img;
            if( s > 1 )
            {
                timg = cvCreateImage(cvSize(img->width*s,img->height*s),
                    img->depth, img->nChannels );
                cvResize( img, timg, CV_INTER_CUBIC );
            }
            result = cvFindChessboardCorners( timg, cvSize(nx, ny),
                &temp[0], &count,
                CV_CALIB_CB_ADAPTIVE_THRESH |
                CV_CALIB_CB_NORMALIZE_IMAGE);
            if( timg != img )
                cvReleaseImage( &timg );
            if( result || s == maxScale )
                for( j = 0; j < count; j++ )
            {
                temp[j].x /= s;
                temp[j].y /= s;
            }
            if( result )
                break;
        }
        if( displayCorners )
        {
            printf("found %d corners in %s\n", count, buf);
            IplImage* cimg = cvCreateImage( imageSize, 8, 3 );
            cvCvtColor( img, cimg, CV_GRAY2BGR );
            cvDrawChessboardCorners( cimg, cvSize(nx, ny), &temp[0],
                count, result );
            // save to file
            snprintf(fns, MSL, "corners-%s", buf);
            cvSaveImage(fns, cimg);
//            cvShowImage( "corners", cimg );
            cvReleaseImage( &cimg );
/*            int c = cvWaitKey(1000);
            if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                exit(-1);*/
        }
        else
            putchar('.');
        N = pts.size();
        pts.resize(N + n, cvPoint2D32f(0,0));
        active[lr].push_back((uchar)result);
    //assert( result != 0 );
        if( result )
        {
         //Calibration will suffer without subpixel interpolation
            cvFindCornerSubPix( img, &temp[0], count,
                cvSize(11, 11), cvSize(-1,-1),
                cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                30, 0.01) );
            copy( temp.begin(), temp.end(), pts.begin() + N );
        }
        cvReleaseImage( &img );
    }
    fclose(f);
    printf("\n");
// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
    nframes = active[0].size();//Number of good chessboads found
    objectPoints.resize(nframes*n);
    for( i = 0; i < ny; i++ )
        for( j = 0; j < nx; j++ )
        objectPoints[i*nx + j] =
        cvPoint3D32f(i*squareSize, j*squareSize, 0);
    for( i = 1; i < nframes; i++ )
        copy( objectPoints.begin(), objectPoints.begin() + n,
        objectPoints.begin() + i*n );
    npoints.resize(nframes,n);
    N = nframes*n;
    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
    cvSetIdentity(&_M1);
    cvSetIdentity(&_M2);
    cvZero(&_D1);
    cvZero(&_D2);

// CALIBRATE THE STEREO CAMERAS
    printf("Running stereo calibration ...");
    fflush(stdout);
    cvStereoCalibrate( &_objectPoints, &_imagePoints1,
        &_imagePoints2, &_npoints,
        &_M1, &_D1, &_M2, &_D2,
        imageSize, &_R, &_T, &_E, &_F,
        cvTermCriteria(CV_TERMCRIT_ITER+
        CV_TERMCRIT_EPS, 100, 1e-5),
        CV_CALIB_FIX_ASPECT_RATIO +
        CV_CALIB_ZERO_TANGENT_DIST +
        CV_CALIB_SAME_FOCAL_LENGTH );
    printf(" done\n");
    // make sure decimal point is '.'
    setlocale(LC_ALL, "C");

    // SAVE TO FILE CAMERA MATRIXES AND DISTORTION COEFFICIENTS --> FILES
    FILE *f_out = fopen("stereocalib.txt","wt");
    FILE *fini = fopen("stereocalib.ini","wt");
    fprintf( f_out, "image size height: %d, width: %d\n", imageSize.height, imageSize.width);
    fprintf( fini, "# image size height: %d, width: %d\n", imageSize.height, imageSize.width);
    fprintf( f_out, "\nleft  camera - intrinsic:\n");
    fprintf( fini, "#left  camera - intrinsic:\n");
    fprintf( f_out, " %7.2f %7.2f %7.2f;\n %7.2f %7.2f %7.2f;\n %7.2f %7.2f %7.2f\n",
            (double)cvGet2D(&_M1,0,0).val[0],(double)cvGet2D(&_M1,0,1).val[0],(double)cvGet2D(&_M1,0,2).val[0],
            (double)cvGet2D(&_M1,1,0).val[0],(double)cvGet2D(&_M1,1,1).val[0],(double)cvGet2D(&_M1,1,2).val[0],
            (double)cvGet2D(&_M1,2,0).val[0],(double)cvGet2D(&_M1,2,1).val[0],(double)cvGet2D(&_M1,2,2).val[0] );
    fprintf( fini, "var stereo.intrinsicLeft=\"%7.2f %7.2f %7.2f; %7.2f %7.2f %7.2f; %7.2f %7.2f %7.2f\"\n",
            (double)cvGet2D(&_M1,0,0).val[0],(double)cvGet2D(&_M1,0,1).val[0],(double)cvGet2D(&_M1,0,2).val[0],
            (double)cvGet2D(&_M1,1,0).val[0],(double)cvGet2D(&_M1,1,1).val[0],(double)cvGet2D(&_M1,1,2).val[0],
            (double)cvGet2D(&_M1,2,0).val[0],(double)cvGet2D(&_M1,2,1).val[0],(double)cvGet2D(&_M1,2,2).val[0] );

    double k3 = 0.0;
    if (estimateK3)
      k3 = (double)cvGet2D(&_D1,0,4).val[0];
    fprintf( f_out, "left  camera - distortion [k1, k2, p1, p2, k3]:\n");
    fprintf( fini, "# left  camera - distortion [k1, k2, p1, p2, k3]:\n");
    fprintf( f_out, "%8.6f %8.6f %g %g %8.6f \n",
            (double)cvGet2D(&_D1,0,0).val[0],(double)cvGet2D(&_D1,0,1).val[0],(double)cvGet2D(&_D1,0,2).val[0],
            (double)cvGet2D(&_D1,0,3).val[0],k3 );
    fprintf( fini, "var stereo.distortionLeft=\"%8.6f %8.6f %g %g %8.6f\"\n",
            (double)cvGet2D(&_D1,0,0).val[0],(double)cvGet2D(&_D1,0,1).val[0],(double)cvGet2D(&_D1,0,2).val[0],
            (double)cvGet2D(&_D1,0,3).val[0],k3 );

    fprintf( f_out, "\nright camera - intrinsic:\n");
    fprintf( fini, "#right camera - intrinsic:\n");
    fprintf( f_out, " %7.2f %7.2f %7.2f;\n %7.2f %7.2f %7.2f;\n %7.2f %7.2f %7.2f\n",
            (double)cvGet2D(&_M2,0,0).val[0],(double)cvGet2D(&_M2,0,1).val[0],(double)cvGet2D(&_M2,0,2).val[0],
            (double)cvGet2D(&_M2,1,0).val[0],(double)cvGet2D(&_M2,1,1).val[0],(double)cvGet2D(&_M2,1,2).val[0],
            (double)cvGet2D(&_M2,2,0).val[0],(double)cvGet2D(&_M2,2,1).val[0],(double)cvGet2D(&_M2,2,2).val[0] );
    fprintf( fini, "var stereo.intrinsicRight=\"%7.2f %7.2f %7.2f; %7.2f %7.2f %7.2f; %7.2f %7.2f %7.2f\"\n",
            (double)cvGet2D(&_M2,0,0).val[0],(double)cvGet2D(&_M2,0,1).val[0],(double)cvGet2D(&_M2,0,2).val[0],
            (double)cvGet2D(&_M2,1,0).val[0],(double)cvGet2D(&_M2,1,1).val[0],(double)cvGet2D(&_M2,1,2).val[0],
            (double)cvGet2D(&_M2,2,0).val[0],(double)cvGet2D(&_M2,2,1).val[0],(double)cvGet2D(&_M2,2,2).val[0] );
    if (estimateK3)
      k3 = (double)cvGet2D(&_D2,0,4).val[0];
    fprintf( f_out, "\nright camera - distortion [k1, k2, p1, p2, k3]:\n");
    fprintf( fini, "#right camera - distortion [k1, k2, p1, p2, k3]:\n");
    fprintf( f_out, "%8.6f %8.6f %g %g %8.6f \n",
            (double)cvGet2D(&_D2,0,0).val[0],(double)cvGet2D(&_D2,0,1).val[0],(double)cvGet2D(&_D2,0,2).val[0],
            (double)cvGet2D(&_D2,0,3).val[0],k3);
    fprintf( fini, "var stereo.distortionRight=\"%8.6f %8.6f %g %g %8.6f\"\n",
            (double)cvGet2D(&_D2,0,0).val[0],(double)cvGet2D(&_D2,0,1).val[0],(double)cvGet2D(&_D2,0,2).val[0],
            (double)cvGet2D(&_D2,0,3).val[0],k3);

    fprintf( f_out, "\nfundamental matrix\n");
    fprintf( fini, "#fundamental matrix\n");
    fprintf( f_out, " %e %e %e;\n %e %e %e;\n %e %e %e\n",
            (double)cvGet2D(&_F,0,0).val[0],(double)cvGet2D(&_F,0,1).val[0],(double)cvGet2D(&_F,0,2).val[0],
            (double)cvGet2D(&_F,1,0).val[0],(double)cvGet2D(&_F,1,1).val[0],(double)cvGet2D(&_F,1,2).val[0],
            (double)cvGet2D(&_F,2,0).val[0],(double)cvGet2D(&_F,2,1).val[0],(double)cvGet2D(&_F,2,2).val[0] );
    fprintf( fini, "var stereo.fundamental=\"%e %e %e; %e %e %e; %e %e %e\"\n",
            (double)cvGet2D(&_F,0,0).val[0],(double)cvGet2D(&_F,0,1).val[0],(double)cvGet2D(&_F,0,2).val[0],
            (double)cvGet2D(&_F,1,0).val[0],(double)cvGet2D(&_F,1,1).val[0],(double)cvGet2D(&_F,1,2).val[0],
            (double)cvGet2D(&_F,2,0).val[0],(double)cvGet2D(&_F,2,1).val[0],(double)cvGet2D(&_F,2,2).val[0] );

    fprintf( f_out, "\nessential matrix\n");
    fprintf( fini, "# essential matrix\n");
    fprintf( f_out, " %e %e %e;\n %e %e %e;\n %e %e %e\n",
            (double)cvGet2D(&_E,0,0).val[0],(double)cvGet2D(&_E,0,1).val[0],(double)cvGet2D(&_E,0,2).val[0],
            (double)cvGet2D(&_E,1,0).val[0],(double)cvGet2D(&_E,1,1).val[0],(double)cvGet2D(&_E,1,2).val[0],
            (double)cvGet2D(&_E,2,0).val[0],(double)cvGet2D(&_E,2,1).val[0],(double)cvGet2D(&_E,2,2).val[0] );
    fprintf( fini, "var stereo.essential=\"%e %e %e; %e %e %e; %e %e %e\"\n",
            (double)cvGet2D(&_E,0,0).val[0],(double)cvGet2D(&_E,0,1).val[0],(double)cvGet2D(&_E,0,2).val[0],
            (double)cvGet2D(&_E,1,0).val[0],(double)cvGet2D(&_E,1,1).val[0],(double)cvGet2D(&_E,1,2).val[0],
            (double)cvGet2D(&_E,2,0).val[0],(double)cvGet2D(&_E,2,1).val[0],(double)cvGet2D(&_E,2,2).val[0] );

    fprintf( f_out, "\nRotation matrix - left-to-right\n");
    fprintf( fini, "# Rotation matrix - left-to-right\n");
    fprintf( f_out, " %e %e %e;\n %e %e %e;\n %e %e %e\n",
            (double)cvGet2D(&_R,0,0).val[0],(double)cvGet2D(&_R,0,1).val[0],(double)cvGet2D(&_R,0,2).val[0],
            (double)cvGet2D(&_R,1,0).val[0],(double)cvGet2D(&_R,1,1).val[0],(double)cvGet2D(&_R,1,2).val[0],
            (double)cvGet2D(&_R,2,0).val[0],(double)cvGet2D(&_R,2,1).val[0],(double)cvGet2D(&_R,2,2).val[0] );
    fprintf( fini, "var stereo.LRrotation=\"%e %e %e; %e %e %e; %e %e %e\"\n",
            (double)cvGet2D(&_R,0,0).val[0],(double)cvGet2D(&_R,0,1).val[0],(double)cvGet2D(&_R,0,2).val[0],
            (double)cvGet2D(&_R,1,0).val[0],(double)cvGet2D(&_R,1,1).val[0],(double)cvGet2D(&_R,1,2).val[0],
            (double)cvGet2D(&_R,2,0).val[0],(double)cvGet2D(&_R,2,1).val[0],(double)cvGet2D(&_R,2,2).val[0] );

    fprintf( f_out, "\nTranslation vector - left-to-right\n");
    fprintf( fini, "# Translation vector - left-to-right\n");
    fprintf( f_out, "%e %e %e\n",
            (double)cvGet2D(&_T,0,0).val[0],(double)cvGet2D(&_T,1,0).val[0],(double)cvGet2D(&_T,2,0).val[0] );
    fprintf( fini, "var stereo.LRtranslate=\"%e %e %e\"\n",
            (double)cvGet2D(&_T,0,0).val[0],(double)cvGet2D(&_T,1,0).val[0],(double)cvGet2D(&_T,2,0).val[0] );

    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    vector<CvPoint3D32f> lines[2];
    points[0].resize(N);
    points[1].resize(N);
    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    lines[0].resize(N);
    lines[1].resize(N);
    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
    //Always work in undistorted space
    cvUndistortPoints( &_imagePoints1, &_imagePoints1,
        &_M1, &_D1, 0, &_M1 );
    cvUndistortPoints( &_imagePoints2, &_imagePoints2,
        &_M2, &_D2, 0, &_M2 );
    cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
    cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
    FILE * fo = stdout;
    double avgErr;
    for (int o=0; o< 2; o++)
    { // print corner errors to console and output file
      fprintf(fo, "\n");
      avgErr = 0;
      for( int f = 0; f < nframes; f++ )
      {
        fprintf(fo, "set %d left: %s, right: %s pixel errors: \n", f,
              imageNames[0][f].c_str(), imageNames[1][f].c_str());
        double imgErry[10] = {0,0,0,0,0,0,0,0,0,0};
        double imgErr = 0;
        for(int jy = 0; jy < ny; jy++ )
        {
          double imgErrx = 0.0;
          for(int jx = 0; jx < nx; jx++ )
          {
            i = f * n + jy * nx + jx;
            double errl = points[0][i].x*lines[1][i].x +
                points[0][i].y*lines[1][i].y + lines[1][i].z;
            double errr = points[1][i].x*lines[0][i].x +
                points[1][i].y*lines[0][i].y + lines[0][i].z;
            imgErrx += fabs(errl) + fabs(errr);
            fprintf(fo, "(%6.2f,%6.2f) ", errl, errr);
            imgErry[jx] += fabs(errl) + fabs(errr);
          }
          avgErr += imgErrx;
          imgErr += imgErrx;
          fprintf(fo, " = %10.1f\n", imgErrx);
        }
        for (int jx = 0; jx < nx; jx++)
          fprintf(fo, "   %10.1f   ", imgErry[jx]);
        fprintf(fo, " = %10.1f\n", imgErr);
      }
      fprintf(fo,  "avg err = %g\n", avgErr/(nframes*n) );
      fo = f_out;
    }
    fclose( f_out );
    fprintf(fini, "#Estimated from %d image sets resulting average pixel error %g pixels\n", nframes, avgErr/(nframes*n)); 
    fclose(fini);
    //COMPUTE AND DISPLAY RECTIFICATION
    if( showUndistorted )
    {
        CvMat* mx1 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* my1 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* mx2 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* my2 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* img1r = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* img2r = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* disp = cvCreateMat( imageSize.height,
            imageSize.width, CV_16S );
        CvMat* vdisp = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* pair;
        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
        CvMat _R2 = cvMat(3, 3, CV_64F, R2);
// IF BY CALIBRATED (BOUGUET'S METHOD)
        if( useUncalibrated == 0 )
        {
            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
            CvMat _P2 = cvMat(3, 4, CV_64F, P2);
            cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
                &_R, &_T,
                &_R1, &_R2, &_P1, &_P2, &matQ,
                0/*CV_CALIB_ZERO_DISPARITY*/ );
            isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
    //Precompute maps for cvRemap()
            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
            cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
        }
//OR ELSE HARTLEY'S METHOD
        else if( useUncalibrated == 1 || useUncalibrated == 2 )
     // use intrinsic parameters of each camera, but
     // compute the rectification transformation directly
     // from the fundamental matrix
        {
            double H1[3][3], H2[3][3], iM[3][3];
            CvMat _H1 = cvMat(3, 3, CV_64F, H1);
            CvMat _H2 = cvMat(3, 3, CV_64F, H2);
            CvMat _iM = cvMat(3, 3, CV_64F, iM);
    //Just to show you could have independently used F
            if( useUncalibrated == 2 )
                cvFindFundamentalMat( &_imagePoints1,
                &_imagePoints2, &_F);
            cvStereoRectifyUncalibrated( &_imagePoints1,
                &_imagePoints2, &_F,
                imageSize,
                &_H1, &_H2, 3);
            cvInvert(&_M1, &_iM);
            cvMatMul(&_H1, &_M1, &_R1);
            cvMatMul(&_iM, &_R1, &_R1);
            cvInvert(&_M2, &_iM);
            cvMatMul(&_H2, &_M2, &_R2);
            cvMatMul(&_iM, &_R2, &_R2);
    //Precompute map for cvRemap()
            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_M1,mx1,my1);

            cvInitUndistortRectifyMap(&_M2,&_D1,&_R2,&_M2,mx2,my2);
        }
        else
            assert(0);

        //cvNamedWindow( "rectified", 1 );
// RECTIFY THE IMAGES AND FIND DISPARITY MAPS
        if( !isVerticalStereo )
            pair = cvCreateMat( imageSize.height, imageSize.width*2,
            CV_8UC3 );
        else
            pair = cvCreateMat( imageSize.height*2, imageSize.width,
            CV_8UC3 );
//Setup for finding stereo corrrespondences
        CvStereoBMState *BMState = cvCreateStereoBMState();
        assert(BMState != 0);
        BMState->preFilterSize=41;
        BMState->preFilterCap=31;
        BMState->SADWindowSize=41;
        BMState->minDisparity=-64;
        BMState->numberOfDisparities=128;
        BMState->textureThreshold=10;
        BMState->uniquenessRatio=15;
        for( i = 0; i < nframes; i++ )
        {
            IplImage* img1=cvLoadImage(imageNames[0][i].c_str(),0);
            IplImage* img2=cvLoadImage(imageNames[1][i].c_str(),0);
            if( img1 && img2 )
            {
                CvMat part;
                cvRemap( img1, img1r, mx1, my1 );
                cvRemap( img2, img2r, mx2, my2 );
                if( !isVerticalStereo || useUncalibrated != 0 )
                {
              // When the stereo camera is oriented vertically,
              // useUncalibrated==0 does not transpose the
              // image, so the epipolar lines in the rectified
              // images are vertical. Stereo correspondence
              // function does not support such a case.
                    cvFindStereoCorrespondenceBM( img1r, img2r, disp,
                        BMState);
                    cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
                    snprintf(fns, MSL, "dispar-%s", imageNames[0][i].c_str());
                    cvSaveImage(fns, vdisp);
                    //cvNamedWindow( "disparity" );
                    //cvShowImage( "disparity", vdisp );
                    print3dcloud(BMState, i, imageSize, disp, mx1, my1, &matQ);
                }
                if( !isVerticalStereo )
                {
                    cvGetCols( pair, &part, 0, imageSize.width );
                    cvCvtColor( img1r, &part, CV_GRAY2BGR );
                    cvGetCols( pair, &part, imageSize.width,
                        imageSize.width*2 );
                    cvCvtColor( img2r, &part, CV_GRAY2BGR );
                    for( j = 0; j < imageSize.height; j += 16 )
                        cvLine( pair, cvPoint(0,j),
                        cvPoint(imageSize.width*2,j),
                        CV_RGB(0,255,0));
                }
                else
                {
                    cvGetRows( pair, &part, 0, imageSize.height );
                    cvCvtColor( img1r, &part, CV_GRAY2BGR );
                    cvGetRows( pair, &part, imageSize.height,
                        imageSize.height*2 );
                    cvCvtColor( img2r, &part, CV_GRAY2BGR );
                    for( j = 0; j < imageSize.width; j += 16 )
                        cvLine( pair, cvPoint(j,0),
                        cvPoint(j,imageSize.height*2),
                        CV_RGB(0,255,0));
                }
                snprintf(fns, MSL, "pair-%s", imageNames[0][i].c_str());
                cvSaveImage(fns, pair);
/*                cvShowImage( "rectified", pair );
                if( cvWaitKey() == 27 )
                    break;*/
            }
            cvReleaseImage( &img1 );
            cvReleaseImage( &img2 );
        }
        cvReleaseStereoBMState(&BMState);
        cvReleaseMat( &mx1 );
        cvReleaseMat( &my1 );
        cvReleaseMat( &mx2 );
        cvReleaseMat( &my2 );
        cvReleaseMat( &img1r );
        cvReleaseMat( &img2r );
        cvReleaseMat( &disp );
    }
}

int main(int argc, char *argv[])
{
  int i;
  int xx = 8, yy=6;
  bool xxOK = 0, yyOK = 0;
  char * filename = NULL;
  double squareSize = 0.0705;
  int estimateK3 = 1;
  for (i = 1; i < argc; i++)
  {
    char * p1 = argv[i], *p2;
    int v;
    if (*p1 == '-')
      break;
    v = strtol(p1, &p2, 10);
    if (p2 > p1)
    {
      if (xxOK and yyOK)
      {
        squareSize = strtof(p1, &p2);
      }
      else if (xxOK)
        yy = v;
      else
        xx = v;
      if (xxOK)
        yyOK = true;
      xxOK = true;
    }
    else
    {
      if (strcasecmp(p1, "k3") == 0)
        estimateK3 = 1;
      else
        filename = p1;
    }
  }
  if (filename == NULL or xx == 0 or yy == 0)
  {
    printf("\nA stereo calibrate utility from openCV\n");
    printf("- requires at least 4 set of images of a checkerboard\n");
    printf("use:\n");
    printf("  stereocalibrate [[xx yy] squareSize] <filename>\n");
    printf("where:\n");
    printf("  filename is name of a file with filename of images from left and right camera, one name on each line, left image first, at least 4 imagesets\n");
    printf("  xx and yy is number of detectable corners in x and y direction on checkerboard (default is %dx x %dy)\n", xx, yy);
    printf("  squareSize is the size of one (black or white) square in meters (default=%g m)\n", squareSize);
    //printf("  k3 if this option is mentioned, then also the 6th order radial distortion is estimated\n");
    printf("Example: a portrait oriented checkerboard in A2 format with 7.05cm squares:\n");
    printf("  stereocalibrate 6 8 0.0705 images.list\n");
    printf("Take some checkerboard images at different distances and images where checkerboard gets out in all corners of image.\n");
    printf("result is in stereocalib.ini that can be used in camera server ini-file\n");
    printf("The error list shows estimated error for each corner (and summed) investigate for outliers, \n");
    printf("  i.e. a single corner with high pixel error compared to neighbour pixels - likely bad \n");
    printf("  corner estimates. Remove such image sets. compare also with saved corner images.\n");
    printf("prints a pixel error estimate at the end, this should be < 1 pixel\n\n");
  }
  else
    StereoCalib(filename, xx, yy, squareSize, 0, estimateK3);
  return 0;
}


/// print 3D cloud
void print3dcloud(CvStereoBMState *BMState, int imgNum, CvSize imageSize, CvMat * disp, CvArr * mx1, CvArr * my1, CvMat * matQ)
{
  int16_t * d16;
  CvScalar upx, upy;
  bool isDebug = true;
  FILE * f3d;
  int maxDisp = 100;
  int minDisp = 0;
  float p3d[3];
  CvMat point3d = cvMat(1, 1, CV_32FC3, p3d);
  float p3d2[3];
  CvMat point3d2 = cvMat(1, 1, CV_32FC3, p3d2);
  const int MSL = 100;
  char s[MSL];
  snprintf(s, MSL, "3dcloud-%02d.txt", imgNum);
  f3d = fopen(s, "w");
  fprintf(f3d, "x=fwd, y=left, z=up, rowOrg, colOrg, rowRectified, colRectified, disparity\n");
  for (int i = 0; i < imageSize.height; i+=1)
  { 
    d16 = (int16_t *)disp->data.i;
    d16 += i * imageSize.width;
    //printf("line %3d:\n", i);
    for (int j = 0 /*imageSize.width/2*/; j < imageSize.width; j+=1 /*32*/)
    { // get original color pixel
      if (*d16 >= BMState->minDisparity)
      { // a 3D can be calculated
        if (*d16 > maxDisp)
          maxDisp = *d16;
        if (*d16 < minDisp)
          minDisp = *d16;
        upx = cvGet2D(mx1, i, j);
        upy = cvGet2D(my1, i, j);
        p3d[0] = j;
        p3d[1] = i;
        p3d[2] = *d16/16.0;
        cvPerspectiveTransform(&point3d, &point3d2, matQ);
        if (isDebug)
        { // coordinates in robot style coordinates x=fwd, y=left, z=up
          fprintf(f3d, "%7.4f %7.4f %7.4f   %5.1f %5.1f   %3d %3d %5.2f\n",
                    -p3d2[2], -p3d2[0], p3d2[1],
                    upy.val[0], upx.val[0],
                    i, j, *d16/16.0);
        }
      }
      d16++;
    }
  }
  fclose(f3d);
}
