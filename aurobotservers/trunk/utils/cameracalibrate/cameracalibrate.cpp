
#ifdef OPENCV2
#include <core/core_c.h>
//#include <imgproc/types_c.h>
#include <imgproc/imgproc_c.h>
#include <calib3d/calib3d.hpp>
#include <highgui/highgui_c.h>
#else
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif
//#include <iostream>
#include <stdio.h>
#include <getopt.h>


bool getCmdLineOptions(int argc, char *argv[],
                       int * board_w, int * board_h, int fileArg[], int * fileCnt);

int main(int argc, char* argv[])
{
  //int n_boards = 0;
  //const int board_dt = 20;
  int board_w;
  int board_h;
  int * fileArg;
  fileArg = (int*)malloc(argc * sizeof(int));
  board_w = 5; // Board width in squares
  board_h = 8; // Board height
  int boardCnt =0;
  //n_boards = 0;
  //CvCapture* capture = cvCreateCameraCapture( 1 );
  //assert( capture );
  bool isOK = getCmdLineOptions(argc, argv, &board_w, &board_h, fileArg, &boardCnt);
  if (isOK and boardCnt >= 3)
  { // board size;
    int board_n = board_w * board_h;
    CvSize board_sz = cvSize( board_w, board_h );
    // Allocate Storage
    CvMat* image_points             = cvCreateMat( boardCnt*board_n, 2, CV_32FC1 );
    CvMat* object_points            = cvCreateMat( boardCnt*board_n, 3, CV_32FC1 );
    CvMat* point_counts             = cvCreateMat( boardCnt, 1, CV_32SC1 );
    CvMat* intrinsic_matrix         = cvCreateMat( 3, 3, CV_32FC1 );
    CvMat* distortion_coeffs        = cvCreateMat( 5, 1, CV_32FC1 );

    CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
    int corner_count;
    int successes = 0;
    int step;
    const char * fileName = argv[fileArg[0]];

    IplImage *image = cvLoadImage(fileName); // cvQueryFrame( capture );
    IplImage *gray_image = cvCreateImage( cvGetSize( image ), 8, 1 );
    const int MSL = 100;
    char s[MSL];

    // Capture Corner views loop until we've got n_boards
    // succesful captures (all corners on the board are found)
    printf("using %d image files\n", boardCnt);
    for (int i = 0; i < boardCnt; i++)
    {
      if (i > 0)
      {
        fileName = argv[fileArg[i]];
        image = cvLoadImage(fileName);
      }
      if (image == NULL)
        printf("Failed to load image %s\n", fileName);
      if (image != NULL)
      { // Find chessboard corners:
        int found = cvFindChessboardCorners( image, board_sz, corners,
                &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
        printf("- %d found %d in a %dx%d frame in file %s\n", i, corner_count, board_h, board_w, fileName);
        // Get subpixel accuracy on those corners
        cvCvtColor( image, gray_image, CV_BGR2GRAY );
        cvFindCornerSubPix( gray_image, corners, corner_count, cvSize( 11, 11 ),
                cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

        // Draw it
        cvDrawChessboardCorners( image, board_sz, corners, corner_count, found );
        //cvShowImage( "Calibration", image );
        snprintf(s, MSL, "cp-%s", fileName);
        //cvShowImage( "Undistort", image ); // Show corrected image
        cvSaveImage(s, image);

        // If we got a good board, add it to our data
        if( corner_count == board_n )
        {
          step = successes*board_n;
          for( int i=step, j=0; j < board_n; ++i, ++j )
          {
                  CV_MAT_ELEM( *image_points, float, i, 0 ) = corners[j].x;
                  CV_MAT_ELEM( *image_points, float, i, 1 ) = corners[j].y;
                  CV_MAT_ELEM( *object_points, float, i, 0 ) = j/board_w;
                  CV_MAT_ELEM( *object_points, float, i, 1 ) = j%board_w;
                  CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
          }
          CV_MAT_ELEM( *point_counts, int, successes, 0 ) = board_n;
          successes++;
        }
      }
    } // End collection while loop

    if (successes > 3)
    {
      FILE * iniFile;
      // Allocate matrices according to how many chessboards found
      CvMat* object_points2 = cvCreateMat( successes*board_n, 3, CV_32FC1 );
      CvMat* image_points2 = cvCreateMat( successes*board_n, 2, CV_32FC1 );
      CvMat* point_counts2 = cvCreateMat( successes, 1, CV_32SC1 );

      // Transfer the points into the correct size matrices
      for( int i = 0; i < successes*board_n; ++i ){
              CV_MAT_ELEM( *image_points2, float, i, 0) = CV_MAT_ELEM( *image_points, float, i, 0 );
              CV_MAT_ELEM( *image_points2, float, i, 1) = CV_MAT_ELEM( *image_points, float, i, 1 );
              CV_MAT_ELEM( *object_points2, float, i, 0) = CV_MAT_ELEM( *object_points, float, i, 0 );
              CV_MAT_ELEM( *object_points2, float, i, 1) = CV_MAT_ELEM( *object_points, float, i, 1 );
              CV_MAT_ELEM( *object_points2, float, i, 2) = CV_MAT_ELEM( *object_points, float, i, 2 );
      }

      for( int i=0; i < successes; ++i ){
              CV_MAT_ELEM( *point_counts2, int, i, 0 ) = CV_MAT_ELEM( *point_counts, int, i, 0 );
      }
      cvReleaseMat( &object_points );
      cvReleaseMat( &image_points );
      cvReleaseMat( &point_counts );

      // At this point we have all the chessboard corners we need
      // Initiliazie the intrinsic matrix such that the two focal lengths
      // have a ratio of 1.0

      CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0;
      CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0;

      // Calibrate the camera
      cvCalibrateCamera2( object_points2, image_points2, point_counts2, cvGetSize( image ),
              intrinsic_matrix, distortion_coeffs, NULL, NULL, CV_CALIB_FIX_ASPECT_RATIO );

      // Save the intrinsics and distortions
      cvSave( "Intrinsics.xml", intrinsic_matrix );
      printf("saved Intrinsics.xml as matrix:\n");
      iniFile = fopen("canXX.ini", "w");
      if (iniFile != NULL)
      {
        fprintf(iniFile, "# camera calibration values for camera XX\n"
                         "# replace the XX with the actual camera device number\n"
                         "# and incooperate the commands in the ucamserver.ini\n");
        fprintf(iniFile, "var campool.deviceXX.intrinsic=\"");
      }                  
      for (int i = 0; i < 3; i++)
      {
        printf(" %9.3f, %9.3f, %9.3f;\n", cvmGet(intrinsic_matrix, i, 0), cvmGet(intrinsic_matrix, i, 1), cvmGet(intrinsic_matrix, i, 2));
        if (iniFile != NULL)
          fprintf(iniFile, " %9.3f, %9.3f, %9.3f;", cvmGet(intrinsic_matrix, i, 0), cvmGet(intrinsic_matrix, i, 1), cvmGet(intrinsic_matrix, i, 2));
      }
      if (iniFile != NULL)
        fprintf(iniFile, "\"\n");
      cvSave( "Distortion.xml", distortion_coeffs );
      printf("saved Distortion.xml with:\n");
      printf(" k1=%9.5f,", cvmGet(distortion_coeffs, 0, 0));
      printf(" k2=%9.5f,", cvmGet(distortion_coeffs, 1, 0));
      printf(" p1=%9.5f,", cvmGet(distortion_coeffs, 2, 0));
      printf(" p2=%9.5f,", cvmGet(distortion_coeffs, 3, 0));
      printf(" k3=%9.5f\n", cvmGet(distortion_coeffs, 4, 0));
      if (iniFile != NULL)
        fprintf(iniFile, "var campool.deviceXX.something=\"%f %f %f %f %f\"\n",
                cvmGet(distortion_coeffs, 0, 0), cvmGet(distortion_coeffs, 1, 0),
                cvmGet(distortion_coeffs, 2, 0), cvmGet(distortion_coeffs, 3, 0),
                cvmGet(distortion_coeffs, 4, 0));
      if (iniFile != NULL)
        fclose(iniFile);
    }

    // Example of loading these matrices back in
    CvMat *intrinsic = (CvMat*)cvLoad( "Intrinsics.xml" );
    CvMat *distortion = (CvMat*)cvLoad( "Distortion.xml" );

    if (intrinsic != NULL)
    { // Build the undistort map that we will use for all subsequent frames
      IplImage* mapx = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
      IplImage* mapy = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
      cvInitUndistortMap( intrinsic, distortion, mapx, mapy );

      for (int i = 0; i < boardCnt; i++)
      {
        fileName = argv[fileArg[i]];
        image = cvLoadImage(fileName);
        IplImage *t = cvCloneImage( image );
        cvRemap( t, image, mapx, mapy ); // undistort image
        cvReleaseImage( &t );
        snprintf(s, MSL, "ud-%s", fileName);
        cvSaveImage(s, image);
      }
    }
    return !(intrinsic != NULL);
  }
  else
    return -1;
}

/////////////////////////////////////////////////////

bool getCmdLineOptions(int argc, char *argv[],
                       int * board_w, int * board_h, int fileArg[], int * fileCnt)
{
  static struct option long_options[] = {
    {"width", 1, 0, 'w'},
    {"height", 1, 0, 'h'},
    {"help", 0, 0, 'a'},
    {0, 0, 0, 0}
  };
  bool ask4help = false;
  int opt;
  int option_index = 0;
  while(true)
  {
    opt = getopt_long(argc, argv, "aw:h:",
                 long_options, &option_index);
    if (opt == -1)
      break;
    switch (opt)
    {
      case 'w': // pattern-width
        if (optarg != NULL)
          *board_w = strtol(optarg, NULL, 10);
        break;
      case 'h': // pattern height
        if (optarg != NULL)
          *board_h = strtol(optarg, NULL, 10);
        break;
      case 'a':
        ask4help = true;
        //printHelp();
        break;
      default:
        break;
    }
  }
  if (ask4help or argc < 3)
  {
    printf("\nFinding intrinsic and distortion parameters from camera images.\n");
    printf("using a checkerboard of size WxH usable corners (5-10 images well into corners).\n");
    printf("Saves images in an undistorded version - with 'un-' as pre-name.\n");
    printf("use:\n");
    printf("  $ %s -h H -w W file1 file2 fileN\n", argv[0]);
    printf("or:\n");
    printf("  $ %s --hight=H --width=W file1 file2 fileN\n", argv[0]);
    printf("or this help\n");
    printf("  $ %s --help\n", argv[0]);
    printf("\nexample using images of a checkerboard with 8x5=40 usable corners (9x6 squares):\n");
    printf("  $ %s -h5 -w8 guppy-10*.png\n\n", argv[0]);
  }
  else if (optind < argc)
  {
    while (optind < argc)
      fileArg[(*fileCnt)++] = optind++;
  }
  return not ask4help;
}

