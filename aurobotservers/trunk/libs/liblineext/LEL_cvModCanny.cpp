/*
This is a modified version of the cvcanny.cpp source file. The original file calls the
sobel filter inside the cvCanny function. This file has been modified to separate the
sobel filtering operation and canny edge detection. Necessary changes have been made
to be able to include this file. without including exotic source files. Those changes
might turn out to be incompatible with different versions of openCV.

Enis BAYRAMOGLU
24.04.2008,
enisbayramoglu@gmail.com

The algorithm is also modified now. The non-maxima supression is changed to look for
horizontal or vertical neighbors in cases where the diagonal neighbors would be checked before
*/


/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include <stdio.h>

//#define OPENCV2
#ifdef OPENCV2
#include <core/core.hpp>
//#include <opencv/core/core.hpp>
//#include <opencv/core/core_c.h>
#include <imgproc/imgproc_c.h>
#else
#include <cxcore.h>
#include <cv.h>
#endif

void cvModCanny( CvMat* dx, CvMat* dy, IplImage* dst,
         double low_thresh, double high_thresh )
{
//    static const int sec_tab[] = { 1, 3, 0, 0, 2, 2, 2, 2 };
    void *buffer = 0;
    uchar **stack_top, **stack_bottom = 0;

    CvSize size;
    int low, high;
    int* mag_buf[3];
    uchar* map;
    int mapstep, maxsize;
    int i, j;
    CvMat mag_row;

    if( low_thresh > high_thresh )
    {
        double t;
        CV_SWAP( low_thresh, high_thresh, t );
    }

    size = cvSize(dst->width,dst->height);


    low = cvFloor( low_thresh );
    high = cvFloor( high_thresh );

    buffer = cvAlloc( (size.width+2)*(size.height+2) +
                                (size.width+2)*3*sizeof(int)) ;

    mag_buf[0] = (int*)buffer;
    mag_buf[1] = mag_buf[0] + size.width + 2;
    mag_buf[2] = mag_buf[1] + size.width + 2;
    map = (uchar*)(mag_buf[2] + size.width + 2);
    mapstep = size.width + 2;

    maxsize = MAX( 1 << 10, size.width*size.height/10 );
    stack_top = stack_bottom = (uchar**)cvAlloc( maxsize*sizeof(stack_top[0]) );

    memset( mag_buf[0], 0, (size.width+2)*sizeof(int) );
    memset( map, 1, mapstep );
    memset( map + mapstep*(size.height + 1), 1, mapstep );

    /* sector numbers
       (Top-Left Origin)

        1   2   3
         *  *  *
          * * *
        0*******0
          * * *
         *  *  *
        3   2   1
    */

    #define CANNY_PUSH(d)    *(d) = (uchar)2, *stack_top++ = (d)
    #define CANNY_POP(d)     (d) = *--stack_top

    mag_row = cvMat( 1, size.width, CV_32F );

    // calculate magnitude and angle of gradient, perform non-maxima supression.
    // fill the map with one of the following values:
    //   0 - the pixel might belong to an edge
    //   1 - the pixel can not belong to an edge
    //   2 - the pixel does belong to an edge
    for( i = 0; i <= size.height; i++ )
    {
        int* _mag = mag_buf[(i > 0) + 1] + 1;
        const short* _dx = (short*)(dx->data.ptr + dx->step*i);
        const short* _dy = (short*)(dy->data.ptr + dy->step*i);
        uchar* _map;
        int x, y;
        int magstep1, magstep2;
        int prev_flag = 0;

        if( i < size.height )
        {
            _mag[-1] = _mag[size.width] = 0;

            for( j = 0; j < size.width; j++ )
                _mag[j] = abs(_dx[j]) + abs(_dy[j]);
        }
        else
            memset( _mag-1, 0, (size.width + 2)*sizeof(int) );

        // at the very beginning we do not have a complete ring
        // buffer of 3 magnitude rows for non-maxima suppression
        if( i == 0 )
            continue;

        _map = map + mapstep*i + 1;
        _map[-1] = _map[size.width] = 1;

        _mag = mag_buf[1] + 1; // take the central row
        _dx = (short*)(dx->data.ptr + dx->step*(i-1));
        _dy = (short*)(dy->data.ptr + dy->step*(i-1));

        magstep1 = (int)(mag_buf[2] - mag_buf[1]);
        magstep2 = (int)(mag_buf[0] - mag_buf[1]);

        if( (stack_top - stack_bottom) + size.width > maxsize )
        {
            uchar** new_stack_bottom;
            maxsize = MAX( maxsize * 3/2, maxsize + size.width );
            new_stack_bottom = (uchar**)cvAlloc( maxsize * sizeof(stack_top[0]));
            memcpy( new_stack_bottom, stack_bottom, (stack_top - stack_bottom)*sizeof(stack_top[0]) );
            stack_top = new_stack_bottom + (stack_top - stack_bottom);
            cvFree( (void **)&stack_bottom );
            stack_bottom = new_stack_bottom;
        }

        for( j = 0; j < size.width; j++ )
        {

            x = _dx[j];
            y = _dy[j];
            int m = _mag[j];

            x = abs(x);
            y = abs(y);
            if( m > low )
            {
		if(y<x)
                {
                    if( m > _mag[j-1] && m >= _mag[j+1] )
                    {
                        if( m > high && !prev_flag && _map[j-mapstep] != 2 )
                        {
                            CANNY_PUSH( _map + j );
                            prev_flag = 1;
                        }
                        else
                            _map[j] = (uchar)0;
                        continue;
                    }
                }
                else
                {
                    if( m > _mag[j+magstep2] && m >= _mag[j+magstep1] )
                    {
                        if( m > high && !prev_flag && _map[j-mapstep] != 2 )
                        {
                            CANNY_PUSH( _map + j );
                            prev_flag = 1;
                        }
                        else
                            _map[j] = (uchar)0;
                        continue;
                    }
                }
            }
            prev_flag = 0;
            _map[j] = (uchar)1;
        }

        // scroll the ring buffer
        _mag = mag_buf[0];
        mag_buf[0] = mag_buf[1];
        mag_buf[1] = mag_buf[2];
        mag_buf[2] = _mag;
    }

    // now track the edges (hysteresis thresholding)
    while( stack_top > stack_bottom )
    {
        uchar* m;
        if( (stack_top - stack_bottom) + 8 > maxsize )
        {
            uchar** new_stack_bottom;
            maxsize = MAX( maxsize * 3/2, maxsize + 8 );
            new_stack_bottom = (uchar**)cvAlloc( maxsize * sizeof(stack_top[0]));
            memcpy( new_stack_bottom, stack_bottom, (stack_top - stack_bottom)*sizeof(stack_top[0]) );
            stack_top = new_stack_bottom + (stack_top - stack_bottom);
            cvFree( (void **)&stack_bottom );
            stack_bottom = new_stack_bottom;
        }

        CANNY_POP(m);

        if( !m[-1] )
            CANNY_PUSH( m - 1 );
        if( !m[1] )
            CANNY_PUSH( m + 1 );
        if( !m[-mapstep-1] )
            CANNY_PUSH( m - mapstep - 1 );
        if( !m[-mapstep] )
            CANNY_PUSH( m - mapstep );
        if( !m[-mapstep+1] )
            CANNY_PUSH( m - mapstep + 1 );
        if( !m[mapstep-1] )
            CANNY_PUSH( m + mapstep - 1 );
        if( !m[mapstep] )
            CANNY_PUSH( m + mapstep );
        if( !m[mapstep+1] )
            CANNY_PUSH( m + mapstep + 1 );
    }

    // the final pass, form the final image
    for( i = 0; i < size.height; i++ )
    {
        const uchar* _map = map + mapstep*(i+1) + 1;
        uchar* _dst = (uchar*) (dst->imageData) + dst->widthStep*i;

        for( j = 0; j < size.width; j++ )
            _dst[j] = (uchar)-(_map[j] >> 1);
    }


    cvFree( &buffer );
    cvFree( (void**)&stack_bottom );
}

void cvModCanny( IplImage* img, IplImage* dst, double low_thresh, double high_thresh, int aperture_size){

	CvMat* dx,*dy;

	dy = cvCreateMat( img->height, img->width, CV_16SC1 );
	dx = cvCreateMat( img->height, img->width, CV_16SC1 );
	cvSobel( img, dx, 1, 0, aperture_size );
	cvSobel( img, dy, 0, 1, aperture_size );

	cvModCanny(dx,dy,dst,200,60);

	cvReleaseMat(&dx);
	cvReleaseMat(&dy);
}


/* End of file. */
