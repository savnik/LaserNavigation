/***************************************************************************
 *   Copyright (C) 2012 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *   Albert Navarro Comes                                                  *
 *   albertnavarro8@gmail.com                                              *
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
 *
 * $Id: ufunctracking.cpp 59 2012-10-21 06:25:02Z jcan $
 * $Rev: 59 $
 ***************************************************************************/
//
#include <urob4/usmltag.h>
//odometry
#include <urob4/uresposehist.h>
#include <sys/time.h>
//getVarPool
#include <urob4/uvarcalc.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "ufunctracking.h"

#define UIMGPOOL_BALL_BASE 45

#ifdef LIBRARY_OPEN_NEEDED

UFunctionBase * createFunc()
{ // called by server to create an object of this type
    // replace 'UFuncPCLTest' with your classname, as used in the headerfile
    return new UFuncTracking();
}

#endif

using pcl::visualization::PointCloudColorHandlerCustom;
typedef pcl::PointXYZRGB PointT;
pcl::visualization::PCLVisualizer *viewer;

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncTracking::~UFuncTracking()
{ // possibly remove allocated variables here - if needed
    printf("PCLTest unloaded\n");
}


///////////////////////////////////////////////////

bool UFuncTracking::handleCommand(UServerInMsg * msg, void * extra)
{ // handle command(s) send to this plug-in
    bool ask4help;
    int camDevice = -1;
    int imgPoolNum = -1;
    USmlTag tag;
    bool result;
    bool debug = true; // default is debug on
    // check for parameters - one parameter is tested for - 'help'
    ask4help = msg->tag.getAttValue("help", NULL, 0);
    if (not ask4help)
    { // get all other parameters
        msg->tag.getAttValueInt("device", &camDevice);
        msg->tag.getAttValueInt("img", &imgPoolNum);
        msg->tag.getAttValueBool("debug", &debug, true);
    }
    // ask4help = false, if no 'help' option were available.
    if (ask4help)
    { // create the reply in XML-like (html - like) format
        sendHelpStart("Track");
        sendText("help              This message\n");
	sendText("img=18            \n");
	sendText("track             Track objects, it will only analyze one image at a time. To perform a proper track\n write poolpush img=18 cmd=track\n");
        sendText("---\n");
        sendHelpDone();
        sendInfo("done");
        result = true;
    }
    else
    { // not help, so - first - get source image
        TrackObjects();
        printf("Capture studied\n");
        result = true;
    }
    // return true if the function is handled with a positive result
    return result;
}

////////////////////////////////////////////////////////////

void UFuncTracking::createBaseVar()
{
    //calling function "track" the program will track objects
}

//////////////////////////////////////////////

void UFuncTracking::GetKinectPointCloudDataRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud )
{

    const int MPC = 2; // parameter count
    double pars[MPC], v;
    bool isOK;
    int n = 1;
    pars[0] = 1;
    pars[1] = -1;
    isOK = callGlobal("kinect.GetPointCloud", "dd", NULL, pars, &v, (UDataBase**) &cloud, &n);
    if (not isOK)
        printf("*** not found error: no kinect.GetPointCloud(d,d)\n");
}


/////////////////////////////////////////////////////////////////////////////////

void UFuncTracking::FirstImage(void)
{
    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ini (new pcl::PointCloud<pcl::PointXYZRGB>),
    cloud_ini_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),cloud_ini_trans_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),
    cloud_ini_pitch (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cluster_ini;
    OBJ obj_img_ini[40];

    build_tree_smr("data/smr");
    build_tree_human("data/human");

    smr_desc = SMR;
    human_desc = HUMAN;
    times_human = 0;
    times_smr = 0;

    timeval first_time;
    gettimeofday(&first_time,NULL);
    START_TIME = ((double)first_time.tv_sec)+((double)first_time.tv_usec)/1000000;
    IMAGES_STUDIED = 1;

    HUMAN_FAR = 20;
    FLOOR = -1.32;

    GetKinectPointCloudDataRGB( cloud_ini );

    if ( cloud_ini->points.size() > 0 )
    {
        UImagePool * imgPool = (UImagePool *)getStaticResource("imgPool", false, false);
        UResPoseHist * odo;
        UPoseTime pose;
        odo = (UResPoseHist *) getStaticResource("odoPose", true);
        //reset pose
        if (imgPool != NULL)
        { // raw depth image
            UVariable * var = getVarPool()->getGlobalVariable("kinect.imagesC3D");
            UImage * depthBW = imgPool->getImage(var->getInt(1), true);
            if (depthBW != NULL)
            {
                // depth time is set, as the first usb-packet is arrived in the freenect driver
                pose = odo->getPoseAtTime(depthBW->imgTime);
            }
        }
        printf("POSE X: %f POSE Y: %f POSE H: %f\n",pose.x,pose.y,pose.h);

        downsampling( cloud_ini, cloud_ini_filtered );

        X_INI = pose.x;
        Y_INI = pose.y;
        YAW_INI = pose.h;
        X = pose.x;
        Y = pose.y;
        YAW = pose.h;
        Z = 0.0;
        ROLL = 0.0;
        update_file_robot_position(1);
        getGlobalValue("kinect.orient[1]",&pitch);
        printf("ANGLE: %f\n",pitch);

        viewer->setCameraPose(-6.0,1.0,10.0,pose.x,pose.y,0.0,0.0,0.0,1.0);
        //filtering
        Eigen::Affine3f t_prev,t;
        pcl::getTransformation ( 0.0, 0.0, 0.0, 0.0, pitch, 0.0, t_prev );
        pcl::getTransformation ( X, Y, Z, ROLL, 0.0, YAW, t );
        pcl::transformPointCloud ( *cloud_ini_filtered, *cloud_ini_pitch, t_prev );

        clustering_floor( &cloud_ini_pitch );
        clustering( cloud_ini_pitch, obj_img_ini, &cluster_ini );

        pcl::transformPointCloud ( *cloud_ini_pitch, *cloud_ini_trans_filtered, t );

        object_classification_frame1 ( obj_img_ini, cluster_ini );
        printf("FIRST FRAME HAS BEEN STUDIED. %d HUMANS, %d SMRS and %d OTHER OBJECTS WERE FOUND.\n",human_ini,smr_ini,rest_ini);

        add = 0;
        STATIC = 0;
        first_image = false;
    }
}

void UFuncTracking::TrackObjects(void)
{
    timeval begin, ini_it, end_it, endclus, endma, endtotal, endget, endtrans, enddown, endvis, tfloor, endobj, endtrans2, tplane;
    gettimeofday(&begin,NULL);
    // Read in the cloud data
    Wall_Image=0;
    //check if it is the first image, because it needs some different functions due to there are no non-trained clusters stored and also 
    //because many global variables need to be initialized
    if ( first_image )
        FirstImage();
    else
    {
        pcl::PCDReader reader;

        UImagePool * imgPool = (UImagePool *)getStaticResource("imgPool", false, false);
        //odometry
	UResPoseHist * odo;
        UPoseTime pose;
        odo = (UResPoseHist *) getStaticResource("odoPose", true);

        color color_matched[10];
        color color_cluster[15];
        color color_new_smr[5];
        color color_new_human[5];

        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > static_cluster;

        gettimeofday(&ini_it,NULL);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_next (new pcl::PointCloud<pcl::PointXYZRGB>);
        GetKinectPointCloudDataRGB( cloud_next );
        if ( cloud_next->points.size() > 0 )
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_next_trans (new pcl::PointCloud<pcl::PointXYZRGB>),
            cloud_next_trans_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_next_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),
            cloud_pitch (new pcl::PointCloud<pcl::PointXYZRGB>);
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > matched_prev;
            if (imgPool != NULL)
            { // raw depth image
                UVariable * var = getVarPool()->getGlobalVariable("kinect.imagesC3D");
                UImage * depthBW = imgPool->getImage(var->getInt(1), true);
                if (depthBW != NULL)
                {
                    pose = odo->getPoseAtTime(depthBW->imgTime);
                }
            }
            printf("POSE X: %f POSE Y: %f POSE H: %f pitch: %f\n",pose.x,pose.y,pose.h,pitch);
            //changing the camera focus
            // transformation of the point cloud to the same reference system as the initial one
            X = pose.x;
            Y = pose.y;
            YAW = pose.h;
            Z = 0.0;
            ROLL = 0.0;
            update_file_robot_position(1);
            gettimeofday(&endget,NULL);
	    //calculating new camera pose of the viewer
            if (YAW<-PI/2)
            {
                viewer->setCameraPose(pose.x+6.0*cos(PI+YAW),pose.y+6.0*sin(PI+YAW),6.0,pose.x,pose.y,0.0,0.0,0.0,1.0);
                printf("Pose.x: %f Pose.y: %f\n",pose.x+6.0*cos(PI+YAW),pose.y+6.0*sin(PI+YAW));
            }
            else if ( (YAW>-PI/2) & (YAW<0) )
            {
                viewer->setCameraPose(pose.x-6.0*cos(YAW),pose.y-6.0*sin(YAW),6.0,pose.x,pose.y,0.0,0.0,0.0,1.0);
                printf("Pose.x: %f Pose.y: %f\n",pose.x-6.0*cos(YAW),pose.y-6.0*sin(YAW));
            }
            else if (YAW>PI/2)
            {
                viewer->setCameraPose(pose.x+6.0*cos(PI-YAW),pose.y-6.0*sin(PI-YAW),6.0,pose.x,pose.y,0.0,0.0,0.0,1.0);
                printf("Pose.x: %f Pose.y: %f\n",pose.x+6.0*cos(PI-YAW),pose.y-6.0*sin(PI-YAW));
            }
            else
            {
                viewer->setCameraPose(pose.x-6.0*cos(YAW),pose.y-6.0*sin(YAW),6.0,pose.x,pose.y,0.0,0.0,0.0,1.0);
                printf("Pose.x: %f Pose.y: %f\n",pose.x-6.0*cos(YAW),pose.y-6.0*sin(YAW));
            }
            //filtering
            downsampling( cloud_next, cloud_next_filtered );
            gettimeofday(&enddown,NULL);
            Eigen::Affine3f t_prev,t;
	    //get t_prev using only pitch
            pcl::getTransformation ( 0.0, 0.0, 0.0, ROLL, pitch, 0.0, t_prev );
	    //get t_prev using X, Y and YAW
            pcl::getTransformation ( X, Y, Z, 0.0, 0.0, YAW, t );
            pcl::transformPointCloud ( *cloud_next_filtered, *cloud_pitch, t_prev );
            gettimeofday(&endtrans,NULL);
            // Floor Clustering
            clustering_floor( &cloud_pitch );
            gettimeofday(&tfloor,NULL);
            gettimeofday(&tplane,NULL);
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cluster_prev;
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cluster_trans;
            OBJ obj_img_trans[40];
	    // extract clusters from the image
            clustering( cloud_pitch, obj_img_trans, &cluster_prev );
            gettimeofday(&endclus,NULL);
	    //transforming point cloud using t
            pcl::transformPointCloud ( *cloud_pitch, *cloud_next_trans_filtered, t );
            gettimeofday(&endtrans2,NULL);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pre_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
            int rest_next, human_next, smr_next, matchcluster;
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pos_human_next;
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pos_smr_next;
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pos_rest_next;
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > matched_cluster;
	    //object recognition
            object_classification ( obj_img_trans, cluster_prev, cloud_next_trans_filtered, &pos_human_next, &pos_smr_next, &pos_rest_next,  &matched_cluster,
                                    &static_cluster, &human_next, &smr_next, &rest_next, &matchcluster, color_cluster );
            printf("THE NEXT FRAME HAS BEEN STUDIED. %d HUMANS, %d SMRS and %d OTHER OBJECTS WERE FOUND. %d cluster were recognized\n",
                   human_next, smr_next, rest_next, matchcluster );
            gettimeofday(&endobj,NULL);

            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cluster_matched;
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > plane_matched;
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > matched_next;
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > new_smr;
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > new_human;
            int match;
	    //object correlation
            matching ( &match, human_next, smr_next, cloud_next_trans_filtered, &matched_next, &new_smr, &new_human, pos_human_next,
                       pos_smr_next, color_matched, color_new_smr, color_new_human );

            gettimeofday(&endma,NULL);

            if (1) 
            {
                viewer->removeAllPointClouds();
                removeAllText3D ( add, matchcluster, viewer );
            }
            gettimeofday(&endtotal,NULL);
            int added;
	    //visualization of all objects, but statics
            object_visualization ( viewer, cloud_next_trans_filtered, matched_next, matched_cluster, new_smr, new_human, pos_rest_next,
                               color_matched, color_cluster, color_new_smr, color_new_human, &added );
            visualize_static ( viewer, cloud_next_trans_filtered );
	    //image with the whole track represented (1 out of 4 recognitions per human)
	    //visualize_front_page ( viewer, cloud_next_trans_filtered );
            IMAGES_STUDIED++;
            gettimeofday(&endvis,NULL);
            add = added;
            rest_ini = rest_next;
            human_ini = human_next;
            smr_ini = smr_next;

            //time calculation
            gettimeofday(&end_it,NULL);
            dTimeItIni = ((double)ini_it.tv_sec)+((double)ini_it.tv_usec)/1000000;
            dTimeGet = ((double)endget.tv_sec)+((double)endget.tv_usec)/1000000;
            dTimeDown = ((double)enddown.tv_sec)+((double)enddown.tv_usec)/1000000;
            dTimeTrans = ((double)endtrans.tv_sec)+((double)endtrans.tv_usec)/1000000;
            dTimeFloor = ((double)tfloor.tv_sec)+((double)tfloor.tv_usec)/1000000;
            dTimeClus = ((double)endclus.tv_sec)+((double)endclus.tv_usec)/1000000;
            dTimeTrans2 = ((double)endtrans2.tv_sec)+((double)endtrans2.tv_usec)/1000000;
            dTimeObj = ((double)endobj.tv_sec)+((double)endobj.tv_usec)/1000000;
            dTimeMatching = ((double)endma.tv_sec)+((double)endma.tv_usec)/1000000;
            dTimeVis = ((double)endvis.tv_sec)+((double)endvis.tv_usec)/1000000;
            dTimeItEnd = ((double)end_it.tv_sec)+((double)end_it.tv_usec)/1000000;

            update_file_time_calculation(1);
            printf("time: GetInfo; Downsampling; Transf pitch; Floor; Clustering; Transf 2; Obj Class; Matching; Visual; Copy; Total; = [%f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f]\n",
                   dTimeGet-dTimeItIni,dTimeDown-dTimeGet,dTimeTrans-dTimeDown,dTimeFloor-dTimeTrans,dTimeClus-dTimeFloor,
                   dTimeTrans2-dTimeClus,dTimeObj-dTimeTrans2,dTimeMatching-dTimeObj,dTimeVis-dTimeMatching,dTimeItEnd-dTimeVis,dTimeItEnd-dTimeItIni);

            printf("HUMANCOUNT: %d, SMRCOUNT: %d, CLUSTERCOUNT: %d, TOTALCLUSTERCOUND: %d STATIC: %d TIMESHUMAN: %d, TIMESSMR: %d\n", humancount,
                   smrcount, clustercount, totalcluster, STATIC, times_human, times_smr);
            printf("\n IMAGES STUDIED : %d \n",IMAGES_STUDIED);
            //updating viewer
	    viewer->spinOnce(100);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void UFuncTracking::downsampling ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ini , pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered )
{
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_ini);
    vg.setLeafSize (lxyz, lxyz, lxyz);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering with leaf 0.03 has: " << cloud_filtered->points.size ()  << " data points." << std::endl;
}

void UFuncTracking::estimating_normals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals )
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.06);

    // Compute the features
    ne.compute (*cloud_normals);
}

void UFuncTracking::estimating_normals_human (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals )
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.09);
    printf("Normals human calculated\n");

    // Compute the features
    ne.compute (*cloud_normals);
}

void UFuncTracking::estimating_normals_smr (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals )
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.06);

    // Compute the features
    ne.compute (*cloud_normals);
}


void UFuncTracking::estimating_vfh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

    estimating_normals(cloud,normals);
    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (normals);
    // alternatively, if cloud is of tpe PointNormal, do vfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    vfh.setSearchMethod (tree);

    // Compute the features
    vfh.compute (vfhs.operator*());
}

void UFuncTracking::estimating_vfh_human(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

    estimating_normals_human(cloud,normals);
    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (normals);
    // alternatively, if cloud is of tpe PointNormal, do vfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    vfh.setSearchMethod (tree);

    // Compute the features
    vfh.compute (vfhs.operator*());
}

void UFuncTracking::estimating_vfh_smr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

    estimating_normals_smr(cloud,normals);
    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (normals);
    // alternatively, if cloud is of tpe PointNormal, do vfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    vfh.setSearchMethod (tree);

    // Compute the features
    vfh.compute (vfhs.operator*());
}

void UFuncTracking::clustering_floor ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr *cloud_filtered )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.08);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle (0.08);

    seg.setInputCloud (*cloud_filtered);
    seg.segment (*inliers, *coefficients);

    float iter=0.08;
    float angle=0.08;
    while ( (inliers->indices.size() == 0) || (iter == 0.2) )
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset. Try again.");
        iter=iter+0.01;
        angle=angle+0.01;
        printf("New iter: %f and angle: %f\n",iter,angle);
        seg.setDistanceThreshold (iter);
        seg.setEpsAngle (angle);
        seg.setInputCloud (*cloud_filtered);
        seg.segment (*inliers, *coefficients);
    }

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (*cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;
    pcl::getMinMax3D(**cloud_filtered,min_pt,max_pt);
    printf("Lowest point in the cloud: %f, Highest point in the cloud: %f\n",min_pt.z,max_pt.z);
    // Write the planar inliers to disk
    extract.filter (*cloud_plane);
    pcl::getMinMax3D(*cloud_plane,min_pt,max_pt);
    printf("Floor plane at initial height: %f, Floor plane at final height: %f\n",max_pt.z,min_pt.z);
    printf("Floor plane at initial distance: %f, Floor plane at final distance: %f\n",min_pt.x,max_pt.x);
    if (max_pt.z<-1.2)
        FLOOR = max_pt.z;
    std::cout << "PointCloud representing the floor plane: " << cloud_plane->points.size () << " data points." << std::endl;
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = cloud_f;
}

void UFuncTracking::clustering ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered , OBJ obj_img[] ,
                                 std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *cluster )
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.1);

    pcl::PointXYZRGB min_pt;
    pcl::PointXYZRGB max_pt;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.05); // 5 cm
    ec.setMinClusterSize (80);
    ec.setMaxClusterSize (1700);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);
    pcl::getMinMax3D(*cloud_filtered,min_pt,max_pt);

    double width,height;
    if ((min_pt.z<0.0)&(max_pt.z>=0.0))
        height=fabs(max_pt.z)+fabs(min_pt.z);
    else
        height=fabs(max_pt.z-min_pt.z);

    int c = 0;
    cluster_detected = 0;
    cluster_processed = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cluster_detected++;
        pcl::getMinMax3D(*cloud_cluster,min_pt,max_pt);

        // calculation of width and height
        if ((min_pt.y>0.0)&(max_pt.y<=0.0))
            width = fabs(max_pt.y)+fabs(min_pt.y);
        else
            width = fabs(min_pt.y-max_pt.y);
        if ((min_pt.z<0.0)&(max_pt.z>=0.0))
            height = fabs(max_pt.z)+fabs(min_pt.z);
        else
            height = fabs(max_pt.z-min_pt.z);
        if ( (width < max_width) & (width > 0.05) & (height > 0.05) & (max_pt.x-min_pt.x>0.05) & (max_pt.x-min_pt.x<0.7) &
                ( (min_pt.z<FLOOR+0.2)||((height>0.8)&(min_pt.z<FLOOR+0.55)) ) & (min_pt.x<7.0) )
        {
            bool exist= false;
            if ( STATIC > 0 )
            {
                int st = 1;
                while ( (st<=STATIC) & (!exist) )
                {
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(*cloud_cluster,centroid);
                    if ( (centroid.x()<STD[st].COG.x()+0.1) & (centroid.x()>STD[st].COG.x()-0.1) & (centroid.y()<STD[st].COG.y()+0.1) &
                            (centroid.y()>STD[st].COG.y()-0.1) & (centroid.z()<STD[st].COG.z()+0.1) & (centroid.z()>STD[st].COG.z()-0.1) &
                            (!STD[st].empty) & (cluster_color(cloud_cluster)>STD[st].colorH-0.05) &
                            (cluster_color(cloud_cluster)<STD[st].colorH+0.05) )
                    {
                        exist=true;
                        STD[st].t.Now();
                        STD[st].x = min_pt.x;
                        STD[st].y = min_pt.y;
                        STD[st].z = min_pt.z;
                        STD[st].detections++;
                        pcl::compute3DCentroid(*cloud_cluster,STD[st].COG);
                        update_file_static_position(st);
                        STD[st].colorH = cluster_color(cloud_cluster);
                        STD[st].colorS = cluster_colorS(cloud_cluster);
                        update_file_static_color(st);
                        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
                        printf("It is the static cluster: %d detected: %d\n",st,STD[st].detections);
                    }
                    st++;
                }
            }
            if ( !exist )
            {
                obj_img[c].width = width;
                obj_img[c].height = height;
                obj_img[c].baseheight = min_pt.z;
                obj_img[c].top = max_pt.z;
                obj_img[c].basewidth_left = max_pt.y;
                obj_img[c].basewidth_right = min_pt.y;
                obj_img[c].dist = min_pt.x;
                obj_img[c].depth = max_pt.x-min_pt.x;
                std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
                printf( "Width(x):%f height(y):%f located at: %f meters at high: %f and top: %f, left position is: %f, right is: %f and depth is: %f\n",
                        obj_img[c].width, obj_img[c].height, obj_img[c].dist, obj_img[c].baseheight, obj_img[c].top, obj_img[c].basewidth_left,
                        obj_img[c].basewidth_right,obj_img[c].depth);
                cluster->push_back(cloud_cluster);
                c++;
            }
        }
        else
        {
        }
    }
    obj_img[0].size = c;
    cluster_processed = c;
    printf("Number of clusters:%d and cluster count: %d\n",obj_img[0].size,clustercount);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UFuncTracking::
RGB2HSV (int r, int g, int b, float& fh, float& fs, float& fv)
{
    // mostly copied from opencv-svn/modules/imgproc/src/color.cpp
    const int hsv_shift = 12;

    static const int div_table[] = {
        0, 1044480, 522240, 348160, 261120, 208896, 174080, 149211,
        130560, 116053, 104448, 94953, 87040, 80345, 74606, 69632,
        65280, 61440, 58027, 54973, 52224, 49737, 47476, 45412,
        43520, 41779, 40172, 38684, 37303, 36017, 34816, 33693,
        32640, 31651, 30720, 29842, 29013, 28229, 27486, 26782,
        26112, 25475, 24869, 24290, 23738, 23211, 22706, 22223,
        21760, 21316, 20890, 20480, 20086, 19707, 19342, 18991,
        18651, 18324, 18008, 17703, 17408, 17123, 16846, 16579,
        16320, 16069, 15825, 15589, 15360, 15137, 14921, 14711,
        14507, 14308, 14115, 13926, 13743, 13565, 13391, 13221,
        13056, 12895, 12738, 12584, 12434, 12288, 12145, 12006,
        11869, 11736, 11605, 11478, 11353, 11231, 11111, 10995,
        10880, 10768, 10658, 10550, 10445, 10341, 10240, 10141,
        10043, 9947, 9854, 9761, 9671, 9582, 9495, 9410,
        9326, 9243, 9162, 9082, 9004, 8927, 8852, 8777,
        8704, 8632, 8561, 8492, 8423, 8356, 8290, 8224,
        8160, 8097, 8034, 7973, 7913, 7853, 7795, 7737,
        7680, 7624, 7569, 7514, 7461, 7408, 7355, 7304,
        7253, 7203, 7154, 7105, 7057, 7010, 6963, 6917,
        6872, 6827, 6782, 6739, 6695, 6653, 6611, 6569,
        6528, 6487, 6447, 6408, 6369, 6330, 6292, 6254,
        6217, 6180, 6144, 6108, 6073, 6037, 6003, 5968,
        5935, 5901, 5868, 5835, 5803, 5771, 5739, 5708,
        5677, 5646, 5615, 5585, 5556, 5526, 5497, 5468,
        5440, 5412, 5384, 5356, 5329, 5302, 5275, 5249,
        5222, 5196, 5171, 5145, 5120, 5095, 5070, 5046,
        5022, 4998, 4974, 4950, 4927, 4904, 4881, 4858,
        4836, 4813, 4791, 4769, 4748, 4726, 4705, 4684,
        4663, 4642, 4622, 4601, 4581, 4561, 4541, 4522,
        4502, 4483, 4464, 4445, 4426, 4407, 4389, 4370,
        4352, 4334, 4316, 4298, 4281, 4263, 4246, 4229,
        4212, 4195, 4178, 4161, 4145, 4128, 4112, 4096
    };
    int hr = 180, hscale = 15;
    int h, s, v = b;
    int vmin = b, diff;
    int vr, vg;

    v = std::max<int> (v, g);
    v = std::max<int> (v, r);
    vmin = std::min<int> (vmin, g);
    vmin = std::min<int> (vmin, r);

    diff = v - vmin;
    vr = v == r ? -1 : 0;
    vg = v == g ? -1 : 0;

    s = diff * div_table[v] >> hsv_shift;
    h = (vr & (g - b)) +
        (~vr & ((vg & (b - r + 2 * diff))
                + ((~vg) & (r - g + 4 * diff))));
    h = (h * div_table[diff] * hscale +
         (1 << (hsv_shift + 6))) >> (7 + hsv_shift);

    h += h < 0 ? hr : 0;
    fh = h / 180.0;
    fs = s / 255.0;
    fv = v / 255.0;
}

void UFuncTracking::dominant_color ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , float *HMean, float *SMean)
{
    uint32_t r, g, b;
    float H , S , V , HSum=0.0, SSum=0.0, VSum=0.0;
    for (size_t i=0;i<cloud->points.size();i++)
    {
        uint32_t rgb = *reinterpret_cast<int*> ( &cloud->points[i].rgb ); 
        r= ( rgb >> 16 ) & 0x0000ff;
        g= ( rgb >> 8 )  & 0x0000ff;
        b= ( rgb )       & 0x0000ff;

        RGB2HSV ( r, g, b , H , S , V );
        HSum= HSum + H;
        SSum= SSum + S;
        VSum= VSum + V;
    }
    *HMean= float(HSum) / float(cloud->points.size());
    *SMean= float(SSum) / float(cloud->points.size());
}

float UFuncTracking::cluster_color ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud )
{
    uint32_t r, g, b;
    float H , S , V , HSum=0.0, SSum=0.0, VSum=0.0;
    for (size_t i=0;i<cloud->points.size();i++)
    {
        uint32_t rgb = *reinterpret_cast<int*> ( &cloud->points[i].rgb );
        r= ( rgb >> 16 ) & 0x0000ff;
        g= ( rgb >> 8 )  & 0x0000ff;
        b= ( rgb )       & 0x0000ff;

        RGB2HSV ( r, g, b , H , S , V );
        HSum= HSum + H;
        SSum= SSum + S;
        VSum= VSum + V;
    }
    return float(HSum)/float(cloud->points.size());
}


float UFuncTracking::cluster_colorS ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud )
{
    uint32_t r, g, b;
    float H , S , V , HSum=0.0, SSum=0.0, VSum=0.0;
    for (size_t i=0;i<cloud->points.size();i++)
    {
        uint32_t rgb = *reinterpret_cast<int*> ( &cloud->points[i].rgb );
        r= ( rgb >> 16 ) & 0x0000ff;
        g= ( rgb >> 8 )  & 0x0000ff;
        b= ( rgb )       & 0x0000ff;

        RGB2HSV ( r, g, b , H , S , V );
        HSum= HSum + H;
        SSum= SSum + S;
        VSum= VSum + V;
    }
    return float(SSum)/float(cloud->points.size());
}

/** \brief Loads an n-D histogram file as a VFH signature
  * \param path the input file name
  * \param vfh the resultant VFH model
**/
bool UFuncTracking::loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{
    int vfh_idx;
    // Load the file as a PCD
    try
    {
        sensor_msgs::PointCloud2 cloud;
        int version;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        pcl::PCDReader r;
        int type;
        int idx; //unsigned int idx;
        r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

        vfh_idx = pcl::getFieldIndex (cloud, "vfh");
        if (vfh_idx == -1)
            return (false);
        if ((int)cloud.width * cloud.height != 1)
            return (false);
    }
    catch (pcl::InvalidConversionException e)
    {
        return (false);
    }

    // Treat the VFH signature as a single Point Cloud
    pcl::PointCloud <pcl::VFHSignature308> point;
    pcl::io::loadPCDFile (path.string (), point);
    vfh.second.resize (308);

    std::vector <sensor_msgs::PointField> fields;
    getFieldIndex (point, "vfh", fields);

    for (size_t i = 0; i < fields[vfh_idx].count; ++i)
    {
        vfh.second[i] = point.points[0].histogram[i];
    }
    vfh.first = path.string ();
    return (true);
}

/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */

inline void UFuncTracking::
nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model,
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
    // Query point
    flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
    memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

    indices = flann::Matrix<int>(new int[k], 1, k);
    distances = flann::Matrix<float>(new float[k], 1, k);
    index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
    delete[] p.ptr ();
}

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */

bool UFuncTracking::loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
    ifstream fs;
    fs.open (filename.c_str ());
    if (!fs.is_open () || fs.fail ())
        return (false);

    std::string line;
    while (!fs.eof ())
    {
        getline (fs, line);
        if (line.empty ())
            continue;
        vfh_model m;
        m.first = line;
        models.push_back (m);
    }
    fs.close ();
    return (true);
}

void UFuncTracking::descriptor_smr ( int k , vfh_model histogram , flann::Matrix<int> &k_indices , flann::Matrix<float> &k_distances )
{
    std::string kdtree_idx_file_name = "/data/smr/kdtree_smr.idx";
    std::string training_data_h5_file_name = "/data/smr/training_smr.h5";
    std::string training_data_list_file_name = "/data/smr/training_smr.list";

    std::vector<vfh_model> models;
    flann::Matrix<float> data;

    loadFileList (models, training_data_list_file_name);
    flann::load_from_file (data, training_data_h5_file_name, "/data/smr/training_smr");
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("/data/smr/kdtree_smr.idx"));
    index.buildIndex ();
    nearestKSearch (index, histogram, k, k_indices, k_distances);
}

void UFuncTracking::descriptor_human ( int k , vfh_model histogram , flann::Matrix<int> &k_indices , flann::Matrix<float> &k_distances )
{
    std::string kdtree_idx_file_name = "/data/human/kdtree_human.idx";
    std::string training_data_h5_file_name = "/data/human/training_human.h5";
    std::string training_data_list_file_name = "/data/human/training_human.list";

    std::vector<vfh_model> models;
    flann::Matrix<float> data;

    loadFileList (models, training_data_list_file_name);
    flann::load_from_file (data, training_data_h5_file_name, "/data/human/training_data");
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("/data/human/kdtree_human.idx"));
    index.buildIndex ();
    nearestKSearch (index, histogram, k, k_indices, k_distances);
}

void UFuncTracking::visualization_neighbors ( flann::Matrix<int> &k_indices , flann::Matrix<float> &k_distances )
{
    // Output the results on screen
    int k=6;
    pcl::console::print_highlight ("The closest %d neighbors for are:\n", k);
    for (int i = 0; i < k; ++i)
        pcl::console::print_info ("    %d - (%d) with a distance of: %f\n", i, k_indices[0][i], k_distances[0][i]);
    // Load the results
    int y_s = (int)floor (sqrt ((double)k));
    int x_s = y_s + (int)ceil ((k / (double)y_s) - y_s);
    double x_step = (double)(1 / (double)x_s);
    double y_step = (double)(1 / (double)y_s);
    pcl::console::print_highlight ("Preparing to load ");
    pcl::console::print_value ("%d", k);
    pcl::console::print_info (" files (");
    pcl::console::print_value ("%d", x_s);
    pcl::console::print_info ("x");
    pcl::console::print_value ("%d", y_s);
    pcl::console::print_info (" / ");
    pcl::console::print_value ("%f", x_step);
    pcl::console::print_info ("x");
    pcl::console::print_value ("%f", y_step);
    pcl::console::print_info (")\n");
}

/* \brief Load a set of VFH features that will act as the model (training data)
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param extension the file extension containing the VFH features
  * \param models the resultant vector of histogram models
  */

void UFuncTracking::loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, std::vector<vfh_model> &models)
{
    if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
        return;

    for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
    {
        if (boost::filesystem::is_directory (it->status ()))
        {
            std::stringstream ss;
            ss << it->path ();
            pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
            loadFeatureModels (it->path (), extension, models);
        }
        if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
        {
            vfh_model m;
            if (loadHist (base_dir / it->path ().filename (), m))
                models.push_back (m);
        }
    }
}

void UFuncTracking::build_tree_smr ( const boost::filesystem::path &path )
{
    remove("data/smr/training_smr.h5");
    std::string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
    std::string kdtree_idx_file_name = "data/smr/kdtree_smr.idx";;
    std::string training_data_h5_file_name = "data/smr/training_smr.h5";
    std::string training_data_list_file_name = "data/smr/training_smr.list";
    std::vector<vfh_model> models;

    // Load the model histograms
    loadFeatureModels ( path, extension, models );

    // Convert data into FLANN format
    flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

    for (size_t i = 0; i < data.rows; ++i)
        for (size_t j = 0; j < data.cols; ++j)
            data[i][j] = models[i].second[j];

    // Save data to disk (list of models)
    flann::save_to_file (data, training_data_h5_file_name, "training_smr");
    std::ofstream fs;
    fs.open (training_data_list_file_name.c_str ());
    for (size_t i = 0; i < models.size (); ++i)
        fs << models[i].first << "\n";
    fs.close ();

    // Build the tree index and save it to disk
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
    index.buildIndex ();
    index.save (kdtree_idx_file_name);
    delete[] data.ptr ();
}

void UFuncTracking::build_tree_human ( const boost::filesystem::path &path )
{
    remove("data/human/training_human.h5");
    std::string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
    std::string kdtree_idx_file_name = "data/human/kdtree_human.idx";
    std::string training_data_h5_file_name = "data/human/training_human.h5";
    std::string training_data_list_file_name = "data/human/training_human.list";
    std::vector<vfh_model> models;
    // Load the model histograms
    loadFeatureModels ( path, extension, models );

    // Convert data into FLANN format
    flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

    for (size_t i = 0; i < data.rows; ++i)
        for (size_t j = 0; j < data.cols; ++j)
            data[i][j] = models[i].second[j];

    // Save data to disk (list of models)
    flann::save_to_file (data, training_data_h5_file_name, "training_human");
    std::ofstream fs;
    fs.open (training_data_list_file_name.c_str ());
    for (size_t i = 0; i < models.size (); ++i)
        fs << models[i].first << "\n";
    fs.close ();

    // Build the tree index and save it to disk
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
    index.buildIndex ();
    index.save (kdtree_idx_file_name);
    delete[] data.ptr ();
}

void UFuncTracking::build_tree_cluster ( const boost::filesystem::path &path , int i)
{
    std::string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
    std::stringstream kdt;
    kdt << "data/cluster_" << i << "/kdtree_cluster_" << i << ".idx";
    std::stringstream h5;
    h5 << "data/cluster_" << i << "/training_cluster_" << i << ".h5";
    std::stringstream list;
    list << "data/cluster_" << i << "/training_cluster_" << i << ".list";
    std::string filename= h5.str();
    std::remove(filename.c_str());
    std::string kdtree_idx_file_name = kdt.str ();
    std::string training_data_h5_file_name = h5.str ();
    std::string training_data_list_file_name = list.str ();
    std::vector<vfh_model> models;
    // Load the model histograms
    loadFeatureModels ( path, extension, models );

    // Convert data into FLANN format
    flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

    for (size_t m = 0; m < data.rows; ++m)
        for (size_t n = 0; n < data.cols; ++n)
            data[m][n] = models[m].second[n];

    // Save data to disk (list of models)
    std::stringstream train;
    train << "training_cluster_"<< i;
    flann::save_to_file (data, training_data_h5_file_name, train.str());
    std::ofstream fs;
    fs.open (training_data_list_file_name.c_str ());
    for (size_t m = 0; m < models.size (); ++m)
        fs << models[m].first << "\n";
    fs.close ();

    // Build the tree index and save it to disk
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
    index.buildIndex ();
    index.save (kdtree_idx_file_name);

    delete[] data.ptr ();
}

void UFuncTracking::build_tree_static ( const boost::filesystem::path &path , int i)
{
    std::string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
    std::stringstream kdt;
    kdt << "data/static_" << i << "/kdtree_static_" << i << ".idx";
    std::stringstream h5;
    h5 << "data/static_" << i << "/training_static_" << i << ".h5";
    std::stringstream list;
    list << "data/static_" << i << "/training_static_" << i << ".list";
    std::string filename= h5.str();
    std::remove(filename.c_str());
    std::string kdtree_idx_file_name = kdt.str ();
    std::string training_data_h5_file_name = h5.str ();
    std::string training_data_list_file_name = list.str ();
    std::vector<vfh_model> models;
    // Load the model histograms
    loadFeatureModels ( path, extension, models );

    // Convert data into FLANN format
    flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

    for (size_t m = 0; m < data.rows; ++m)
        for (size_t n = 0; n < data.cols; ++n)
            data[m][n] = models[m].second[n];

    // Save data to disk (list of models)
    std::stringstream train;
    train << "training_static_"<< i;
    flann::save_to_file (data, training_data_h5_file_name, train.str());
    std::ofstream fs;
    fs.open (training_data_list_file_name.c_str ());
    for (size_t m = 0; m < models.size (); ++m)
        fs << models[m].first << "\n";
    fs.close ();

    // Build the tree index and save it to disk
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
    index.buildIndex ();
    index.save (kdtree_idx_file_name);

    delete[] data.ptr ();
}



void UFuncTracking::nearest_neighbors_smr ( const boost::filesystem::path &path , flann::Matrix<int> &k_indices , flann::Matrix<float> &k_distances)
{
    int k = 1;

    std::string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

    vfh_model histogram;
    loadHist (path, histogram);

    flann::Matrix<float> data;
    std::vector<vfh_model> models;

    std::string kdtree_idx_file_name = "data/smr/kdtree_smr.idx";
    std::string training_data_h5_file_name = "data/smr/training_smr.h5";
    std::string training_data_list_file_name = "data/smr/training_smr.list";

    loadFileList (models, training_data_list_file_name);
    flann::load_from_file (data, training_data_h5_file_name, "training_smr");

    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("data/smr/kdtree_smr.idx"));
    index.buildIndex ();
    nearestKSearch (index, histogram, k, k_indices, k_distances);
}

void UFuncTracking::nearest_neighbors_human ( const boost::filesystem::path &path , flann::Matrix<int> &k_indices ,
        flann::Matrix<float> &k_distances )
{
    int k = 1;

    std::string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

    vfh_model histogram;
    loadHist (path, histogram);

    flann::Matrix<float> data;
    std::vector<vfh_model> models;

    std::string kdtree_idx_file_name = "data/human/kdtree_human.idx";
    std::string training_data_h5_file_name = "data/human/training_human.h5";
    std::string training_data_list_file_name = "data/human/training_human.list";

    loadFileList (models, training_data_list_file_name);
    flann::load_from_file (data, training_data_h5_file_name, "training_human");

    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("data/human/kdtree_human.idx"));
    index.buildIndex ();
    nearestKSearch (index, histogram, k, k_indices, k_distances);

    if ( k_distances[0][0] < 70.0 )
        update_file_human_match(models,k_indices,k_distances);
}

void UFuncTracking::nearest_neighbors_cluster ( const boost::filesystem::path &path , pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhsi,
        pcl::PointCloud<pcl ::PointXYZRGB>::Ptr cloud ,	pcl::PointCloud<pcl ::PointXYZRGB>::Ptr big_cloud,
        flann::Matrix<int> &k_indices, flann::Matrix<float> &k_distances, int place, int *pos )
{
    int k = 1;

    std::string extension (".pcd");
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

    vfh_model histogram;
    loadHist (path, histogram);

    flann::Matrix<float> data;
    std::vector<vfh_model> models;
    int i=1;
    bool found=false;
    // instead of 3 should be the number of kdtree created
    if ( STATIC > 0 )
    {
        while ( (i <= STATIC) & (!found) )
        {
            std::stringstream rst;
            rst << "data/static_" <<  i << "/kdtree_static_" << i << ".idx";
            std::stringstream rsst;
            rsst << "data/static_" << i << "/training_static_" << i << ".h5";
            std::stringstream fst;
            fst << "data/static_" << i << "/training_static_" << i << ".list";
            std::string kdtree_idx_file_name = rst.str();
            std::string training_data_h5_file_name = rsst.str();
            std::string training_data_list_file_name = fst.str();
            loadFileList (models, training_data_list_file_name);
            std::stringstream traint;
            traint << "training_static_"<< i;
            flann::load_from_file (data, training_data_h5_file_name, traint.str());

            flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams (rst.str()));
            index.buildIndex ();
            nearestKSearch (index, histogram, k, k_indices, k_distances);
            if ( (k_distances[0][0] < thresh_static) & (k_distances[0][1] < thresh_static) & (cluster_color(cloud)<STD[i].colorH+0.04) &
                    (cluster_color(cloud)>STD[i].colorH-0.04) )
            {
                static_size[i]++;
                STD[i].detections++;
                STD[i].empty = false;
                printf("The cluster: %d is the Static Cluster %d with k_distance1: %f and k_distance2: %f DETECTED: %d TIMES\n",place,i,k_distances[0][0],
                       k_distances[0][1],STD[i].detections);
                found = true;
                *pos=101;
                STD[i].cloud = cloud;
                STD[i].t.Now();
                pcl::compute3DCentroid (*cloud, STD[i].COG);
                //if already 5 descriptors stored, no need for more
                if ( STD[i].detections < 20 )
                {
                    std::stringstream st;
                    st << "data/static_" << i << "/cluster_" << STD[i].detections << ".pcd";
                    pcl::io::savePCDFile(st.str (),*cloud);
                    std::stringstream sst;
                    sst << "data/static_" << i << "/cluster_" << STD[i].detections << "_vfh.pcd";
                    pcl::io::savePCDFile(sst.str (),*vfhsi);
                    std::stringstream ft;
                    ft << "data/static_" << i;
                    build_tree_static(ft.str(),i);
                }
            }
            else
            {
            }
            i=i+1;
        }
    }
    if ( !found )
    {
        i = 1;
        while ( (i < clustercount) & (!found) & (!CD[i].empty) )
        {
            std::stringstream check;
            check << "data/cluster_" << i;
            struct stat st;
            std::string oldstring = check.str();
            const char *nameold = oldstring.c_str();
            if (stat(nameold,&st) == 0)
            {
                std::stringstream rs;
                rs << "data/cluster_" << i << "/kdtree_cluster_" << i << ".idx";
                std::stringstream rss;
                rss << "data/cluster_" << i << "/training_cluster_" << i << ".h5";
                std::stringstream fs;
                fs << "data/cluster_" << i << "/training_cluster_" << i << ".list";

                std::string kdtree_idx_file_name = rs.str();
                std::string training_data_h5_file_name = rss.str();
                std::string training_data_list_file_name = fs.str();

                loadFileList (models, training_data_list_file_name);
                std::stringstream train;
                train << "training_cluster_"<< i;
                flann::load_from_file (data, training_data_h5_file_name, train.str());

                flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams (rs.str()));
                index.buildIndex ();
                nearestKSearch (index, histogram, k, k_indices, k_distances);
                if ( (k_distances[0][0] < thresh_cluster) & (cluster_color(cloud)<CD[i].colorH+0.04) & (cluster_color(cloud)>CD[i].colorH-0.04) )
                {
                    pcl::PointXYZRGB min, max;
                    pcl::getMinMax3D(*cloud,min,max);
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid (*cloud, centroid);
                    if ( (centroid.x()<CD[i].COG.x()+0.1) & (centroid.x()>CD[i].COG.x()-0.1) & (centroid.y()<CD[i].COG.y()+0.1) &
                            (centroid.y()>CD[i].COG.y()-0.1) & (centroid.z()<CD[i].COG.z()+0.1) & (centroid.z()>CD[i].COG.z()-0.1) &
                            (!STD[i].empty) )
                    {
                        CD[i].detections++;
                        CD[i].t.Now();
                        *pos=i;
                        if ( (CD[i].detections > 3) & (k_distances[0][1] < thresh_cluster) )
                        {
                            found = true;
                            STATIC++;
                            staticcount++;
                            STD[staticcount].x = cluster_pos[i].minx;
                            STD[staticcount].y = cluster_pos[i].miny;
                            STD[staticcount].z = cluster_pos[i].minz;
                            printf("X: %f Y: %f Z: %f\n", cluster_pos[i].minx, cluster_pos[i].miny, cluster_pos[i].minz);
                            printf("Cluster %d has been detected enough times to make it STATIC: %d, place: %d\n \n \n \n", i, STATIC, place);
                            STD[staticcount].cloud = cloud;
                            pcl::compute3DCentroid (*cloud, STD[staticcount].COG);
                            printf("Centroid x: %f y: %f z: %f\n",STD[staticcount].COG.x(),STD[staticcount].COG.y(),STD[staticcount].COG.z());
                            STD[staticcount].t.Now();
                            STD[staticcount].r = CD[i].r;
                            STD[staticcount].g = CD[i].g;
                            STD[staticcount].b = CD[i].b;
                            STD[staticcount].detections = CD[i].detections;
                            STD[staticcount].empty = false;
                            float H,S;
                            dominant_color(cloud,&H,&S);
                            STD[staticcount].colorH = H;
                            STD[staticcount].colorS = S;
                            update_file_static_position(staticcount);
                            update_file_static_color(staticcount);
                            std::stringstream f;
                            f << "data/static_" << STATIC;
                            std::string folder = f.str();
                            int result = rename(oldstring.c_str(),folder.c_str());
                            if (result == 0)
                                printf("File succesfully renamed.\n");
                            else
                                printf("Error renaming file.\n");
                            std::stringstream st;
                            st << "data/static_" << STATIC << "/cluster_5.pcd";
                            pcl::io::savePCDFile(st.str (),*cloud);
                            std::stringstream sst;
                            sst << "data/static_" << STATIC << "/cluster_5_vfh.pcd";
                            pcl::io::savePCDFile(sst.str (),*vfhsi);
                            build_tree_static(f.str(),STATIC);
                            CD[i].empty = true;
                            CD[i].detections = 0;
                        }
                        else
                        {
                            printf("Cluster %d has been recognized %d times now with k_distance: %f, place: %d and colorH : %f and colorS: %f\n",
                                   i,CD[i].detections,k_distances[0][0],place,CD[i].colorH,CD[i].colorS);
                            found=true;
                            std::stringstream sss;
                            sss << "data/cluster_" << i << "/cluster_" << CD[i].detections << ".pcd";
                            pcl::io::savePCDFile(sss.str (),*cloud);
                            std::stringstream ss;
                            ss << "data/cluster_" << i << "/cluster_" << CD[i].detections << "_vfh.pcd";
                            pcl::io::savePCDFile(ss.str (),*vfhsi);
                            std::stringstream f;
                            f << "data/cluster_" << i;
                            pcl::compute3DCentroid (*cloud, CD[i].COG);
                            update_file_cluster_position(i);
                            build_tree_cluster(f.str(),i);
                        }
                    }
                }
                else
                {
                }
            }
            else
                printf("Folder does not exist\n");
            i++;
        }
    }
    if (!found)
        *pos=100;
}

void UFuncTracking::visualize_static( pcl::visualization::PCLVisualizer *viewer, pcl::PointCloud<pcl ::PointXYZRGB>::Ptr cloud )
{

    for ( int c=1; c<=STATIC; c++ )
    {
        double dt=STD[c].t.getTimePassed();
        if ( dt > 8.0 )
        {
               STD[c].empty = true;
        }
    }
    for ( int j=1; j<=STATIC; j++)
    {
        if ( !STD[j].empty )
        {
            PointCloudColorHandlerCustom<PointT> cluster (cloud, STD[j].r, STD[j].g, STD[j].b);
            std::stringstream nnn;
            nnn << "static_" << j << "_pcd";
            viewer->addPointCloud ( STD[j].cloud, nnn.str () );
            printf("Static cluster :%d printed in color r: %d g: %d and b: %d\n",j,STD[j].r,STD[j].g,STD[j].b);
            printf("X: %f Y: %f Z: %f\n", STD[j].x, STD[j].y, STD[j].z);
        }
    }
}


void UFuncTracking::visualize_front_page( pcl::visualization::PCLVisualizer *viewer, pcl::PointCloud<pcl ::PointXYZRGB>::Ptr cloud )
{

    for ( int j=1; j<=humancount; j++)
    {
        PointCloudColorHandlerCustom<PointT> cluster (cloud, HP[j].r, HP[j].g, HP[j].b);
        printf("Suma: %d\n", HP[j].suma);
        for (int i=0; i<HP[j].suma; i++)
        {
            std::stringstream nnn;
            nnn << "human_" << j << "_" << i << "_pcd";
            if ( i%4 == 0 )
            {
                viewer->addPointCloud ( HP[j].cloud[i], cluster, nnn.str () );
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,nnn.str());
                printf("Human cluster :%d printed in color r: %d g: %d and b: %d\n",j,HP[j].r,HP[j].g,HP[j].b);
            }
        }
    }
}


void UFuncTracking::object_visualization( pcl::visualization::PCLVisualizer *viewer, pcl::PointCloud<pcl ::PointXYZRGB>::Ptr cloud,
                                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > matched_next,
                                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > matched_cluster,
                                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > smr_new,
                                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > human_new,
                                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cluster_new,
                                        color color_matched[], color color_cluster[], color color_new_smr[], color color_new_human[], int *added )
{
    int j=0;
    int count=1;
    viewer->addCoordinateSystem(0.1);
    printf("Sizes of STATIC CLUSTER: %d, MATCHED: %d, MATCHED CLUSTER: %d, NEW HUMANS: %d, NEW SMRs: %d\n",STATIC,
           matched_next.size(),matched_cluster.size(),human_new.size(),smr_new.size());
    for (int i=0;i<int(matched_next.size());i++)
    {
        PointCloudColorHandlerCustom<PointT> cluster (cloud, color_matched[count].r, color_matched[count].g, color_matched[count].b);
        pcl::PointXYZRGB min_pt;
        pcl::PointXYZRGB max_pt;
        std::stringstream nnn;
        nnn << "cluster_" << count << "_pcd";
        viewer->addPointCloud ( matched_next[i], cluster, nnn.str () );
        std::stringstream ss_next;
        ss_next << "MATCHED_" << count;
        pcl::getMinMax3D(*matched_next[i],min_pt,max_pt);
        viewer->addText3D( ss_next.str (), max_pt, 0.03, color_matched[count].r, color_matched[count].g, color_matched[count].b, ss_next.str() );
        printf("MATCHED:%d, count:%d, r:%d, g:%d, b:%d\n",i,count,color_matched[count].r,color_matched[count].g,color_matched[count].b);
        count++;
        j++;
    }
    int countc = 1;
    for (int i=0;i<int(matched_cluster.size());i++)
    {
        PointCloudColorHandlerCustom<PointT> cluster (cloud, color_cluster[countc].r, color_cluster[countc].g, color_cluster[countc].b);
        pcl::PointXYZRGB min_pt;
        pcl::PointXYZRGB max_pt;
        std::stringstream nnn;
        nnn << "cluster_" << count << "_pcd";
        viewer->addPointCloud ( matched_cluster[i], nnn.str () );
        std::stringstream ss_next;
        ss_next << "MATCHED_CLUSTER_" << countc;
        pcl::getMinMax3D(*matched_cluster[i],min_pt,max_pt);
        viewer->addText3D( ss_next.str (), max_pt, 0.03, color_cluster[countc].r, color_cluster[countc].g, color_cluster[countc].b, ss_next.str() );
        printf("CLUSTER MATCHED:%d, count:%d, r:%d, g:%d, b:%d\n",i,countc,color_cluster[countc].r,color_cluster[countc].g,color_cluster[countc].b);
        count++;
        countc++;
        j++;
    }
    int counts = 1;
    for ( int i=0; i<int(smr_new.size()); i++ )
    {
        PointCloudColorHandlerCustom<PointT> cluster (cloud, color_new_smr[counts].r, color_new_smr[counts].g, color_new_smr[counts].b);
        pcl::PointXYZRGB min_pt;
        pcl::PointXYZRGB max_pt;
        std::stringstream nnn;
        nnn << "cluster_" << count << "_pcd";
        viewer->addPointCloud ( smr_new[i], cluster, nnn.str () );
        std::stringstream ss_next;
        ss_next << "SMR_" << counts;
        pcl::getMinMax3D(*smr_new[i],min_pt,max_pt);
        viewer->addText3D( ss_next.str (), max_pt, 0.03, color_new_smr[counts].r, color_new_smr[counts].g, color_new_smr[counts].b, ss_next.str() );
        printf("SMR:%d, count:%d, r:%d, g:%d, b:%d\n",i,counts,color_new_smr[counts].r, color_new_smr[counts].g, color_new_smr[counts].b);
        j++;
        count++;
        counts++;
    }
    int counth = 1;
    for ( int i=0; i<int(human_new.size()); i++)
    {
        PointCloudColorHandlerCustom<PointT> cluster (cloud, color_new_human[counth].r, color_new_human[counth].g, color_new_human[counth].b);
        pcl::PointXYZRGB min_pt;
        pcl::PointXYZRGB max_pt;
        std::stringstream nnn;
        nnn << "cluster_" << count << "_pcd";
        viewer->addPointCloud ( human_new[i], cluster, nnn.str () );
        std::stringstream ss_next;
        ss_next << "HUMAN_" << counth;
        pcl::getMinMax3D(*human_new[i],min_pt,max_pt);
        viewer->addText3D( ss_next.str (), max_pt, 0.03, color_new_human[counth].r, color_new_human[counth].g, color_new_human[counth].b, ss_next.str() );
        printf("HUMAN:%d, count:%d, r:%d, g:%d, b:%d\n",i,count,color_new_human[counth].r, color_new_human[counth].g, color_new_human[counth].b);
        j++;
        count++;
        counth++;
    }
    *added = j;
    printf( "added: %d\n", *added);
}

void UFuncTracking:: call_tree_cluster ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhsi, int i )
{
    std::stringstream f;
    f << "data/cluster_" << i;
    std::string folder = f.str();
    const char *name = folder.c_str();
    mkdir(name,0777);
    std::stringstream rs;
    rs << "data/cluster_" << i << "/cluster_" << CD[i].detections << ".pcd";
    pcl::io::savePCDFile(rs.str (),*cluster);
    std::stringstream rss;
    rss << "data/cluster_" << i << "/cluster_" << CD[i].detections << "_vfh.pcd";
    pcl::io::savePCDFile(rss.str (),*vfhsi);
    std::stringstream fs;
    fs << "data/cluster_" << i;
    build_tree_cluster(fs.str(),i);
    pcl::PointXYZRGB min, max;
    pcl::getMinMax3D(*cluster,min,max);
    cluster_pos[i].minx = min.x;
    cluster_pos[i].miny = min.y;
    cluster_pos[i].minz = min.z;
}

void UFuncTracking::object_classification ( OBJ obj_img[], std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pre_cluster,
        pcl::PointCloud<pcl ::PointXYZRGB>::Ptr cloud,
        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *pos_human,
        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *pos_smr,
        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *pos_rest,
        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *matched_cluster,
        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *static_cluster,
        int *human, int *smr, int *rest, int *matchcluster, color color_cluster[] )
{
    int i = 0;
    int chuman = 0;
    int csmr = 0;
    int crest = 0, matchedcluster = 0;
    int pos;
    int c=1;
    bool found=false;
    Eigen::Affine3f t;
    pcl::getTransformation ( X, Y, Z, 0.0, 0.0, YAW, t );
    while ( i < obj_img[0].size )
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*pre_cluster[i], *cluster, t);
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*pre_cluster[i],centroid);
        printf("PRE centroid x: %f, y: %f \n",centroid.x(),centroid.y());
        pcl::compute3DCentroid(*cluster,centroid);
        printf("POST centroid x: %f, y: %f \n",centroid.x(),centroid.y());
        printf("points cluster: %d\n", int(cluster->points.size()));
        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhsi (new pcl::PointCloud<pcl::VFHSignature308> ());
        flann::Matrix<float> k_distances;
        flann::Matrix<int> k_indices;
        printf("CLUSTER POINTS: %d Height: %f widht: %f, dist: %f\n",int(cluster->points.size()),obj_img[i].height,obj_img[i].width,obj_img[i].dist);
        if ( (obj_img[i].height < 0.45) & (obj_img[i].height > 0.05) & (obj_img[i].width < 0.45) & (obj_img[i].width > 0.05) & (obj_img[i].depth > 0.05) & (obj_img[i].baseheight < (FLOOR+0.2)) )
        {
            estimating_vfh_smr(cluster,vfhsi);
            pcl::io::savePCDFile("vfh1.pcd",*vfhsi);
            nearest_neighbors_smr( "vfh1.pcd", k_indices, k_distances);
            times_smr++;
            if ( (k_distances[0][0] < thresh_smr) && (k_distances[0][1] < thresh_smr) )
            {
                pos_smr->push_back(cluster);
                printf("Object %d is a SMR with the minimum distance: %f and %f\n",i,k_distances[0][0],k_distances[0][1]);
                csmr++;
                if ( (k_distances[0][0] > 10.0) & (k_distances[0][0] < thresh_smr_add) & (smr_desc < 200) )
                {
                    smr_desc++;
                    std::stringstream sss;
                    sss << "data/smr/smr_" << smr_desc << ".pcd";
                    pcl::io::savePCDFile(sss.str (),*cluster);
                    std::stringstream ss;
                    ss << "data/smr/smr_vfh_" << smr_desc << ".pcd";
                    pcl::io::savePCDFile(ss.str (),*vfhsi);
                    build_tree_smr("data/smr");
                }
            }
            else
            {
                printf("Object %d is NOT A SMR with the minimum distance: %f and %f\n",i,k_distances[0][0],k_distances[0][1]);
                estimating_vfh(cluster,vfhsi);
                pcl::io::savePCDFile("vfh1.pcd",*vfhsi);
                nearest_neighbors_cluster ( "vfh1.pcd", vfhsi, cluster, cloud, k_indices, k_distances, i, &pos );
                if ( (pos != 100) & (pos != 101) )
                {
                    matchedcluster++;
                    printf("Position:%d and clustercount:%d and STATIC:%d\n", pos, clustercount, STATIC);
                    matched_cluster->push_back(cluster);
                    color_cluster[matchedcluster].r = CD[pos].r;
                    color_cluster[matchedcluster].g = CD[pos].g;
                    color_cluster[matchedcluster].b = CD[pos].b;
                    CD[pos].t.Now();
                    CD[pos].colorH = cluster_color(cluster);
                    pcl::compute3DCentroid (*cluster, CD[pos].COG);
                }
                else if ( pos == 101 )
                {
                    //matched as static
                }
                else
                {
                    c=1;
                    found=false;
                    while ( (c<=clustercount) & (!found) )
                    {
                        double dt=CD[c].t.getTimePassed();
                        if ( dt > 7.0 )
                        {
                            CD[c].empty = true;
                            found=true;
                            std::stringstream del;
                            del << "data/cluster_" << c << "/";
                            boost::filesystem::remove_all(del.str());
                        }
                        c++;
                    }
                    float found=false;
                    int zz=1;
                    while ( (!found) & (zz<=clustercount) )
                    {
                        if (CD[zz].empty)
                        {
                            found = CD[zz].empty;
                        } else
                            zz++;
                    }
                    if (zz>clustercount)
                        clustercount++;
                    totalcluster++;
                    CD[zz].t.Now();
                    CD[zz].colorH = cluster_color(cluster);
                    CD[zz].colorS = cluster_colorS(cluster);
                    pcl::compute3DCentroid (*cluster, CD[zz].COG);
                    CD[zz].empty = false;
                    CD[zz].detections = 1;
                    assign_color_cluster(zz);
                    printf("New cluster CD[%d] created with color: %f and colorS: %f\n",zz,CD[zz].colorH,CD[zz].colorS);
                    call_tree_cluster(cluster,vfhsi,zz);
                }
            }
        }
        else if ( ((obj_img[i].height > 1.15)||((obj_img[i].height > 0.8)&(obj_img[i].dist < 2.0))) & (obj_img[i].height < 1.80) & (obj_img[i].width > 0.15) & (obj_img[i].depth > 0.05) & (obj_img[i].baseheight < FLOOR+0.35) )
        {
            printf("Object i:%d is maybe a human\n",i);
            estimating_vfh_human(cluster,vfhsi);
            pcl::io::savePCDFile("vfh1.pcd",*vfhsi);
            nearest_neighbors_human( "vfh1.pcd", k_indices, k_distances);
            if ( (k_distances[0][0] < thresh_human) || ( (obj_img[i].dist < 2.1) & (k_distances[0][0] < thresh_human) ) )
            {
                pos_human->push_back(cluster);
                printf("Object %d is a HUMAN with the minimum distance:%f and %f at distance: %f\n",i,k_distances[0][0],k_distances[0][1],obj_img[i].dist);
                chuman++;
                times_human++;
                std::stringstream h;
                h << "humanexample_" << times_human << ".pcd";
                if ( (k_distances[0][0] < thresh_human_add) & (obj_img[i].height > 1.30) & (k_distances[0][0] > 15.0) & (human_desc < 150) )
                {
                    human_desc++;
                    std::stringstream sss;
                    sss << "data/human/human_" << human_desc << ".pcd";
                    pcl::io::savePCDFile(sss.str (),*cluster);
                    std::stringstream ss;
                    ss << "data/human/human_" << human_desc << "_vfh.pcd";
                    pcl::io::savePCDFile(ss.str (),*vfhsi);
                    build_tree_human("data/human");
                }
            }
            else
            {
                printf("Object %d is NOT a HUMAN with the minimum distance:%f and %f at distance: %f\n",i,k_distances[0][0],k_distances[0][1],obj_img[i].dist);
                estimating_vfh(cluster,vfhsi);
                pcl::io::savePCDFile("vfh1.pcd",*vfhsi);
                nearest_neighbors_cluster ( "vfh1.pcd", vfhsi, cluster, cloud, k_indices, k_distances, i, &pos );
                if ( (pos != 100) & (pos != 101) )
                {
                    matchedcluster++;
                    printf("matchedcluster:%d and position:%d and clustercount:%d and STATIC: %d\n",matchedcluster,pos,clustercount,STATIC);
                    matched_cluster->push_back(cluster);
                    color_cluster[matchedcluster].r = CD[pos].r;
                    color_cluster[matchedcluster].g = CD[pos].g;
                    color_cluster[matchedcluster].b = CD[pos].b;
                    CD[pos].colorH = cluster_color(cluster);
                    CD[pos].t.Now();
                    pcl::compute3DCentroid (*cluster, CD[pos].COG);
                }
                else if ( pos == 101 )
                {
                    //matched as static
                }
                else
                {
                    int c=1;
                    bool found=false;
                    while ( (c<=clustercount) & (!found) )
                    {
                        double dt = CD[c].t.getTimePassed();
                        if ( dt > 7.0 )
                        {
                            CD[c].empty = true;
                            found = true;
                            std::stringstream del;
                            del << "data/cluster_" << c << "/";
                            boost::filesystem::remove_all(del.str());
                        }
                        c++;
                    }
                    found=false;
                    int zz=1;
                    while ( (!found) & (zz<=clustercount) )
                    {
                        if (CD[zz].empty)
                        {
                            found = CD[zz].empty;
                        } else
                            zz++;
                    }
                    if (zz>clustercount)
                        clustercount++;
                    assign_color_cluster(zz);
                    totalcluster++;
                    CD[zz].t.Now();
                    CD[zz].colorH = cluster_color(cluster);
                    CD[zz].colorS = cluster_colorS(cluster);
                    CD[zz].empty = false;
                    CD[zz].detections = 1;
                    pcl::compute3DCentroid (*cluster, CD[zz].COG);
                    printf("New cluster CD[%d] created with color: %f and colorS: %f\n",zz,CD[zz].colorH,CD[zz].colorS);
                    call_tree_cluster(cluster,vfhsi,zz);
                }
            }
        }
        else
        {
            estimating_vfh(cluster,vfhsi);
            pcl::io::savePCDFile("vfh1.pcd",*vfhsi);
            nearest_neighbors_cluster ( "vfh1.pcd", vfhsi, cluster, cloud, k_indices, k_distances, i, &pos );
            if ( (pos != 100) & (pos != 101) )
            {
                matchedcluster++;
                printf("matchedcluster:%d and position:%d and clustercount:%d and STATIC: %d\n",matchedcluster,pos,clustercount,STATIC);
                matched_cluster->push_back(cluster);
                color_cluster[matchedcluster].r=CD[pos].r;
                color_cluster[matchedcluster].g=CD[pos].g;
                color_cluster[matchedcluster].b=CD[pos].b;
                CD[pos].colorH = cluster_color(cluster);
                CD[pos].t.Now();
                pcl::compute3DCentroid (*cluster, CD[pos].COG);
            }
            else if ( pos == 101 )
            {
                //matched as static
            }
            else
            {
                c=1;
                found=false;
                while ( (c<=clustercount) & (!found) )
                {
                    double dt=CD[c].t.getTimePassed();
                    if ( dt > 7.0 )
                    {
                        CD[c].empty=true;
                        found=true;
                        std::stringstream del;
                        del << "data/cluster_" << c << "/";
                        boost::filesystem::remove_all(del.str());
                    }
                    c++;
                }
                found=false;
                int zz=1;
                while ( (!found) & (zz<=clustercount) )
                {
                    if (CD[zz].empty)
                    {
                        found=CD[zz].empty;
                    } else
                        zz++;
                }
                if (zz>clustercount)
                    clustercount++;
                assign_color_cluster(zz);
                totalcluster++;
                CD[zz].t.Now();
                CD[zz].colorH = cluster_color(cluster);
                CD[zz].colorS = cluster_colorS(cluster);
                CD[zz].empty = false;
                CD[zz].detections = 1;
                pcl::compute3DCentroid (*cluster, CD[zz].COG);
                printf("New cluster CD[%d] created with colorH: %f and colorS: %f\n",zz,CD[zz].colorH,CD[zz].colorS);
                call_tree_cluster(cluster,vfhsi,zz);
            }
        }
        i++;
    }
    *human = chuman;
    *smr = csmr;
    *rest = crest;
    *matchcluster = matchedcluster;
}


void UFuncTracking::assign_color_cluster ( int index )
{
    if ( LASTCOLORCLUSTER == 3 )
    {
        CD[index].r = 0;
        CD[index].g = 0;
        CD[index].b = 255;
        LASTCOLORCLUSTER=1;
    }
    else if ( LASTCOLORCLUSTER == 1 )
    {
        CD[index].r = 255;
        CD[index].g = 0;
        CD[index].b = 0;
        LASTCOLORCLUSTER=2;
    }
    else {
        CD[index].r = 0;
        CD[index].g = 255;
        CD[index].b = 0;
        LASTCOLORCLUSTER=3;
    }
}

void UFuncTracking::assign_color_smr ( int index )
{
    if ( LASTCOLORSMR==3 )
    {
        SD[index].r = 255;
        SD[index].g = 0;
        SD[index].b = 0;
        LASTCOLORSMR = 1;
    }
    else if ( LASTCOLORSMR==1 )
    {
        SD[index].r = 0;
        SD[index].g = 255;
//         SD[index].b = 0;
        LASTCOLORSMR = 2;
    }
    else
    {
        SD[index].r = 0;
        SD[index].g = 0;
        SD[index].b = 255;
        LASTCOLORSMR = 3;
    }
}

void UFuncTracking::assign_color_human ( int index )
{
    if ( LASTCOLORHUMAN == 3 )
    {
        HD[index].r=255;
        HD[index].g=0;
        HD[index].b=0;
        LASTCOLORHUMAN = 1;
    }
    else if ( LASTCOLORHUMAN == 1 )
    {
        HD[index].r=0;
        HD[index].g=255;
        HD[index].b=0;
        LASTCOLORHUMAN = 2;
    }
    else
    {
        HD[index].r=0;
        HD[index].g=0;
        HD[index].b=255;
        LASTCOLORHUMAN = 3;
    }
}

void UFuncTracking::object_classification_frame1 ( OBJ obj_img[], std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pre_cluster )
{
    int i = 0;
    int chuman = 0;
    int csmr = 0;
    int crest = 0;
    HUMANDETECTED = 0;
    SMRDETECTED = 0;
    humancount = 0;
    smrcount = 0;
    staticcount = 0;
    clustercount = 0;
    LASTCOLORSMR = 3;
    LASTCOLORHUMAN = 3;
    LASTCOLORCLUSTER = 3;
    pcl::PointXYZRGB min_pt, max_pt;
    Eigen::Affine3f t;
    pcl::getTransformation ( X, Y, Z, ROLL, 0.0, YAW, t );
    while ( i < obj_img[0].size )
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*pre_cluster[i], *cluster, t);
        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhsi (new pcl::PointCloud<pcl::VFHSignature308> ());
        flann::Matrix<float> k_distances;
        flann::Matrix<int> k_indices;
        if ( (obj_img[i].height < 0.50) & (obj_img[i].height > 0.05) & (obj_img[i].width < 0.5) & (obj_img[i].width > 0.05) & (obj_img[i].depth > 0.05) & (obj_img[i].baseheight < (FLOOR+0.2)) )
        {
            estimating_vfh_smr(cluster,vfhsi);
            pcl::io::savePCDFile("vfh1.pcd",*vfhsi);
            nearest_neighbors_smr( "vfh1.pcd", k_indices, k_distances);
            if ( k_distances[0][0] < thresh_smr )
            {
                SMRDETECTED++;
                smrcount++;
                float H,S;
                dominant_color(cluster,&H,&S);
                SD[smrcount].colorH = H;
                SD[smrcount].colorS = S;
                printf("Object %d is a SMR with color: %f ans colorS: %f\n",i,SD[smrcount].colorH,SD[smrcount].colorS);
                pcl::getMinMax3D(*cluster,min_pt,max_pt);
                SD[smrcount].x=max_pt.x;
                SD[smrcount].y=max_pt.y;
                SD[smrcount].z=max_pt.z;
                SD[smrcount].empty=false;
                SD[smrcount].t.Now();
                pcl::compute3DCentroid(*cluster,SD[smrcount].COG);
                update_file_smr_position(smrcount);
                update_file_smr_color(smrcount);
                assign_color_smr(smrcount);
                printf("SMRDETECTED:%d, r:%d, g:%d, b:%d and x:%f, y:%f, z:%f\n",SMRDETECTED,SD[smrcount].r,SD[smrcount].g,
                       SD[smrcount].b,SD[smrcount].x,SD[smrcount].y,SD[smrcount].z);
                //updating database if match is good enough
                if ( (k_distances[0][0] < thresh_smr/1.5) & (SMR<50) )
                {
                    smr_desc++;
                    std::stringstream sss;
                    sss << "data/smr/smr_" << smr_desc << ".pcd";
                    pcl::io::savePCDFile(sss.str (),*cluster);
                    std::stringstream ss;
                    ss << "data/smr/smr_" << smr_desc << "_vfh.pcd";
                    pcl::io::savePCDFile(ss.str (),*vfhsi);
                    build_tree_smr("data/smr");
                }
                csmr++;
            }
            else
            {
                estimating_vfh(cluster,vfhsi);
                pcl::io::savePCDFile("vfh1.pcd",*vfhsi);
                crest++;
                clustercount++;
                printf("Clustercount:%d\n",clustercount);
                assign_color_cluster(clustercount);
                CD[clustercount].empty=false;
                CD[clustercount].t.Now();
                float H,S;
                dominant_color(cluster,&H,&S);
                CD[clustercount].colorH = H;
                CD[clustercount].colorS = S;
                pcl::compute3DCentroid(*cluster,CD[clustercount].COG);
                update_file_cluster_position(clustercount);
                update_file_cluster_color(clustercount);
                call_tree_cluster(cluster,vfhsi,clustercount);
            }
        }
        else if ( (obj_img[i].height > 1.1) & (obj_img[i].height < 1.90) & (obj_img[i].depth > 0.05) & (obj_img[i].baseheight < FLOOR+0.35) ) //& (obj_img[i].top > 0.5) )
        {
            estimating_vfh_human(cluster,vfhsi);
            pcl::io::savePCDFile("vfh1.pcd",*vfhsi);
            nearest_neighbors_human( "vfh1.pcd", k_indices, k_distances);
            if ( k_distances[0][0] < thresh_human )
            {
                printf("Object %d is a HUMAN\n",i);
                HUMANDETECTED++;
                humancount++;
                float H,S;
                dominant_color(cluster,&H,&S);
                HD[humancount].colorH = H;
                HD[humancount].colorS = S;
                HD[humancount].empty=false;
                pcl::compute3DCentroid(*cluster,HD[humancount].COG);
                update_file_human_position(humancount);
                update_file_human_color(humancount);
                HD[humancount].t.Now();
                assign_color_human(humancount);
                if ( (k_distances[0][0] < thresh_human/1.5) & (HUMAN<50) )
                {
                    human_desc=human_desc+1;
                    std::stringstream sss;
                    sss << "data/human/human_" << human_desc << ".pcd";
                    pcl::io::savePCDFile(sss.str (),*cluster);
                    std::stringstream ss;
                    ss << "data/human/human_" << human_desc << "_vfh.pcd";
                    pcl::io::savePCDFile(ss.str (),*vfhsi);
                    build_tree_human("data/human");
                }
                chuman++;
                printf("HUMANDETECTED:%d, r:%d, g:%d, b:%d\n",HUMANDETECTED,HD[humancount].r,HD[humancount].g, HD[humancount].b);
            }
            else
            {
                estimating_vfh(cluster,vfhsi);
                pcl::io::savePCDFile("vfh1.pcd",*vfhsi);
                crest++;
                clustercount++;
                assign_color_cluster(clustercount);
                CD[clustercount].empty=false;
                CD[clustercount].t.Now();
                float H,S;
                dominant_color(cluster,&H,&S);
                CD[clustercount].colorH = H;
                CD[clustercount].colorS = S;
                pcl::compute3DCentroid(*cluster,CD[clustercount].COG);
                update_file_cluster_position(clustercount);
                update_file_cluster_color(clustercount);
                call_tree_cluster(cluster,vfhsi,clustercount);
            }
        }
        else
        {

            estimating_vfh(cluster,vfhsi);
            pcl::io::savePCDFile("vfh1.pcd",*vfhsi);
            crest++;
            clustercount++;
            assign_color_cluster(clustercount);
            CD[clustercount].empty = false;
            CD[clustercount].t.Now();
            float H,S;
            dominant_color(cluster,&H,&S);
            CD[clustercount].colorH = H;
            CD[clustercount].colorS = S;
            pcl::compute3DCentroid(*cluster,CD[clustercount].COG);
            update_file_cluster_position(clustercount);
            update_file_cluster_color(clustercount);
            call_tree_cluster(cluster,vfhsi,clustercount);
        }
        i++;
    }
    printf("\n");
    human_ini = chuman;
    smr_ini = csmr;
    rest_ini = crest;
    totalcluster = clustercount;
}

void UFuncTracking::matching ( int *match, int human_next, int smr_next, pcl::PointCloud<pcl ::PointXYZRGB>::Ptr cloud,
                               std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *matched_next,std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *new_smr,
                               std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *new_human, std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pos_human_next,
                               std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pos_smr_next, color color_matched[], color color_new_smr[],
                               color color_new_human[] )
{
    pcl::PointXYZRGB min_ini, min_next, max_ini, max_next;
    float minabs, dist;
    int m_min, n_min, match_human=0, match_smr=0;
    int used_smr[4], minm[4], m_notused[4];
    for (int u=0;u<4;u++)
    {
        used_smr[u] = 15;
        minm[u] = 0;
    }
    //SMRS
    int realscount = 0;
    for ( int c=1; c<=smrcount; c++)
    {
        if ( !SD[c].empty )
        {
            double dt = SD[c].t.getTimePassed();
            printf("TimePassed SMR:%f\n",dt);
            if ( dt > 5.0 )
            {
                SD[c].empty = true;
                printf("SMR :%d no longer stored\n",c);
            }
            else
            {
                realscount++;
                SD[c].exp_x = SD[c].COG.x()+(SD[c].speed*cos(SD[c].angle))/2;
                SD[c].exp_y = SD[c].COG.y()+(SD[c].speed*sin(SD[c].angle))/2;
            }
        }
    }
    int notfound = 0;
    int countfor = 0;
    timeval ini,fin;

    printf("Pos smr size: %d\n",int(pos_smr_next.size()));

    if ( (smr_next>0) & (smrcount>0) )
    {
        gettimeofday(&ini,NULL);
        while ( countfor < smr_next )
        {
            minabs=100.0;
            for ( int m=0; m<smr_next; m++)
            {
                if ( minm[m]==0 )
                {
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(*pos_smr_next[m],centroid);
                    for (int n=1; n<=smrcount; n++)
                    {
                        if ( !SD[n].empty )
                        {
                            dist = ((SD[n].exp_y-centroid.y())*(SD[n].exp_y-centroid.y()))+((SD[n].exp_x-centroid.x())*(SD[n].exp_x-centroid.x()));
                            dist = sqrt(dist);
                            if ( (dist < minabs) & (minm[m]==0) & (n != used_smr[0]) & (n != used_smr[1]) & (n != used_smr[2]) & (n != used_smr[3]) )
                            {
                                minabs = dist;
                                m_min = m;
                                n_min = n;
                            }
                        }
                    }
                    if ( (countfor >= realscount) & (minabs == 100.0) & (minm[m] == 0) )
                    {
                        notfound++;
                        m_notused[notfound] = m;
                        printf("Notfound:%d at m:%d\n", notfound, m);
                        minm[m] = 1;
                    }
                }
            }
            countfor++;
            if ( minabs != 100.0 )
            {
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*pos_smr_next[m_min],centroid);
                float angle, speed;
                if ( centroid.x()-SD[n_min].COG.x()>=0 )
                    angle = atan( (centroid.y()-SD[n_min].COG.y())/(centroid.x()-SD[n_min].COG.x()) );
                else
                    angle = atan( (centroid.y()-SD[n_min].COG.y())/(centroid.x()-SD[n_min].COG.x()) ) + 3.1416;
                float distreal;
                distreal = (SD[n_min].COG.y()-centroid.y())*(SD[n_min].COG.y()-centroid.y())+(SD[n_min].COG.x()-centroid.x())*(SD[n_min].COG.x()-centroid.x());
                distreal = sqrt(distreal);
                speed = distreal/SD[n_min].t.getTimePassed();
                if ( (speed > 0.8) || ( (SD[n_min].t.getTimePassed()>2.0)&(speed>0.35) ) )
                {
                    notfound++;
                    m_notused[notfound] = m_min;
                    minm[m_min] = 1;
                    printf("Speed too fast: %f. Dist real: %f and timePassed: %f. Notfound:%d at m:%d and n:%d\n",speed,distreal,SD[n_min].t.getTimePassed(),notfound,m_min,n_min);
                }
                else
                {
                    used_smr[m_min] = n_min;
                    minm[m_min] = 1;
                    SD[n_min].angle = angle;
                    SD[n_min].speed = speed;
                    printf("angle: %f and speed: %f, dist: %f and timePassed: %f\n",SD[n_min].angle,SD[n_min].speed,distreal,SD[n_min].t.getTimePassed());
                    matched_next->push_back(pos_smr_next[m_min]);
                    pcl::getMinMax3D(*pos_smr_next[m_min],min_next,max_next);
                    SD[n_min].x = max_next.x;
                    SD[n_min].y = max_next.y;
                    SD[n_min].z = max_next.z;
                    SD[n_min].t.Now();
                    pcl::compute3DCentroid(*pos_smr_next[m_min],SD[n_min].COG);
                    update_file_smr_position(n_min);
                    float H,S;
                    dominant_color(pos_smr_next[m_min],&H,&S);
                    SD[n_min].colorH = H;
                    SD[n_min].colorS = S;
                    printf("Color SMR now: %f",SD[n_min].colorH);
                    match_smr++;
                    color_matched[match_smr].r = SD[n_min].r;
                    color_matched[match_smr].g = SD[n_min].g;
                    color_matched[match_smr].b = SD[n_min].b;
                    update_file_smr_color(n_min);
                    update_file_smr_expected(n_min);
                    printf("SMRCOUNT:%d, the matched number:%d, distance driven:%f, with r:%d, g:%d, b:%d, x:%f, y:%f, z:%f, m_min:%d and n_min:%d and color: %f\n",
                           smrcount, match_smr, minabs, color_matched[match_smr].r, color_matched[match_smr].g, color_matched[match_smr].b, SD[n_min].COG.x(),
                           SD[n_min].COG.y(), SD[n_min].COG.z(), m_min, n_min, SD[n_min].colorH);
                    float dist_cam = (SD[n_min].COG.y()-Y)*(SD[n_min].COG.y()-Y)+(SD[n_min].COG.x()-X)*(SD[n_min].COG.x()-X);
                    dist_cam = sqrt(dist_cam);
                }
            }
            else
            {
                printf("New smr id: %d detected in %d\n",notfound,m_notused[notfound]);
            }
        }
        gettimeofday(&fin,NULL);
    }
    printf("SMRcount :%d and SMRnext:%d and match_smr: %d\n",smrcount,smr_next,match_smr);
    if ( notfound > 0 || (realscount==0) )
    {
        if (realscount>0)
        {
            SMRDETECTED=0;
            for ( int s=1; s<=notfound; s++ )
            {
                new_smr->push_back(pos_smr_next[m_notused[s]]);
                smrcount++;
                SMRDETECTED++;
                float H,S;
                dominant_color(pos_smr_next[m_notused[s]],&H,&S);
                SD[smrcount].colorH = H;
                SD[smrcount].colorS = S;
                SD[smrcount].speed = 0;
                SD[smrcount].angle = 0;
                update_file_smr_color(smrcount);
                printf("Color new SMR: %f\n",SD[s].colorH);
                color_new_smr[s].r = SD[smrcount].r;
                color_new_smr[s].g = SD[smrcount].g;
                color_new_smr[s].b = SD[smrcount].b;
                pcl::getMinMax3D(*pos_smr_next[m_notused[s]],min_ini,max_ini);
                SD[smrcount].x = max_ini.x;
                SD[smrcount].y = max_ini.y;
                SD[smrcount].z = max_ini.z;
                SD[smrcount].empty = false;
                SD[smrcount].t.Now();
                pcl::compute3DCentroid(*pos_smr_next[m_notused[s]],SD[smrcount].COG);
                assign_color_smr(smrcount);
                update_file_smr_position(smrcount);
                update_file_smr_expected(smrcount);
                printf("SMRDETECTED:%d, r:%d, g:%d, b:%d, x:%f, y:%f, z:%f and LASTCOLOR:%d\n",SMRDETECTED,SD[smrcount].r,SD[smrcount].g, SD[smrcount].b,SD[smrcount].COG.x(),SD[smrcount].COG.y(),SD[smrcount].COG.z(),LASTCOLORSMR);
                float dist_cam = (SD[smrcount].COG.y()-Y)*(SD[smrcount].COG.y()-Y)+(SD[smrcount].COG.x()-X)*(SD[smrcount].COG.x()-X);
                dist_cam = sqrt(dist_cam);
            }
        }
        else
        {
            for ( int s=0; s<smr_next; s++ )
            {
                smrcount++;
                float H,S;
                dominant_color(pos_smr_next[s],&H,&S);
                SD[smrcount].colorH = H;
                SD[smrcount].colorS = S;
                new_smr->push_back(pos_smr_next[s]);
                pcl::getMinMax3D(*pos_smr_next[s],min_ini,max_ini);
                SD[smrcount].x = max_ini.x;
                SD[smrcount].y = max_ini.y;
                SD[smrcount].z = max_ini.z;
                SD[smrcount].empty = false;
                SD[smrcount].speed = 0.0;
                SD[smrcount].angle = 0.0;
                SD[smrcount].t.Now();
                pcl::compute3DCentroid(*pos_smr_next[s],SD[smrcount].COG);
                assign_color_smr(smrcount);
                update_file_smr_position(smrcount);
                update_file_smr_color(smrcount);
                update_file_smr_expected(smrcount);
                color_new_smr[s+1].r = SD[smrcount].r;
                color_new_smr[s+1].g = SD[smrcount].g;
                color_new_smr[s+1].b = SD[smrcount].b;
                printf("SMRDETECTED:%d, r:%d, g:%d, b:%d, x:%f, y:%f, z:%f and LASTCOLOR:%d\n",SMRDETECTED,SD[smrcount].r,SD[smrcount].g,
                       SD[smrcount].b,SD[smrcount].x, SD[smrcount].y,SD[smrcount].z,LASTCOLORSMR);
                float dist_cam = (SD[smrcount].COG.y()-Y)*(SD[smrcount].COG.y()-Y)+(SD[smrcount].COG.x()-X)*(SD[smrcount].COG.x()-X);
                dist_cam = sqrt(dist_cam);
            }
        }
    }

    int used_human[5];
    countfor = 0;
    notfound = 0;
    for (int u=0;u<4;u++)
    {
        used_human[u]=10;
        minm[u]=0;
    }
    //HUMANS
    int realhcount = 0;
    for ( int c=1; c<=humancount; c++)
    {
        if (!HD[c].empty)
        {
            double dt = HD[c].t.getTimePassed();
            if ( dt > 4.0 )
            {
                HD[c].empty = true;
                printf("Human :%d no longer stored\n",c);
            }
            else
            {
                realhcount++;
                HD[c].exp_x = HD[c].COG.x()+(HD[c].speed*cos(HD[c].angle))/2;
                HD[c].exp_y = HD[c].COG.y()+(HD[c].speed*sin(HD[c].angle))/2;
            }
        }
    }
    printf("RealHCount: %d, HUMANcount :%d and HUMANnext:%d and match_human:%d\n",realhcount,humancount,human_next,match_human);
    if ( (human_next>0) & (realhcount>0) )
    {
        while ( countfor < human_next )
        {
            minabs=100.0;
            for (int m=0;m<human_next;m++)
            {
                if ( minm[m]==0 )
                {
                    //float H_next = cluster_color(pos_human_next[m]);
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(*pos_human_next[m],centroid);
                    for (int n=1; n<=humancount; n++)
                    {
                        //chose the best match
                        if ( !HD[n].empty )
                        {
                            dist = ((HD[n].exp_y-centroid.y())*(HD[n].exp_y-centroid.y()))+((HD[n].exp_x-centroid.x())*(HD[n].exp_x-centroid.x()));
                            dist = sqrt(dist);
                            printf("for m:%d and n: %d dist is:%f\n",m,n,dist);
                            if ( (dist < minabs) & (n != used_human[0]) & (n != used_human[1]) & (n != used_human[2]) & (n != used_human[3]))
                            {
                                minabs = dist;
                                m_min = m;
                                n_min = n;
                            }
                        }
                    }
                    printf("Countfor: %d realhumancount: %d minabs: %f, minm[m]: %d\n",countfor,realhcount,minabs,minm[m]);
                    if ( (countfor>=realhcount) & (minabs == 100.0) & (minm[m]==0) )
                    {
                        notfound++;
                        m_notused[notfound] = m;
                        printf("Notfound:%d at m:%d\n",notfound,m);
                        minm[m] = 1;
                    }
                }
            }
            countfor++;
            if ( minabs != 100.0 )
            {
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*pos_human_next[m_min],centroid);
                float angle, speed;
                if ( centroid.x()-HD[n_min].COG.x()>=0 )
                    angle = atan( (centroid.y()-HD[n_min].COG.y())/(centroid.x()-HD[n_min].COG.x()) );
                else
                    angle = atan( (centroid.y()-HD[n_min].COG.y())/(centroid.x()-HD[n_min].COG.x()) ) + 3.1416;
                float distreal;
                distreal = (HD[n_min].COG.z()-centroid.z())*(HD[n_min].COG.z()-centroid.z())+(HD[n_min].COG.y()-centroid.y())*(HD[n_min].COG.y()-centroid.y())
                           +(HD[n_min].COG.x()-centroid.x())*(HD[n_min].COG.x()-centroid.x());
                distreal = sqrt(distreal);
                speed = distreal/HD[n_min].t.getTimePassed();
                if ( (speed > 2.0) || ( (HD[n_min].t.getTimePassed()>2.0)&(speed>0.8) ) )
                {
                    notfound++;
                    m_notused[notfound] = m_min;
                    minm[m_min] = 1;
                    printf("Speed too fast: %f. Dist real: %f and timePassed: %f. Notfound:%d at m:%d and n:%d\n",speed,distreal,HD[n_min].t.getTimePassed(),notfound,m_min,n_min);
                }
                else
                {
                    minm[m_min] = 1;
                    used_human[m_min] = n_min;
                    HD[n_min].angle = angle;
                    HD[n_min].speed = speed;
                    printf("angle: %f and speed: %f and dist: %f\n",HD[n_min].angle,HD[n_min].speed,distreal);
                    float H,S;
                    dominant_color(pos_human_next[m_min],&H,&S);
                    HD[n_min].colorH = H;
                    HD[n_min].colorS = S;
                    pcl::getMinMax3D(*pos_human_next[m_min],min_next,max_next);
                    HD[n_min].x = max_next.x;
                    HD[n_min].y = max_next.y;
                    HD[n_min].z = max_next.z;
                    matched_next->push_back(pos_human_next[m_min]);
                    HP[n_min].cloud[HP[n_min].suma]=pos_human_next[m_min];
                    HP[n_min].suma++;
                    match_human++;
                    color_matched[match_smr+match_human].r = HD[n_min].r;
                    color_matched[match_smr+match_human].g = HD[n_min].g;
                    color_matched[match_smr+match_human].b = HD[n_min].b;

                    pcl::compute3DCentroid(*pos_human_next[m_min],HD[n_min].COG);
                    HD[n_min].t.Now();
                    update_file_human_position(n_min);
                    update_file_human_color(n_min);
                    update_file_human_expected(n_min);
                    printf("m_min:%d; n_min: %d distanced moved:%f and speed: %f, with r:%d, g:%d, b:%d\n",m_min, n_min, minabs, HD[n_min].speed,
                           color_matched[match_smr+match_human].r, color_matched[match_smr+match_human].g, color_matched[match_smr+match_human].b);
                    float dist_cam = (HD[n_min].COG.y()-Y)*(HD[n_min].COG.y()-Y)+(HD[n_min].COG.x()-X)*(HD[n_min].COG.x()-X);
                    dist_cam = sqrt(dist_cam);
                }
            }
            else
            {
                printf("New human id:%d detected in %d\n", notfound, m_notused[notfound]);
            }
        }
    }
    if ( (notfound>0) || (realhcount==0) )
    {
        if ( realhcount>0 )
        {
            HUMANDETECTED = 0;
            for ( int s=1; s<=notfound; s++ )
            {
                new_human->push_back(pos_human_next[m_notused[s]]);
                humancount++;
                HUMANDETECTED++;
                float H,S;
                dominant_color(pos_human_next[m_notused[s]],&H,&S);
                HD[humancount].colorH = H;
                HD[humancount].colorS = S;
                HD[humancount].empty = false;
                HD[humancount].speed = 0.0;
                HD[humancount].angle = 0.0;
                HD[humancount].t.Now();
                assign_color_human(humancount);
                color_new_human[s].r = HD[humancount].r;
                color_new_human[s].g = HD[humancount].g;
                color_new_human[s].b = HD[humancount].b;
                HP[humancount].suma = 0;
                HP[humancount].cloud[HP[humancount].suma]=pos_human_next[m_notused[s]];
                HP[humancount].r = HD[humancount].r;
                HP[humancount].g = HD[humancount].g;
                HP[humancount].b = HD[humancount].b;
                HP[humancount].suma++;
                pcl::compute3DCentroid(*pos_human_next[m_notused[s]],HD[humancount].COG);
                update_file_human_position(humancount);
                update_file_human_color(humancount);
                update_file_human_expected(humancount);
                printf("HUMANDETECTED:%d, r:%d, g:%d, b:%d\n",HUMANDETECTED,HD[humancount].r,HD[humancount].g, HD[humancount].b);
                float dist_cam = (HD[humancount].COG.y()-Y)*(HD[humancount].COG.y()-Y)+(HD[humancount].COG.x()-X)*(HD[humancount].COG.x()-X);
                dist_cam = sqrt(dist_cam);
            }
        }
        else
        {
            for ( int s=0; s<human_next; s++ )
            {
                humancount++;
                HUMANDETECTED++;
                float H,S;
                dominant_color(pos_human_next[s],&H,&S);
                HD[humancount].colorH = H;
                HD[humancount].colorS = S;
                HD[humancount].empty = false;
                HD[humancount].speed = 0.0;
                HD[humancount].angle = 0.0;
                HD[humancount].t.Now();
                new_human->push_back(pos_human_next[s]);
                assign_color_human(humancount);
                color_new_human[s+1].r = HD[humancount].r;
                color_new_human[s+1].g = HD[humancount].g;
                color_new_human[s+1].b = HD[humancount].b;
                HP[humancount].suma = 0;
                HP[humancount].cloud[HP[humancount].suma]=pos_human_next[s];
                HP[humancount].r = HD[humancount].r;
                HP[humancount].g = HD[humancount].g;
                HP[humancount].b = HD[humancount].b;
                HP[humancount].suma++;
                pcl::compute3DCentroid(*pos_human_next[s],HD[humancount].COG);
                update_file_human_position(humancount);
                update_file_human_color(humancount);
                update_file_human_expected(humancount);
                printf("HUMANDETECTED:%d, r:%d, g:%d, b:%d\n",HUMANDETECTED,HD[humancount].r,HD[humancount].g, HD[humancount].b);
                float dist_cam = (HD[humancount].COG.y()-Y)*(HD[humancount].COG.y()-Y)+(HD[humancount].COG.x()-X)*(HD[humancount].COG.x()-X);
                dist_cam = sqrt(dist_cam);
            }
        }
    }
    *match=match_human+match_smr;
}


void UFuncTracking::update_file_cluster_position ( int i )
{
    std::stringstream list;
    list << "data/cluster_" << i << "/positions.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << CD[i].COG.x() << " " << CD[i].COG.y() << " " << CD[i].COG.z() << "\n";
    fs.close ();
}

void UFuncTracking::update_file_static_position ( int i )
{
    std::stringstream list;
    list << "data/static_" << i << "/positions.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    //std::fstream::ate fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << STD[i].COG.x() << " " << STD[i].COG.y() << " " << STD[i].COG.z() << "\n";
    fs.close ();
}

void UFuncTracking::update_file_human_position ( int i )
{
    std::stringstream list;
    list << "data/human/human_" << i << "_positions.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    //std::fstream::ate fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << HD[i].COG.x() << " " << HD[i].COG.y() << " " << HD[i].COG.z() << "\n";
    fs.close ();
}

void UFuncTracking::update_file_robot_position ( int i )
{
    std::stringstream list;
    list << "data/robot_positions.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    //std::fstream::ate fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << X << " " << Y << " " << YAW << "\n";
    fs.close ();
}

void UFuncTracking::update_file_time_calculation ( int i )
{
    std::stringstream list;
    list << "data/time_calculations.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    //std::fstream::ate fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << dTimeGet-dTimeItIni << " " << dTimeDown-dTimeGet << " " << dTimeTrans-dTimeDown << " " << dTimeFloor-dTimeTrans << " "
    << dTimeClus-dTimeFloor << " " <<  dTimeTrans2-dTimeClus << " " << dTimeObj-dTimeTrans2 << " " << dTimeMatching-dTimeObj << " "
    << dTimeVis-dTimeMatching <<" " << " " << dTimeItEnd-dTimeVis << " " << dTimeItEnd-dTimeItIni << " " << cluster_detected << " "
    << cluster_processed << " " <<"\n";
    fs.close ();
}

void UFuncTracking::update_file_human_expected ( int i )
{
    std::stringstream list;
    list << "data/human/human_" << i << "_expected_positions.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    //std::fstream::ate fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << HD[i].COG.x() << " " << HD[i].exp_x << " " << HD[i].COG.y() << " " << HD[i].exp_y << " " << HD[i].speed << " " << HD[i].angle << " " << IMAGES_STUDIED << "\n";
    fs.close ();
}

void UFuncTracking::update_file_human_match ( std::vector<vfh_model> models, flann::Matrix<int> k_indices, flann::Matrix<float> &k_distances )
{
    std::stringstream list;
    list << "data/human/human_matches.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    //std::fstream::ate fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << models.at(k_indices[0][0]).first.c_str() << " " << k_distances[0][0] << "\n";
    fs.close ();
}

void UFuncTracking::update_file_smr_position ( int i )
{
    std::stringstream list;
    list << "data/smr/smr_" << i << "_positions.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    //std::fstream::ate fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << SD[i].COG.x() << " " << SD[i].COG.y() << " " << SD[i].COG.z() << "\n";
    fs.close ();
}

void UFuncTracking::update_file_smr_expected ( int i )
{
    std::stringstream list;
    list << "data/smr/smr_" << i << "_expected_positions.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    //std::fstream::ate fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << SD[i].COG.x() << " " << SD[i].exp_x << " " << SD[i].COG.y() << " " << SD[i].exp_y << " " << SD[i].speed << " " << SD[i].angle << " " << IMAGES_STUDIED << "\n";
    fs.close ();
}

void UFuncTracking::update_file_smr_color ( int i )
{
    std::stringstream list;
    list << "data/smr/smr_" << i << "_color.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    //std::fstream::ate fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << SD[i].colorH << " " << SD[i].colorS << "\n";
    fs.close ();
}

void UFuncTracking::update_file_human_color ( int i )
{
    std::stringstream list;
    list << "data/human/human_" << i << "_color.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    //std::fstream::ate fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << HD[i].colorH << " " << HD[i].colorS << "\n";
    fs.close ();
}

void UFuncTracking::update_file_cluster_color ( int i )
{
    std::stringstream list;
    list << "data/cluster_" << i << "/color.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    //std::fstream::ate fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << CD[i].colorH << " " << CD[i].colorS << "\n";
    fs.close ();
}

void UFuncTracking::update_file_static_color ( int i )
{
    std::stringstream list;
    list << "data/static_" << i << "/color.list";
    std::string position_data_list_file_name = list.str ();

    std::ofstream fs;
    //std::fstream::ate fs;
    fs.open (position_data_list_file_name.c_str (), ios::app);
    fs << STD[i].colorH << " " << STD[i].colorS << "\n";
    fs.close ();
}


void UFuncTracking::removeAllText3D ( int add, int matchcluster, pcl::visualization::PCLVisualizer *viewer )
{
    int added=add+matchcluster;
    printf("added:%d add:%d matchcluster:%d\n",added,add,matchcluster);
    for (int del=1;del<=added;del++)
    {
        std::stringstream textm;
        textm << "MATCHED_" << del;
        viewer->removeText3D(textm.str());
        std::stringstream textmc;
        textmc << "MATCHED_CLUSTER_" << del;
        viewer->removeText3D(textmc.str());
        std::stringstream textst;
        textst << "STATIC_" << del;
        viewer->removeText3D(textst.str());
        std::stringstream textsmr;
        textsmr << "SMR_" << del;
        viewer->removeText3D(textsmr.str());
        std::stringstream texth;
        texth << "HUMAN_" << del;
        viewer->removeText3D(texth.str());
        std::stringstream textn;
        textn << "NEW_" << del;
        viewer->removeText3D(textn.str());
    }
}