//***************************************************************************
/*   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *   Albert Navarro Comes                                                  *
 *   albertnavarro8@gmail.com                                              *
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
 ***************************************************************************/
#ifndef UFUNC_Tracking_H
#define UFUNC_Tracking_H

#define BOOST_SYMBOL_VISIBLE __attribute__((visibility("default")))

#include <cstdlib>

#include <ucam4/ufunctioncambase.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <ulms4/ufunclaserbase.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/cvfh.h>

#include <pcl/common/transforms.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_representation.h>
#include <pcl/point_cloud.h>
#include <boost/make_shared.hpp>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <flann/io/hdf5.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <boost/filesystem.hpp>
#include <sys/stat.h>
#include <stdio.h>

// //////////////////////////////////////////////////////
// //////////////////////////////////////////////////////
// //////////////////////////////////////////////////////

#define PI 3.14159

// class of objects with sizes (initialized in the clustering process)
class OBJ {
public:
    float width;
    float height;
    float baseheight;
    float top;
    float basewidth_left;
    float basewidth_right;
    float dist;
    float depth;
    int size;
    int human;
    int smr;
};
// class of detected objects
class DETECTED {
public:
    bool empty;
    int r;
    int g;
    int b;
    int detections;
    float x;
    float y;
    float z;
    float exp_x;
    float exp_y;
    Eigen::Vector4f COG;
    float speed;
    float angle;
    float colorH;
    float colorS;
    UTime t;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
};

//front page picture
class PICTURE {
public:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud[500];
  int r;
  int g;
  int b;
  int suma;
};

PICTURE HP[15];

DETECTED HD[15];
DETECTED SD[25];
DETECTED CD[100];
DETECTED STD[50];
int humancount;
int smrcount;
int clustercount;
int staticcount;
int totalcluster;

double thresh_smr = 160.0; //threshold to accept as valid a SMR
double thresh_smr_add = 110.0; //threshold to add a SMR to the database
double thresh_human = 70.0; //threshold to accept as valid a Human
double thresh_human_add = 50.0; //threshold to add a Human to the database
double thresh_cluster = 150.0; // threshold to accept a cluster as valid
double thresh_static = 80.0; // threshold to accept a static cluster as valid

double FLOOR;  //height at what the floor is found
double max_width = 0.9; //maximum width allowed for an object

double pitch;

int SMR = 150; //SMR descriptors in the databse (when setting to 200 or higher no descriptors will be added)
int HUMAN = 150; //Human descriptors in the databse (when setting to 150 or higher no descriptors will be added)
int smr_desc;
int human_desc;

int times_human;
int times_smr;

////////////////////////////////
int human_ini;
int smr_ini;
int rest_ini;
int add;
float lxyz = 0.03; //downsampling size
bool first_image = true;

double START_TIME;
int IMAGES_STUDIED;
bool WALL;

int HUMAN_FAR;
int Wall_Image;
////////////////////////////////

float X, X_INI, Xf;
float Y, Y_INI, Yf;
float YAW, YAW_INI, YAWf;
float Z;
float ROLL;

class position {
public:
    float minx;
    float miny;
    float minz;
};
class color {
public:
    int r;
    int g;
    int b;
};
position cluster_pos[1000];
int cluster_size[1000];
int STATIC;
int static_size[100];
int SMRDETECTED;
int HUMANDETECTED;

int LASTCOLORSMR;
int LASTCOLORHUMAN;
int LASTCOLORCLUSTER;

// HSV struct
struct colorHSV
{
    double H,S,V;
};

//time variables
double dTimeItIni, dTimeGet, dTimeDown, dTimeTrans, dTimeFloor, dTimeClus, dTimeTrans2, dTimeObj, dTimeMatching, dTimeVis, dTimeItEnd, dTimeIni, dTimeEnd;
int cluster_detected, cluster_processed;

typedef std::pair<std::string, std::vector<float> > vfh_model;

/**
Example plugin that demonstrates a plugin that tracks humans, SMRs and other objects.

@author Christian Andersen & Albert Navarro
*/
class UFuncTracking : public UFunctionCamBase
{ // NAMING convention recommend that the main plugin function class
    // starts with UFunc followed by
    // a descriptive extension for this specific plugin
public:
    /**
    Constructor */
    UFuncTracking()
    {
        setCommand("Track", "Track", "tracking objects with kinect (" __DATE__ " " __TIME__ ")");
        viewer = new pcl::visualization::PCLVisualizer ("3DViewer");
        createBaseVar();
    };
    /**
    Destructor */
    virtual ~UFuncTracking();
    /**
    Handle incomming command
    Must return true if the function is handled -
    otherwise the client will get a 'failed' reply */
    virtual bool handleCommand(UServerInMsg * msg, void * extra);

private:
    pcl::visualization::PCLVisualizer *viewer;
    /**
    Create create globally  available status and configuration variables */
    void createBaseVar();
    /**
    Get kinect pointcloud data from the image pool RGB*/
    void GetKinectPointCloudDataRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud );
    /**
    Main function of the algorithm */
    void TrackObjects(void);
    /**
    studies the first image */
    void FirstImage(void);
    /**
    downsamples the image */
    void downsampling ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ini , pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered );
    /**
    estimates normals of the cluster */
    void estimating_normals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals );
    /**
    estimates normals of a human*/
    void estimating_normals_human (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals );
    /**
    estimates normals of a SMR*/
    void estimating_normals_smr (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals );
    /**
    estimating the vfh cluster */
    void estimating_vfh (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs);
    /**
    estimating the vfh human */
    void estimating_vfh_human(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs);
    /**
    estimating the vfh smr */
    void estimating_vfh_smr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs);
    /**
    clustering the floor */
    void clustering_floor ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr *cloud_filtered );
    /**
    clustering the point cloud, extract the cluster that are going to be studied */
    void clustering ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered , OBJ obj_img[] ,
                      std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *cluster );
    /**
    remove all3DText */
    void removeAllText3D ( int add, int matchcluster, pcl::visualization::PCLVisualizer *viewer );
    /**
    converts RGB color format to HSV */
    void RGB2HSV (int r, int g, int b, float& fh, float& fs, float& fv);
    /**
    finds the dominant color of a cluster */
    void dominant_color ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , float *HMean, float *SMean );
    /**
    finds the Hue color of a cluster */
    float cluster_color ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud );
    /**
    finds de Saturation color of a cluster */
    float cluster_colorS ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud );
    /**
    assigning a color to the cluster for representing it later on */
    void assign_color_cluster ( int index );
    /**
    assigning a color to the smr for representing it later on */
    void assign_color_smr ( int index );
    /**
    assigning a color to the human for representing it later on */
    void assign_color_human ( int index );
    /**
    load a Histogram */
    bool loadHist (const boost::filesystem::path &path, vfh_model &vfh);
    /**
    searches for the K nearest neighbors */
    void nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model,int k, flann::Matrix<int> &indices,
                         flann::Matrix<float> &distances);
    /**
    loads the file .list */
    bool loadFileList (std::vector<vfh_model> &models, const std::string &filename);
    /**
    creates the descriptor of a SMR */
    void descriptor_smr ( int k , vfh_model histogram , flann::Matrix<int> &k_indices , flann::Matrix<float> &k_distances );
    /**
    creates the descriptor of a human */
    void descriptor_human ( int k , vfh_model histogram , flann::Matrix<int> &k_indices , flann::Matrix<float> &k_distances );
    /**
    show the closest neighbors after the search */
    void visualization_neighbors ( flann::Matrix<int> &k_indices , flann::Matrix<float> &k_distances );
    /**
    loads the models */
    void loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, std::vector<vfh_model> &models);
    /**
    builds the kdtree structure of the SMR descriptor */
    void build_tree_smr ( const boost::filesystem::path &path );
    /**
    builds the kdtree structure of the HUMAN descriptor */
    void build_tree_human ( const boost::filesystem::path &path );
    /**
    builds the kdtree structure of a Cluster descriptor */
    void build_tree_cluster ( const boost::filesystem::path &path , int i);
    /**
    builds the kdtree structure of a Static descriptor */
    void build_tree_static ( const boost::filesystem::path &path , int i);
    /**
    calculates the nearest neighbors of a SMR */
    void nearest_neighbors_smr ( const boost::filesystem::path &path , flann::Matrix<int> &k_indices , flann::Matrix<float> &k_distances);
    /**
    calculates the nearest neighbors of a Human */
    void nearest_neighbors_human ( const boost::filesystem::path &path , flann::Matrix<int> &k_indices , flann::Matrix<float> &k_distances);
    /**
    calculates the nearest neighbors of a cluster */
    void nearest_neighbors_cluster ( const boost::filesystem::path &path , pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhsi,
                                     pcl::PointCloud<pcl ::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl ::PointXYZRGB>::Ptr big_cloud, 
				     flann::Matrix<int> &k_indices, flann::Matrix<float> &k_distances, int place, int *pos );
    /**
    visualization of all the objects, but the Static ones */
    void object_visualization ( pcl::visualization::PCLVisualizer *viewer, pcl::PointCloud<pcl ::PointXYZRGB>::Ptr cloud,
                              std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > matched_next,
                              std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > matched_cluster,
                              std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > smr_new,
                              std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > human_new,
                              std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cluster_new,
                              color color_matched[], color color_cluster[], color color_new_smr[], color color_new_human[], int *added );
    /**
    visualization of static objects */
    void visualize_static ( pcl::visualization::PCLVisualizer *viewer, pcl::PointCloud<pcl ::PointXYZRGB>::Ptr cloud );
    /**
    visualization function to obtain 1 out 4 representations of the path followed by a human */
    void visualize_front_page( pcl::visualization::PCLVisualizer *viewer, pcl::PointCloud<pcl ::PointXYZRGB>::Ptr cloud );
    /**
    calls the kdtree cluster */
    void call_tree_cluster ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhsi, int i );
    /**
    object classification/recognition, trying to recognize clusters as human, smr or any other type of cluster */
    void object_classification ( OBJ obj_img[], std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pre_cluster,
				 pcl::PointCloud<pcl ::PointXYZRGB>::Ptr cloud,
                                 std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *pos_human,
                                 std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *pos_smr,
                                 std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *pos_rest,
                                 std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *matched_cluster,
                                 std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *static_cluster,
                                 int *human, int *smr, int *rest, int *matchcluster, color color_cluster[] );
    /**
    object classification/recognition for the objects found in the first frame, when no clusters have been stored yet */
    void object_classification_frame1 ( OBJ obj_img[], std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pre_cluster );
    /**
    matching/correlation of the objects detected in different frames */
    void matching ( int *match, int human_next, int smr_next, pcl::PointCloud<pcl ::PointXYZRGB>::Ptr big_cloud,
                    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *matched_next, std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *new_smr,
                    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > *new_human, std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pos_human_next,
                    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pos_smr_next, color color_matched[], color color_new_smr[], color color_new_human[] );
    /**
    files with position, color and times of the clusters for Matlab representation*/
    void update_file_cluster_position ( int i );
    void update_file_static_position ( int i );
    void update_file_smr_position (int i);
    void update_file_human_position (int i);
    void update_file_cluster_color(int i);
    void update_file_static_color(int i);
    void update_file_smr_color(int i);
    void update_file_human_color(int i);
    void update_file_human_expected(int i);
    void update_file_smr_expected(int i);
    void update_file_robot_position(int i);
    void update_file_time_calculation(int i);
    void update_file_human_match( std::vector<vfh_model> models, flann::Matrix<int> k_indices, flann::Matrix<float> &k_distances );
    void update_file_time_tree(int i);

    
private:


    /// resulting map limits
    UVariable * varMinMapX;
    UVariable * varMaxMapX;
    UVariable * varMinMapY;
    UVariable * varMaxMapY;
    // map square size note this is also double the resolution of the 3D data
    UVariable * varSquareSize;
    // after estimation of a plane, this is the height limit of what is part of the plane and what is not
    UVariable * varPlaneHeight;
    // anything above this height is ignored
    UVariable * varRobotHeight;

    // finished map
    UVariable * varKinectMapPool;

    // optimizing variables
    UVariable * varShowFloorPlane;
    UVariable * varShowRoof;


};


#endif

