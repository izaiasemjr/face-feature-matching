
#include "utils.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d_omp.h>


// List the available keypoints
static const string SIFT_NORMAL  = "sift_normal";
static const string SIFT_Z  = "sift_z";
static const string FROM_FILE = "fromFile";


using namespace std;


template<typename KeyPointT>
void getKeyPointsFromFile(string filename,
                          typename pcl::PointCloud<KeyPointT>::Ptr keypoints_choice
                         ){

    PointCloudXYZPtr keypoints(new PointCloudXYZ);
    pcl::io::loadPCDFile (filename.c_str(), *keypoints);

    vector<int> keypoints_index = {
//        1,  // eye corner up_left-extern
//        2,  // eye corner up_left-intern
//        3,  // eye corner up_right-intern
//        4,  // eye corner up_right-extern
//        0,  //
//        5, //
//        10, //
//        11, //
//        12, //
//        14, //
//        18, //
//        19, //
//        21, //
//        22, //
//        23, //
        6,  // eye corner left-extern
        7,  // eye corner left-intern
        8,  // eye corner right-intern
        9,  // eye corner right-extern
        13, // nose_tip
        15, // mouth corner left
        16, // mouth center up
        17, // mouth corner right
        20  // mouth center down
    };

    for(int i =0;i<keypoints_index.size();i++){
        auto p =keypoints->points[keypoints_index[i]];
        if(pcl::isFinite(p) )
            keypoints_choice->push_back(p);

    }


}


template<typename KeyPointT>
void getKeyPointsFromNose(string filename,
                          typename pcl::PointCloud<KeyPointT>::Ptr keypoints_choice
                         ){

    typename pcl::PointCloud<KeyPointT>::Ptr keypoints(new typename pcl::PointCloud<KeyPointT>);
    pcl::io::loadPCDFile (filename.c_str(), *keypoints);
    KeyPointT nose = keypoints->points[13];

    keypoints_choice->push_back(nose);
    keypoints_choice->push_back(KeyPointT(nose.x + 43.15,nose.y + 30.76,nose.z  -50.44)); // EYE_RIGHT_EXTERN
    keypoints_choice->push_back(KeyPointT(nose.x + 12.75,nose.y + 29.22,nose.z  -42.09)); // EYE_RIGHT_INTERN
    keypoints_choice->push_back(KeyPointT(nose.x - 50.45,nose.y + 26.77,nose.z  -45.81)); // EYE_LEFT_EXTERN
    keypoints_choice->push_back(KeyPointT(nose.x - 19.66,nose.y + 27.69,nose.z  -40.48)); // EYE_LEFT_INTERN
    keypoints_choice->push_back(KeyPointT(nose.x - 24.48,nose.y - 39.36,nose.z  -27.24)); // MOUTH_LEFT
    keypoints_choice->push_back(KeyPointT(nose.x + 25.27,nose.y - 36.84,nose.z  -29.99)); // MOUTH_RIGHT
    keypoints_choice->push_back(KeyPointT(nose.x + 0.63, nose.y - 29.12,nose.z  -12.6)); // MOUTH_CENTER_UP
    keypoints_choice->push_back(KeyPointT(nose.x + 1.23, nose.y - 44.05,nose.z  -14.51)); // MOUTH_CENTER_DOWN

}

template<typename PointT,typename KeyPointT>
void computeSiftNormal(typename pcl::PointCloud<PointT>::Ptr cloud,
                      typename pcl::PointCloud<KeyPointT>::Ptr cloud_keypoints,
                       pcl::PointCloud<Normal>::Ptr cloud_normals,
                      vector<string> params
                      ){

    using namespace pcl;

    // Parameters for sift computation
    float min_scale = 4;
    int n_octaves = 3;
    int n_scales_per_octave = 4;
    float min_contrast = 0.0001;
    if(params.size()==5){
        min_scale = stof(params[1]);
        n_octaves = stoi(params[2]);
        n_scales_per_octave = stoi(params[3]);
        min_contrast = stof(params[4]);
    }

////    Estimate the normals of the input cloud
//    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);;
//    estimateNormals<PointT>(cloud,cloud_normals,normal_radius);
//    cout<<"Normals computed: "<<cloud_normals->points.size()<<endl;

    // concat Points and Normals to form PointNormal object
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_sift(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *cloud_normals, *cloud_normals_sift);

    // Estimate the sift interest points using normals values from xyz as the Intensity variants
    SIFTKeypoint<PointNormal, PointWithScale> sift;
    PointCloud<PointWithScale>::Ptr keypoints(new PointCloud<PointWithScale>);
    search::KdTree<PointNormal>::Ptr tree(new search::KdTree<PointNormal> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud_normals_sift);
    sift.compute(*keypoints);

    // copy cloud type PointWithScale to cloud XYZ
    copyPointCloud(*keypoints,*cloud_keypoints);

    // Sanity check
    if (keypoints->points.size() == 0 ){
      cout<<"0 zero keypoints computed, the program will finish"<<endl;
      exit(1);
    }

}




template<typename PointT,typename KeyPointT>
void computeKeypoints(typename pcl::PointCloud<PointT>::Ptr& cloud,
                       typename pcl::PointCloud<KeyPointT>::Ptr& cloud_keypoints,
                       pcl::PointCloud<Normal>::Ptr normals,
                       vector<string> params
                    ){
    if(params.size()<1)
        cout<<"You should especify at least the methode name like: -keypoints sift_normal"<<endl;

    if (params[0]==SIFT_NORMAL){
        //cout<<"computing SIFT ..."<<endl;
        computeSiftNormal<PointT, KeyPointT>(cloud,cloud_keypoints,normals,params);
    }
    else {
        cout<<"We not support this method. Methods Supported:"<<endl;
    }

}

