#ifndef UTILS_H
#define UTILS_H

// basics PCL manipulations
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>


typedef pcl::PointWithScale PointWithScale;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::Normal Normal;
typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointUV PointUV;
typedef pcl::PointNormal PointNormal;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;

typedef pcl::PointCloud<pcl::PointWithScale> PointCloudWithScale;
typedef pcl::PointCloud<pcl::PointWithScale>::Ptr PointCloudWithScalePtr;

typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
typedef pcl::PointCloud<pcl::Normal>::Ptr PointCloudNormalPtr;

using namespace std;


vector<string> split(const std::string& s, char delimiter)
{
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

string bosphorusLabels(string label){

    string filename = split(label,'-')[0];
    vector<string> results = split(filename,'_');
    string r;
    for(int i=0;i<results.size();++i){
       r = i==results.size()-1 ?r + results[i]: r + results[i] + string(",");
    }
    return r;
}

template<typename PointType>
void estimateNormals(typename pcl::PointCloud<PointType>::Ptr cloud,
                     pcl::PointCloud<Normal>::Ptr normals,
                     float radius_search
                     ){


      pcl::NormalEstimationOMP<PointType, Normal> normal_estimation_omp;
      normal_estimation_omp.setInputCloud(cloud);
      normal_estimation_omp.setRadiusSearch(radius_search);
      typename pcl::search::KdTree<PointType>::Ptr kdtree_omp(new typename pcl::search::KdTree<PointType>);
      normal_estimation_omp.setSearchMethod(kdtree_omp);
      normal_estimation_omp.compute(*normals);
}


/** Put a symbol for verification (ex: "-icp" ), a delimiter to separate the parameters(",") and the numbers of parameters
 * separeted by this delimiter. The function return a vector of strings (tokens) where each parameter is a string.
 * example: get_parameters(argc,argv,"-icp", ",");
*/
vector<string> get_parameters(int argc, char **argv ,const char *symbol, const char *delimiter ){

    std::string params_string;
    bool match = pcl::console::parse_argument (argc, argv, symbol, params_string) > 0;
    if (!match){PCL_ERROR ("Couldn't read %s parameters \n (exit is called) \n ", symbol);
        exit(1);
    }

    std::vector<std::string> tokens;
    boost::split (tokens, params_string, boost::is_any_of(delimiter), boost::token_compress_on);

    return tokens;
}

/**
 * Save cloud in pcd.
*/
void saveCloudPCD(const char* file,PointCloudXYZPtr cloud){
    cloud->width=cloud->points.size();
    cloud->height = 1;
    pcl::io::savePCDFileASCII (file, *cloud);
}

/** Open cloud in pcd. If the cannot open the file, the program finish
*/
void openCloudPCD(const char* file, PointCloudXYZPtr cloud ){

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1){
        PCL_ERROR ("Couldn't read this path: %s \n (exit is called)\n", file);
        exit(1);
    }
}


#endif // KEYPOINT_H

