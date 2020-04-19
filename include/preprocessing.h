#ifndef PREPROCESSING_H
#define PREPROCESSING_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// List the available keypoints
static const string VOXEL_GRID  = "voxel_grid";
static const string UNIFORM_SAMP    = "uniformSampling";

using namespace std;

template<typename PointT>
void preprocessing(pcl::PointCloud<PointT>::Ptr cloud_in ,
                   vector<string> params,
                   pcl::PointCloud<PointT>::Ptr cloud_out){
    if (params[0]==UNIFORM_SAMP){
        downsample(cloud_in,params_,cloud_out);
    }
    else if (params[0]==VOXEL_GRID){
        ;
    }else {
        cout<<"We not support this method. Methods Supported:"
        <<endl;
    }
}

template< typename PointT>
void downsample (typename pcl::PointCloud<PointT>::Ptr &points,
                 vector<string> params,
                 typename pcl::PointCloud<PointT>::Ptr &downsampled_out)
{
  if(params_.size()<1 || params_.size()>3){
    cout<<"Usage for voxel grid should be: downsample -downsample voxel_grid,<leaf_size_x>,<leaf_size_y>,<leaf_size_z>"<<endl;
    exit(1);
  }

  pcl::VoxelGrid<PointT> vox_grid;
  vox_grid.setLeafSize(stof(params_[0]), stof(params_[1]), stof(params_[2]));
  vox_grid.setInputCloud (points);
  vox_grid.filter (*downsampled_out);
}



#endif  // PREPROCESSING_H
