
#include "utils.h"
#include "pcl/features/fpfh.h"


// List the available keypoints
static const string FPFH  = "fpfh";
static const string PFH   = "PFH";

using namespace std;


float dist_manhattan(float *source_histogram,float *target_histogram, size_t size){
    float sum=0;
    for (int i=0; i<size; i++){
        sum+= abs(source_histogram[i] - target_histogram[i]);
    }
    return sqrt(sum);
}


template <typename FeatureType>
void find_feature_correspondences (typename pcl::PointCloud<FeatureType>::Ptr descriptors1,
                              typename pcl::PointCloud<FeatureType>::Ptr descriptors2,
                              std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{
    // let be correspondences univoque
    typename pcl::PointCloud<FeatureType>::Ptr source_descriptors(descriptors1);
    typename pcl::PointCloud<FeatureType>::Ptr target_descriptors(descriptors2);
    if(descriptors1->points.size() > descriptors2->points.size()){
        source_descriptors = descriptors2;
        target_descriptors = descriptors1;
    }


  // Resize the output vector
  correspondences_out.resize (source_descriptors->size ());
  correspondence_scores_out.resize (source_descriptors->size ());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<FeatureType> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  float dist;
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source_descriptors->size (); ++i)
  {

    auto first_value = source_descriptors->points[i].histogram[0];
    if (first_value!=first_value)
        continue;

    descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
    correspondences_out[i] = k_indices[0];
    //correspondence_scores_out[i] = k_squared_distances[0];
    dist = dist_manhattan(source_descriptors->points[i].histogram,target_descriptors->points[k_indices[0]].histogram,33);
    correspondence_scores_out[i] = dist;
  }
}

template <typename FeatureType>
void find_feature_correspondences_fixes(
                              typename pcl::PointCloud<FeatureType>::Ptr descriptors1,
                              typename pcl::PointCloud<FeatureType>::Ptr descriptors2,
                              std::vector<int> &correspondences_out,
                              std::vector<float> &correspondence_scores_out,
                              string metric
                              )
{
    // let be correspondences univoque
    typename pcl::PointCloud<FeatureType>::Ptr source_descriptors(descriptors1);
    typename pcl::PointCloud<FeatureType>::Ptr target_descriptors(descriptors2);
    if(descriptors1->points.size() > descriptors2->points.size()){
        source_descriptors = descriptors2;
        target_descriptors = descriptors1;
    }


  // Resize the output vector
  correspondences_out.resize (source_descriptors->size (),0);
  correspondence_scores_out.resize (source_descriptors->size (), numeric_limits<float>::max() );

  // Use a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<FeatureType> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  float dist;
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);

  for (size_t i = 0; i < source_descriptors->size (); ++i)
  {

    auto first_value = source_descriptors->points[i]. histogram[0];
    if (first_value!=first_value)
         continue;

    dist = dist_manhattan(source_descriptors->points[i].histogram,target_descriptors->points[i].histogram,33);
    correspondences_out[i] = i;
    correspondence_scores_out[i] = dist;
  }
}



pcl::PointIndicesPtr getIndicesKeypoints(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints
        ){

    pcl::PointIndicesPtr indices (new pcl::PointIndices);
    /** get indexes **/
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    vector<float> dist;
    vector<int> idx;
    for (int i =0;i<cloud_keypoints->points.size();++i){
        // if point is invalid, pass to next iteration
        if(!pcl::isFinite(cloud_keypoints->points[i]))
            continue;
        tree.nearestKSearch(cloud_keypoints->points [i],1, idx,dist);
        indices->indices.push_back(idx[0]);
    }
    return indices;

}

template<typename PointT,typename KeyPointsType>
void compute_FPFH_features_at_keypoints (typename pcl::PointCloud<PointT>::Ptr cloud,
                                         typename pcl::PointCloud<KeyPointsType>::Ptr &keypoints,
                                         pcl::PointCloud<Normal>::Ptr normals,
                                         pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptors_out,
                                         vector<string> params
                                         ){
    using namespace  pcl;
    float feature_radius = 25;
    if(params.size()>1)
        feature_radius = stof(params[1]);

    // PFH estimation object.
    typename search::KdTree<PointT>::Ptr kdtree(new search::KdTree<PointT>);
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(kdtree);
    fpfh.setRadiusSearch(feature_radius);

    //
    fpfh.setSearchSurface(cloud);
    fpfh.setInputCloud(keypoints);
    /*/
    fpfh.setInputCloud(cloud);
    fpfh.setIndices(getIndicesKeypoints(cloud,keypoints));
    //*/

    // Compute the features
    fpfh.compute (*descriptors_out);
}





template<typename PointT,typename KeyPointType,typename FeatureType>
void computeFeatures(typename pcl::PointCloud<PointT>::Ptr& cloud,
                     typename pcl::PointCloud<KeyPointType>::Ptr cloud_keypoints,
                     pcl::PointCloud<Normal>::Ptr &normals,
                     pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors_out,
                     vector<string> params
                    ){
    if(params.size()<1)
        cout<<"You should especify at least the methode name like: -keypoints sift_normal"<<endl;

    if (params[0]==FPFH){
        //cout<<"computing FPFH ..."<<endl;
        compute_FPFH_features_at_keypoints<PointT, KeyPointType>(cloud,cloud_keypoints,normals,descriptors_out,params);
    }
    else if (params[0]==PFH){
        ;
    }else {
        cout<<"We not support this method. Methods Supported:"<<endl;
    }

    return ;
}

