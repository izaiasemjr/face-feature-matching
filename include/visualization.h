


#include "utils.h"


// basics PCL manipulations
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

class Visualize{

public:
    Visualize(){};
    ~Visualize(){};

    template<typename PointType,typename KeypointType>
    void visualize_keypoints (const typename pcl::PointCloud<PointType>::Ptr points,
                              const typename pcl::PointCloud<KeypointType>::Ptr keypoints
                              );

    void visualize_normals (const PointCloudXYZPtr points,
                            const PointCloudXYZPtr normal_points,
                            const PointCloudNormalPtr normals);

    template<typename PointType,typename KeypointType>
    void visualize_correspondences (const typename pcl::PointCloud<PointType>::Ptr points1,
                                    const typename pcl::PointCloud<KeypointType>::Ptr keypoints1,
                                    const typename pcl::PointCloud<PointType>::Ptr points2,
                                    const typename pcl::PointCloud<KeypointType>::Ptr keypoints2,
                                    const std::vector<int> &correspondences,
                                    const std::vector<float> &correspondence_scores,
                                    float thr_corr
                                    );

};

template<typename PointType,typename KeypointType>
void Visualize::visualize_keypoints (const typename pcl::PointCloud<PointType>::Ptr points,
                                     const typename pcl::PointCloud<KeypointType>::Ptr keypoints
                                     ){


  // Add the points to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");

  //Draw each keypoint as a sphere
  for (size_t i = 0; i < keypoints->size (); ++i)
  {
    // Get the point data
    const KeypointType & p = keypoints->points[i];

    // Pick the radius of the sphere *
    float r = 1.0 ;//2 * p.scale;
    // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
    //   radius of 2*p.scale is a good illustration of the extent of the keypoint

    // Generate a unique string for each sphere
    std::stringstream ss ("keypoint");
    ss << i;

    // Add a sphere at the keypoint
    viz.addSphere(p, r, 1.0, 0.0, 0.0, ss.str ());

    }

  // Give control over to the visualizer
  viz.spin ();
}


void Visualize::visualize_normals (const PointCloudXYZPtr points,
                        const PointCloudXYZPtr normal_points,
                        const PointCloudNormalPtr normals)
{
  // Add the points and normals to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");
  viz.addPointCloud (normal_points, "normal_points");

  viz.addPointCloudNormals<PointXYZ, Normal> (normal_points, normals, 10, 10, "normals");

  // Give control over to the visualizer
  viz.spin ();
}




template<typename PointType,typename KeypointType>
void Visualize::visualize_correspondences (const typename pcl::PointCloud<PointType>::Ptr points1,
                                const typename pcl::PointCloud<KeypointType>::Ptr keypoints1,
                                const typename pcl::PointCloud<PointType>::Ptr points2,
                                const typename pcl::PointCloud<KeypointType>::Ptr keypoints2,
                                const std::vector<int> &correspondences,
                                const std::vector<float> &correspondence_scores,
                                float thr_corr
                                ){

    using namespace pcl;

    typename PointCloud<PointType>::Ptr points_source (points1);
    typename PointCloud<PointType>::Ptr points_target (points2);
    typename PointCloud<KeypointType>::Ptr keypoints_source (keypoints1);
    typename PointCloud<KeypointType>::Ptr keypoints_target(keypoints2);
    if(correspondences.size() != keypoints1->points.size()){
        points_target=points1;keypoints_target=keypoints1;
        points_source=points2;keypoints_source=keypoints2;
    }

  // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
  // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points

  // Create some new point clouds to hold our transformed data
  typename PointCloud<PointType>::Ptr points_left (new PointCloudXYZ);
  typename PointCloud<PointType>::Ptr points_right (new PointCloudXYZ);
  typename PointCloud<KeypointType>::Ptr keypoints_left  (new PointCloud<KeypointType>);
  typename PointCloud<KeypointType>::Ptr keypoints_right (new PointCloud<KeypointType>);

  // Shift the first clouds' points to the left
  //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
  const Eigen::Vector3f translate (65.4, 15.0, 0.0);
  const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
  pcl::transformPointCloud (*points_source, *points_left, -translate, no_rotation);
  pcl::transformPointCloud (*keypoints_source, *keypoints_left, -translate, no_rotation);

  // Shift the second clouds' points to the right
  pcl::transformPointCloud (*points_target, *points_right, translate, no_rotation);
  pcl::transformPointCloud (*keypoints_target, *keypoints_right, translate, no_rotation);

  // Add the clouds to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points_left, "points_left");
  viz.addPointCloud (points_right, "points_right");

  // Compute the median correspondence score
  std::vector<float> temp (correspondence_scores);
  std::sort (temp.begin (), temp.end ());
  float median_score = temp[temp.size ()/2];

  // Draw lines between the best corresponding points
  int count=0;
  cout<<"th: "<<thr_corr<<endl;
  for (size_t i = 0; i < keypoints_left->points.size(); i++)
  {
    cout<<"correspondence_scores[i]: "<<correspondence_scores[i]<<endl;
    if(correspondence_scores[i] > temp[5] )
    //if (correspondence_scores[i] > median_score*thr_corr  )
//    if (correspondence_scores[i] > thr_corr  )
    {
      continue; // Don't draw weak correspondences
    }

    // Get the pair of points
    const KeypointType & p_left = keypoints_left->points[i];
    const KeypointType & p_right = keypoints_right->points[correspondences[i]];

    // Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max (r, std::max (g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;

    // Generate a unique string for each line
    std::stringstream ss ("line");
    ss << i;

    // Draw the line
    viz.addLine (p_left, p_right, r, g, b, ss.str ());
  }

  /// Parameters for recognition
  std::vector<float> best_correspondences_scores;
  std::vector<float> corr_sort (correspondence_scores);
  std::sort (corr_sort.begin (), corr_sort.end ());
  float mediana_score = corr_sort[corr_sort.size()/2];
//    int max=9;
//    int size = corr_sort.size()>= max ? max: corr_sort.size() ;

  float average=0,dist=0,mediana=0;
//    for(int i=0;i<size;i++){
  size_t inliers=0;
  for(size_t i=0;i<corr_sort.size();i++){
      //if(corr_sort[i]< corr_sort[9] ){
      //if(corr_sort[i]< mediana_score*thr_corr ){
      if(corr_sort[i]< corr_sort[5]/*thr_corr*/ ){
          best_correspondences_scores.push_back(corr_sort[i]);
          average+=corr_sort[i];
          dist+= static_cast<float>(pow(corr_sort[i],2));
          inliers++;
      }
  }

  if (inliers !=0){
      average=average/inliers;
      dist=sqrt(dist/inliers);
      float dist2=sqrt(dist)/inliers;

      if (best_correspondences_scores.size()>0)
          mediana= best_correspondences_scores.size()%2 == 0 ? (best_correspondences_scores[inliers/2] + best_correspondences_scores[(inliers/2)-1])/2: best_correspondences_scores[inliers/2];
      cout<<"dist media: "<<average<<"\ndist quadratica: "<<dist<<"\ndist quadratica2: "<<dist2<<"\nmediana: "<<mediana<<endl;
      cout<<"inliers: "<<inliers<<endl;
  }
  else{
      average=numeric_limits<float>::max();
      dist=numeric_limits<float>::max();
      mediana=numeric_limits<float>::max();
  }

  // Give control over to the visualizer
  viz.spin ();
}

//#endif VISUALIZATION_H

