
#include "./include/utils.h"
#include "./include/keypoints.h"
#include "./include/features.h"


typedef PointXYZ PointDemo;

int main (int argc, char ** argv)
{
    /** exemple params
     * -clouds cloud/bs009/bs009_N_N_0.pcd -downsample voxel_grid,3    -normal 10  -keypoints from_file,keypoints/bs009/bs009_N_N_0.pcd  -keypointss  sift_normal,3,3,4,0.001  -features fpfh,25 -output bs009_N_N_0_desc.pcd
    **/

    using namespace  pcl;

    /// parameters
    vector<string> clouds_params = get_parameters(argc,argv, "-cloud",",");
    vector<string> normal_params = get_parameters(argc,argv, "-normal",",");
    vector<string> keypoints_params = get_parameters(argc,argv, "-keypoints",",");
    vector<string> features_params = get_parameters(argc,argv, "-features",",");
    vector<string> output_params = get_parameters(argc,argv, "-output",",");
    vector<string> debug_params = get_parameters(argc,argv, "-debug",",");

    /// Dada for source
    PointCloud<PointDemo>::Ptr points (new PointCloud<PointDemo>);
    PointCloud<PointDemo>::Ptr downsampled (new PointCloud<PointDemo>);
    PointCloudNormalPtr normals (new PointCloudNormal);
    PointCloud<PointDemo>::Ptr keypoints (new PointCloud<PointDemo>);
    PointCloud<FPFHSignature33>::Ptr descriptors (new PointCloud<FPFHSignature33>);

    /// Load the pair of point clouds
    io::loadPCDFile (clouds_params[0].c_str(), *points);
    if(debug_params[0]=="1")cout<<"points      -> "<<points->points.size()<<endl;

    /// normals
    if (normal_params.size()<1) {cout<<"You need especify radius for normal computing, like: -normal 25"<<endl;exit(1);}
    estimateNormals<PointDemo>(points,normals,stof(normal_params[0]));
    if(debug_params[0]=="1")cout<<"normals     -> "<<normals->points.size()<<endl;

    /// keypoints
    if(keypoints_params[0]=="from_file"){
        getKeyPointsFromNose<PointDemo>(keypoints_params[1],keypoints);
    }else {
        computeKeypoints<PointDemo,PointDemo>(points,keypoints,normals,keypoints_params);
    }
    if(debug_params[0]=="1")cout<<"keypoints   -> "<<keypoints->points.size()<<endl;


    /// features
    computeFeatures<PointDemo,PointDemo,FPFHSignature33>(points,keypoints,normals,descriptors,features_params);
    if(debug_params[0]=="1")cout<<"descriptors -> "<<descriptors->points.size()<<endl;

    pcl::io::savePCDFileASCII(output_params[0].c_str(),*descriptors);
    if(debug_params[0]=="1")cout<<"file saved  -> "<<output_params[0]<<endl;

    return 0;

}



