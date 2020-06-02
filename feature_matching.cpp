
#include "./include/utils.h"
#include "./include/features.h"
#include "./include/keypoints.h"
#include "./include/visualization.h"

#include<chrono>

using namespace std::chrono;

typedef PointXYZ PointDemo;

void demoCorrespondence(int argc, char ** argv);

int main (int argc, char ** argv){


    /** examples parameters
      -features  bs071_O_MOUTH_0-desc.pcd,bs071_N_N_0-desc.pcd  -output  matches.dat:bs071,O,MOUTH,0:bs101,N,N,0   -th 440 -debug 1
     **/
    using namespace  pcl;

    /*/
    demoCorrespondence(argc,argv);
    /*/

    auto total_start = chrono::steady_clock::now();
    vector<string> debug_params = get_parameters(argc,argv, "-debug",",");
    vector<string> features_params = get_parameters(argc,argv, "-features",",");
    vector<string> th_params = get_parameters(argc,argv, "-th",",");
    vector<string> output_params = get_parameters(argc,argv, "-output",":");

    /// load features
    PointCloud<FPFHSignature33>::Ptr descriptors1 (new PointCloud<FPFHSignature33>);
    PointCloud<FPFHSignature33>::Ptr descriptors2 (new PointCloud<FPFHSignature33>);
    io::loadPCDFile (features_params[0].c_str(), *descriptors1);
    io::loadPCDFile (features_params[1].c_str(), *descriptors2);
    if(debug_params[0]=="1")
    cout<<"\ndescriptors -> source: "<<descriptors1->points.size()<<" -- target: "<<descriptors2->points.size()<<endl;
    if(debug_params[0]=="1")
    cout<<"source: "<<features_params[0]<<" -- target: "<<features_params[1]<<endl;

    /// correspondences
    std::vector<int> correspondences;
    std::vector<float> correspondence_scores;
    find_feature_correspondences<FPFHSignature33> (descriptors1, descriptors2, correspondences, correspondence_scores);
    if(debug_params[0]=="1")cout<<"correspondences size: "<<correspondences.size()<<endl;


    /// Parameters for recognition
    std::vector<int> correspondences_histogram(correspondences.size(),0);
    std::vector<float> best_correspondences_scores;
    std::vector<float> corr_sort (correspondence_scores);
    std::sort (corr_sort.begin (), corr_sort.end ());
    float mediana_score = corr_sort[corr_sort.size()/2];
    float average=0,sumSquared=0,mediana=0,distN2=0,distN=0,dist=0;

    float threshold=0.0;
    if (th_params[0]=="th_fixe")
        threshold = stof(th_params[1]);
    else if (th_params[0]=="th_median")
        threshold = mediana_score*stof(th_params[1]);
    else if (th_params[0]=="th_k"){
        int k = stoi(th_params[1]);
        int idx = k >= corr_sort.size() ? corr_sort.size()-1: k-1 ;
        threshold = corr_sort[idx];
    }else
        threshold = corr_sort[corr_sort.size()-1];

    size_t inliers=0;
    for(size_t i=0;i<corr_sort.size();i++){
        if(corr_sort[i]<=  threshold ){
            correspondences_histogram[i]+=1;
            best_correspondences_scores.push_back(corr_sort[i]);
            average+=corr_sort[i];
            sumSquared+= static_cast<float>(pow(corr_sort[i],2));
            inliers++;
        }
    }

    if (inliers !=0){
        average=average/inliers;
        dist=sqrt(sumSquared);
        distN=sqrt(sumSquared)/inliers;
        distN2=sqrt(sumSquared)/(inliers*inliers);
        if (best_correspondences_scores.size()>0)
            mediana= best_correspondences_scores.size()%2 == 0 ? (best_correspondences_scores[inliers/2] + best_correspondences_scores[(inliers/2)-1])/2: best_correspondences_scores[inliers/2];
        if(debug_params[0]=="1")cout<<"dist media: "<<average<<"\ndist "<<dist<<"\ndist/N: "<<distN<<"\ndist/N2: "<<distN2<<"\nmediana: "<<mediana<<endl;
    }
    else{
        average=numeric_limits<float>::max();
        mediana=numeric_limits<float>::max();
        dist=numeric_limits<float>::max();
        distN=numeric_limits<float>::max();
        distN2=numeric_limits<float>::max();
    }


    /// save to file
    std::ofstream file;
    file.open(output_params[0].c_str(), std::ios_base::app);
    file<<dist<<","<<distN<<","<<distN2<<","<<average<<","<<mediana<<","<<correspondence_scores.size()<<","<<inliers<<","
       <<output_params[1]<<","
       <<output_params[2]<<endl;

    if(debug_params[0]=="1")cout<<"inliers: "<<inliers<<endl;
    if(debug_params[0]=="1")cout<<"total time: " << static_cast<duration<double>>(chrono::steady_clock::now() - total_start).count() << " s\n"<<
                               "--------------------------------------------------"<<endl;

    /// save histogram shit
    std::ofstream file_hist;
    file_hist.open((output_params[0] + ".hist").c_str(), std::ios_base::app);
    for(int h =0;h<correspondences_histogram.size();h++){
        file_hist<<correspondences_histogram[h]<<",";
    }
    file_hist<<output_params[1]<<","<<output_params[2]<<endl;


    return 0;
}


void demoCorrespondence(int argc, char ** argv){
    /** example prams in terminal
     *   -clouds cloud/bs071/bs071_O_MOUTH_0.pcd,cloud/bs101/bs101_N_N_0.pcd -downsample voxel_grid,3   -keypoints from_file,keypoints/bs071/bs071_O_MOUTH_0.pcd,keypoints/bs101/bs101_N_N_0.pcd  -keypoints  sift_normal,3,3,4,0.001  -features fpfh,40,440 -normal 10
     **/

    vector<string> clouds_params = get_parameters(argc,argv, "-clouds",",");
    vector<string> normal_params = get_parameters(argc,argv, "-normal",",");
    vector<string> downsample_params = get_parameters(argc,argv, "-downsample",",");
    vector<string> keypoints_params = get_parameters(argc,argv, "-keypoints",",");
    vector<string> features_params = get_parameters(argc,argv, "-features",",");


    using namespace  pcl;

    /// Dada for source
    PointCloud<PointDemo>::Ptr points1 (new PointCloud<PointDemo>);
    PointCloud<PointDemo>::Ptr downsampled1 (new PointCloud<PointDemo>);
    PointCloudNormalPtr normals1 (new PointCloudNormal);
    PointCloud<PointDemo>::Ptr keypoints1 (new PointCloud<PointDemo>);
    PointCloud<FPFHSignature33>::Ptr descriptors1 (new PointCloud<FPFHSignature33>);
    /// Dada for target
    PointCloud<PointDemo>::Ptr points2 (new PointCloud<PointDemo>);
    PointCloud<PointDemo>::Ptr downsampled2 (new PointCloud<PointDemo>);
    PointCloudNormalPtr normals2 (new PointCloudNormal);
    PointCloud<PointDemo>::Ptr keypoints2 (new PointCloud<PointDemo>);
    PointCloud<FPFHSignature33>::Ptr descriptors2 (new PointCloud<FPFHSignature33>);

    // Load the pair of point clouds
    io::loadPCDFile (clouds_params[0].c_str(), *points1);
    io::loadPCDFile (clouds_params[1].c_str(), *points2);
    cout<<"points      -> source: "<<points1->points.size()<<" -- target: "<<points2->points.size()<<endl;

    // normals
    if (normal_params.size()<1) {cout<<"You need especify radius for normal computing, like: -normal 25"<<endl;exit(1);}
    estimateNormals<PointDemo>(points1,normals1,stof(normal_params[0]));
    estimateNormals<PointDemo>(points2,normals2,stof(normal_params[0]));
    cout<<"normals     -> source: "<<normals1->points.size()<<" -- target: "<<normals2->points.size()<<endl;

    // keypoints
    if(keypoints_params[0]=="from_file"){
        getKeyPointsFromFile<PointDemo>(keypoints_params[1],keypoints1);
        getKeyPointsFromFile<PointDemo>(keypoints_params[2],keypoints2);
    }else {
        computeKeypoints<PointDemo,PointDemo>(points1,keypoints1,normals1,keypoints_params);
        computeKeypoints<PointDemo,PointDemo>(points2,keypoints2,normals2,keypoints_params);
    }
    cout<<"keypoints   -> source: "<<keypoints1->points.size()<<" -- target: "<<keypoints2->points.size()<<endl;

    // features
    computeFeatures<PointDemo,PointDemo,FPFHSignature33>(points1,keypoints1,normals1,descriptors1,features_params);
    computeFeatures<PointDemo,PointDemo,FPFHSignature33>(points2,keypoints2,normals2,descriptors2,features_params);
    cout<<"descriptors -> source: "<<descriptors1->points.size()<<" -- target: "<<descriptors2->points.size()<<endl;

    //// FEATURE MATCHING ///////////////////////////////////////////
    // correspondences
    std::vector<int> correspondences;
    std::vector<float> correspondence_scores;
    find_feature_correspondences<FPFHSignature33> (descriptors1, descriptors2, correspondences, correspondence_scores);
    cout<<"\ncorrespondences size: "<<correspondences.size()<<endl;

    Visualize view ;
    view.visualize_keypoints<PointDemo,PointDemo>(points1,keypoints1);
    view.visualize_keypoints<PointDemo,PointDemo>(points2,keypoints2);
    view.visualize_correspondences<PointDemo,PointDemo>(points1,keypoints1,points2,keypoints2,correspondences,correspondence_scores,stof(features_params[2]));

}


