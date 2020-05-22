

#include <pcl/point_cloud.h>

#include <pcl/search/kdtree.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/registration/icp_nl.h>

#include <pcl/registration/gicp.h>

#include <pcl/filters/normal_space.h>

#include <pcl/filters/covariance_sampling.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>


#include <Eigen/dense>


#include "registration.h"


Registration::Registration(float sample_porportion, 
                           double icp_transformation_epsilon, 
                           float icp_euclidean_fitness_epsilon, 
                           int icp_maximum_iterations, 
                           float icp_ransac_outlier_rejection_threshold, 
                           float icp_max_correspondence_distance, 
                           int normals_nn_search) 

    : sample_porportion_{sample_porportion}
    , icp_transformation_epsilon_{icp_transformation_epsilon} 
    , icp_euclidean_fitness_epsilon_{icp_euclidean_fitness_epsilon} 
    , icp_maximum_iterations_{icp_maximum_iterations} 
    , icp_ransac_outlier_rejection_threshold_{icp_ransac_outlier_rejection_threshold}
    , icp_max_correspondence_distance_{icp_max_correspondence_distance}
    , normals_nn_search_{normals_nn_search}

    {}


///////////////////////////////////////////////// Normals estimation ////////////////////////////////////////////////////


void Registration::normals_estimation(pcl::PointCloud <pcl::PointNormal>::Ptr cloud) {


     pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud <pcl::PointXYZ>);

     pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

     pcl::copyPointCloud(*cloud, *cloud_xyz);


     pcl::NormalEstimationOMP <pcl::PointXYZ, pcl::Normal> ne;

     pcl::search::KdTree <pcl::PointXYZ>::Ptr kd_tree (new pcl::search::KdTree <pcl::PointXYZ> ());

     ne.setSearchMethod(kd_tree);

     ne.setInputCloud(cloud_xyz);

     ne.setKSearch(normals_nn_search_);

     ne.compute(*normals);

     pcl::concatenateFields <pcl::PointNormal, pcl::Normal, pcl::PointNormal> (*cloud, *normals, *cloud);


}


////////////////////////////////////////////////// Alignment Non-linear ICP //////////////////////////////////////////////////////


void Registration::alignment_icp_nl(pcl::PointCloud <pcl::PointNormal>::Ptr target, pcl::PointCloud <pcl::PointNormal>::Ptr source, Eigen::Matrix4f & transformation) {


     pcl::PointCloud <pcl::PointNormal>::Ptr ICP_result (new pcl::PointCloud <pcl::PointNormal>);


     pcl::IterativeClosestPointNonLinear <pcl::PointNormal, pcl::PointNormal> ICP;
 

     ICP.setTransformationEpsilon(icp_transformation_epsilon_);

     ICP.setMaxCorrespondenceDistance(icp_max_correspondence_distance_); 

     ICP.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_); 

     ICP.setMaximumIterations(icp_maximum_iterations_);

     ICP.setRANSACOutlierRejectionThreshold(icp_ransac_outlier_rejection_threshold_);


     ICP.setInputTarget(target);

     ICP.setInputSource(source);      

     ICP.align(*ICP_result);


     // Source to target transformation matrix

     transformation = ICP.getFinalTransformation();

}

//////////////////////////////////////////// Alignment Generalized ICP ///////////////////////////////////////////////


void Registration::alignment_gicp(pcl::PointCloud <pcl::PointNormal>::Ptr target, pcl::PointCloud <pcl::PointNormal>::Ptr source, Eigen::Matrix4f & transformation) {


     pcl::PointCloud <pcl::PointNormal>::Ptr icp_result (new pcl::PointCloud <pcl::PointNormal>);


     pcl::GeneralizedIterativeClosestPoint <pcl::PointNormal, pcl::PointNormal> icp;

     icp.setTransformationEpsilon(icp_transformation_epsilon_);

     icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_); 

     icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_); 

     icp.setMaximumIterations(icp_maximum_iterations_);

     icp.setRANSACOutlierRejectionThreshold(icp_ransac_outlier_rejection_threshold_);


     icp.setInputTarget(target);

     icp.setInputSource(source);      

     icp.align(*icp_result);


     // Source to target transformation matrix

     transformation = icp.getFinalTransformation();

}


////////////////////////////////////////// Visualization ////////////////////////////////////////////////////


void Registration::visualize(pcl::PointCloud <pcl::PointNormal>::Ptr cloud_1, pcl::PointCloud <pcl::PointNormal>::Ptr cloud_2) {


     pcl::visualization::PCLVisualizer viz;

     viz.setBackgroundColor(255, 255, 255);


     // Cloud 1

     pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_1_xyz (new pcl::PointCloud <pcl::PointXYZ>);

     pcl::copyPointCloud(*cloud_1, *cloud_1_xyz);


     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_1_color (cloud_1_xyz, 100, 149, 237);

     viz.addPointCloud<pcl::PointXYZ> (cloud_1_xyz, cloud_1_color, "cloud 1");

     viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "cloud 1");
    

     // Cloud 2

     if (cloud_2 != nullptr) {


        pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_2_xyz (new pcl::PointCloud <pcl::PointXYZ>);

        pcl::copyPointCloud(*cloud_2, *cloud_2_xyz);


        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_2_color (cloud_2_xyz, 128, 0, 128);

        viz.addPointCloud<pcl::PointXYZ> (cloud_2_xyz, cloud_2_color, "cloud 2");

        viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "cloud 2");


     }

 
     viz.spin();


}


///////////////////////////////////// Normal space sampling ////////////////////////////////////////////////////////////////////////////////


void Registration::normal_space_sampling(pcl::PointCloud <pcl::PointNormal>::Ptr input_cloud, pcl::PointCloud <pcl::PointNormal>::Ptr output_cloud) {


     pcl::NormalSpaceSampling <pcl::PointNormal, pcl::PointNormal> normal_space_sampling_estimator;

     normal_space_sampling_estimator.setInputCloud(input_cloud);

     normal_space_sampling_estimator.setNormals(input_cloud);

     normal_space_sampling_estimator.setBins(4, 4, 4);

     normal_space_sampling_estimator.setSeed(0);

     normal_space_sampling_estimator.setSample(input_cloud -> points.size() * sample_porportion_);

     pcl::IndicesPtr indices (new std::vector <int> ());

     normal_space_sampling_estimator.filter(*indices);


     // Extract indices

     pcl::ExtractIndices <pcl::PointNormal> extraction;

     extraction.setInputCloud(input_cloud);

     extraction.setIndices(indices);

     extraction.setNegative(false);

     extraction.filter(*output_cloud);


}


//////////////////////////////////////// Covariance sampling ///////////////////////////////////////////////////////////////////////////////////


void Registration::covariance_sampling(pcl::PointCloud <pcl::PointNormal>::Ptr input_cloud, pcl::PointCloud <pcl::PointNormal>::Ptr output_cloud) {


     pcl::CovarianceSampling <pcl::PointNormal, pcl::PointNormal> covariance_sampling_estimator;

     covariance_sampling_estimator.setInputCloud(input_cloud);

     covariance_sampling_estimator.setNormals(input_cloud);

     covariance_sampling_estimator.setNumberOfSamples(input_cloud -> points.size() * sample_porportion_);

     pcl::IndicesPtr indices (new std::vector <int> ());

     covariance_sampling_estimator.filter(*indices);


     // Extract indices

     pcl::ExtractIndices <pcl::PointNormal> extraction;

     extraction.setInputCloud(input_cloud);

     extraction.setIndices(indices);

     extraction.setNegative(false);

     extraction.filter(*output_cloud);


}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////



