

#pragma once


#include <pcl/point_cloud.h>


#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif


class Registration {


private:


    // Registration parameters 

    float sample_porportion_; 

    double icp_transformation_epsilon_; 
    
    float icp_euclidean_fitness_epsilon_; 

    int icp_maximum_iterations_; 

    float icp_ransac_outlier_rejection_threshold_; 

    float icp_max_correspondence_distance_; 


    // Normals estimation parameter

    int normals_nn_search_;


public: 


    Registration(float sample_porportion, 
                 double icp_transformation_epsilon, 
                 float icp_euclidean_fitness_epsilon, 
                 int icp_maximum_iterations, 
                 float icp_ransac_outlier_rejection_threshold, 
                 float icp_max_correspondence_distance, 
                 int normals_nn_search);


    void normals_estimation(pcl::PointCloud <pcl::PointNormal>::Ptr cloud);


    void normal_space_sampling(pcl::PointCloud <pcl::PointNormal>::Ptr input_cloud, pcl::PointCloud <pcl::PointNormal>::Ptr output_cloud);


    void covariance_sampling(pcl::PointCloud <pcl::PointNormal>::Ptr input_cloud, pcl::PointCloud <pcl::PointNormal>::Ptr output_cloud);


    void alignment_icp_nl(const pcl::PointCloud <pcl::PointNormal>::Ptr target, const pcl::PointCloud <pcl::PointNormal>::Ptr source, Eigen::Matrix4f & transformation); 


    void alignment_gicp(const pcl::PointCloud <pcl::PointNormal>::Ptr target, const pcl::PointCloud <pcl::PointNormal>::Ptr source, Eigen::Matrix4f & transformation); 


    void visualize(pcl::PointCloud <pcl::PointNormal>::Ptr cloud_1, pcl::PointCloud <pcl::PointNormal>::Ptr cloud_2);


};



