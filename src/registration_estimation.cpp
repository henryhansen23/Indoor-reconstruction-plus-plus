

#include <algorithm>

#include <fstream>

#include <iostream>

#include <string> 

#include <vector>


#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <pcl/registration/transforms.h>


#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif


#include "load_data.h"

#include "registration.h"


#include <boost/filesystem.hpp>

using namespace boost::filesystem;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////


void translation_estimation(const std::string data_path, Eigen::Vector3f & translation) {


     Registration reg(0.25, 1e-8, 0.01, 5, 0.1, 0.5, 10); 


     // Data variables

     pcl::PointCloud <pcl::PointNormal>::Ptr target (new pcl::PointCloud <pcl::PointNormal>);

     pcl::PointCloud <pcl::PointNormal>::Ptr source (new pcl::PointCloud <pcl::PointNormal>);

     pcl::PointCloud <pcl::PointNormal>::Ptr target_sampled (new pcl::PointCloud <pcl::PointNormal>);

     pcl::PointCloud <pcl::PointNormal>::Ptr source_sampled (new pcl::PointCloud <pcl::PointNormal>);


     /////////////////////////////// Non-incremental pairwise registration for translation estimation /////////////////////////////////////////////


     // The first point cloud is the source

     pcl::io::loadPCDFile <pcl::PointNormal> (data_path + "/scan_0.pcd", *source);


     // Align all other point clouds pairwise


     int scans = number_of_scans(data_path); 


     for (int i = 1; i <= scans; ++i) {


          std::string scan = "/scan_" + std::to_string(i); 


          // Import new target cloud 

          pcl::io::loadPCDFile <pcl::PointNormal> (data_path + scan + ".pcd", *target);

                 
          // Prepare target cloud for alignment

          reg.normals_estimation(target); 

          reg.normal_space_sampling(target, target_sampled);


          // Prepare source cloud for alignment 

          reg.normals_estimation(source); 

          reg.normal_space_sampling(source, source_sampled); 

 
          // Align source and target

          Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity(); 

          reg.alignment_icp_nl(target_sampled, source_sampled, transformation_matrix);
 
 
          // Store translation
                      
          translation += transformation_matrix.col(3).head <3> ();


          // Target is the source in the next alignment

          source -> swap(*target);

 
          }


}


////////////////////////////// Incremental pairwise registration ///////////////////////////////////////////////////////////////////////////


void incremental_pairwise_registration(const std::vector <pcl::PointCloud <pcl::PointXYZ> > & clouds, const std::vector <Eigen::Vector3f> & translations, const std::string data_dir, const bool visualization) {


     Registration reg(0.25, 1e-8, 0.01, 5, 0.1, 1, 10); 


     // Data variables

     pcl::PointCloud <pcl::PointNormal>::Ptr target (new pcl::PointCloud <pcl::PointNormal>);

     pcl::PointCloud <pcl::PointNormal>::Ptr source (new pcl::PointCloud <pcl::PointNormal>);

     pcl::PointCloud <pcl::PointNormal>::Ptr target_sampled (new pcl::PointCloud <pcl::PointNormal>);

     pcl::PointCloud <pcl::PointNormal>::Ptr source_sampled (new pcl::PointCloud <pcl::PointNormal>);



     // The first point cloud is the source

     pcl::copyPointCloud(clouds[0], *source);


     // Align all other point clouds pairwise

     for (std::size_t i = 1; i < clouds.size(); ++i) {


         // Initital alignment 

         Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity(); 

         transformation_matrix.col(3).head <3> () = translations[i-1];

         pcl::transformPointCloud(*source, *source, transformation_matrix);


         // Import new target cloud 

         pcl::copyPointCloud(clouds[i], *target);


         // Align source and target

         std::cout << "Fragment " << i-1 << " and " << i << " to be aligned." << std::endl; 

         if (visualization) {reg.visualize(target, source); std::cout << "Close the visualization to align." << std::endl;} 


         // Prepare target cloud for alignment

         reg.normals_estimation(target); 

         reg.covariance_sampling(target, target_sampled);


         // Prepare source cloud for alignment 

         reg.normals_estimation(source); 

         reg.covariance_sampling(source, source_sampled); 


         reg.alignment_gicp(target_sampled, source_sampled, transformation_matrix);


         // Transform the source cloud and append the target cloud

         pcl::transformPointCloud(*source, *source, transformation_matrix);

         *source += *target;


         std::cout << "The alignment is completed" << std::endl;

         if (visualization) {reg.visualize(target, source); std::cout << "Close the visualization to procede." << std::endl;} 

         
     }


     pcl::io::savePCDFileBinary(data_dir + "/combined_cloud.pcd", *source);  


     // Visualize all clouds combined 

     reg.visualize(source, nullptr);


}

