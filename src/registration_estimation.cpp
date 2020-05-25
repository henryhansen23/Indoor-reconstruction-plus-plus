

#include <algorithm>

#include <fstream>

#include <iostream>

#include <string> 

#include <vector>


#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <pcl/registration/transforms.h>


#include <Eigen/dense>


#include "registration.h"

#include "data_types.h"


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


     ///////////////////////////////// Store odometry files //////////////////////////////////////////////


     std::vector <Dir> files; 
 

     path p(data_path);
 
     for (auto i = directory_iterator(p); i != directory_iterator(); ++i) {


         std::string pcd_file = i -> path().filename().string(); 

         if (pcd_file == ".DS_Store") {continue;} //If Apple

         int no = std::stoi(pcd_file.substr(5, pcd_file.find("."))); 

         files.push_back({pcd_file, no}); 


     }

     std::sort(files.begin(), files.end(), [](Dir i, Dir j) {return i.number < j.number;}); // Sort by number


     /////////////////////////////// Non-incremental pairwise registration for translation estimation /////////////////////////////////////////////


     // The first point cloud is the source

     pcl::io::loadPCDFile <pcl::PointNormal> (data_path + "/" + files[0].name, *source);


     // Align all other point clouds pairwise

     for (std::size_t i = 1; i < files.size(); ++i) {

 
          // Import new target cloud 

          pcl::io::loadPCDFile <pcl::PointNormal> (data_path + "/" + files[i].name, *target);

                 
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

         reg.normal_space_sampling(target, target_sampled);


         // Prepare source cloud for alignment 

         reg.normals_estimation(source); 

         reg.normal_space_sampling(source, source_sampled); 


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

