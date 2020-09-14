

#include <string> 

#include <utility>

#include <vector> 


#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <pcl/common/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>


#include "transformation.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


std::vector <std::vector <Eigen::Vector4d> > quaternions_scan_assignment(const std::vector <Eigen::Vector4d> & quaternions, std::vector <std::vector <pcl::PointCloud <pcl::PointXYZ> > > & datapacket_clouds) {


                                             int number, current_number = 0, last_number = 0; 

                                             std::vector <std::vector <Eigen::Vector4d> > quaternions_scans; 

                                             std::vector <Eigen::Vector4d> quaternions_data_packets;
   

                                             // Assign the qauternions to the respective datapackets and scans  

                                             for (std::size_t i = 0; i < datapacket_clouds.size(); ++i) {

         
                                                 number = (int) datapacket_clouds[i].size(); 

                                                 current_number += number;


                                                 for (int j = last_number; j < current_number; ++j) {

                                                     quaternions_data_packets.push_back(quaternions[j]); 
 
                                                 }


                                                 quaternions_scans.push_back(quaternions_data_packets);

                                                 last_number += number;

                                                 quaternions_data_packets.clear(); 


                                             }


                                             return quaternions_scans; 


}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void combine_datapackets_to_scans(std::vector <std::vector <pcl::PointCloud <pcl::PointXYZ> > > datapacket_clouds, const std::vector <Eigen::Vector4d> & quaternions, const std::string path) {


     const std::vector <std::vector <Eigen::Vector4d> > quaternions_scans = quaternions_scan_assignment(quaternions, datapacket_clouds);


     std::vector <Eigen::Matrix4d> transformation_matrices;


     boost::filesystem::create_directory(path + "/scans");
 
        
     for (std::size_t i = 0; i < datapacket_clouds.size(); ++i) {

        
         // Make transformation matrices from quaternions 

         std::vector <Eigen::Matrix4d> transformation_matrices = make_transformation_matrices(quaternions_scans[i]); 


         pcl::PointCloud <pcl::PointXYZ>::Ptr datapackets_combined (new pcl::PointCloud <pcl::PointXYZ>);


         for (std::size_t j = 0; j < datapacket_clouds[i].size(); ++j) {


             pcl::transformPointCloud(datapacket_clouds[i][j], datapacket_clouds[i][j], transformation_matrices[j]);

             *datapackets_combined += datapacket_clouds[i][j]; 


         }
 
        
         // Save scan 

         pcl::PointCloud <pcl::PointNormal>::Ptr scan (new pcl::PointCloud <pcl::PointNormal>);

         pcl::copyPointCloud(*datapackets_combined, *scan);

         pcl::io::savePCDFileBinary(path + "/scans/scan_" + std::to_string(i) + ".pcd", *scan);  


         transformation_matrices.clear(); 


     }


}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void combine_datapackets_to_fragment(std::vector <std::vector <pcl::PointCloud <pcl::PointXYZ> > > datapacket_clouds, const std::vector <Eigen::Vector4d> & quaternions, const std::string path) {


     const std::vector <std::vector <Eigen::Vector4d> > quaternions_scans = quaternions_scan_assignment(quaternions, datapacket_clouds);
  

     pcl::PointCloud <pcl::PointXYZ>::Ptr datapackets_combined (new pcl::PointCloud <pcl::PointXYZ>);


     std::vector <Eigen::Matrix4d> transformation_matrices;

        
     for (std::size_t i = 0; i < datapacket_clouds.size(); ++i) {

        
         // Make transformation matrices from quaternions 

         std::vector <Eigen::Matrix4d> transformation_matrices = make_transformation_matrices(quaternions_scans[i]); 


         for (std::size_t j = 0; j < datapacket_clouds[i].size(); ++j) {


             pcl::transformPointCloud(datapacket_clouds[i][j], datapacket_clouds[i][j], transformation_matrices[j]);

             *datapackets_combined += datapacket_clouds[i][j]; 


         }
   

         transformation_matrices.clear(); 


     }

     
     // Save fragment 
  
     pcl::io::savePCDFileBinary(path + "/fragment.pcd", *datapackets_combined); 

     
     // Visualize fragment 

     pcl::visualization::PCLVisualizer viz;

     viz.setBackgroundColor(255, 255, 255);

     pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> cloud_color (datapackets_combined, 0, 255, 0);

     viz.addPointCloud <pcl::PointXYZ> (datapackets_combined, cloud_color, "cloud 1");

     viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "cloud 1");
    
     viz.spin();


}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


