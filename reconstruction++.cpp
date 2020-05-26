

#include <iostream>

#include <string> 

#include <vector> 


#include <pcl/point_types.h>


#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif


#include <boost/filesystem.hpp>


#include "load_data.h"

#include "quaternion_interpolation.h"

#include "combine_datapackets.h"

#include "registration_estimation.h"


///////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv) {


    //////////////////////////////// Command line arguments /////////////////////////////////////////


    std::string data_dir = argv[1];

    bool visualization = false;

    if (argc > 2) {std::string arg = argv[2]; if (arg == "v") {visualization = true;}}


    //////////////////////////////////// Fragments ///////////////////////////////////////////////////

  
    std::cout << std::endl << "Fragments" << std::endl << std::endl; 


    int fragments = number_of_directories(data_dir + "/fragments"); 


    for (int i = 0; i <= fragments; ++i) {
     
                
        std::cout << i << std::endl; 
        
       
        std::string fragment = "fragment_" + std::to_string(i); 
            

        // Read quaternions 

        const std::vector <std::pair <Eigen::Vector4d, double> > quaternions = read_quaternions_file(data_dir + "/fragments/" + fragment + "/quaternions"); 


        // Interpolate quaternions 

        const std::vector <std::pair <Eigen::Vector4d, double> > interpolated_quaternions = interpolate_quaternions(quaternions);

      
        // Load datapackets 

        std::vector <std::vector <pcl::PointCloud <pcl::PointXYZ> > > datapackets_clouds = load_datapackets(data_dir + "/fragments/" + fragment + "/datapackets"); 

    
        // Combine datapackets to fragment

        combine_datapackets_to_fragment(datapackets_clouds, interpolated_quaternions, data_dir + "/fragments/" + fragment); 
      
        
    }


    std::cout << std::endl << std::endl; 


    //////////////////////////////////// Odometry ///////////////////////////////////////////////////////////


    if (boost::filesystem::exists(data_dir + "/odometry")) {


       std::cout << "Odometry" << std::endl << std::endl; 

        
       int odometries = number_of_directories(data_dir + "/odometry"); 


       std::vector <Eigen::Vector3f> translations; 


       for (int i = 0; i <= odometries; ++i) {


           std::cout << i << std::endl; 
        
       
           std::string odometry = "odometry_" + std::to_string(i); 
            

           // Read quaternions

           const std::vector <std::pair <Eigen::Vector4d, double> > quaternions = read_quaternions_file(data_dir + "/odometry/" + odometry  + "/quaternions"); 


           // Interpolate quaternions

           const std::vector <std::pair <Eigen::Vector4d, double> > interpolated_quaternions = interpolate_quaternions(quaternions);


           // Load datapackets

           std::vector <std::vector <pcl::PointCloud <pcl::PointXYZ> > > datapacket_clouds = load_datapackets(data_dir + "/odometry/" + odometry + "/datapackets"); 


           // Combine datapackets to scans
 
           combine_datapackets_to_scans(datapacket_clouds, interpolated_quaternions, data_dir + "/odometry/" + odometry); 


           // Estimate translation 

           Eigen::Vector3f translation {0, 0, 0}; 

           translation_estimation(data_dir + "/odometry/" + odometry + "/scans", translation); 

           translations.push_back(translation); 

    
       }


       std::cout << std::endl << std::endl; 


       /////////////////////////// Fragment pairwise registration with odometry ////////////////////////////////////////////////////////


       std::cout << "Fragment pairwise registration" << std::endl << std::endl; 


       std::vector <pcl::PointCloud <pcl::PointXYZ> > fragment_clouds = load_fragments(data_dir + "/fragments", fragments); 

       incremental_pairwise_registration(fragment_clouds, translations, data_dir, visualization); 


    }


    //////////////////////////////////////////// END ////////////////////////////////////////////////////////////////////////

  
    return 0; 
    

}






