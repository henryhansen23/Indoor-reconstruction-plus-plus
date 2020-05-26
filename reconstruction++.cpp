

#include <algorithm>

#include <iostream>

#include <string> 

#include <vector> 


#include <pcl/point_types.h>

#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <pcl/console/parse.h>


#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif


#include "data_types.h"

#include "load_data.h"

#include "quaternion_interpolation.h"

#include "combine_datapackets.h"

#include "registration_estimation.h"


#include <boost/filesystem.hpp>

using namespace boost::filesystem;
 

///////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv) {


    //////////////////////////////// Command line arguments /////////////////////////////////////////


    std::string data_dir = argv[1];

    bool visualization = false;

    if (argc > 2) {std::string arg = argv[2]; if (arg == "v") {visualization = true;}}


    //////////////////////////////////// Fragments ///////////////////////////////////////////////////

  
    std::cout << std::endl << "Fragments" << std::endl << std::endl; 

   
    std::vector <Dir> fragments; 


    path p_fragment(data_dir + "/fragments"); 

    for (auto i = directory_iterator(p_fragment); i != directory_iterator(); ++i) {


        std::string fragments_dir = i -> path().filename().string(); 

        if (fragments_dir == ".DS_Store") {continue;} // If Apple

        int no = std::stoi(fragments_dir.substr(fragments_dir.find("_") + 1, fragments_dir.length())); 

        fragments.push_back({fragments_dir, no});  


    }

    std::sort(fragments.begin(), fragments.end(), [](Dir i, Dir j) {return i.number < j.number;}); // Sort by number
      
  
    for (std::size_t i = 0; i < fragments.size(); ++i) {


        std::cout << fragments[i].name << std::endl; 


        // Read quaternions 

        const std::vector <Quaternion_file> quaternions = read_quaternions_file(data_dir + "/fragments/" + fragments[i].name + "/quaternions"); 


        // Interpolate quaternions 

        const std::vector <Quaternion_file> interpolated_quaternions = interpolate_quaternions(quaternions);

      
        // Load datapackets 

        std::vector <std::vector <pcl::PointCloud <pcl::PointXYZ> > > datapackets_clouds = load_datapackets(data_dir + "/fragments/" + fragments[i].name + "/datapackets"); 

    
        // Combine datapackets to fragment

        combine_datapackets_to_fragment(datapackets_clouds, interpolated_quaternions, data_dir + "/fragments/" + fragments[i].name); 
      
        
    }


    std::cout << std::endl << std::endl; 


    //////////////////////////////////// Odometry ///////////////////////////////////////////////////////////


    if (exists(data_dir + "/odometry")) {


       std::cout << "Odometry" << std::endl << std::endl; 


       std::vector <Dir> odometry; 


       path p_odometry(data_dir + "/odometry"); 

       for (auto i = directory_iterator(p_odometry); i != directory_iterator(); ++i) {


           std::string odometry_dir = i -> path().filename().string(); 

           if (odometry_dir == ".DS_Store") {continue;} // If Apple

           int no = std::stoi(odometry_dir.substr(odometry_dir.find("_") + 1, odometry_dir.length())); 

           odometry.push_back({odometry_dir, no});  


       }

       std::sort(odometry.begin(), odometry.end(), [](Dir i, Dir j) {return i.number < j.number;}); // Sort by number


       std::vector <Eigen::Vector3f> translations; 


       for (std::size_t i = 0; i < odometry.size(); ++i) {


           std::cout << odometry[i].name << std::endl; 


           // Read quaternions

           const std::vector <Quaternion_file> quaternions = read_quaternions_file(data_dir + "/odometry/" + odometry[i].name + "/quaternions"); 


           // Interpolate quaternions

           const std::vector <Quaternion_file> interpolated_quaternions = interpolate_quaternions(quaternions);


           // Load datapackets

           std::vector <std::vector <pcl::PointCloud <pcl::PointXYZ> > > datapacket_clouds = load_datapackets(data_dir + "/odometry/" + odometry[i].name + "/datapackets"); 


           // Combine datapackets to scans
 
           combine_datapackets_to_scans(datapacket_clouds, interpolated_quaternions, data_dir + "/odometry/" + odometry[i].name); 


           // Estimate translation 

           Eigen::Vector3f translation {0, 0, 0}; 

           translation_estimation(data_dir + "/odometry/" + odometry[i].name + "/scans", translation); 

           translations.push_back(translation); 

    
       }


       std::cout << std::endl << std::endl; 


       /////////////////////////// Fragment pairwise registration with odometry ////////////////////////////////////////////////////////


       std::cout << "Fragment pairwise registration" << std::endl << std::endl; 


       std::vector <pcl::PointCloud <pcl::PointXYZ> > fragment_clouds = load_fragments(data_dir + "/fragments"); 

       incremental_pairwise_registration(fragment_clouds, translations, data_dir, visualization); 


    }


    //////////////////////////////////////////// END ////////////////////////////////////////////////////////////////////////

  
    return 0; 
    

}






