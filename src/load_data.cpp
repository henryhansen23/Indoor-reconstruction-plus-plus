

#include <string> 

#include <vector> 


#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>


#include <Eigen/dense>
 

#include "data_types.h"


#include <boost/filesystem.hpp>

using namespace boost::filesystem;


///////////////////////////////////////////////////////////////////////////////////////////


int number_of_scans(const std::string data_path) {


    int number = 0, new_number; 

    const int pos_number = 5; 

    std::string scan_number;

      
    path p(data_path); 

    for (auto i = directory_iterator(p); i != directory_iterator(); ++i) {


        std::string file = i -> path().filename().string(); 

        if (file == ".DS_Store") {continue;} // If Apple
  
  
        scan_number = file.substr(pos_number, file.find("_", pos_number) - pos_number);     

        new_number = std::stoi(scan_number); 


        if (new_number > number) {

           number = new_number; 

        }

                                
    }

 
    return number; 
   

}


////////////////////////////////////////////////////////////////////////////////////////////////////////


std::vector <pcl::PointCloud <pcl::PointXYZ> > load_fragments(const std::string fragments_path) {


                                               path p(fragments_path); 

                                               std::vector <Dir> fragments; 
	

                                               for (auto i = directory_iterator(p); i != directory_iterator(); ++i) {


                                                   std::string fragments_dir = i -> path().filename().string(); 

                                                   if (fragments_dir == ".DS_Store") {continue;} // If Apple

                                                   int no = std::stoi(fragments_dir.substr(fragments_dir.find("_") + 1, fragments_dir.find("."))); 

                                                   fragments.push_back({fragments_dir, no});  


                                               } 


                                               // Sort by order 

                                               std::sort(fragments.begin(), fragments.end(), [](Dir i, Dir j) {return i.number < j.number;});


                                               // Read clouds

                                               std::vector <pcl::PointCloud <pcl::PointXYZ> > clouds; 
      
                                               for (std::size_t i = 0; i < fragments.size(); ++i) {


                                                   pcl::PointCloud <pcl::PointXYZ> cloud; 

                                                   pcl::io::loadPCDFile <pcl::PointXYZ> (fragments_path + "/" + fragments[i].name + "/fragment.pcd", cloud);

                                                   clouds.push_back(cloud); 


                                               }


                                               return clouds;                     
  
   
}


////////////////////////////////////////////////////////////////////////////////////////////////////////


std::vector <pcl::PointCloud <pcl::PointXYZ> > load_datapackets_in_one_scan(const int scan_number, const std::string datapackets_path) {

	
	                                       path p(datapackets_path); 

                                               std::vector <pcl::PointCloud <pcl::PointXYZ> > clouds; 

                                               const int pos_number = 5;

                                               std::string file_number;  


                                               for (auto i = directory_iterator(p); i != directory_iterator(); ++i) {


                                                   std::string file = i -> path().filename().string(); 

                                                   if (file == ".DS_Store") {continue;} // If Apple


                                                   file_number = file.substr(pos_number, file.find("_", pos_number) - pos_number); 
   

                                                   if (scan_number == std::stoi(file_number)) {


                                                      pcl::PointCloud <pcl::PointXYZ> cloud;

                                                      pcl::io::loadPCDFile <pcl::PointXYZ> (datapackets_path + "/" + file, cloud);

                                                      clouds.push_back(cloud);

        
                                                   }

                                            
                                               }
 

                                               return clouds;                     
  
   
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


std::vector <std::vector <pcl::PointCloud <pcl::PointXYZ> > > load_datapackets(const std::string path) {

     
                                                              std::vector <std::vector <pcl::PointCloud <pcl::PointXYZ> > > fragment_clouds; 

                                                              std::vector <pcl::PointCloud <pcl::PointXYZ> > datapackets_scan_clouds; 

                                                              int n_scans = number_of_scans(path); 


                                                              for (int i = 0; i <= n_scans; ++i) {

              
                                                                  datapackets_scan_clouds = load_datapackets_in_one_scan(i, path); 

                                                                  fragment_clouds.push_back(datapackets_scan_clouds);



                                                              }


                                                              return fragment_clouds; 


}


////////////////////////////////////////////////////////////////////////////////////////////////////////////


std::vector <Quaternion_file> read_quaternions_file(const std::string path) {

                
                              std::vector <Quaternion_file> quaternions;

                              Eigen::Vector4d quat;

                              double time;               
        
 
                              std::ifstream input(path + "/" + "quaternions_datapacket.csv");

                              std::string line;

                              const std::string delimiter = ",";


                              // Read line 

                              std::getline(input, line); // Read first line

                              while (std::getline(input, line)) {

          
	                            int pos = 0;

	                            double number;

                                    std::vector <double> row;
 

                                    // Split numbers by delimiter 

    	                            while ((pos = line.find(delimiter)) != std::string::npos)  {

    
		                          number = std::stod(line.substr(0, pos));

                                          row.push_back(number);

                                          line.erase(0, pos + delimiter.length());


                                    }


                                    // Store row      

                                    for (int j = 0; j < 4; ++j) {

                                        quat(j) = row[j]; 

                                    }

                                    time = row[4]; 

                                    quaternions.push_back({quat, time});

        
                                }


                                return quaternions; 


}


////////////////////////////////////////////////////////////////////////////////////////////////////////////


