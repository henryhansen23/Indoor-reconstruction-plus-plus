

#pragma once


#include <string> 

#include <vector> 


#include <pcl/point_cloud.h>


#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif
 

///////////////////////////////////////////////////////////////////////////////////////////



int number_of(const std::string data_path, const int pos_start_number, const std::string sep);

std::vector <pcl::PointCloud <pcl::PointXYZ> > load_fragments(const std::string fragments_path, const int fragments_number); 

std::vector <std::vector <pcl::PointCloud <pcl::PointXYZ> > > load_datapackets(const std::string path);

std::vector <std::pair <Eigen::Vector4d, double> > read_quaternions_file(const std::string path); 


///////////////////////////////////////////////////////////////////////////////////////////
