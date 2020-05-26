

#pragma once


#include <string> 

#include <vector> 


#include <pcl/point_cloud.h>


#include <Eigen/dense>
 

///////////////////////////////////////////////////////////////////////////////////////////


std::vector <pcl::PointCloud <pcl::PointXYZ> > load_fragments(const std::string fragments_path); 

std::vector <pcl::PointCloud <pcl::PointXYZ> > load_scans(const std::string path);

std::vector <std::vector <pcl::PointCloud <pcl::PointXYZ> > > load_datapackets(const std::string path);

std::vector <std::pair <Eigen::Vector4d, double> read_quaternions_file(const std::string path); 


///////////////////////////////////////////////////////////////////////////////////////////
